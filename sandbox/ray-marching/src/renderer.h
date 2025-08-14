#ifndef _RENDERER_H
#define _RENDERER_H
#include <algorithm>
#include <random>
#include <vector>

#include "glad/gl.h"
#include "glm/glm.hpp"
//
#include "gcss/camera.h"
#include "gcss/quad.h"
#include "gcss/texture.h"

#include "bvh.h"

using namespace gcss;

class Renderer {
private:
    glm::uvec2 resolution;
    Camera camera;

    glm::vec3 backgroundColor;

    // multi spheres
    std::vector<SphereData> spheres;
    std::vector<SphereDataPacked> packedSpheres;
    GLuint sphereBuffer = 0;
    static constexpr int MAX_SPHERES = 64;

    float stepSize = 20.0f;

    // BVH
    BVH bvh;
    GLuint bvhBuffer = 0;
    GLuint sphereIndexBuffer = 0;
    bool useBVH = false;

    // Multi-pass rendering textures
    Texture primaryTexture; // First pass output

    // Pass: Final display
    Quad quad;
    VertexShader vertexShader;
    FragmentShader fragmentShader;
    Pipeline renderPipeline;

    // Spatial hash parameters
    static constexpr int HASH_TABLE_SIZE = 4096 * 256; // Should be power of 2
    static constexpr float CELL_SIZE = 20.0f;          // Size of each grid cell
    bool useSpatialHash = false;                       // Toggle acceleration
    GLuint hashTableBuffer = 0;                        // OpenGL buffer for hash table

    // Compute shader for building the hash table
    ComputeShader hashBuildShader;
    Pipeline hashBuildPipeline;

    // deprecated
    ComputeShader timeConsumeShader;
    Pipeline timeConsumePipeline;

    ComputeShader cullStatisticShader;
    Pipeline cullStatisticPipeline;
    Texture cullStatisticTexture;

    ComputeShader rayMarchingBVHShader;
    Pipeline rayMarchingBVHPipeline;

    ComputeShader rayMarchingHashShader;
    Pipeline rayMarchingHashPipeline;

    ComputeShader rayMarchingBruteForceShader;
    Pipeline rayMarchingBruteForcePipeline;

public:
    Renderer()
        : resolution{512, 512}, primaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          cullStatisticTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.vert"),
          fragmentShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.frag"),
          hashBuildShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "build-spatial-hash.comp"),
          timeConsumeShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "time-consume.comp"),
          cullStatisticShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "cull-statistics.comp"),
          rayMarchingBVHShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "ray-marching-bvh-step.comp"),
          rayMarchingHashShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "ray-marching-hash.comp"),
          rayMarchingBruteForceShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "ray-marching-brute-force.comp") {
        // Setup pipelines
        renderPipeline.attachVertexShader(vertexShader);
        renderPipeline.attachFragmentShader(fragmentShader);
        hashBuildPipeline.attachComputeShader(hashBuildShader);
        timeConsumePipeline.attachComputeShader(timeConsumeShader);
        cullStatisticPipeline.attachComputeShader(cullStatisticShader);

        rayMarchingBVHPipeline.attachComputeShader(rayMarchingBVHShader);
        rayMarchingHashPipeline.attachComputeShader(rayMarchingHashShader);
        rayMarchingBruteForcePipeline.attachComputeShader(rayMarchingBruteForceShader);

        // create spheres
        generateCloudSpheres();
        createSphereBuffer();

        // create hash table buffer
        createHashTableBuffer();

        // create BVH buffer
        createBVHBuffer();
        buildBVH();
    }

    ~Renderer() {
        // Clean up OpenGL resources
        if (sphereBuffer != 0) {
            glDeleteBuffers(1, &sphereBuffer);
            sphereBuffer = 0;
        }
        // Existing cleanup...
        if (hashTableBuffer != 0) {
            glDeleteBuffers(1, &hashTableBuffer);
            hashTableBuffer = 0;
        }
        if (bvhBuffer != 0) {
            glDeleteBuffers(1, &bvhBuffer);
        }
        if (sphereIndexBuffer != 0) {
            glDeleteBuffers(1, &sphereIndexBuffer);
        }
    }

    glm::uvec2 getResolution() const {
        return this->resolution;
    }

    void setResolution(const glm::uvec2& resolutionArg) {
        this->resolution = resolutionArg;
        primaryTexture.resize(resolutionArg);
        cullStatisticTexture.resize(resolutionArg);
    }

    void setBackgroundColor(const glm::vec3& color) {
        backgroundColor = color;
    }

    glm::vec3 getBackgroundColor() const {
        return backgroundColor;
    }

    void move(const CameraMovement& movement_direction, float delta_time) {
        camera.move(movement_direction, delta_time);
    }

    void lookAround(float d_phi, float d_theta) {
        camera.lookAround(d_phi, d_theta);
    }

    void render() {
        if (useBVH) {
            rayMarchingBVH();
        } else if (useSpatialHash) {
            buildSpatialHash();
            rayMarchingHash();
        } else {
            rayMarchingBruteForce();
        }
        // Final display
        renderPassDisplay();
    }

private:
    void rayMarchingBVH() {
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        if (sphereBuffer != 0) {
            glBindBufferBase(GL_UNIFORM_BUFFER, 1, sphereBuffer);
        }

        if (bvhBuffer != 0 && sphereIndexBuffer != 0 && !bvh.getGPUNodes().empty()) {
            glBindBufferBase(GL_UNIFORM_BUFFER, 2, bvhBuffer);
            glBindBufferBase(GL_UNIFORM_BUFFER, 3, sphereIndexBuffer);
        }

        rayMarchingBVHShader.setUniform("cameraPosition", camera.camPos);
        rayMarchingBVHShader.setUniform("cameraFront", camera.camForward);
        rayMarchingBVHShader.setUniform("cameraUp", camera.camUp);
        rayMarchingBVHShader.setUniform("cameraRight", camera.camRight);
        rayMarchingBVHShader.setUniform("backgroundColor", backgroundColor);
        rayMarchingBVHShader.setUniform("aabbMin", aabbMin);
        rayMarchingBVHShader.setUniform("aabbMax", aabbMax);

        rayMarchingBVHShader.setUniform("stepSize", stepSize);

        rayMarchingBVHPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "Ray Marching BVH");
        glDispatchCompute(
            (GLuint)std::ceil(resolution.x / 8.0f), (GLuint)std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        rayMarchingBVHPipeline.deactivate();

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void rayMarchingBruteForce() {
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        if (sphereBuffer != 0) {
            glBindBufferBase(GL_UNIFORM_BUFFER, 1, sphereBuffer);
        }

        rayMarchingBruteForceShader.setUniform("cameraPosition", camera.camPos);
        rayMarchingBruteForceShader.setUniform("cameraFront", camera.camForward);
        rayMarchingBruteForceShader.setUniform("cameraUp", camera.camUp);
        rayMarchingBruteForceShader.setUniform("cameraRight", camera.camRight);
        rayMarchingBruteForceShader.setUniform("backgroundColor", backgroundColor);
        rayMarchingBruteForceShader.setUniform("aabbMin", aabbMin);
        rayMarchingBruteForceShader.setUniform("aabbMax", aabbMax);

        rayMarchingBruteForceShader.setUniform("stepSize", stepSize);

        rayMarchingBruteForcePipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "Ray Marching Brute Force");
        glDispatchCompute(
            (GLuint)std::ceil(resolution.x / 8.0f), (GLuint)std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        rayMarchingBruteForcePipeline.deactivate();

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void rayMarchingHash() {
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        if (sphereBuffer != 0) {
            glBindBufferBase(GL_UNIFORM_BUFFER, 1, sphereBuffer);
        }

        if (hashTableBuffer != 0) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);
        }

        rayMarchingHashShader.setUniform("cameraPosition", camera.camPos);
        rayMarchingHashShader.setUniform("cameraFront", camera.camForward);
        rayMarchingHashShader.setUniform("cameraUp", camera.camUp);
        rayMarchingHashShader.setUniform("cameraRight", camera.camRight);
        rayMarchingHashShader.setUniform("backgroundColor", backgroundColor);
        rayMarchingHashShader.setUniform("aabbMin", aabbMin);
        rayMarchingHashShader.setUniform("aabbMax", aabbMax);

        rayMarchingHashShader.setUniform("stepSize", stepSize);
        rayMarchingHashShader.setUniform("cellSize", CELL_SIZE);

        rayMarchingHashPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "Ray Marching Hash");
        glDispatchCompute(
            (GLuint)std::ceil(resolution.x / 8.0f), (GLuint)std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        rayMarchingHashPipeline.deactivate();

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void timeConsumePass() {
        // Bind output texture
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        // Bind sphere buffer
        if (sphereBuffer != 0) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
        }

        // Bind hash table if using acceleration
        if (useSpatialHash) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);
        }

        if (useBVH) {
            if (bvhBuffer != 0 && sphereIndexBuffer != 0 && !bvh.getGPUNodes().empty()) {
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, bvhBuffer);
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, sphereIndexBuffer);
            } else {
                useBVH = false;
            }
        }

        // Pass acceleration toggle to shader
        timeConsumeShader.setUniform("useSpatialHash", useSpatialHash);
        timeConsumeShader.setUniform("useBVH", useBVH);
        timeConsumeShader.setUniform("cellSize", CELL_SIZE);

        // Set uniforms - add AABB data
        timeConsumeShader.setUniform("cameraPosition", camera.camPos);
        timeConsumeShader.setUniform("cameraFront", camera.camForward);
        timeConsumeShader.setUniform("cameraUp", camera.camUp);
        timeConsumeShader.setUniform("cameraRight", camera.camRight);
        timeConsumeShader.setUniform("aabbMin", aabbMin);
        timeConsumeShader.setUniform("aabbMax", aabbMax);

        timeConsumePipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "time consume");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        timeConsumePipeline.deactivate();

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void cullStatisticsPass() {

        if (!useSpatialHash) {
            return;
        }

        // Bind output texture
        cullStatisticTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        // Bind sphere buffer
        if (sphereBuffer != 0) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
        }

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, bvhBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, sphereIndexBuffer);

        cullStatisticShader.setUniform("cellSize", CELL_SIZE);

        // Set uniforms - add AABB data
        cullStatisticShader.setUniform("cameraPosition", camera.camPos);
        cullStatisticShader.setUniform("cameraFront", camera.camForward);
        cullStatisticShader.setUniform("cameraUp", camera.camUp);
        cullStatisticShader.setUniform("cameraRight", camera.camRight);
        cullStatisticShader.setUniform("aabbMin", aabbMin);
        cullStatisticShader.setUniform("aabbMax", aabbMax);

        cullStatisticPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "cull statistics");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        cullStatisticPipeline.deactivate();

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void buildSpatialHash() {
        glBindBuffer(GL_UNIFORM_BUFFER, hashTableBuffer);
        std::vector<uint64_t> emptyTable(HASH_TABLE_SIZE, 0);
        glBufferData(
            GL_UNIFORM_BUFFER, emptyTable.size() * sizeof(uint64_t), emptyTable.data(),
            GL_DYNAMIC_DRAW);

        // Bind input spheres and output hash table
        glBindBufferBase(GL_UNIFORM_BUFFER, 1, sphereBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);

        // Set uniforms
        hashBuildShader.setUniform("cellSize", CELL_SIZE);
        hashBuildShader.setUniform("aabbMin", aabbMin);
        hashBuildShader.setUniform("aabbMax", aabbMax);

        // Dispatch shader - one thread per sphere
        hashBuildPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 1, -1, "Build Spatial Hash");
        glDispatchCompute((GLuint)std::ceil(MAX_SPHERES / 64.0f), 1, 1);
        glPopDebugGroup();
        hashBuildPipeline.deactivate();

        // Memory barrier
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }

    void buildBVH() {
        bvh.build(packedSpheres);
        bvh.printStats();

        const auto& nodes = bvh.getGPUNodes();
        const auto& indices = bvh.getGPUSphereIndices();

        const GLintptr bvhDataOffset = 16;

        glBindBuffer(GL_UNIFORM_BUFFER, bvhBuffer);
        glBufferData(
            GL_UNIFORM_BUFFER, 16 + nodes.size() * sizeof(BVHNodeGPU), nullptr, GL_STATIC_DRAW);

        int nodeCount = static_cast<int>(nodes.size());
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(int), &nodeCount);
        glBufferSubData(
            GL_UNIFORM_BUFFER, bvhDataOffset, nodes.size() * sizeof(BVHNodeGPU), nodes.data());

        glBindBuffer(GL_UNIFORM_BUFFER, sphereIndexBuffer);
        std::vector<uint32_t> sphereIndices;
        sphereIndices.reserve(indices.size() * 4); // 4 uint32_t per sphere index
        for (auto i : indices) {
            sphereIndices.push_back(i);
            sphereIndices.push_back(0);
            sphereIndices.push_back(0);
            sphereIndices.push_back(0);
        }

        glBufferData(
            GL_UNIFORM_BUFFER, sphereIndices.size() * sizeof(uint32_t), sphereIndices.data(),
            GL_STATIC_DRAW);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    }

    void renderPassDisplay() {
        // Clear and setup viewport
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, resolution.x, resolution.y);

        primaryTexture.bindToTextureUnit(0);

        // Draw quad
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 3, -1, "Display");
        quad.draw(renderPipeline);
        glPopDebugGroup();
    }

    void generateCloudSpheres() {
        spheres.clear();

        std::mt19937 gen(10);

        // Cloud-like distribution parameters
        std::uniform_real_distribution<float> xDist(-1000.0f, 1000.0f);
        std::uniform_real_distribution<float> yDist(100.0f, 1000.0f);
        std::uniform_real_distribution<float> zDist(-4000.0f, -10.0f);
        std::uniform_real_distribution<float> radiusDist(50.0f, 200.0f);
        std::uniform_real_distribution<float> densityDist(0.0f, 1.0f);

        // Cloud color variations (white to light gray)
        std::uniform_real_distribution<float> brightnessVariation(0.7f, 1.0f);
        std::uniform_real_distribution<float> colorTint(0.95f, 1.0f);

        for (int i = 0; i < MAX_SPHERES; i++) {
            SphereData sphere;

            // Position: scattered in sky-like formation
            sphere.centerX = xDist(gen);
            sphere.centerY = yDist(gen);
            sphere.centerZ = zDist(gen);

            // Add some clustering effect for more realistic cloud formation
            if (densityDist(gen) < 0.3f) {
                if (!spheres.empty()) {
                    int clusterTarget = gen() % spheres.size();
                    sphere.centerX = spheres[clusterTarget].centerX
                                     + std::uniform_real_distribution<float>(-8.0f, 8.0f)(gen);
                    sphere.centerY = spheres[clusterTarget].centerY
                                     + std::uniform_real_distribution<float>(-3.0f, 3.0f)(gen);
                    sphere.centerZ = spheres[clusterTarget].centerZ
                                     + std::uniform_real_distribution<float>(-5.0f, 5.0f)(gen);
                }
            }

            // Size: varied like real clouds
            sphere.radius = radiusDist(gen);

            // Color: cloud-like whites and light grays with subtle variations
            float brightness = brightnessVariation(gen);
            float rTint = colorTint(gen);
            float gTint = colorTint(gen);
            float bTint = std::uniform_real_distribution<float>(0.98f, 1.0f)(gen);

            sphere.colorR = brightness * rTint;
            sphere.colorG = brightness * gTint;
            sphere.colorB = brightness * bTint;
            sphere.padding = 0.0f;

            spheres.push_back(sphere);
        }

        // auto sortStart = std::chrono::high_resolution_clock::now();
        // SpatialSorter::sortSpheres(spheres);
        // auto sortEnd = std::chrono::high_resolution_clock::now();

        // auto sortDuration = std::chrono::duration_cast<std::chrono::microseconds>(sortEnd - sortStart);
        // spdlog::info("Spatial sorting completed in {} μs", sortDuration.count());

        packedSpheres.reserve(spheres.size());
        for (const auto& sphere : spheres) {
            packedSpheres.emplace_back(sphere);
        }
    }

    void createSphereBuffer() {
        glGenBuffers(1, &sphereBuffer);
        updateSphereBuffer();
    }

    void updateSphereBuffer() {
        // uniform buffer object
        if (sphereBuffer == 0) {
            std::cerr << "Sphere buffer not created!" << std::endl;
            return;
        }
        calculateAABB();
        // Bind the buffer
        glBindBuffer(GL_UNIFORM_BUFFER, sphereBuffer);
        // Create buffer data: first int for count, then sphere array
        const GLintptr sphereDataOffset = 16;
        const GLsizeiptr sphereDataSize = spheres.size() * sizeof(SphereDataPacked);
        const GLsizeiptr bufferSize = sphereDataOffset + sphereDataSize;
        glBufferData(GL_UNIFORM_BUFFER, bufferSize, nullptr, GL_STATIC_DRAW);
        int numSpheres = static_cast<int>(packedSpheres.size());
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(int), &numSpheres);
        if (!spheres.empty()) {
            glBufferSubData(
                GL_UNIFORM_BUFFER, sphereDataOffset, sphereDataSize, packedSpheres.data());
        }
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    }

    void calculateAABB() {
        if (spheres.empty()) {
            aabbMin = glm::vec3(-1.0f);
            aabbMax = glm::vec3(1.0f);
            return;
        }

        // Start with the first sphere
        aabbMin = glm::vec3(
            spheres[0].centerX - spheres[0].radius, spheres[0].centerY - spheres[0].radius,
            spheres[0].centerZ - spheres[0].radius);

        aabbMax = glm::vec3(
            spheres[0].centerX + spheres[0].radius, spheres[0].centerY + spheres[0].radius,
            spheres[0].centerZ + spheres[0].radius);

        // Expand for all other spheres
        for (size_t i = 1; i < spheres.size(); i++) {
            glm::vec3 sphereMin(
                spheres[i].centerX - spheres[i].radius, spheres[i].centerY - spheres[i].radius,
                spheres[i].centerZ - spheres[i].radius);

            glm::vec3 sphereMax(
                spheres[i].centerX + spheres[i].radius, spheres[i].centerY + spheres[i].radius,
                spheres[i].centerZ + spheres[i].radius);

            aabbMin = glm::min(aabbMin, sphereMin);
            aabbMax = glm::max(aabbMax, sphereMax);
        }
    }

    void createHashTableBuffer() {
        glGenBuffers(1, &hashTableBuffer);

        // Initialize hash table with zeros
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, hashTableBuffer);

        // Each entry is a 64-bit mask (uint64_t or uvec2 in GLSL)
        std::vector<uint64_t> emptyTable(HASH_TABLE_SIZE, 0);
        glBufferData(
            GL_SHADER_STORAGE_BUFFER, emptyTable.size() * sizeof(uint64_t), emptyTable.data(),
            GL_DYNAMIC_DRAW);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void createBVHBuffer() {
        glGenBuffers(1, &bvhBuffer);
        glGenBuffers(1, &sphereIndexBuffer);
    }

private:
    glm::vec3 aabbMin;
    glm::vec3 aabbMax;

public:
    void setUseSpatialHash(bool use) {
        useSpatialHash = use;
    }

    bool getUseSpatialHash() const {
        return useSpatialHash;
    }

    void setUseBVH(bool use) {
        useBVH = use;
    }

    bool getUseBVH() const {
        return useBVH;
    }

public:
};

#endif
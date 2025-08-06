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

    // BVH
    BVH bvh;
    GLuint bvhBuffer = 0;
    GLuint sphereIndexBuffer = 0;
    bool useBVH = false;

    // Multi-pass rendering textures
    Texture primaryTexture; // First pass output

    // Pass: Primary ray tracing
    ComputeShader rayTracingShader;
    Pipeline rayTracingPipeline;

    // Pass: Post-processing (denoising, tone mapping, etc.)
    ComputeShader postProcessShader;
    Pipeline postProcessPipeline;

    // Pass: Final display
    Quad quad;
    VertexShader vertexShader;
    FragmentShader fragmentShader;
    Pipeline renderPipeline;

    // Spatial hash parameters
    static constexpr int HASH_TABLE_SIZE = 4096 * 256; // Should be power of 2
    static constexpr float CELL_SIZE = 10.0f;          // Size of each grid cell
    bool useSpatialHash = true;                        // Toggle acceleration
    GLuint hashTableBuffer = 0;                        // OpenGL buffer for hash table

    // Compute shader for building the hash table
    ComputeShader hashBuildShader;
    Pipeline hashBuildPipeline;

    ComputeShader timeConsumeShader;
    Pipeline timeConsumePipeline;

public:
    Renderer()
        : resolution{512, 512}, primaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          rayTracingShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "ray-marching-bvh.comp"),
          postProcessShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "post-process.comp"),
          vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.vert"),
          fragmentShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.frag"),
          hashBuildShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "build-spatial-hash.comp"),
          timeConsumeShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "time-consume.comp") {
        // Setup pipelines
        rayTracingPipeline.attachComputeShader(rayTracingShader);
        postProcessPipeline.attachComputeShader(postProcessShader);
        renderPipeline.attachVertexShader(vertexShader);
        renderPipeline.attachFragmentShader(fragmentShader);
        hashBuildPipeline.attachComputeShader(hashBuildShader);
        timeConsumePipeline.attachComputeShader(timeConsumeShader);

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
        if (useSpatialHash) {
            // Build spatial hash table
            buildSpatialHash();
        }

        // if (useBVH) {
        //     buildBVH();
        // }

        timeConsumePass();

        // ray marching multiple spheres
        renderPassRayMarching();

        // Final display
        renderPassDisplay();
    }

private:
    // render multiple spheres with ray marching
    void renderPassRayMarching() {
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
        rayTracingShader.setUniform("useSpatialHash", useSpatialHash);
        rayTracingShader.setUniform("useBVH", useBVH);
        rayTracingShader.setUniform("cellSize", CELL_SIZE);

        // Set uniforms - add AABB data
        rayTracingShader.setUniform("cameraPosition", camera.camPos);
        rayTracingShader.setUniform("cameraFront", camera.camForward);
        rayTracingShader.setUniform("cameraUp", camera.camUp);
        rayTracingShader.setUniform("cameraRight", camera.camRight);
        rayTracingShader.setUniform("backgroundColor", backgroundColor);
        rayTracingShader.setUniform("aabbMin", aabbMin);
        rayTracingShader.setUniform("aabbMax", aabbMax);

        rayTracingPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "mult sphere Ray Marching");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        rayTracingPipeline.deactivate();

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

    void buildSpatialHash() {
        // Clear hash table
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, hashTableBuffer);
        std::vector<uint64_t> emptyTable(HASH_TABLE_SIZE, 0);
        glBufferData(
            GL_SHADER_STORAGE_BUFFER, emptyTable.size() * sizeof(uint64_t), emptyTable.data(),
            GL_DYNAMIC_DRAW);

        // Bind input spheres and output hash table
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);

        // Set uniforms
        hashBuildShader.setUniform("cellSize", CELL_SIZE);
        hashBuildShader.setUniform("aabbMin", aabbMin);
        hashBuildShader.setUniform("aabbMax", aabbMax);

        // Dispatch shader - one thread per sphere
        hashBuildPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 1, -1, "Build Spatial Hash");
        glDispatchCompute(std::ceil(MAX_SPHERES / 64.0f), 1, 1);
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

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, bvhBuffer);
        glBufferData(
            GL_SHADER_STORAGE_BUFFER, 16 + nodes.size() * sizeof(BVHNodeGPU), nullptr,
            GL_DYNAMIC_DRAW);

        int nodeCount = static_cast<int>(nodes.size());
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(int), &nodeCount);
        glBufferSubData(
            GL_SHADER_STORAGE_BUFFER, bvhDataOffset, nodes.size() * sizeof(BVHNodeGPU),
            nodes.data());

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, sphereIndexBuffer);
        glBufferData(
            GL_SHADER_STORAGE_BUFFER, indices.size() * sizeof(uint32_t), indices.data(),
            GL_DYNAMIC_DRAW);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
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

        packedSpheres.reserve(spheres.size());
        for (const auto& sphere : spheres) {
            packedSpheres.emplace_back(sphere);
        }

        // Override the first sphere with specific values
        // spheres[0].centerX = 0.0f;
        // spheres[0].centerY = 0.0f;
        // spheres[0].centerZ = -5.0f;
        // spheres[0].radius = 1.0f;
        // spheres[0].colorR = 1.0f;  // Red
        // spheres[0].colorG = 0.0f;
        // spheres[0].colorB = 0.0f;
    }

    void createSphereBuffer() {
        glGenBuffers(1, &sphereBuffer);
        updateSphereBuffer();
    }

    // TODO: discard the struct SphereData and use SphereDataPacked instead
    void updateSphereBuffer() {
        if (sphereBuffer == 0) {
            std::cerr << "Sphere buffer not created!" << std::endl;
            return;
        }

        calculateAABB();

        // Bind the buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, sphereBuffer);
        // Create buffer data: first int for count, then sphere array

        const GLintptr sphereDataOffset = 16;
        const GLsizeiptr sphereDataSize = spheres.size() * sizeof(SphereDataPacked);
        const GLsizeiptr bufferSize = sphereDataOffset + sphereDataSize;

        glBufferData(GL_SHADER_STORAGE_BUFFER, bufferSize, nullptr, GL_DYNAMIC_DRAW);

        int numSpheres = static_cast<int>(packedSpheres.size());

        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(int), &numSpheres);

        if (!spheres.empty()) {
            glBufferSubData(
                GL_SHADER_STORAGE_BUFFER, sphereDataOffset, sphereDataSize, packedSpheres.data());
        }

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
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
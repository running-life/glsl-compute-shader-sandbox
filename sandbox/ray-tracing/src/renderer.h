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

using namespace gcss;

// struct SphereData {
//   glm::vec3 center;
//   float radius;
//   glm::vec3 color;
//   float padding; // For alignment
// };

struct SphereData {
    // Explicitly use individual floats instead of vec3
    float centerX, centerY, centerZ;
    float radius;
    float colorR, colorG, colorB;
    float padding;

    // Constructor for convenience
    SphereData(const glm::vec3& center, float r, const glm::vec3& color)
        : centerX(center.x), centerY(center.y), centerZ(center.z), radius(r), colorR(color.r),
          colorG(color.g), colorB(color.b), padding(0.0f) {}

    SphereData() = default;
};

struct SphereDataPacked {
    glm::vec4 centerAndRadius;
    glm::vec4 color;

    SphereDataPacked(const SphereData& data)
        : centerAndRadius(data.centerX, data.centerY, data.centerZ, data.radius),
          color(data.colorR, data.colorG, data.colorB, data.padding) {}
};

class Renderer {
private:
    glm::uvec2 resolution;
    Camera camera;

    // Sphere parameters
    glm::vec3 sphereCenter;
    float sphereRadius;
    glm::vec3 sphereColor;
    glm::vec3 backgroundColor;

    // multi spheres
    std::vector<SphereData> spheres;
    GLuint sphereBuffer = 0;
    static constexpr int MAX_SPHERES = 64;

    // Multi-pass rendering textures
    Texture primaryTexture;   // First pass output
    Texture secondaryTexture; // Second pass output
    Texture finalTexture;     // Final output

    // Pass 1: Primary ray tracing
    ComputeShader rayTracingShader;
    Pipeline rayTracingPipeline;

    // Pass 2: Post-processing (denoising, tone mapping, etc.)
    ComputeShader postProcessShader;
    Pipeline postProcessPipeline;

    // Pass 3: Final display
    Quad quad;
    VertexShader vertexShader;
    FragmentShader fragmentShader;
    Pipeline renderPipeline;

    // Spatial hash parameters
    static constexpr int HASH_TABLE_SIZE = 4096 * 16; // Should be power of 2
    static constexpr float CELL_SIZE = 10.0f;   // Size of each grid cell
    bool useAccelerationStructure = true;        // Toggle acceleration
    GLuint hashTableBuffer = 0;                  // OpenGL buffer for hash table

    // Compute shader for building the hash table
    ComputeShader hashBuildShader;
    Pipeline hashBuildPipeline;

public:
    Renderer()
        : resolution{512, 512}, sphereCenter{0.0f, 0.0f, -5.0f}, sphereRadius{1.0f},
          sphereColor{0.8f, 0.3f, 0.3f}, backgroundColor{0.1f, 0.1f, 0.2f},
          primaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          secondaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          finalTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
          rayTracingShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "multi-sphere-ray-marching.comp"),
          postProcessShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "post-process.comp"),
          vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.vert"),
          fragmentShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders" / "render.frag"),
          hashBuildShader(
              std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "shaders"
              / "build-spatial-hash.comp") {
        // Setup pipelines
        rayTracingPipeline.attachComputeShader(rayTracingShader);
        postProcessPipeline.attachComputeShader(postProcessShader);
        renderPipeline.attachVertexShader(vertexShader);
        renderPipeline.attachFragmentShader(fragmentShader);
        hashBuildPipeline.attachComputeShader(hashBuildShader);

        // create spheres
        generateCloudSpheres();
        createSphereBuffer();

        // create hash table buffer
        createHashTableBuffer();
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
    }

    glm::uvec2 getResolution() const {
        return this->resolution;
    }

    void setResolution(const glm::uvec2& resolution) {
        this->resolution = resolution;
        primaryTexture.resize(resolution);
        secondaryTexture.resize(resolution);
        finalTexture.resize(resolution);
    }

    glm::vec3 getSphereCenter() const {
        return sphereCenter;
    }
    void setSphereCenter(const glm::vec3& center) {
        this->sphereCenter = center;
    }

    float getSphereRadius() const {
        return sphereRadius;
    }
    void setSphereRadius(float radius) {
        this->sphereRadius = std::max(0.01f, radius);
    }

    glm::vec3 getSphereColor() const {
        return sphereColor;
    }
    void setSphereColor(const glm::vec3& color) {
        this->sphereColor = color;
    }

    glm::vec3 getBackgroundColor() const {
        return backgroundColor;
    }
    void setBackgroundColor(const glm::vec3& color) {
        this->backgroundColor = color;
    }

    void move(const CameraMovement& movement_direction, float delta_time) {
        camera.move(movement_direction, delta_time);
    }

    void lookAround(float d_phi, float d_theta) {
        camera.lookAround(d_phi, d_theta);
    }

    void render() {
        if (useAccelerationStructure) {
            // Build spatial hash table
            buildSpatialHash();
        }

        // ray marching multiple spheres
        renderPassRayMarching2();
        // Pass 2: Post-processing
        renderPass2_PostProcess();
        // Pass 3: Final display
        renderPass3_Display();
    }

private:
    // render one sphere with ray tracing
    void renderPass1_RayTracing() {
        // Bind output texture
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        // Set uniforms
        rayTracingShader.setUniform("cameraPosition", camera.camPos);
        rayTracingShader.setUniform("cameraFront", camera.camForward);
        rayTracingShader.setUniform("cameraUp", camera.camUp);
        rayTracingShader.setUniform("cameraRight", camera.camRight);
        rayTracingShader.setUniform("sphereCenter", sphereCenter);
        rayTracingShader.setUniform("sphereRadius", sphereRadius);
        rayTracingShader.setUniform("sphereColor", sphereColor);
        rayTracingShader.setUniform("backgroundColor", backgroundColor);

        rayTracingPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 1, -1, "Pass 1: Ray Tracing");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        rayTracingPipeline.deactivate();

        // Memory barrier to ensure pass 1 is complete before pass 2
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    // render multiple spheres with ray marching
    void renderPassRayMarching() {
        // Bind output texture
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        // Bind sphere buffer - add safety check
        if (sphereBuffer != 0) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
        }

        // Set uniforms
        rayTracingShader.setUniform("cameraPosition", camera.camPos);
        rayTracingShader.setUniform("cameraFront", camera.camForward);
        rayTracingShader.setUniform("cameraUp", camera.camUp);
        rayTracingShader.setUniform("cameraRight", camera.camRight);
        rayTracingShader.setUniform("backgroundColor", backgroundColor);

        rayTracingPipeline.activate();
        // glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 4, -1, "mult sphere Ray Marching");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        // glPopDebugGroup();
        rayTracingPipeline.deactivate();

        // Memory barrier to ensure pass 1 is complete before pass 2
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void renderPassRayMarching2() {
        // Bind output texture
        primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);

        // Bind sphere buffer
        if (sphereBuffer != 0) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
        }

        // Bind hash table if using acceleration
        if (useAccelerationStructure) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, hashTableBuffer);
        }

        // Pass acceleration toggle to shader
        rayTracingShader.setUniform("useAccelerationStructure", useAccelerationStructure);
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

    void renderPass2_PostProcess() {
        // Bind input texture from pass 1
        primaryTexture.bindToImageUnit(0, GL_READ_ONLY);
        // Bind output texture for this pass
        finalTexture.bindToImageUnit(1, GL_WRITE_ONLY);

        // Set post-processing uniforms (e.g., exposure, gamma correction)
        postProcessShader.setUniform("exposure", 1.0f);
        postProcessShader.setUniform("gamma", 2.2f);

        postProcessPipeline.activate();
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 3, -1, "Post Process");
        glDispatchCompute(std::ceil(resolution.x / 8.0f), std::ceil(resolution.y / 8.0f), 1);
        glPopDebugGroup();
        postProcessPipeline.deactivate();

        // Memory barrier to ensure pass 2 is complete before pass 3
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }

    void renderPass3_Display() {
        // Clear and setup viewport
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, resolution.x, resolution.y);

        // Draw quad
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 3, -1, "Pass 3: Display");

        // Choose which texture to display (for debugging)
        if (debugShowPass1) {
            primaryTexture.bindToTextureUnit(0);
        } else {
            finalTexture.bindToTextureUnit(0);
        }

        // Draw quad
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 3, -1, "Pass 3: Display");
        quad.draw(renderPipeline);
        glPopDebugGroup();
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

    void updateSphereBuffer() {
        if (sphereBuffer == 0) {
            std::cerr << "Sphere buffer not created!" << std::endl;
            return;
        }

        calculateAABB();

        // create sphere data packed
        std::vector<SphereDataPacked> packedSpheres;
        packedSpheres.reserve(spheres.size());
        for (const auto& sphere : spheres) {
            packedSpheres.emplace_back(sphere);
        }

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

private:
    bool debugShowPass1 = false;
    glm::vec3 aabbMin;
    glm::vec3 aabbMax;

public:
    void setDebugShowPass1(bool show) {
        debugShowPass1 = show;
    }
    void setUseAccelerationStructure(bool use) {
        useAccelerationStructure = use;
    }

    bool getUseAccelerationStructure() const {
        return useAccelerationStructure;
    }

public:
};

#endif
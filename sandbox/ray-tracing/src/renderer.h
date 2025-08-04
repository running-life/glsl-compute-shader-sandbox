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

struct SphereData {
  glm::vec3 center;
  float radius;
  glm::vec3 color;
  float padding; // For alignment
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
  GLuint sphereBuffer;
  static constexpr int MAX_SPHERES = 64;

  // Multi-pass rendering textures
  Texture primaryTexture;    // First pass output
  Texture secondaryTexture;  // Second pass output
  Texture finalTexture;      // Final output
  
  // Pass 1: Primary ray tracing
  ComputeShader rayTracingShader;
  Pipeline rayTracingPipeline;
  
  // Pass 2: Post-processing (denoising, tone mapping, etc.)
  ComputeShader postProcessShader;
  Pipeline postProcessPipeline;
  
  // Pass 3: Final display
  
  // Pass 2: Post-processing (denoising, tone mapping, etc.)
  ComputeShader postProcessShader;
  Pipeline postProcessPipeline;
  
  // Pass 3: Final display
  Quad quad;
  VertexShader vertexShader;
  FragmentShader fragmentShader;
  Pipeline renderPipeline;

 public:
  Renderer()
      : resolution{512, 512},
        sphereCenter{0.0f, 0.0f, -5.0f},
        sphereRadius{1.0f},
        sphereColor{0.8f, 0.3f, 0.3f},
        backgroundColor{0.1f, 0.1f, 0.2f},
        primaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        secondaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        finalTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        primaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        secondaryTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        finalTexture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        rayTracingShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                         "shaders" / "ray-tracing.comp"),
        postProcessShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                          "shaders" / "post-process.comp"),
        postProcessShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                          "shaders" / "post-process.comp"),
        vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                     "shaders" / "render.vert"),
        fragmentShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                       "shaders" / "render.frag") {
    // Setup pipelines
    // Setup pipelines
    rayTracingPipeline.attachComputeShader(rayTracingShader);
    postProcessPipeline.attachComputeShader(postProcessShader);
    
    postProcessPipeline.attachComputeShader(postProcessShader);
    
    renderPipeline.attachVertexShader(vertexShader);
    renderPipeline.attachFragmentShader(fragmentShader);
  }

  glm::uvec2 getResolution() const { return this->resolution; }

  void setResolution(const glm::uvec2& resolution) {
    this->resolution = resolution;
    primaryTexture.resize(resolution);
    secondaryTexture.resize(resolution);
    finalTexture.resize(resolution);
    primaryTexture.resize(resolution);
    secondaryTexture.resize(resolution);
    finalTexture.resize(resolution);
  }

  glm::vec3 getSphereCenter() const { return sphereCenter; }
  void setSphereCenter(const glm::vec3& center) { this->sphereCenter = center; }

  float getSphereRadius() const { return sphereRadius; }
  void setSphereRadius(float radius) { this->sphereRadius = std::max(0.01f, radius); }

  glm::vec3 getSphereColor() const { return sphereColor; }
  void setSphereColor(const glm::vec3& color) { this->sphereColor = color; }

  glm::vec3 getBackgroundColor() const { return backgroundColor; }
  void setBackgroundColor(const glm::vec3& color) { this->backgroundColor = color; }

  void move(const CameraMovement& movement_direction, float delta_time) {
    camera.move(movement_direction, delta_time);
  }

  void lookAround(float d_phi, float d_theta) {
    camera.lookAround(d_phi, d_theta);
  }

  void render() {
    // Pass 1: Primary ray tracing
    renderPass1_RayTracing();
    
    // Pass 2: Post-processing
    renderPass2_PostProcess();
    
    // Pass 3: Final display
    renderPass3_Display();
  }

private:
  void renderPass1_RayTracing() {
    // Bind output texture
    primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);
    // Pass 1: Primary ray tracing
    renderPass1_RayTracing();
    
    // Pass 2: Post-processing
    renderPass2_PostProcess();
    
    // Pass 3: Final display
    renderPass3_Display();
  }

private:
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
    glDispatchCompute(std::ceil(resolution.x / 8.0f),
                      std::ceil(resolution.y / 8.0f), 1);
    glPopDebugGroup();
    rayTracingPipeline.deactivate();

    // Memory barrier to ensure pass 1 is complete before pass 2
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
  }

  void renderPass1RayMarching() {
    // Bind output texture
    primaryTexture.bindToImageUnit(0, GL_WRITE_ONLY);
    
    // Bind sphere buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sphereBuffer);
    
    // Set uniforms
    rayTracingShader.setUniform("cameraPosition", camera.camPos);
    rayTracingShader.setUniform("cameraFront", camera.camForward);
    rayTracingShader.setUniform("cameraUp", camera.camUp);
    rayTracingShader.setUniform("cameraRight", camera.camRight);
    rayTracingShader.setUniform("backgroundColor", backgroundColor);
    
    rayTracingPipeline.activate();
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 1, -1, "Pass 1: Ray Tracing");
    glDispatchCompute(std::ceil(resolution.x / 8.0f),
                      std::ceil(resolution.y / 8.0f), 1);
    glPopDebugGroup();
    rayTracingPipeline.deactivate();

    // Memory barrier to ensure pass 1 is complete before pass 2
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
  }

  void renderPass2RayMarching() {

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
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 2, -1, "Pass 2: Post Process");
    glDispatchCompute(std::ceil(resolution.x / 8.0f),
                      std::ceil(resolution.y / 8.0f), 1);
    glPopDebugGroup();
    postProcessPipeline.deactivate();

    // Memory barrier to ensure pass 2 is complete before pass 3
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
  }
  
  void renderPass3_Display() {
    // Clear and setup viewport
  }
  
  void renderPass3_Display() {
    // Clear and setup viewport
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, resolution.x, resolution.y);
    
    // Choose which texture to display (for debugging)
    if (debugShowPass1) {
      primaryTexture.bindToTextureUnit(0);
    } else {
      finalTexture.bindToTextureUnit(0);
    }
    
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
   std::uniform_real_distribution<float> xDist(-100.0f, 100.0f);
   std::uniform_real_distribution<float> yDist(100.0f, 1000.0f);   // Higher in the sky
   std::uniform_real_distribution<float> zDist(100.0f, 4000.0f); // Spread in depth
   std::uniform_real_distribution<float> radiusDist(5.0f, 20.0f); // Varied cloud sizes
   std::uniform_real_distribution<float> densityDist(0.0f, 1.0f);
   
   // Cloud color variations (white to light gray)
   std::uniform_real_distribution<float> brightnessVariation(0.7f, 1.0f);
   std::uniform_real_distribution<float> colorTint(0.95f, 1.0f); // Slight blue/warm tint
   
   for (int i = 0; i < MAX_SPHERES; i++) {
     SphereData sphere;
     
     // Position: scattered in sky-like formation
     sphere.center.x = xDist(gen);
     sphere.center.y = yDist(gen);
     sphere.center.z = zDist(gen);
     
     // Add some clustering effect for more realistic cloud formation
     if (densityDist(gen) < 0.3f) { // 30% chance for clustered clouds
       // Create clusters near existing spheres
       if (!spheres.empty()) {
         int clusterTarget = gen() % spheres.size();
         sphere.center.x = spheres[clusterTarget].center.x + std::uniform_real_distribution<float>(-8.0f, 8.0f)(gen);
         sphere.center.y = spheres[clusterTarget].center.y + std::uniform_real_distribution<float>(-3.0f, 3.0f)(gen);
         sphere.center.z = spheres[clusterTarget].center.z + std::uniform_real_distribution<float>(-5.0f, 5.0f)(gen);
       }
     }
     
     // Size: varied like real clouds
     sphere.radius = radiusDist(gen);
     
     // Color: cloud-like whites and light grays with subtle variations
     float brightness = brightnessVariation(gen);
     float rTint = colorTint(gen);
     float gTint = colorTint(gen);
     float bTint = std::uniform_real_distribution<float>(0.98f, 1.0f)(gen); // Slightly more blue
     
     sphere.color = glm::vec3(brightness * rTint, brightness * gTint, brightness * bTint);
     sphere.padding = 0.0f;
     
     spheres.push_back(sphere);
   }
  }

  void createSphereBuffer() {
    glGenBuffers(1, &sphereBuffer);
    updateSphereBuffer();
  }

  void updateSphereBuffer() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, sphereBuffer);
    
    // Create buffer data: first int for count, then sphere array
    size_t bufferSize = sizeof(int) + spheres.size() * sizeof(SphereData);
    glBufferData(GL_SHADER_STORAGE_BUFFER, bufferSize, nullptr, GL_DYNAMIC_DRAW);
    
    // Upload sphere count
    int numSpheres = static_cast<int>(spheres.size());
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(int), &numSpheres);
    
    // Upload sphere data
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, sizeof(int), 
                    spheres.size() * sizeof(SphereData), spheres.data());
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }

private:
  bool debugShowPass1 = false;

public:
  void setDebugShowPass1(bool show) { debugShowPass1 = show; }

public:
};

#endif
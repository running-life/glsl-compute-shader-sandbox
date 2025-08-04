#ifndef _RENDERER_H
#define _RENDERER_H
#include <algorithm>

#include "glad/gl.h"
#include "glm/glm.hpp"
//
#include "gcss/camera.h"
#include "gcss/quad.h"
#include "gcss/texture.h"

using namespace gcss;

class Renderer {
 private:
  glm::uvec2 resolution;
  Camera camera;
  
  // Sphere parameters
  glm::vec3 sphereCenter;
  float sphereRadius;
  glm::vec3 sphereColor;
  glm::vec3 backgroundColor;

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
        rayTracingShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                         "shaders" / "ray-tracing.comp"),
        postProcessShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                          "shaders" / "post-process.comp"),
        vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                     "shaders" / "render.vert"),
        fragmentShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                       "shaders" / "render.frag") {
    // Setup pipelines
    rayTracingPipeline.attachComputeShader(rayTracingShader);
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
    quad.draw(renderPipeline);
    glPopDebugGroup();
  }

private:
  bool debugShowPass1 = false; // 调试开关

public:
  void setDebugShowPass1(bool show) { debugShowPass1 = show; }

public:
};

#endif
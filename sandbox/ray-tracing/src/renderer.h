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

  // OpenGL objects
  Texture texture;
  ComputeShader rayTracingShader;
  Pipeline rayTracingPipeline;

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
        texture{glm::vec2(512, 512), GL_RGBA32F, GL_RGBA, GL_FLOAT},
        rayTracingShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                         "shaders" / "ray-tracing.comp"),
        vertexShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                     "shaders" / "render.vert"),
        fragmentShader(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                       "shaders" / "render.frag") {
    rayTracingPipeline.attachComputeShader(rayTracingShader);

    renderPipeline.attachVertexShader(vertexShader);
    renderPipeline.attachFragmentShader(fragmentShader);
  }

  glm::uvec2 getResolution() const { return this->resolution; }

  void setResolution(const glm::uvec2& resolution) {
    this->resolution = resolution;
    texture.resize(resolution);
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
    // run ray tracing compute shader
    texture.bindToImageUnit(0, GL_WRITE_ONLY);
    
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
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 1, -1, "Ray Tracing");
    glDispatchCompute(std::ceil(resolution.x / 8.0f),
                      std::ceil(resolution.y / 8.0f), 1);
    glPopDebugGroup();
    rayTracingPipeline.deactivate();

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // render quad with the ray traced result
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, resolution.x, resolution.y);
    texture.bindToTextureUnit(0);
    quad.draw(renderPipeline);
  }
};

#endif
#ifndef _RENDERER_H
#define _RENDERER_H
#include <random>
#include <vector>
#include <cmath>
#include <filesystem>

#include "glad/gl.h"
#include "glm/glm.hpp"
//
#include "gcss/buffer.h"
#include "gcss/camera.h"
//
#include "cloth.h"

using namespace gcss;

class Renderer {
 private:
  glm::uvec2 resolution;
  glm::uvec2 clothSize;  // 布料尺寸 (width, height)
  
  // 物理参数
  float dt;
  float gravity;
  float damping;
  float stiffness;
  int solverIterations;
  bool pause;
  bool windEnabled;
  glm::vec3 windForce;
  
  // 渲染参数
  glm::vec3 clothColor;
  bool wireframeMode;
  bool showPoints;
  
  Camera camera;
  
  // 布料数据
  Cloth cloth;
  Buffer particleBuffer;
  Buffer constraintBuffer;
  Buffer triangleVertexBuffer;
  
  uint32_t nParticles;
  uint32_t nConstraints;
  uint32_t nTriangleVertices;
  
  // Compute shaders
  ComputeShader predictPositions;
  ComputeShader satisfyConstraints;
  ComputeShader updatePositions;
  ComputeShader updateTriangleVertices;
  
  Pipeline predictPositionsPipeline;
  Pipeline satisfyConstraintsPipeline;
  Pipeline updatePositionsPipeline;
  Pipeline updateTriangleVerticesPipeline;
  
  // 渲染管线
  VertexShader vertexShader;
  FragmentShader fragmentShader;
  VertexShader pointVertexShader;
  FragmentShader pointFragmentShader;
  
  Pipeline renderPipeline;
  Pipeline pointRenderPipeline;
  
  float elapsed_time;
  
 public:
  Renderer()
      : resolution{800, 600},
        clothSize{32, 32},
        dt{0.008f},           // 减小时间步长
        gravity{-9.8f},
        damping{0.99f},
        stiffness{0.8f},      // 减小刚度
        solverIterations{5},   // 增加迭代次数
        pause{false},
        windEnabled{false},
        windForce{0.0f, 0.0f, 5.0f},
        clothColor{0.8f, 0.4f, 0.2f},
        wireframeMode{false},
        showPoints{true},
        // 加载着色器
        predictPositions{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                        "shaders" / "predict-positions.comp"},
        satisfyConstraints{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                          "shaders" / "satisfy-constraints.comp"},
        updatePositions{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                       "shaders" / "update-positions.comp"},
        updateTriangleVertices{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                              "shaders" / "update-triangle-vertices.comp"},
        vertexShader{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                     "shaders" / "render-cloth.vert"},
        fragmentShader{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                       "shaders" / "render-cloth.frag"},
        pointVertexShader{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                         "shaders" / "render-points.vert"},
        pointFragmentShader{std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) /
                           "shaders" / "render-points.frag"},
        elapsed_time{0} {
    
    // 设置compute shader管线
    predictPositionsPipeline.attachComputeShader(predictPositions);
    satisfyConstraintsPipeline.attachComputeShader(satisfyConstraints);
    updatePositionsPipeline.attachComputeShader(updatePositions);
    updateTriangleVerticesPipeline.attachComputeShader(updateTriangleVertices);
    
    // 设置渲染管线
    renderPipeline.attachVertexShader(vertexShader);
    renderPipeline.attachFragmentShader(fragmentShader);
    
    pointRenderPipeline.attachVertexShader(pointVertexShader);
    pointRenderPipeline.attachFragmentShader(pointFragmentShader);
    
    // OpenGL设置
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    initializeCloth();
  }
  
  // Getters
  glm::uvec2 getResolution() const { return resolution; }
  glm::uvec2 getClothSize() const { return clothSize; }
  float getDt() const { return dt; }
  float getGravity() const { return gravity; }
  float getDamping() const { return damping; }
  float getStiffness() const { return stiffness; }
  int getSolverIterations() const { return solverIterations; }
  bool isPaused() const { return pause; }
  bool isWireframeMode() const { return wireframeMode; }
  bool isShowPoints() const { return showPoints; }
  glm::vec3 getClothColor() const { return clothColor; }
  glm::vec3 getWindForce() const { return windForce; }
  bool isWindEnabled() const { return windEnabled; }
  
  // Setters
  void setResolution(const glm::uvec2& resolution) { this->resolution = resolution; }
  void setClothSize(const glm::uvec2& clothSize) {
    this->clothSize = clothSize;
    initializeCloth();
  }
  void setDt(float dt) { this->dt = dt; }
  void setGravity(float gravity) { this->gravity = gravity; }
  void setDamping(float damping) { this->damping = damping; }
  void setStiffness(float stiffness) { this->stiffness = stiffness; }
  void setSolverIterations(int iterations) { this->solverIterations = iterations; }
  void setPause(bool pause) { this->pause = pause; }
  void setWireframeMode(bool wireframe) { this->wireframeMode = wireframe; }
  void setShowPoints(bool show) { this->showPoints = show; }
  void setClothColor(const glm::vec3& color) { this->clothColor = color; }
  void setWindForce(const glm::vec3& wind) { this->windForce = wind; }
  void setWindEnabled(bool enabled) { this->windEnabled = enabled; }
  
  void move(const CameraMovement& movement_direction, float delta_time) {
    camera.move(movement_direction, delta_time);
  }

  void lookAround(float d_phi, float d_theta) {
    camera.lookAround(d_phi, d_theta);
  }
  
  void initializeCloth() {
    nParticles = clothSize.x * clothSize.y;
    nConstraints = 0;
    
    // 计算约束数量
    // 结构约束 (水平和垂直)
    nConstraints += (clothSize.x - 1) * clothSize.y;  // 水平
    nConstraints += clothSize.x * (clothSize.y - 1);  // 垂直
    
    // 剪切约束 (对角线)
    nConstraints += (clothSize.x - 1) * (clothSize.y - 1) * 2;
    
    // 弯曲约束 (隔一个粒子的连接)
    nConstraints += (clothSize.x - 2) * clothSize.y;  // 水平弯曲
    nConstraints += clothSize.x * (clothSize.y - 2);  // 垂直弯曲
    
    // 初始化粒子
    std::vector<ClothParticle> particles(nParticles);
    float spacing = 0.1f;
    
    for (uint32_t y = 0; y < clothSize.y; ++y) {
      for (uint32_t x = 0; x < clothSize.x; ++x) {
        uint32_t index = y * clothSize.x + x;
        
        particles[index].position = glm::vec4(
          (x - clothSize.x * 0.5f) * spacing,
          2.0f,
          (y - clothSize.y * 0.5f) * spacing,
          1.0f
        );
        particles[index].oldPosition = particles[index].position;
        particles[index].velocity = glm::vec4(0.0f);
        particles[index].mass = 1.0f;
        
        // 固定顶部两个角
        if (y == 0 && (x == 0 || x == clothSize.x - 1)) {
          particles[index].invMass = 0.0f;  // 固定粒子
        } else {
          particles[index].invMass = 1.0f / particles[index].mass;
        }
      }
    }
    
    particleBuffer.setData(particles, GL_DYNAMIC_DRAW);
    
    // 初始化约束
    std::vector<Constraint> constraints;
    constraints.reserve(nConstraints);
    
    auto addConstraint = [&](uint32_t a, uint32_t b, float stiffness) {
      glm::vec3 posA = glm::vec3(particles[a].position);
      glm::vec3 posB = glm::vec3(particles[b].position);
      float restLength = glm::length(posA - posB);
      
      constraints.push_back({a, b, restLength, stiffness});
    };
    
    // 结构约束
    for (uint32_t y = 0; y < clothSize.y; ++y) {
      for (uint32_t x = 0; x < clothSize.x; ++x) {
        uint32_t current = y * clothSize.x + x;
        
        // 水平约束
        if (x < clothSize.x - 1) {
          addConstraint(current, current + 1, 1.0f);
        }
        
        // 垂直约束
        if (y < clothSize.y - 1) {
          addConstraint(current, current + clothSize.x, 1.0f);
        }
      }
    }
    
    // 剪切约束
    for (uint32_t y = 0; y < clothSize.y - 1; ++y) {
      for (uint32_t x = 0; x < clothSize.x - 1; ++x) {
        uint32_t current = y * clothSize.x + x;
        addConstraint(current, current + clothSize.x + 1, 0.5f);
        addConstraint(current + 1, current + clothSize.x, 0.5f);
      }
    }
    
    // 弯曲约束
    for (uint32_t y = 0; y < clothSize.y; ++y) {
      for (uint32_t x = 0; x < clothSize.x - 2; ++x) {
        uint32_t current = y * clothSize.x + x;
        addConstraint(current, current + 2, 0.2f);
      }
    }
    
    for (uint32_t y = 0; y < clothSize.y - 2; ++y) {
      for (uint32_t x = 0; x < clothSize.x; ++x) {
        uint32_t current = y * clothSize.x + x;
        addConstraint(current, current + 2 * clothSize.x, 0.2f);
      }
    }
    
    constraintBuffer.setData(constraints, GL_STATIC_DRAW);
    
    // 生成三角形顶点数据（不使用索引）
    std::vector<ClothParticle> triangleVertices;
    
    for (uint32_t y = 0; y < clothSize.y - 1; ++y) {
      for (uint32_t x = 0; x < clothSize.x - 1; ++x) {
        uint32_t topLeft = y * clothSize.x + x;
        uint32_t topRight = topLeft + 1;
        uint32_t bottomLeft = topLeft + clothSize.x;
        uint32_t bottomRight = bottomLeft + 1;
        
        // 第一个三角形
        triangleVertices.push_back(particles[topLeft]);
        triangleVertices.push_back(particles[bottomLeft]);
        triangleVertices.push_back(particles[topRight]);
        
        // 第二个三角形
        triangleVertices.push_back(particles[topRight]);
        triangleVertices.push_back(particles[bottomLeft]);
        triangleVertices.push_back(particles[bottomRight]);
      }
    }
    
    nTriangleVertices = triangleVertices.size();
    triangleVertexBuffer.setData(triangleVertices, GL_DYNAMIC_DRAW);
    
    // 设置布料对象
    cloth.setBuffers(&particleBuffer, &constraintBuffer, &triangleVertexBuffer,
                     nParticles, nConstraints, nTriangleVertices);
  }
  
  void render(float delta_time) {
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    
    // 清屏
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, resolution.x, resolution.y);
    
    // 更新物理
    elapsed_time += delta_time;
    if (elapsed_time > dt && !pause) {
      elapsed_time = 0;
      updatePhysics();
    }
    
    // 渲染布料
    glm::mat4 viewProjection = camera.computeViewProjectionmatrix(resolution.x, resolution.y);
    
    if (wireframeMode) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    vertexShader.setUniform("viewProjection", viewProjection);
    fragmentShader.setUniform("clothColor", clothColor);
    cloth.drawTriangles(renderPipeline);
    
    // 渲染点
    if (showPoints) {
      glPointSize(3.0f);
      pointVertexShader.setUniform("viewProjection", viewProjection);
      pointFragmentShader.setUniform("pointColor", glm::vec3(1.0f, 0.0f, 0.0f));
      cloth.drawPoints(pointRenderPipeline);
    }
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  
 private:
  void updatePhysics() {
    // 1. 预测位置
    particleBuffer.bindToShaderStorageBuffer(0);
    predictPositions.setUniform("dt", dt);
    predictPositions.setUniform("gravity", gravity);
    predictPositions.setUniform("damping", damping);
    predictPositions.setUniform("windEnabled", windEnabled);
    predictPositions.setUniform("windForce", windForce);
    
    predictPositionsPipeline.activate();
    glDispatchCompute((nParticles + 127) / 128, 1, 1);
    predictPositionsPipeline.deactivate();
    
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    
    // 2. 满足约束 (多次迭代)
    for (int i = 0; i < solverIterations; ++i) {
      particleBuffer.bindToShaderStorageBuffer(0);
      constraintBuffer.bindToShaderStorageBuffer(1);
      satisfyConstraints.setUniform("stiffness", stiffness);
      
      satisfyConstraintsPipeline.activate();
      glDispatchCompute((nConstraints + 127) / 128, 1, 1);
      satisfyConstraintsPipeline.deactivate();
      
      glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
    
    // 3. 更新位置和速度
    particleBuffer.bindToShaderStorageBuffer(0);
    updatePositions.setUniform("dt", dt);
    
    updatePositionsPipeline.activate();
    glDispatchCompute((nParticles + 127) / 128, 1, 1);
    updatePositionsPipeline.deactivate();
    
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    
    // 4. 更新三角形顶点缓冲区
    updateTriangleVertexBuffer();
  }
  
  void updateTriangleVertexBuffer() {
    // 使用GPU compute shader来更新三角形顶点
    particleBuffer.bindToShaderStorageBuffer(0);
    triangleVertexBuffer.bindToShaderStorageBuffer(1);
    
    updateTriangleVertices.setUniform("clothWidth", static_cast<uint32_t>(clothSize.x));
    updateTriangleVertices.setUniform("clothHeight", static_cast<uint32_t>(clothSize.y));
    
    uint32_t totalTriangles = (clothSize.x - 1) * (clothSize.y - 1) * 2;
    
    updateTriangleVerticesPipeline.activate();
    glDispatchCompute((totalTriangles + 127) / 128, 1, 1);
    updateTriangleVerticesPipeline.deactivate();
    
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }
};

#endif
#ifndef _CLOTH_H
#define _CLOTH_H
#include <vector>
#include "gcss/buffer.h"
#include "gcss/shader.h"
#include "gcss/vertex-array-object.h"
#include "glm/glm.hpp"

using namespace gcss;

// 布料粒子结构
struct alignas(16) ClothParticle {
  glm::vec4 position = glm::vec4(0);      // 当前位置
  glm::vec4 oldPosition = glm::vec4(0);   // 上一帧位置
  glm::vec4 velocity = glm::vec4(0);      // 速度
  float mass = 1.0f;                      // 质量
  float invMass = 1.0f;                   // 逆质量 (固定点为0)
  float padding[2];                       // 对齐填充
};

// 约束结构
struct alignas(16) Constraint {
  uint32_t particleA;                     // 粒子A索引
  uint32_t particleB;                     // 粒子B索引
  float restLength;                       // 静止长度
  float stiffness;                        // 刚度
};

class Cloth {
 private:
  VertexArrayObject triangleVAO;
  VertexArrayObject pointVAO;
  const Buffer* particleBuffer;
  const Buffer* constraintBuffer;
  const Buffer* triangleVertexBuffer;
  uint32_t nParticles;
  uint32_t nConstraints;
  uint32_t nTriangleVertices;

 public:
  Cloth() : nParticles(0), nConstraints(0), nTriangleVertices(0) {}

  void setBuffers(const Buffer* particleBuffer, const Buffer* constraintBuffer, 
                  const Buffer* triangleVertexBuffer, uint32_t nParticles, 
                  uint32_t nConstraints, uint32_t nTriangleVertices) {
    this->particleBuffer = particleBuffer;
    this->constraintBuffer = constraintBuffer;
    this->triangleVertexBuffer = triangleVertexBuffer;
    this->nParticles = nParticles;
    this->nConstraints = nConstraints;
    this->nTriangleVertices = nTriangleVertices;

    // 设置三角形渲染的VAO
    triangleVAO.bindVertexBuffer(*triangleVertexBuffer, 0, 0, sizeof(ClothParticle));
    triangleVAO.activateVertexAttribution(0, 0, 3, GL_FLOAT, 0);
    
    // 设置点渲染的VAO
    pointVAO.bindVertexBuffer(*particleBuffer, 0, 0, sizeof(ClothParticle));
    pointVAO.activateVertexAttribution(0, 0, 3, GL_FLOAT, 0);
  }

  void drawTriangles(const Pipeline& pipeline) const {
    if (nTriangleVertices == 0) return;
    
    pipeline.activate();
    triangleVAO.activate();
    glDrawArrays(GL_TRIANGLES, 0, nTriangleVertices);
    triangleVAO.deactivate();
    pipeline.deactivate();
  }

  void drawPoints(const Pipeline& pipeline) const {
    if (nParticles == 0) return;
    
    pipeline.activate();
    pointVAO.activate();
    glDrawArrays(GL_POINTS, 0, nParticles);
    pointVAO.deactivate();
    pipeline.deactivate();
  }

  uint32_t getParticleCount() const { return nParticles; }
  uint32_t getConstraintCount() const { return nConstraints; }
};

#endif
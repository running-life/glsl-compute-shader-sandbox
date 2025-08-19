#version 460 core

// 重新声明gl_PerVertex块
out gl_PerVertex {
    vec4 gl_Position;
    float gl_PointSize;
    float gl_ClipDistance[];
};

layout(location = 0) in vec3 position;

uniform mat4 viewProjection;

out vec3 worldPos;
out vec3 normal;

void main() {
  worldPos = position;
  
  // 简单的法线计算（可以在几何着色器中改进）
  normal = vec3(0.0, 0.0, 1.0);
  
  gl_Position = viewProjection * vec4(position, 1.0);
}

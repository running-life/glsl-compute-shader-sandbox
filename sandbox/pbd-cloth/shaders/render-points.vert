#version 460 core

// 重新声明gl_PerVertex块
out gl_PerVertex {
    vec4 gl_Position;
    float gl_PointSize;
    float gl_ClipDistance[];
};

layout(location = 0) in vec3 position;

uniform mat4 viewProjection;

void main() {
  gl_Position = viewProjection * vec4(position, 1.0);
  gl_PointSize = 3.0;
}

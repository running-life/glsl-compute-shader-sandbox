#version 460 core

uniform vec3 pointColor;

out vec4 fragColor;

void main() {
  fragColor = vec4(pointColor, 1.0);
}
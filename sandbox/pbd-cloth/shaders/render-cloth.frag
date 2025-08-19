#version 460 core

in vec3 worldPos;
in vec3 normal;

uniform vec3 clothColor;

out vec4 fragColor;

void main() {
  // 简单的光照
  vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
  vec3 norm = normalize(normal);
  
  float diffuse = max(dot(norm, lightDir), 0.2);
  
  vec3 finalColor = clothColor * diffuse;
  
  fragColor = vec4(finalColor, 1.0);
}
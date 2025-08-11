#version 460 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoord;

out gl_PerVertex {
    vec4 gl_Position;
};

out vec2 fragTexCoord;

void main() {
    fragTexCoord = texCoord;
    gl_Position = vec4(position, 1.0);
}
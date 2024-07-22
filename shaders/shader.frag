#version 450

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in float transparency;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

layout(location = 0) out vec4 outColor;

void main() {
	outColor = vec4(vec3(texture(texSampler, fragTexCoord)), transparency);
}
#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 inTexCoord;

layout(location = 0) flat out vec4 fragColor;

layout(set = 0, binding = 0) uniform UniformBufferObject {
	mat4 notUsed;
	mat4 view;
	mat4 proj;
} ubo;

layout(push_constant) uniform pushConstant {
	vec3 color;
	bool isOneColor;
	float transparency;	
} ps;


void main() {
	gl_Position = ubo.proj * ubo.view * vec4(inPosition, 1.0);

	if (ps.isOneColor) {
		fragColor = vec4(ps.color, ps.transparency);
	} else {
		fragColor = vec4(inColor, ps.transparency);
	}
}
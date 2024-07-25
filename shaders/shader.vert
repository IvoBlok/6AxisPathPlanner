#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 inTexCoord;

layout(location = 0) out vec2 fragTexCoord;
layout(location = 1) out float fragTransparency;
layout(location = 2) out vec3 fragColor;
layout(location = 3) out int isOneColor;

layout(set = 0, binding = 0) uniform UniformBufferObject {
	mat4 notUsed;
	mat4 view;
	mat4 proj;
} ubo;

layout( push_constant ) uniform constants {
	mat4 model;	
	vec3 color;
	bool isOneColor;
	float transparency;
} ps;

void main() {
	gl_Position = ubo.proj * ubo.view * ps.model * vec4(inPosition, 1.0);

	fragTexCoord = inTexCoord;
	fragTransparency = ps.transparency;
	fragColor = ps.color;
	isOneColor = int(ps.isOneColor);
}
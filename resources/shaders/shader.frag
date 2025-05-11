#version 450

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in float transparency;
layout(location = 2) in vec3 fragColor;
layout(location = 3) flat in int isOneColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

layout(location = 0) out vec4 outColor;

void main() {
	if(isOneColor > 0.5) {
		outColor = vec4(fragColor, transparency);
	} else {
		outColor = vec4(vec3(texture(texSampler, fragTexCoord)), transparency);
	}
}
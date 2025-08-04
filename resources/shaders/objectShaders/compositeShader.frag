#version 450

layout(binding = 0) uniform sampler2D accumTexture;
layout(binding = 1) uniform sampler2D revealageTexture;

layout(location = 0) in vec2 uv;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 accum = texture(accumTexture, uv);
    float reveal = texture(revealageTexture, uv).r;
    outColor = vec4(accum.rgb / max(accum.a, 1e-5), 1.0 - reveal);
}
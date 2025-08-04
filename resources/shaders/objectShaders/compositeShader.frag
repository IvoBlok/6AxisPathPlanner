#version 450

layout(input_attachment_index = 0, binding = 0) uniform subpassInput accumulationBuffer;
layout(input_attachment_index = 1, binding = 1) uniform subpassInput revealageBuffer;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 accum = subpassLoad(accumulationBuffer);
    float reveal = subpassLoad(revealageBuffer).r;
    outColor = vec4(accum.rgb / max(accum.a, 1e-5), 1.0 - reveal);
}
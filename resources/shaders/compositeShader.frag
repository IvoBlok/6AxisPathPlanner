#version 450

layout(input_attachment_index = 0, set = 0, binding = 0) uniform subpassInput accumulationBuffer;
layout(input_attachment_index = 1, set = 0, binding = 1) uniform subpassInput revealageBuffer;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 accum = subpassLoad(accumulationBuffer);
    float reveal = subpassLoad(revealageBuffer).r;
    
    vec3 color = (accum.rgb / clamp(accum.a, 1e-5, 5e4)) * (1 - reveal);
    
    outColor = vec4(color, 1 - reveal);
}
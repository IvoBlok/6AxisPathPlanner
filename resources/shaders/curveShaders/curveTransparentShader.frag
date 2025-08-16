#version 450

layout(location = 0) flat in vec3 fragColor;
layout(location = 1) in float transparency;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;

void main() {
    if (transparency < 0.001) discard;

    // using pre-multiplied colors for easier brightness-maintaining color blending
    vec3 color = fragColor * transparency;

    // WB-OIT buffer values
    float weight = transparency * max(1e-2, 1e3 * pow(1 - gl_FragCoord.z, 3));

    outAccumulation = vec4(color * weight, transparency * weight);
    outRevealage = 1.0 - transparency;
}
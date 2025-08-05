#version 450

layout(location = 0) flat in vec3 fragColor;
layout(location = 1) in float transparency;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;

void main() {
    float depthScale = 1.0;

    if (transparency < 0.001) discard;

    vec3 color = fragColor * transparency;
    
    float maxChannel = max(max(color.r, color.g), color.b);
    float weight = clamp(max(maxChannel, transparency) * 10.0 + 0.01, 1e-2, 1.0);

    // depth weighting
    weight *= pow(1.0 - gl_FragCoord.z * depthScale, 3);

    outAccumulation = vec4(color * weight, weight);
    outRevealage = 1.0 - transparency;
}
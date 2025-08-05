#version 450

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in float transparency;
layout(location = 2) in vec3 fragColor;
layout(location = 3) flat in int isOneColor;
layout(location = 4) in vec3 fragNormal;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;

// Hardcoded sun direction (normalized, coming from top-right)
const vec3 sunDirection = normalize(vec3(0.62, 0.91, 0.5));

void main() {
    float weightParam = 3.0;
    float depthScale = 1.0;
    
    if (transparency < 0.001) discard;

    vec3 baseColor = (isOneColor > 0.5) ? fragColor : vec3(texture(texSampler, fragTexCoord));
    float diffuse = max(dot(normalize(fragNormal), sunDirection), 0.0);
    vec3 lighting = vec3(0.3) + vec3(0.7) * diffuse; // 30% ambient light
    
    vec3 opaqueColor = baseColor * lighting;
    vec3 color = opaqueColor * transparency;

    float maxChannel = max(max(opaqueColor.r, opaqueColor.g), opaqueColor.b);
    float weight = clamp(max(maxChannel, transparency) * 10.0 + 0.01, 1e-2, 1.0);

    // depth weighting
    weight *= pow(1.0 - gl_FragCoord.z * depthScale, weightParam);

    outAccumulation = vec4(opaqueColor * weight, weight);
    outRevealage = 1.0 - transparency;
}
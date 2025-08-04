#version 450

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in float transparency;
layout(location = 2) in vec3 fragColor;
layout(location = 3) flat in int isOneColor;
layout(location = 4) in vec3 fragNormal;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

layout(location = 0) out vec4 outColor;

// Hardcoded sun direction (normalized, coming from top-right)
const vec3 sunDirection = normalize(vec3(0.5, 1.0, 0.5));

void main() {
    if (transparency < 0.01) discard;

    vec3 baseColor = (isOneColor > 0.5) ? fragColor : vec3(texture(texSampler, fragTexCoord));
    float diffuse = max(dot(normalize(fragNormal), sunDirection), 0.0);
    vec3 lighting = vec3(0.3) + vec3(0.7) * diffuse; // 30% ambient light
    
    outColor = vec4(baseColor * lighting, 1.0);

    gl_FragDepth = gl_FragCoord.z;
}
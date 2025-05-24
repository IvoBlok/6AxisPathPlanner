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
    vec3 baseColor;
    
    if(isOneColor > 0.5) {
        baseColor = fragColor;
    } else {
        baseColor = vec3(texture(texSampler, fragTexCoord));
    }
    
    // Simple diffuse lighting
    float diffuse = max(dot(normalize(fragNormal), sunDirection), 0.0);
    
    // Ambient + diffuse lighting
    vec3 ambient = vec3(0.3);  // 30% ambient light
    vec3 lighting = ambient + vec3(0.7) * diffuse;  // 70% diffuse
    
    // Apply lighting to color
    vec3 finalColor = baseColor * lighting;
    
    outColor = vec4(finalColor, transparency);
}
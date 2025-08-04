#version 450

layout(location = 0) out vec2 uv;

void main() {
    // Generate a full-screen (domain for -1 <= x,y <= 1) triangle without needing a vertex buffer
    vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2(3.0, -1.0),
        vec2(-1.0, 3.0)
    );
    
    // Set vertex position
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    
    // Calculate uv coordinates in (0 <= x,y <= 1) range
    uv = positions[gl_VertexIndex] * 0.5 + 0.5;
}
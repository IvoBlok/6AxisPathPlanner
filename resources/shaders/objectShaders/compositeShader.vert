#version 450

void main() {
    // Generate a full-screen (domain for -1 <= x,y <= 1) triangle without needing a vertex buffer
    vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2(3.0, -1.0),
        vec2(-1.0, 3.0)
    );
    
    // Set vertex position
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
}
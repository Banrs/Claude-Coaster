#version 450

layout(location = 0) in vec2 inPos;   // pixels, top-left origin
layout(location = 1) in vec2 inUV;
layout(location = 2) in vec4 inColor;  // rgb + alpha

layout(push_constant) uniform PC {
    vec2 screen;                       // framebuffer size in pixels
} pc;

layout(location = 0) out vec2 fragUV;
layout(location = 1) out vec4 fragColor;

void main() {
    vec2 ndc = inPos / pc.screen * 2.0 - 1.0;   // Vulkan NDC y is down: no flip
    gl_Position = vec4(ndc, 0.0, 1.0);
    fragUV = inUV;
    fragColor = inColor;
}

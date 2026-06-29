#version 450
// Depth-only pass: project geometry into the sun's light space.
layout(location = 0) in vec3 inPos;
layout(push_constant) uniform PC { mat4 lightVP; } pc;
void main(){ gl_Position = pc.lightVP * vec4(inPos, 1.0); }

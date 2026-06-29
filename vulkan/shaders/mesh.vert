#version 450
// World mesh vertex stage. Scene constants come from a UBO (set 0, binding 0);
// also emits the light-space position for shadow-map sampling.
layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;     // xyz
    vec4 camPos;     // xyz, w = fog end
} u;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec3 vColor;
layout(location = 3) out vec4 vLightPos;

void main(){
    vWorldPos = inPos;
    vNormal   = inNormal;
    vColor    = inColor;
    vLightPos = u.lightVP * vec4(inPos, 1.0);
    gl_Position = u.viewProj * vec4(inPos, 1.0);
}

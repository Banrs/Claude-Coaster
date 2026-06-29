#version 450
// Water surface vertex stage. Shares the scene UBO contract with mesh.vert
// (set 0, binding 0). Passes world pos / normal through and emits the
// light-space position so water.frag can sample the shadow map.
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
layout(location = 2) out vec4 vLightPos;

void main(){
    vWorldPos   = inPos;
    vNormal     = inNormal;
    vLightPos   = u.lightVP * vec4(inPos, 1.0);
    gl_Position = u.viewProj * vec4(inPos, 1.0);
}

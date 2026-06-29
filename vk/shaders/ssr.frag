#version 450
// Screen-space reflections. For reflective solid pixels (metal), march the
// reflection ray in world space, project to screen, and test against the
// G-buffer position to find a hit, then blend the lit colour there. Passes
// non-reflective pixels straight through.
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;
    vec4 camPos;     // xyz
} u;
layout(set = 0, binding = 1) uniform sampler2D litColor;   // lit HDR (with water)
layout(set = 0, binding = 2) uniform sampler2D gPosition;
layout(set = 0, binding = 3) uniform sampler2D gNormalRough;
layout(set = 0, binding = 4) uniform sampler2D gAlbedo;    // .a = metalness

vec3 F_Schlick(float c, vec3 F0){ return F0 + (1.0-F0)*pow(clamp(1.0-c,0.0,1.0),5.0); }

void main(){
    vec3 base = texture(litColor, uv).rgb;
    vec4 pf = texture(gPosition, uv);
    float metal = texture(gAlbedo, uv).a;
    vec4 nr = texture(gNormalRough, uv);
    float rough = nr.a;
    if(pf.a < 0.5 || metal < 0.1){ outColor = vec4(base,1.0); return; }

    vec3 P = pf.xyz;
    vec3 N = normalize(nr.xyz);
    vec3 V = normalize(u.camPos.xyz - P);
    vec3 R = reflect(-V, N);

    const int STEPS = 48;
    const float stepLen = 1.6;     // world units per step
    const float thickness = 3.0;   // hit tolerance behind the surface
    vec3 sp = P + N*0.2;
    bool hit = false; vec2 hitUV = vec2(0.0);
    for(int i=0;i<STEPS;i++){
        sp += R*stepLen;
        vec4 clip = u.viewProj * vec4(sp,1.0);
        if(clip.w <= 0.0) break;
        vec2 suv = (clip.xy/clip.w)*0.5 + 0.5;
        if(suv.x<0.0||suv.x>1.0||suv.y<0.0||suv.y>1.0) break;
        vec4 g = texture(gPosition, suv);
        if(g.a < 0.5) continue;                       // sky texel: no occluder here
        float sceneDist = distance(u.camPos.xyz, g.xyz);
        float sampDist  = distance(u.camPos.xyz, sp);
        if(sampDist > sceneDist && (sampDist - sceneDist) < thickness){ hit = true; hitUV = suv; break; }
    }

    if(hit){
        vec3 albedo = pow(texture(gAlbedo, uv).rgb, vec3(2.2));
        vec3 F0 = mix(vec3(0.04), albedo, metal);
        float NoV = max(dot(N,V), 1e-3);
        vec3 Fr = F_Schlick(NoV, F0);
        vec3 refl = texture(litColor, hitUV).rgb;
        vec2 e = abs(hitUV*2.0-1.0); float edge = clamp(1.0 - max(e.x,e.y)*max(e.x,e.y), 0.0, 1.0);
        float k = clamp(metal*(1.0-rough)*edge, 0.0, 1.0);
        base = mix(base, refl*Fr + refl*0.15, k);     // tinted metal reflection
    }
    outColor = vec4(base, 1.0);
}

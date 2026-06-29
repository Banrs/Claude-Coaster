#version 450
// PBR (Cook-Torrance GGX) forward shading for the sun, with a PCF shadow map and
// hemispherical ambient. Outputs LINEAR HDR; the post pass does exposure+ACES.
layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vColor;
layout(location = 3) in vec4 vLightPos;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;     // xyz
    vec4 camPos;     // xyz, w = fog end
} u;
layout(set = 0, binding = 1) uniform sampler2D shadowMap;

layout(location = 0) out vec4 outColor;

const float PI = 3.14159265359;
float D_GGX(float NoH, float a){ float a2=a*a; float d=(NoH*NoH)*(a2-1.0)+1.0; return a2/(PI*d*d); }
float G_SchlickGGX(float NoX, float k){ return NoX/(NoX*(1.0-k)+k); }
float G_Smith(float NoV, float NoL, float r){ float k=(r+1.0); k=(k*k)/8.0; return G_SchlickGGX(NoV,k)*G_SchlickGGX(NoL,k); }
vec3  F_Schlick(float c, vec3 F0){ return F0 + (1.0-F0)*pow(clamp(1.0-c,0.0,1.0),5.0); }

float sampleShadow(vec4 lp, float ndl){
    vec3 p = lp.xyz / lp.w;
    vec2 uv = p.xy * 0.5 + 0.5;
    if(uv.x<0.0||uv.x>1.0||uv.y<0.0||uv.y>1.0||p.z>1.0) return 1.0;
    float bias = max(0.0030*(1.0-ndl), 0.0009);
    vec2 texel = 1.0 / vec2(textureSize(shadowMap,0));
    float vis = 0.0;
    for(int y=-1;y<=1;y++) for(int x=-1;x<=1;x++){
        float d = texture(shadowMap, uv + vec2(x,y)*texel).r;
        vis += (p.z - bias > d) ? 0.0 : 1.0;
    }
    return vis / 9.0;
}

void main(){
    vec3 N = normalize(vNormal);
    vec3 V = normalize(u.camPos.xyz - vWorldPos);
    vec3 L = normalize(u.sunDir.xyz);
    vec3 H = normalize(V + L);

    vec3  albedo = pow(vColor, vec3(2.2));
    float rough = 0.82, metal = 0.0;
    vec3  F0 = mix(vec3(0.04), albedo, metal);

    float NoL=max(dot(N,L),0.0), NoV=max(dot(N,V),1e-4), NoH=max(dot(N,H),0.0), VoH=max(dot(V,H),0.0);
    float D=D_GGX(NoH,rough*rough), G=G_Smith(NoV,NoL,rough);
    vec3  F=F_Schlick(VoH,F0);
    vec3  spec=(D*G*F)/(4.0*NoV*NoL+1e-4);
    vec3  kd=(vec3(1.0)-F)*(1.0-metal);
    vec3  diffuse=kd*albedo/PI;

    float shadow = sampleShadow(vLightPos, NoL);
    vec3 sunRad = vec3(3.4,3.1,2.7);
    vec3 Lo = (diffuse + spec) * sunRad * NoL * shadow;

    vec3 sky=vec3(0.34,0.45,0.66), ground=vec3(0.16,0.15,0.13);
    vec3 amb = mix(ground, sky, clamp(N.y*0.5+0.5,0.0,1.0)) * albedo;
    vec3 c = Lo + amb;

    float d = length(vWorldPos - u.camPos.xyz);
    float fogEnd = max(u.camPos.w, 1.0);
    float fog = clamp((d - fogEnd*0.65)/(fogEnd*0.35), 0.0, 1.0); fog *= fog;
    c = mix(c, vec3(0.18,0.34,0.85), fog*0.75);

    outColor = vec4(c, 1.0);
}

#version 450
// Screen-space ambient occlusion in world space. Samples a hemisphere of points
// around each fragment, projects them with the scene viewProj and compares the
// stored G-buffer surface distance to detect occluders. Output: R8 visibility.
layout(location = 0) in vec2 uv;
layout(location = 0) out float outAO;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;
    vec4 camPos;     // xyz
} u;
layout(set = 0, binding = 1) uniform sampler2D gPosition;
layout(set = 0, binding = 2) uniform sampler2D gNormalRough;

// 16 cosine-ish hemisphere offsets (z up), scaled toward the origin so near taps dominate.
const vec3 KERNEL[16] = vec3[16](
    vec3( 0.045, 0.031, 0.072), vec3(-0.082, 0.061, 0.040), vec3( 0.097,-0.054, 0.110),
    vec3(-0.041,-0.092, 0.090), vec3( 0.140, 0.120, 0.060), vec3(-0.150, 0.040, 0.170),
    vec3( 0.020,-0.180, 0.150), vec3( 0.190, 0.090, 0.200), vec3(-0.210,-0.130, 0.110),
    vec3( 0.060, 0.250, 0.240), vec3(-0.270, 0.140, 0.090), vec3( 0.230,-0.220, 0.310),
    vec3(-0.120, 0.330, 0.280), vec3( 0.360, 0.060, 0.150), vec3(-0.340,-0.300, 0.360),
    vec3( 0.180, 0.380, 0.430)
);

float hash(vec2 p){ return fract(sin(dot(p,vec2(41.3,289.1)))*43758.5453); }

void main(){
    vec4 pf = texture(gPosition, uv);
    if(pf.a < 0.5){ outAO = 1.0; return; }            // background: fully lit
    vec3 P = pf.xyz;
    vec3 N = normalize(texture(gNormalRough, uv).xyz);

    // random tangent basis, rotated per pixel to hide banding
    float ang = hash(uv*vec2(textureSize(gPosition,0)))*6.2831853;
    vec3 randv = vec3(cos(ang), sin(ang), 0.0);
    vec3 T = normalize(randv - N*dot(randv,N));
    vec3 B = cross(N, T);
    mat3 TBN = mat3(T, B, N);

    const float radius = 3.2;        // world units
    float occ = 0.0;
    for(int i=0;i<16;i++){
        vec3 sp = P + (TBN * KERNEL[i]) * radius;
        vec4 clip = u.viewProj * vec4(sp,1.0);
        if(clip.w <= 0.0) continue;
        vec2 su = (clip.xy/clip.w)*0.5 + 0.5;
        if(su.x<0.0||su.x>1.0||su.y<0.0||su.y>1.0) continue;
        vec4 spf = texture(gPosition, su);
        if(spf.a < 0.5) continue;                      // sky sample, no occluder
        float surfDist = distance(u.camPos.xyz, spf.xyz);
        float sampDist = distance(u.camPos.xyz, sp);
        if(surfDist < sampDist - 0.05){
            float rangeCheck = smoothstep(0.0, 1.0, radius / (abs(sampDist - surfDist) + 1e-3));
            occ += rangeCheck;
        }
    }
    float ao = 1.0 - (occ / 16.0) * 1.05;
    outAO = clamp(ao, 0.0, 1.0);
}

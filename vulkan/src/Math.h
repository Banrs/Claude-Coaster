// Math.h — minimal column-major vec/mat math for the Vulkan renderer.
// Replaces raylib's raymath for the renderer-agnostic core. Vulkan clip space
// (Y-down, depth 0..1) is handled in perspectiveVk().
#pragma once
#include <cmath>

struct Vec3 {
    float x=0,y=0,z=0;
    Vec3(){} Vec3(float a,float b,float c):x(a),y(b),z(c){}
};
static inline Vec3 operator+(Vec3 a, Vec3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline Vec3 operator-(Vec3 a, Vec3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline Vec3 operator*(Vec3 a, float s){ return {a.x*s,a.y*s,a.z*s}; }
static inline float dot(Vec3 a, Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline Vec3 cross(Vec3 a, Vec3 b){ return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static inline float length(Vec3 a){ return sqrtf(dot(a,a)); }
static inline Vec3 normalize(Vec3 a){ float l=length(a); return l>1e-8f ? a*(1.0f/l) : Vec3{0,1,0}; }
static inline Vec3 lerp(Vec3 a, Vec3 b, float t){ return a + (b-a)*t; }

// column-major 4x4 (m[col*4+row]), GLSL-compatible
struct Mat4 {
    float m[16];
    static Mat4 identity(){ Mat4 r{}; for(int i=0;i<16;i++) r.m[i]=0; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1; return r; }
};
static inline Mat4 mul(const Mat4&a, const Mat4&b){
    Mat4 r{};
    for(int c=0;c<4;c++) for(int row=0;row<4;row++){
        float s=0; for(int k=0;k<4;k++) s += a.m[k*4+row]*b.m[c*4+k];
        r.m[c*4+row]=s;
    }
    return r;
}
static inline Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up){
    Vec3 f = normalize(center - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 r = Mat4::identity();
    r.m[0]=s.x; r.m[4]=s.y; r.m[8]=s.z;
    r.m[1]=u.x; r.m[5]=u.y; r.m[9]=u.z;
    r.m[2]=-f.x;r.m[6]=-f.y;r.m[10]=-f.z;
    r.m[12]=-dot(s,eye); r.m[13]=-dot(u,eye); r.m[14]=dot(f,eye);
    return r;
}
// Vulkan orthographic (RH, depth [0,1]) — used for the sun shadow map.
static inline Mat4 orthoVk(float halfW, float halfH, float zn, float zf){
    Mat4 r{}; for(int i=0;i<16;i++) r.m[i]=0;
    r.m[0]=1.0f/halfW; r.m[5]=1.0f/halfH;
    r.m[10]=-1.0f/(zf-zn); r.m[14]=-zn/(zf-zn); r.m[15]=1.0f;
    return r;
}
// Vulkan perspective: right-handed, depth [0,1], with Y flip baked in.
static inline Mat4 perspectiveVk(float fovyRad, float aspect, float zn, float zf){
    float t = 1.0f/tanf(fovyRad*0.5f);
    Mat4 r{}; for(int i=0;i<16;i++) r.m[i]=0;
    r.m[0]  = t/aspect;
    r.m[5]  = -t;                 // flip Y for Vulkan
    r.m[10] = zf/(zn-zf);
    r.m[11] = -1.0f;
    r.m[14] = (zn*zf)/(zn-zf);
    return r;
}

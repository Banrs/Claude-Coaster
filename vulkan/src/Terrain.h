// Terrain.h — renderer-agnostic world generation, ported from the base game
// (terrainH / biome palette in ../../src/main.cpp + pathtrace.cpp). Produces a
// triangle mesh (positions + normals + colors) the Vulkan renderer uploads once.
#pragma once
#include "Math.h"
#include <vector>
#include <cstdint>

struct Vertex { Vec3 pos; Vec3 nrm; Vec3 col; };
struct Mesh   { std::vector<Vertex> verts; std::vector<uint32_t> idx; };

namespace world {

static const float WATER_Y   = 64.0f;   // global sea level (Minecraft-style)
static const float TERRA_MAX  = 320.0f;

static inline float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }
static inline float smooth01(float a,float b,float x){ float t=clampf((x-a)/(b-a),0,1); return t*t*(3-2*t); }
static inline float hashf(int x,int z){
    uint32_t h=(uint32_t)x*374761393u+(uint32_t)z*668265263u; h=(h^(h>>13))*1274126177u; h^=h>>16;
    return (h&0xffffff)/16777215.0f;
}
static inline float vnoise(float x,float z){
    int xi=(int)floorf(x),zi=(int)floorf(z); float xf=x-xi,zf=z-zi;
    xf=xf*xf*(3-2*xf); zf=zf*zf*(3-2*zf);
    float a=hashf(xi,zi),b=hashf(xi+1,zi),c=hashf(xi,zi+1),d=hashf(xi+1,zi+1);
    return a+(b-a)*xf+(c-a)*zf+(a-b-c+d)*xf*zf;
}
static inline float fbm(float x,float z,int o){ float a=0,amp=1,fr=1,n=0; for(int i=0;i<o;i++){a+=amp*vnoise(x*fr,z*fr);n+=amp;amp*=0.5f;fr*=2;} return a/n; }
static inline float ridgef(float x,float z,int o){ float a=0,amp=1,fr=1,n=0; for(int i=0;i<o;i++){float v=1.0f-fabsf(vnoise(x*fr,z*fr)*2-1);a+=amp*v*v;n+=amp;amp*=0.5f;fr*=2;} return a/n; }

static inline int terrainH(float x,float z){
    float warpX=(vnoise(x*0.0011f+17.5f,z*0.0011f+91.0f)-0.5f)*220.0f;
    float warpZ=(vnoise(x*0.0011f+53.0f,z*0.0011f+11.5f)-0.5f)*220.0f;
    float wx=x+warpX,wz=z+warpZ;
    float c=fbm(wx*0.0015f+0.5f,wz*0.0015f+0.5f,3);
    float e=fbm(wx*0.0040f+31.7f,wz*0.0040f+12.3f,2);
    float pv=ridgef(wx*0.0048f+5.0f,wz*0.0048f+9.0f,3);
    float det=fbm(wx*0.020f,wz*0.020f,2);
    float mesaMask=smooth01(0.58f,0.82f,fbm(wx*0.0010f+101.0f,wz*0.0010f+44.0f,2));
    float basin=smooth01(0.72f,0.94f,1.0f-ridgef(wx*0.0022f+3.7f,wz*0.0022f+8.1f,2));
    float mountainRegion=smooth01(0.50f,0.84f,fbm(wx*0.00085f+9.0f,wz*0.00085f+73.0f,2));
    float valleyMask=smooth01(0.62f,0.90f,ridgef(wx*0.0017f+61.0f,wz*0.0017f+19.0f,2));
    float midHill=fbm(wx*0.008f+32.0f,wz*0.008f+77.0f,3)-0.5f;
    float base=24.0f+powf(c,1.30f)*150.0f;
    float mAmp=powf(1.0f-e,1.62f);
    float mtn=powf(pv,2.36f)*mAmp*(92.0f+142.0f*mountainRegion);
    float h=base+mtn+(det-0.5f)*14.0f+midHill*22.0f;
    h+=powf(pv,5.0f)*smooth01(0.48f,0.92f,mountainRegion)*(42.0f+46.0f*(1.0f-e));
    h-=basin*(22.0f+48.0f*(1.0f-c));
    h-=valleyMask*(1.0f-mesaMask)*(8.0f+18.0f*(1.0f-c));
    float terraceStep=5.0f+8.0f*vnoise(wx*0.0018f+211.0f,wz*0.0018f+37.0f);
    float terraced=floorf(h/terraceStep)*terraceStep+(det-0.5f)*3.0f;
    h=h+(terraced-h)*mesaMask*0.58f;
    h+=mesaMask*smooth01(0.35f,0.70f,c)*18.0f;
    if(h<1)h=1; if(h>TERRA_MAX)h=TERRA_MAX;
    return (int)h;
}

// Biome classification — ported verbatim from the base game (../../src/main.cpp,
// the per-cell block-colour + tree pass): bio/humid/temp noise + elevation pick a
// cap (top) colour, a side colour, and a tree type + density. This is the single
// source of truth so terrain colours and trees agree, the way Minecraft biomes do.
struct Biome { Vec3 cap, side; int treeType; float treeDen; bool grassCap; };
static inline Vec3 C8(int r,int g,int b){ return Vec3{r/255.f,g/255.f,b/255.f}; }
static inline Biome biomeAt(float wx, float wz, int h){
    Biome bm; bm.treeType=-1; bm.treeDen=0.0f; bm.grassCap=true;
    bool beach = ((float)h+1.0f) <= WATER_Y+0.6f;
    Vec3 GRASS=C8(130,206,102), SAND=C8(242,228,184), DIRT=C8(158,116,82);
    Vec3 capC=GRASS, colC=DIRT;
    float bio   = vnoise(wx*0.0045f+91.3f, wz*0.0045f+23.1f);
    float humid = fbm(wx*0.0028f+44.0f, wz*0.0028f+108.0f, 2);
    float temp  = fbm(wx*0.0019f+12.0f, wz*0.0019f+204.0f, 2);
    if      (h>=260)                       { capC=C8(204,214,224); colC=C8(132,140,154); bm.grassCap=false; }
    else if (h>=158)                       { capC=C8(128,138,146); colC=C8(108,116,126); bm.grassCap=false; }
    else if (beach)                        { capC=SAND;                                   bm.grassCap=false; }
    else if (humid<0.23f && temp>0.42f)    { capC=C8(214,196,108); colC=C8(162,126, 72); bm.grassCap=false; bm.treeType=3; bm.treeDen=0.003f; } // savanna
    else if (humid>0.72f && bio<0.72f)     { capC=C8( 76,176, 92); colC=C8(118, 96, 72);                    bm.treeType=0; bm.treeDen=0.032f; } // dense forest
    else if (bio<0.34f)                    {                                                                bm.treeType=0; bm.treeDen=0.007f; } // plains/oak
    else if (bio<0.58f)                    { capC=C8(118,206,108);                                          bm.treeType=1; bm.treeDen=0.022f; } // birch
    else if (bio<0.78f)                    { capC=C8(210,202,132);                                          bm.treeType=3; bm.treeDen=0.004f; } // dry/acacia
    else                                   { capC=C8(112,150,112); colC=C8(118,104, 86);                    bm.treeType=2; bm.treeDen=0.010f; } // taiga/spruce
    if (bm.grassCap){
        float patch = vnoise(wx*0.03f+7.7f, wz*0.03f+4.2f);
        capC = lerp(capC, lerp(C8(96,188,96), C8(196,206,120), patch), 0.35f);
    }
    bm.cap=capC; bm.side=colC;
    return bm;
}

// One flat-shaded quad (a,b,c,d CCW) with an explicit face normal + color.
inline void addQuad(Mesh& out, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n, Vec3 col){
    uint32_t base=(uint32_t)out.verts.size();
    out.verts.push_back({a,n,col}); out.verts.push_back({b,n,col});
    out.verts.push_back({c,n,col}); out.verts.push_back({d,n,col});
    out.idx.insert(out.idx.end(), { base,base+1,base+2, base,base+2,base+3 });
}

// An oriented box: center C, orthonormal axes (R,U,F), half-extents (hr,hu,hf).
inline void addBox(Mesh& out, Vec3 C, Vec3 R, Vec3 U, Vec3 F,
                   float hr, float hu, float hf, Vec3 col){
    Vec3 r=R*hr, u=U*hu, f=F*hf;
    Vec3 c[8] = { C-r-u-f, C+r-u-f, C+r+u-f, C-r+u-f, C-r-u+f, C+r-u+f, C+r+u+f, C-r+u+f };
    auto quad=[&](int a,int b,int d,int e,Vec3 n){ addQuad(out,c[a],c[b],c[d],c[e],n,col); };
    quad(0,3,2,1, F*-1.0f); quad(4,5,6,7, F);
    quad(0,1,5,4, U*-1.0f); quad(3,7,6,2, U);
    quad(0,4,7,3, R*-1.0f); quad(1,2,6,5, R);
}

// Build a *blocky* (voxel) terrain surface over a (2*half)^2 area of 1 m cells,
// matching the base game's look: each cell is a flat-topped block (cap quad at
// its integer height) plus vertical side faces dropping only to the lower
// neighbour. Faces are flat-normal, so the result reads as cubes, and nothing
// below a neighbour's surface is ever built (never-visible = not drawn).
inline void buildTerrain(float cx, float cz, float half, float /*step*/, Mesh& out){
    out.verts.clear(); out.idx.clear();
    int R = (int)half;
    int ox = (int)floorf(cx), oz = (int)floorf(cz);
    for(int dz=-R; dz<=R; dz++) for(int dx=-R; dx<=R; dx++){
        int iwx=ox+dx, iwz=oz+dz; float wx=(float)iwx, wz=(float)iwz;
        int h=terrainH(wx,wz);
        float top=(float)h+1.0f;
        Biome bm=biomeAt(wx,wz,h);
        float sh=0.89f + 0.13f*hashf(iwx*5+1, iwz*5+2);   // per-cell shade (base game)
        // subtle Minecraft-style grass-colour blend: soften biome borders with a
        // light center-weighted 5-tap over the cap colour (much less than before).
        const float D=2.0f;
        Vec3 capBlend = bm.cap*0.56f
            + biomeAt(wx+D,wz,terrainH(wx+D,wz)).cap*0.11f
            + biomeAt(wx-D,wz,terrainH(wx-D,wz)).cap*0.11f
            + biomeAt(wx,wz+D,terrainH(wx,wz+D)).cap*0.11f
            + biomeAt(wx,wz-D,terrainH(wx,wz-D)).cap*0.11f;
        Vec3 cap=capBlend*sh;
        Vec3 body=bm.side*(sh*0.95f);             // distinct dirt/rock side colour (unblended, like MC)
        float xm=wx-0.5f, xp=wx+0.5f, zm=wz-0.5f, zp=wz+0.5f;

        addQuad(out, {xm,top,zm},{xp,top,zm},{xp,top,zp},{xm,top,zp}, {0,1,0}, cap);

        int hpx=terrainH(wx+1,wz), hnx=terrainH(wx-1,wz);
        int hpz=terrainH(wx,wz+1), hnz=terrainH(wx,wz-1);
        if(h>hpx){ float b=(float)hpx+1.0f; addQuad(out, {xp,b,zm},{xp,top,zm},{xp,top,zp},{xp,b,zp}, { 1,0,0}, body); }
        if(h>hnx){ float b=(float)hnx+1.0f; addQuad(out, {xm,b,zm},{xm,top,zm},{xm,top,zp},{xm,b,zp}, {-1,0,0}, body); }
        if(h>hpz){ float b=(float)hpz+1.0f; addQuad(out, {xm,b,zp},{xp,b,zp},{xp,top,zp},{xm,top,zp}, {0,0, 1}, body); }
        if(h>hnz){ float b=(float)hnz+1.0f; addQuad(out, {xm,b,zm},{xp,b,zm},{xp,top,zm},{xm,top,zm}, {0,0,-1}, body); }
    }
}

// Flat water surface quad at WATER_Y over the same area.
inline void appendWater(float cx, float cz, float half, Mesh& out){
    uint32_t base=(uint32_t)out.verts.size();
    Vec3 wn={0,1,0}, wc={0.20f,0.42f,0.55f};
    out.verts.push_back({ Vec3{cx-half,WATER_Y,cz-half}, wn, wc });
    out.verts.push_back({ Vec3{cx+half,WATER_Y,cz-half}, wn, wc });
    out.verts.push_back({ Vec3{cx-half,WATER_Y,cz+half}, wn, wc });
    out.verts.push_back({ Vec3{cx+half,WATER_Y,cz+half}, wn, wc });
    out.idx.insert(out.idx.end(), { base,base+2,base+1, base+1,base+2,base+3 });
}

// A single tree of `type` (0 oak, 1 birch, 2 spruce, 3 acacia) at (x,top,z) —
// trunk + tiered leaf boxes, ported verbatim from the base game's switch(treeType)
// (drawCubeTex dims are full extents; addBox takes half-extents).
inline void addTree(Mesh& out, int type, float x, float top, float z, float sh){
    Vec3 I{1,0,0}, J{0,1,0}, K{0,0,1};
    auto cl=[](Vec3 c){ return Vec3{ c.x>1?1:c.x, c.y>1?1:c.y, c.z>1?1:c.z }; };
    auto box=[&](float cy,float w,float hh,float l,Vec3 col){
        addBox(out, Vec3{x,top+cy,z}, I,J,K, w*0.5f, hh*0.5f, l*0.5f, cl(col*sh)); };
    if(type==0){               // oak
        Vec3 wood=C8(124,96,62), leaf=C8(108,192,98);
        box(2.6f, 0.8f,5.2f,0.8f, wood);
        box(6.6f, 4.6f,2.6f,4.6f, leaf);
        box(8.8f, 3.0f,1.9f,3.0f, leaf*1.08f);
    } else if(type==1){        // birch (pale bark)
        Vec3 wood=C8(214,209,194), leaf=C8(112,162,81);
        box(3.3f, 0.7f,6.6f,0.7f, wood);
        box(7.8f, 3.6f,2.4f,3.6f, leaf);
        box(10.2f, 2.3f,1.6f,2.3f, leaf*1.07f);
    } else if(type==2){        // spruce / taiga conifer (tapered tiers)
        Vec3 wood=C8(82,60,40), leaf=C8(65,101,65);
        box(3.2f, 0.7f,6.4f,0.7f, wood);
        box(4.4f, 4.4f,1.8f,4.4f, leaf);
        box(6.6f, 3.4f,1.8f,3.4f, leaf*1.05f);
        box(8.8f, 2.4f,1.7f,2.4f, leaf*1.10f);
        box(10.8f, 1.3f,1.6f,1.3f, leaf*1.15f);
    } else {                   // acacia (flat wide canopy)
        Vec3 wood=C8(106,82,53), leaf=C8(131,144,65);
        box(1.9f, 0.65f,3.8f,0.65f, wood);
        box(4.6f, 5.2f,2.0f,5.2f, leaf);
        box(6.0f, 3.4f,1.4f,3.4f, leaf*1.07f);
    }
}

// Place trees by biome — ported from the base game's tree pass: an 8-cell grid,
// per-node density = min(treeDen*64, 0.9), jittered, with the bio<0.58 birch
// half-converted to oak. Tree type/density come from the same biomeAt() used for
// the ground colours, so the forest matches its biome (Minecraft-style).
inline void buildTrees(float cx, float cz, float half, Mesh& out){
    const int TG=8;
    int R=(int)half; int ox=(int)floorf(cx), oz=(int)floorf(cz);
    for(int dz=-R; dz<=R; dz++) for(int dx=-R; dx<=R; dx++){
        int iwx=ox+dx, iwz=oz+dz;
        if((iwx%TG)!=0 || (iwz%TG)!=0) continue;          // tree-node grid
        float wx=(float)iwx, wz=(float)iwz;
        int h=terrainH(wx,wz); float top=(float)h+1.0f;
        if(top<=WATER_Y+0.6f) continue;                   // no trees on beach/water
        Biome bm=biomeAt(wx,wz,h);
        int treeType=bm.treeType; if(treeType<0) continue;
        float nodeDen=fminf(bm.treeDen*(float)(TG*TG), 0.90f);
        float th=hashf(iwx*9+7, iwz*9+3);
        if(th>=nodeDen) continue;
        if(treeType==1 && th>nodeDen*0.5f) treeType=0;    // thin birch -> oak
        float jx=(hashf(iwx*3+1, iwz*7+5)-0.5f)*(float)(TG-5);
        float jz=(hashf(iwx*5+9, iwz*3+2)-0.5f)*(float)(TG-5);
        float sh=0.89f + 0.13f*hashf(iwx*5+1, iwz*5+2);
        addTree(out, treeType, wx+jx, top, wz+jz, sh);
    }
}

} // namespace world

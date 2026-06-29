#include "VoxelScene.h"
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Noise / terrain — ported verbatim from ../../opengl/src/main.cpp (lines 71..138)
// ---------------------------------------------------------------------------
static inline float clampf(float v, float a, float b){ return v<a?a:(v>b?b:v); }
static inline float srgb2lin(float c){ return powf(c, 2.2f); }

static float smooth01(float a, float b, float x){
    float t = clampf((x - a) / (b - a), 0.0f, 1.0f);
    return t*t*(3.0f - 2.0f*t);
}
static float hashf(int x, int z){
    uint32_t h = (uint32_t)x*374761393u + (uint32_t)z*668265263u;
    h = (h ^ (h>>13))*1274126177u; h ^= h>>16;
    return (h & 0xffffff)/16777215.0f;
}
static float vnoise(float x, float z){
    int xi=(int)floorf(x), zi=(int)floorf(z);
    float xf=x-xi, zf=z-zi;
    xf=xf*xf*(3-2*xf); zf=zf*zf*(3-2*zf);
    float a=hashf(xi,zi), b=hashf(xi+1,zi), c=hashf(xi,zi+1), d=hashf(xi+1,zi+1);
    return a + (b-a)*xf + (c-a)*zf + (a-b-c+d)*xf*zf;
}
static float fbm(float x, float z, int oct){
    float a=0, amp=1, fr=1, norm=0;
    for(int i=0;i<oct;i++){ a+=amp*vnoise(x*fr,z*fr); norm+=amp; amp*=0.5f; fr*=2.0f; }
    return a/norm;
}
static float ridgef(float x, float z, int oct){
    float a=0, amp=1, fr=1, norm=0;
    for(int i=0;i<oct;i++){ float n=1.0f-fabsf(vnoise(x*fr,z*fr)*2.0f-1.0f); a+=amp*n*n; norm+=amp; amp*=0.5f; fr*=2.0f; }
    return a/norm;
}
static int terrainH(float x, float z){
    float warpX=(vnoise(x*0.0011f+17.5f, z*0.0011f+91.0f)-0.5f)*220.0f;
    float warpZ=(vnoise(x*0.0011f+53.0f, z*0.0011f+11.5f)-0.5f)*220.0f;
    float wx=x+warpX, wz=z+warpZ;
    float c =fbm(wx*0.0015f+0.5f,  wz*0.0015f+0.5f, 3);
    float e =fbm(wx*0.0040f+31.7f, wz*0.0040f+12.3f, 2);
    float pv=ridgef(wx*0.0048f+5.0f, wz*0.0048f+9.0f, 3);
    float det=fbm(wx*0.020f, wz*0.020f, 2);
    float mesaMask=smooth01(0.58f,0.82f, fbm(wx*0.0010f+101.0f, wz*0.0010f+44.0f, 2));
    float basin   =smooth01(0.72f,0.94f, 1.0f-ridgef(wx*0.0022f+3.7f, wz*0.0022f+8.1f, 2));
    float mountainRegion=smooth01(0.50f,0.84f, fbm(wx*0.00085f+9.0f, wz*0.00085f+73.0f, 2));
    float valleyMask=smooth01(0.62f,0.90f, ridgef(wx*0.0017f+61.0f, wz*0.0017f+19.0f, 2));
    float midHill=fbm(wx*0.008f+32.0f, wz*0.008f+77.0f, 3)-0.5f;
    float base=24.0f+powf(c,1.30f)*150.0f;
    float mAmp=powf(1.0f-e,1.62f);
    float mtn=powf(pv,2.36f)*mAmp*(92.0f+142.0f*mountainRegion);
    float h=base+mtn+(det-0.5f)*14.0f+midHill*22.0f;
    h+=powf(pv,5.0f)*smooth01(0.48f,0.92f,mountainRegion)*(42.0f+46.0f*(1.0f-e));
    h-=basin*(22.0f+48.0f*(1.0f-c));
    h-=valleyMask*(1.0f-mesaMask)*(8.0f+18.0f*(1.0f-c));
    float terraceStep=5.0f+8.0f*vnoise(wx*0.0018f+211.0f, wz*0.0018f+37.0f);
    float terraced=floorf(h/terraceStep)*terraceStep+(det-0.5f)*3.0f;
    h=h+(terraced-h)*mesaMask*0.58f;
    h+=mesaMask*smooth01(0.35f,0.70f,c)*18.0f;
    if(h<1)h=1; if(h>(float)TERRA_MAX)h=(float)TERRA_MAX;
    return (int)h;
}

// ---------------------------------------------------------------------------
// Biome palette — ported from pt_biomeColor (../../opengl/src/pathtrace.cpp 708..732)
// colors are 0..255; we convert to linear on store.
// ---------------------------------------------------------------------------
struct C8 { float r,g,b; };
static C8 mixc(C8 a, C8 b, float t){ return { a.r+(b.r-a.r)*t, a.g+(b.g-a.g)*t, a.b+(b.b-a.b)*t }; }
static C8 shade(C8 a, float s){ return { a.r*s, a.g*s, a.b*s }; }

static void biomeColor(float wx, float wz, int h, C8& capC, C8& bodyC)
{
    bool beach = (h + 1.0f) <= (float)WATER_Y + 0.6f;
    float bio  = vnoise(wx*0.0045f+91.3f, wz*0.0045f+23.1f);
    float humid= fbm(wx*0.0028f+44.0f, wz*0.0028f+108.0f, 2);
    float temp = fbm(wx*0.0019f+12.0f, wz*0.0019f+204.0f, 2);
    C8 GRASS={130,206,102}, DIRT={158,116,82}, SAND={242,228,184};
    capC=GRASS; bodyC=DIRT;
    if(h>=260)      { capC={204,214,224}; bodyC={132,140,154}; }
    else if(h>=158) { capC={128,138,146}; bodyC={108,116,126}; }
    else if(beach)  { capC=SAND; }
    else if(humid<0.23f && temp>0.42f){ capC={214,196,108}; bodyC={162,126,72}; }
    else if(humid>0.72f && bio<0.72f) { capC={76,176,92};  bodyC={118,96,72}; }
    else if(bio<0.34f){}
    else if(bio<0.58f){ capC={118,206,108}; }
    else if(bio<0.78f){ capC={210,202,132}; }
    else { capC={112,150,112}; bodyC={118,104,86}; }
    if(capC.r==GRASS.r && capC.g==GRASS.g && capC.b==GRASS.b){
        float patch=vnoise(wx*0.03f+7.7f, wz*0.03f+4.2f);
        C8 lush={96,188,96}, dry={196,206,120};
        capC=mixc(capC, mixc(lush,dry,patch), 0.35f);
    }
    int cx=(int)floorf(wx), cz=(int)floorf(wz);
    float s=0.89f+0.13f*hashf(cx*5+1, cz*5+2);
    capC=shade(capC,s); bodyC=shade(bodyC,s*0.95f);
}

// ---------------------------------------------------------------------------
void VoxelScene::putVoxel(int gx,int gy,int gz, float r,float g,float b,float mat)
{
    if(gx<0||gy<0||gz<0||gx>=PT_NX||gy>=PT_NY||gz>=PT_NZ) return;
    size_t i = idx(gx,gy,gz)*4;
    grid[i+0]=srgb2lin(r); grid[i+1]=srgb2lin(g); grid[i+2]=srgb2lin(b); grid[i+3]=mat;
}
void VoxelScene::putWorld(float wx,float wy,float wz, float r,float g,float b,float mat)
{
    int gx=(int)floorf((wx-gridMinX)/(float)PT_VOX);
    int gy=(int)floorf((wy-gridMinY)/(float)PT_VOX);
    int gz=(int)floorf((wz-gridMinZ)/(float)PT_VOX);
    putVoxel(gx,gy,gz, r,g,b, mat);
}

// ---------------------------------------------------------------------------
void VoxelScene::bake(float camX, float camZ, float trackU)
{
    std::fill(grid.begin(), grid.end(), 0.0f);

    int baseY = (int)floorf((float)WATER_Y - 8.0f);
    gridMinX = floorf(camX/(float)PT_VOX)*(float)PT_VOX - (PT_NX/2)*(float)PT_VOX;
    gridMinY = (float)baseY;
    gridMinZ = floorf(camZ/(float)PT_VOX)*(float)PT_VOX - (PT_NZ/2)*(float)PT_VOX;

    // ---- terrain columns + water (ported: bakeVoxelsCPU column fill) ----
    for(int gz=0; gz<PT_NZ; gz++)
    for(int gx=0; gx<PT_NX; gx++){
        float wx=gridMinX+(gx+0.5f)*(float)PT_VOX;
        float wz=gridMinZ+(gz+0.5f)*(float)PT_VOX;
        int h=terrainH(wx,wz);
        C8 cap,body; biomeColor(wx,wz,h,cap,body);
        float colDepth=42.0f;
        int topG=(int)floorf((h+0.5f-gridMinY)/(float)PT_VOX);
        int botG=(int)floorf(((float)h-colDepth-gridMinY)/(float)PT_VOX); if(botG<0)botG=0;
        for(int gy=botG; gy<topG; gy++)
            putVoxel(gx,gy,gz, body.r/255.0f, body.g/255.0f, body.b/255.0f, 1.0f);
        if(topG>=0 && topG<PT_NY)
            putVoxel(gx,topG,gz, cap.r/255.0f, cap.g/255.0f, cap.b/255.0f, 1.0f);
        if(h+1.0f < (float)WATER_Y){
            int wG=(int)floorf(((float)WATER_Y-gridMinY)/(float)PT_VOX);
            if(wG>=0 && wG<PT_NY) putVoxel(gx,wG,gz, 0.10f,0.20f,0.26f, 0.6f);
        }
    }

    // ---- trees (ported & condensed: bakeVoxelsCPU tree loop) ----
    int ccx=(int)floorf(camX), ccz=(int)floorf(camZ);
    int R = PT_NX/2;
    for(int dz=-R; dz<=R; dz++)
    for(int dx=-R; dx<=R; dx++){
        int cx=ccx+dx, cz=ccz+dz;
        float wx=cx+0.5f, wz=cz+0.5f;
        int h=terrainH(wx,wz); float top=h+1.0f;
        if(top<=(float)WATER_Y+0.6f) continue;
        float bio=vnoise(wx*0.0045f+91.3f, wz*0.0045f+23.1f);
        float humid=fbm(wx*0.0028f+44.0f, wz*0.0028f+108.0f, 2);
        float temp =fbm(wx*0.0019f+12.0f, wz*0.0019f+204.0f, 2);
        int treeType=-1; float treeDen=0;
        if(h>=158){}
        else if(humid<0.23f && temp>0.42f){ treeType=3; treeDen=0.003f; }
        else if(humid>0.72f && bio<0.72f) { treeType=0; treeDen=0.065f; }
        else if(bio<0.34f){ treeType=0; treeDen=0.012f; }
        else if(bio<0.58f){ treeType=1; treeDen=0.045f; }
        else if(bio<0.78f){ treeType=3; treeDen=0.006f; }
        else { treeType=2; treeDen=0.018f; }
        if(treeType<0) continue;
        if(hashf(cx*9+7, cz*9+3) >= treeDen) continue;
        C8 trunk, leaf;
        switch(treeType){
            case 0: trunk={124,96,62};  leaf={108,192,98}; break;
            case 1: trunk={214,209,194};leaf={112,162,81}; break;
            case 2: trunk={82,60,40};   leaf={65,101,65};  break;
            default:trunk={106,82,53};  leaf={131,144,65}; break;
        }
        for(float ty=top; ty<top+3.0f; ty+=0.6f) putWorld(wx,ty,wz, trunk.r/255,trunk.g/255,trunk.b/255, 1.0f);
        for(int ly=0; ly<=3; ly++)
        for(int lx=-2; lx<=2; lx++)
        for(int lz=-2; lz<=2; lz++){
            float rad=2.4f-ly*0.5f;
            if(lx*lx+lz*lz>rad*rad) continue;
            putWorld(wx+lx*0.9f, top+3.0f+ly*0.9f, wz+lz*0.9f, leaf.r/255,leaf.g/255,leaf.b/255, 1.0f);
        }
    }

    // ---- demo track ribbon (representative; see header note) ----
    // a banked sine+helix spine of PROXY voxels threading the grid centre.
    {
        const float PROXY = (float)MAT_PROXY;
        float cx = gridMinX + (PT_NX*0.5f)*(float)PT_VOX;
        float cz = gridMinZ + (PT_NZ*0.5f)*(float)PT_VOX;
        for(float s=0.0f; s<1.0f; s+=0.0015f){
            float u = s*6.2831853f*3.0f + trackU;
            float wx = cx + sinf(u)*52.0f + sinf(u*0.37f)*18.0f;
            float wz = cz + cosf(u*0.8f)*52.0f;
            float base = (float)terrainH(wx,wz) + 1.0f;
            float wy = base + 26.0f + sinf(u*1.7f)*14.0f + (cosf(u*0.5f)*0.5f+0.5f)*22.0f;
            // rails (two), spine, ties
            for(int rr=-1; rr<=1; rr+=2)
                putWorld(wx+rr*0.6f, wy, wz, 0.74f,0.78f,0.83f, PROXY);
            putWorld(wx, wy-0.3f, wz, 0.46f,0.10f,0.30f, PROXY);
            if(((int)(s*2000.0f))&1) putWorld(wx, wy-0.15f, wz, 0.38f,0.39f,0.42f, PROXY);
            // a couple of support columns down to the ground
            if(((int)(s*200.0f))%7==0)
                for(float yy=base; yy<wy; yy+=1.0f) putWorld(wx, yy, wz, 0.46f,0.48f,0.51f, PROXY);
        }
    }

    buildBricks();
}

// ---------------------------------------------------------------------------
void VoxelScene::buildBricks()
{
    bricks.clear(); brickCoords.clear();
    for(int by=0; by<PT_CNY; by++)
    for(int bz=0; bz<PT_CNZ; bz++)
    for(int bx=0; bx<PT_CNX; bx++){
        int gx0=bx*PT_MK, gy0=by*PT_MK, gz0=bz*PT_MK;
        bool occ=false;
        for(int dy=0; dy<PT_MK && !occ; dy++){ int gy=gy0+dy; if(gy>=PT_NY)break;
        for(int dz=0; dz<PT_MK && !occ; dz++){ int gz=gz0+dz; if(gz>=PT_NZ)break;
        for(int dx=0; dx<PT_MK; dx++){ int gx=gx0+dx; if(gx>=PT_NX)break;
            size_t i=idx(gx,gy,gz)*4;
            if(grid[i+3]>0.001f && (grid[i+0]+grid[i+1]+grid[i+2])>0.004f){ occ=true; break; }
        }}}
        if(!occ) continue;
        float mnx=gridMinX+gx0*(float)PT_VOX;
        float mny=gridMinY+gy0*(float)PT_VOX;
        float mnz=gridMinZ+gz0*(float)PT_VOX;
        int ex=std::min(gx0+PT_MK,PT_NX), ey=std::min(gy0+PT_MK,PT_NY), ez=std::min(gz0+PT_MK,PT_NZ);
        Aabb a{ mnx, mny, mnz,
                gridMinX+ex*(float)PT_VOX, gridMinY+ey*(float)PT_VOX, gridMinZ+ez*(float)PT_VOX };
        bricks.push_back(a);
        brickCoords.push_back(Int4{ gx0, gy0, gz0, 0 });
    }
}

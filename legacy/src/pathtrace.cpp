#ifndef MINECOASTER_PATHTRACE_CPP
#define MINECOASTER_PATHTRACE_CPP

#include <thread>
#include <mutex>
#include <condition_variable>

static const int   PT_NX = 176;
static const int   PT_NZ = 176;
static const int   PT_NY = 168;
static const float PT_VOX = CELL;

static int PT_TILES_X = 0, PT_TILES_Y = 0, PT_ATLAS_W = 0, PT_ATLAS_H = 0;

static const int PT_MK = 8;
static int PT_CNX=0, PT_CNY=0, PT_CNZ=0;
static int PT_CTX=0, PT_CTY=0;
static int PT_COARSE_Y0=0;

struct PTSys {
    Shader trace{};
    Shader resolve{};
    Shader rt{};
    Shader rtBlit{};
    RenderTexture2D accum{};
    RenderTexture2D ping{};
    RenderTexture2D rtBuf{};
    Texture2D vox{};
    int W = 0, H = 0;
    int rtW = 0, rtH = 0;

    int locCamPos=-1, locCamDir=-1, locCamRight=-1, locCamUp=-1, locTan=-1, locAspect=-1;
    int locSunDir=-1, locRes=-1, locFrame=-1, locGridMin=-1, locVoxAtlas=-1;
    int locAtlasSize=-1, locTiles=-1, locGridN=-1, locVoxSize=-1, locPrev=-1;

    int locRAccum=-1, locRRes=-1;

    int rCamPos=-1, rCamDir=-1, rCamRight=-1, rCamUp=-1, rTan=-1, rAspect=-1;
    int rSunDir=-1, rGridMin=-1, rGridN=-1, rVoxSize=-1, rAtlasSize=-1, rTiles=-1;
    int rLightVP=-1, rShadowMap=-1, rShadowTexel=-1;

    int bDepthTex=-1, bInvRes=-1;

    void initShaders();
    void initBuffers(int w, int h);
    void allocAtlas();
    void initLive(int rw, int rh);
    void releaseLiveTarget();
    void setCoarseUniforms();
    void shutdown();
};
static PTSys gPT;

static inline float pt_srgb2lin(float c) { return powf(c, 2.2f); }

static int   terrainH(float x, float z);

void PTSys::initShaders() {

    static const char *PT_VS =
        "#version 330\n"
        "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
        "uniform mat4 mvp; out vec2 uv;\n"
        "void main(){ uv = vertexTexCoord; gl_Position = mvp*vec4(vertexPosition,1.0); }\n";

    static const char *PT_FS =
        "#version 330\n"
        "in vec2 uv; out vec4 finalColor;\n"
        "uniform vec3 camPos; uniform vec3 camDir; uniform vec3 camRight; uniform vec3 camUp;\n"
        "uniform float tanHalfFovY; uniform float aspect;\n"
        "uniform vec3 sunDir; uniform vec2 resolution; uniform int frameIdx;\n"
        "uniform vec3 gridMin; uniform ivec3 gridN; uniform float voxSize;\n"

        "uniform sampler2D texture0; uniform vec2 atlasSize; uniform ivec2 tiles;\n"
        "uniform sampler2D prevAccum;\n"

        "uniform ivec3 coarseN; uniform ivec2 coarseTiles; uniform int coarseY0; uniform int macroK;\n"

        "uint hashU(uint x){ x^=x>>16; x*=0x7feb352dU; x^=x>>15; x*=0x846ca68bU; x^=x>>16; return x; }\n"
        "uint g_seed;\n"
        "float rnd(){ g_seed = hashU(g_seed); return float(g_seed) * (1.0/4294967296.0); }\n"

        "vec4 voxFetch(ivec3 c){\n"
        "  if(c.x<0||c.y<0||c.z<0||c.x>=gridN.x||c.y>=gridN.y||c.z>=gridN.z) return vec4(0.0);\n"
        "  int tx = c.y % tiles.x; int ty = c.y / tiles.x;\n"
        "  vec2 base = vec2(float(tx*gridN.x), float(ty*gridN.z));\n"
        "  vec2 px = base + vec2(float(c.x)+0.5, float(c.z)+0.5);\n"
        "  return texture(texture0, px/atlasSize);\n"
        "}\n"

        "bool solid(ivec3 c){ vec4 v = voxFetch(c); return v.a > 0.5 && (v.r+v.g+v.b) > 0.004; }\n"

        "bool macroEmpty(ivec3 c){\n"
        "  ivec3 cc = c / macroK;\n"
        "  if(cc.x<0||cc.y<0||cc.z<0||cc.x>=coarseN.x||cc.y>=coarseN.y||cc.z>=coarseN.z) return false;\n"
        "  int tx = cc.y % coarseTiles.x, ty = cc.y / coarseTiles.x;\n"
        "  vec2 px = vec2(float(tx*coarseN.x + cc.x)+0.5, float(coarseY0 + ty*coarseN.z + cc.z)+0.5);\n"
        "  return texture(texture0, px/atlasSize).a < 0.5;\n"
        "}\n"

        "bool trace(vec3 ro, vec3 rd, out ivec3 hitC, out vec3 nrm, out float tHit){\n"
        "  vec3 p = (ro - gridMin)/voxSize;\n"
        "  ivec3 c = ivec3(floor(p));\n"
        "  vec3 inv = 1.0/max(abs(rd),vec3(1e-6));\n"
        "  ivec3 step = ivec3(sign(rd));\n"
        "  vec3 tMax, tDelta = inv;\n"
        "  for(int k=0;k<3;k++){\n"
        "    float bound = float(c[k]) + (rd[k]>0.0?1.0:0.0);\n"
        "    tMax[k] = (bound - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "    if(rd[k]==0.0) tMax[k] = 1e9;\n"
        "  }\n"
        "  nrm = vec3(0.0); tHit = 0.0;\n"
        "  float mk = float(macroK);\n"
        "  for(int i=0;i<512;i++){\n"
        "    if(macroEmpty(c)){\n"
        "      ivec3 mc = c / macroK; vec3 mt;\n"
        "      for(int k=0;k<3;k++){\n"
        "        if(rd[k]==0.0){ mt[k]=1e9; continue; }\n"
        "        float mb = float(mc[k]*macroK) + (rd[k]>0.0?mk:0.0);\n"
        "        mt[k] = (mb - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "      }\n"
        "      float tj = min(mt.x, min(mt.y, mt.z));\n"
        "      if(mt.x<=mt.y && mt.x<=mt.z) nrm=vec3(-float(step.x),0,0);\n"
        "      else if(mt.y<=mt.z) nrm=vec3(0,-float(step.y),0);\n"
        "      else nrm=vec3(0,0,-float(step.z));\n"
        "      c = ivec3(floor(p + rd*(tj + 1e-3)));\n"
        "      tHit = tj;\n"
        "      for(int k=0;k<3;k++){\n"
        "        float bnd = float(c[k]) + (rd[k]>0.0?1.0:0.0);\n"
        "        tMax[k] = (bnd - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "        if(rd[k]==0.0) tMax[k] = 1e9;\n"
        "      }\n"
        "      if(c.x<-1||c.y<-1||c.z<-1||c.x>gridN.x||c.y>gridN.y||c.z>gridN.z) break;\n"
        "      continue;\n"
        "    }\n"
        "    if(solid(c)){ hitC = c; tHit *= voxSize; return true; }\n"
        "    if(tMax.x < tMax.y){\n"
        "      if(tMax.x < tMax.z){ c.x+=step.x; tHit=tMax.x; tMax.x+=tDelta.x; nrm=vec3(-float(step.x),0,0);}\n"
        "      else               { c.z+=step.z; tHit=tMax.z; tMax.z+=tDelta.z; nrm=vec3(0,0,-float(step.z));}\n"
        "    } else {\n"
        "      if(tMax.y < tMax.z){ c.y+=step.y; tHit=tMax.y; tMax.y+=tDelta.y; nrm=vec3(0,-float(step.y),0);}\n"
        "      else               { c.z+=step.z; tHit=tMax.z; tMax.z+=tDelta.z; nrm=vec3(0,0,-float(step.z));}\n"
        "    }\n"
        "    if(c.x<-1||c.y<-1||c.z<-1||c.x>gridN.x||c.y>gridN.y||c.z>gridN.z) break;\n"
        "  }\n"
        "  return false;\n"
        "}\n"

        "bool occluded(vec3 ro, vec3 rd){\n"
        "  ivec3 hc; vec3 hn; float ht;\n"
        "  return trace(ro + rd*0.02, rd, hc, hn, ht);\n"
        "}\n"

        "float h13(vec3 p){ p=fract(p*0.1031); p+=dot(p,p.yzx+33.33); return fract((p.x+p.y)*p.z); }\n"
        "float vn3(vec3 x){ vec3 i=floor(x), f=fract(x); f=f*f*(3.0-2.0*f);\n"
        "  return mix(mix(mix(h13(i+vec3(0,0,0)),h13(i+vec3(1,0,0)),f.x), mix(h13(i+vec3(0,1,0)),h13(i+vec3(1,1,0)),f.x),f.y),\n"
        "             mix(mix(h13(i+vec3(0,0,1)),h13(i+vec3(1,0,1)),f.x), mix(h13(i+vec3(0,1,1)),h13(i+vec3(1,1,1)),f.x),f.y), f.z); }\n"
        "float cloudDens(vec3 p){\n"
        "  float cb=300.0, ct=470.0; float hf=(p.y-cb)/(ct-cb);\n"
        "  if(hf<0.0||hf>1.0) return 0.0;\n"
        "  hf = smoothstep(0.0,0.32,hf)*smoothstep(1.0,0.5,hf);\n"
        "  vec3 q=p*0.0012; vec3 qb=floor(q*5.0)/5.0;\n"
        "  float base=mix(vn3(q*5.0+3.1), vn3(qb*5.0+3.1), 0.5);\n"
        "  float det=vn3(q*4.0)*0.5+vn3(q*9.5)*0.28+vn3(q*20.0)*0.14;\n"
        "  return smoothstep(0.42,0.95, base*0.72+det*0.5) * hf;\n"
        "}\n"
        "vec4 cloudVolume(vec3 ro, vec3 rd, vec3 sun, float lift){\n"
        "  if(rd.y < 0.03) return vec4(0.0);\n"
        "  float cb=300.0, ct=470.0; float t0=(cb-ro.y)/rd.y, t1=(ct-ro.y)/rd.y;\n"
        "  float tn=max(min(t0,t1),0.0), tf=max(t0,t1);\n"
        "  if(tf<=tn) return vec4(0.0); tf=min(tf,tn+1500.0);\n"
        "  const int N=14; float dt=(tf-tn)/float(N);\n"
        "  float t=tn+dt*h13(vec3(gl_FragCoord.xy,1.0));\n"
        "  float trans=1.0; vec3 acc=vec3(0.0);\n"
        "  vec3 sunC=mix(vec3(1.0,0.85,0.70), vec3(1.0,0.97,0.92), lift); vec3 ambC=vec3(0.60,0.67,0.80);\n"
        "  for(int i=0;i<N;i++){ vec3 p=ro+rd*t; float d=cloudDens(p);\n"
        "    if(d>0.01){ float ld=cloudDens(p+sun*42.0)+cloudDens(p+sun*95.0)*0.6; float sh=exp(-ld*1.5);\n"
        "      vec3 col=ambC+sunC*sh*1.6; float a=clamp(d*0.9,0.0,1.0);\n"
        "      acc+=trans*a*col; trans*=(1.0-a); if(trans<0.03) break; }\n"
        "    t+=dt; }\n"
        "  return vec4(acc, 1.0-trans);\n"
        "}\n"
        "vec3 skyCol(vec3 d, vec3 sun){\n"
        "  const vec3 ZEN=vec3(0.035,0.22,0.62), MID=vec3(0.22,0.50,0.86), HOR=vec3(0.72,0.86,1.0);\n"
        "  const vec3 HAZE=vec3(1.0,0.78,0.48), GND=vec3(0.30,0.38,0.47);\n"
        "  float h = clamp(d.y*0.5+0.5,0.0,1.0); float t = smoothstep(0.03,0.92,h);\n"
        "  vec3 c = mix(HOR,MID,smoothstep(0.0,0.55,t)); c = mix(c,ZEN,smoothstep(0.34,1.0,t));\n"
        "  float airMass = exp(-max(d.y,0.0)*2.6); c = mix(c, HOR, airMass*0.42);\n"
        "  float hz = exp(-abs(d.y)*4.2); float lift = smoothstep(-0.12,0.55,sun.y);\n"
        "  c += HOR*hz*0.20; c += HAZE*hz*(0.08+0.18*(1.0-lift));\n"
        "  c = mix(GND,c,smoothstep(-0.10,0.04,d.y));\n"
        "  float mu = clamp(dot(d,sun),-1.0,1.0); float fwd = max(mu,0.0);\n"
        "  c *= 0.55+0.45*mu*mu;\n"
        "  vec3 sunTint = mix(vec3(1.0,0.62,0.32),vec3(1.0,0.90,0.70),lift);\n"
        "  c += sunTint*pow(fwd,22.0)*(0.42+0.36*hz);\n"
        "  c += sunTint*pow(fwd,5.0)*(0.10+0.16*(1.0-lift));\n"
        "  c += sunTint*pow(fwd,8.0)*0.10*lift;\n"
        "  float disc = smoothstep(0.9994,0.99986,mu);\n"
        "  c += vec3(1.0,0.95,0.80)*disc*6.0*lift;\n"
        "  vec4 cl = cloudVolume(camPos, d, sun, lift);\n"
        "  c = c*(1.0-cl.a) + cl.rgb;\n"
        "  return c;\n"
        "}\n"

        "const vec3 SUN_RAD = vec3(3.6,3.15,2.55);\n"

        "void main(){\n"
        "  uint px = uint(uv.x*resolution.x), py = uint(uv.y*resolution.y);\n"
        "  g_seed = hashU(px + hashU(py + hashU(uint(frameIdx)*9781U)));\n"

        "  vec2 j = (vec2(rnd(),rnd())-0.5)/resolution;\n"
        "  vec2 ndc = (uv + j)*2.0 - 1.0;\n"
        "  vec3 rd = normalize(camDir + camRight*ndc.x*tanHalfFovY*aspect + camUp*(-ndc.y)*tanHalfFovY);\n"
        "  vec3 ro = camPos; vec3 sun = normalize(sunDir);\n"
        "  vec3 primaryDir = rd;\n"

        "  vec3 col = vec3(0.0); vec3 thr = vec3(1.0);\n"
        "  for(int bounce=0; bounce<4; bounce++){\n"
        "    ivec3 hc; vec3 n; float t;\n"
        "    if(!trace(ro, rd, hc, n, t)){\n"
        "      vec3 sc = skyCol(rd, sun);\n"

        "      if(bounce>0) sc = min(sc, vec3(2.0));\n"
        "      col += thr * sc;\n"
        "      break;\n"
        "    }\n"
        "    vec3 hitP = ro + rd*t + n*0.001;\n"
        "    vec4 vox = voxFetch(hc); vec3 albedo = vox.rgb;\n"
        "    bool water = vox.a < 0.8;\n"

        "    if(water){\n"
        "      vec3 wn = vec3(0.0,1.0,0.0);\n"
        "      float fres = 0.04 + 0.96*pow(1.0 - max(dot(-rd, wn),0.0), 5.0);\n"
        "      fres = clamp(fres, 0.10, 0.98);\n"
        "      vec3 deep = albedo * 1.4;\n"
        "      float ndl = max(dot(wn, sun), 0.0);\n"
        "      if(ndl > 0.0 && !occluded(hitP, sun)) deep += albedo * SUN_RAD * ndl * 0.5;\n"
        "      vec3 spec = reflect(rd, wn);\n"
        "      deep += vec3(1.0,0.95,0.82) * pow(max(dot(spec, sun),0.0), 220.0) * 5.0;\n"
        "      col += thr * deep * (1.0 - fres);\n"
        "      thr *= fres * vec3(0.92,0.96,1.0);\n"
        "      rd = spec; ro = hitP + wn*0.02;\n"
        "      if(max(thr.r,max(thr.g,thr.b)) < 0.02) break;\n"
        "      continue;\n"
        "    }\n"

        "    float ndl = max(dot(n, sun), 0.0);\n"
        "    vec3 direct = vec3(0.0);\n"
        "    if(ndl > 0.0 && !occluded(hitP, sun)) direct = SUN_RAD * ndl;\n"

        "    vec3 amb = mix(vec3(0.16,0.15,0.13), vec3(0.42,0.54,0.74), clamp(n.y*0.5+0.5,0.0,1.0));\n"
        "    vec3 surf = albedo * (direct + amb);\n"

        "    if(bounce==0){\n"

        "      float fog = clamp((t-50.0)/110.0, 0.0, 1.0); fog *= fog;\n"
        "      surf = mix(surf, skyCol(rd, sun), fog);\n"
        "    }\n"
        "    col += thr * surf;\n"

        "    float r1 = 6.2831853*rnd(), r2 = rnd(), r2s = sqrt(r2);\n"
        "    vec3 w = n;\n"
        "    vec3 uax = normalize(cross(abs(w.x)>0.1?vec3(0,1,0):vec3(1,0,0), w));\n"
        "    vec3 vax = cross(w, uax);\n"
        "    rd = normalize(uax*cos(r1)*r2s + vax*sin(r1)*r2s + w*sqrt(1.0-r2));\n"
        "    ro = hitP;\n"
        "    thr *= albedo;\n"
        "    if(max(thr.r,max(thr.g,thr.b)) < 0.02) break;\n"
        "  }\n"

        "  if(any(isnan(col)) || any(isinf(col))) col = skyCol(primaryDir, sun);\n"
        "  col = min(col, vec3(12.0));\n"

        "  vec3 prev = texture(prevAccum, uv).rgb; float n = texture(prevAccum, uv).a;\n"
        "  if(any(isnan(prev))||any(isinf(prev))||isnan(n)||isinf(n)||n<0.0){ prev=vec3(0.0); n=0.0; }\n"
        "  vec3 mean = (prev*n + col)/(n+1.0);\n"
        "  finalColor = vec4(mean, n+1.0);\n"
        "}\n";

    static const char *RS_VS =
        "#version 330\n"
        "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
        "uniform mat4 mvp; out vec2 uv;\n"
        "void main(){ uv = vertexTexCoord; gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
    static const char *RS_FS =
        "#version 330\n"
        "in vec2 uv; out vec4 finalColor; uniform sampler2D texture0;\n"
        "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"
        "void main(){\n"
        "  vec3 hdr = texture(texture0, uv).rgb;\n"
        "  if(any(isnan(hdr))||any(isinf(hdr))){ finalColor=vec4(1.0,0.0,0.0,1.0); return; }\n"
        "  vec3 c = aces(hdr*0.62);\n"
        "  c = pow(c, vec3(1.0/2.2));\n"
        "  float l = dot(c, vec3(0.299,0.587,0.114));\n"
        "  c = mix(vec3(l), c, 1.12);\n"
        "  finalColor = vec4(c, 1.0);\n"
        "}\n";

    static const char *RT_VS = PT_VS;
    static const char *RT_FS =
        "#version 330\n"
        "in vec2 uv; out vec4 finalColor;\n"
        "uniform vec3 camPos; uniform vec3 camDir; uniform vec3 camRight; uniform vec3 camUp;\n"
        "uniform float tanHalfFovY; uniform float aspect;\n"
        "uniform vec3 sunDir;\n"
        "uniform vec3 gridMin; uniform ivec3 gridN; uniform float voxSize;\n"
        "uniform sampler2D texture0; uniform vec2 atlasSize; uniform ivec2 tiles;\n"

        "uniform ivec3 coarseN; uniform ivec2 coarseTiles; uniform int coarseY0; uniform int macroK;\n"

        "uniform mat4 lightVP; uniform sampler2D shadowMap; uniform vec2 shadowTexel;\n"

        "vec4 voxFetch(ivec3 c){\n"
        "  if(c.x<0||c.y<0||c.z<0||c.x>=gridN.x||c.y>=gridN.y||c.z>=gridN.z) return vec4(0.0);\n"
        "  int tx = c.y % tiles.x; int ty = c.y / tiles.x;\n"
        "  vec2 base = vec2(float(tx*gridN.x), float(ty*gridN.z));\n"
        "  vec2 px = base + vec2(float(c.x)+0.5, float(c.z)+0.5);\n"
        "  return texture(texture0, px/atlasSize);\n"
        "}\n"

        "bool macroEmpty(ivec3 c){\n"
        "  ivec3 cc = c / macroK;\n"
        "  if(cc.x<0||cc.y<0||cc.z<0||cc.x>=coarseN.x||cc.y>=coarseN.y||cc.z>=coarseN.z) return false;\n"
        "  int tx = cc.y % coarseTiles.x, ty = cc.y / coarseTiles.x;\n"
        "  vec2 px = vec2(float(tx*coarseN.x + cc.x)+0.5, float(coarseY0 + ty*coarseN.z + cc.z)+0.5);\n"
        "  return texture(texture0, px/atlasSize).a < 0.5;\n"
        "}\n"

        "float shadowMapVis(vec3 wp, vec3 N, vec3 sun){\n"
        "  vec4 lp = lightVP*vec4(wp,1.0);\n"
        "  vec3 p = lp.xyz/lp.w; p = p*0.5+0.5;\n"
        "  if(p.z>1.0) return 1.0;\n"
        "  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"
        "  float NoL = max(dot(N,sun),0.0);\n"
        "  float bias = max(0.0024*(1.0-NoL), 0.0010);\n"
        "  float ang = fract(sin(dot(wp.xz, vec2(12.9898,78.233)))*43758.5453)*6.2831853;\n"
        "  float ca=cos(ang), sa=sin(ang); mat2 rot=mat2(ca,-sa,sa,ca);\n"
        "  vec2 o = shadowTexel*0.9;\n"
        "  vec2 box[4] = vec2[4](vec2(-0.5,-0.5),vec2(0.5,-0.5),vec2(-0.5,0.5),vec2(0.5,0.5));\n"
        "  float s=0.0;\n"
        "  for(int i=0;i<4;i++)\n"
        "    s += (p.z-bias > texture(shadowMap, p.xy + rot*box[i]*o).r)?0.0:1.0;\n"
        "  return s*0.25;\n"
        "}\n"

        "bool trace(vec3 ro, vec3 rd, int maxSteps, float thr, out ivec3 hitC, out vec3 nrm, out float tHit){\n"
        "  vec3 p = (ro - gridMin)/voxSize;\n"
        "  ivec3 c = ivec3(floor(p));\n"
        "  vec3 inv = 1.0/max(abs(rd),vec3(1e-6));\n"
        "  ivec3 stp = ivec3(sign(rd));\n"
        "  vec3 tDelta = inv;\n"
        "  vec3 tMax;\n"
        "  for(int k=0;k<3;k++){\n"
        "    float bound = float(c[k]) + (rd[k]>0.0?1.0:0.0);\n"
        "    tMax[k] = (bound - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "    if(rd[k]==0.0) tMax[k] = 1e9;\n"
        "  }\n"
        "  nrm = vec3(0.0); tHit = 0.0;\n"
        "  float mk = float(macroK);\n"
        "  for(int i=0;i<maxSteps;i++){\n"
        "    if(macroEmpty(c)){\n"

        "      ivec3 mc = c / macroK;\n"
        "      vec3 mt;\n"
        "      for(int k=0;k<3;k++){\n"
        "        if(rd[k]==0.0){ mt[k]=1e9; continue; }\n"
        "        float mbound = float(mc[k]*macroK) + (rd[k]>0.0?mk:0.0);\n"
        "        mt[k] = (mbound - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "      }\n"
        "      float tj = min(mt.x, min(mt.y, mt.z));\n"

        "      if(mt.x<=mt.y && mt.x<=mt.z) nrm=vec3(-float(stp.x),0,0);\n"
        "      else if(mt.y<=mt.z) nrm=vec3(0,-float(stp.y),0);\n"
        "      else nrm=vec3(0,0,-float(stp.z));\n"

        "      c = ivec3(floor(p + rd*(tj + 1e-3)));\n"
        "      tHit = tj;\n"
        "      for(int k=0;k<3;k++){\n"
        "        float bnd = float(c[k]) + (rd[k]>0.0?1.0:0.0);\n"
        "        tMax[k] = (bnd - p[k]) * (rd[k]>0.0?inv[k]:-inv[k]);\n"
        "        if(rd[k]==0.0) tMax[k] = 1e9;\n"
        "      }\n"
        "      if(c.x<-1||c.y<-1||c.z<-1||c.x>gridN.x||c.y>gridN.y||c.z>gridN.z) break;\n"
        "      continue;\n"
        "    }\n"
        "    vec4 vv = voxFetch(c);\n"
        "    if(vv.a > thr && (vv.r+vv.g+vv.b) > 0.004){ hitC = c; tHit *= voxSize; return true; }\n"
        "    if(tMax.x < tMax.y){\n"
        "      if(tMax.x < tMax.z){ c.x+=stp.x; tHit=tMax.x; tMax.x+=tDelta.x; nrm=vec3(-float(stp.x),0,0);}\n"
        "      else               { c.z+=stp.z; tHit=tMax.z; tMax.z+=tDelta.z; nrm=vec3(0,0,-float(stp.z));}\n"
        "    } else {\n"
        "      if(tMax.y < tMax.z){ c.y+=stp.y; tHit=tMax.y; tMax.y+=tDelta.y; nrm=vec3(0,-float(stp.y),0);}\n"
        "      else               { c.z+=stp.z; tHit=tMax.z; tMax.z+=tDelta.z; nrm=vec3(0,0,-float(stp.z));}\n"
        "    }\n"
        "    if(c.x<-1||c.y<-1||c.z<-1||c.x>gridN.x||c.y>gridN.y||c.z>gridN.z) break;\n"
        "  }\n"
        "  return false;\n"
        "}\n"

        "bool occludedOpaque(vec3 ro, vec3 rd){ ivec3 hc; vec3 hn; float ht;\n"
        "  return trace(ro + rd*0.02, rd, 96, 0.5, hc, hn, ht); }\n"

        "float h13(vec3 p){ p=fract(p*0.1031); p+=dot(p,p.yzx+33.33); return fract((p.x+p.y)*p.z); }\n"
        "float vn3(vec3 x){ vec3 i=floor(x), f=fract(x); f=f*f*(3.0-2.0*f);\n"
        "  return mix(mix(mix(h13(i+vec3(0,0,0)),h13(i+vec3(1,0,0)),f.x), mix(h13(i+vec3(0,1,0)),h13(i+vec3(1,1,0)),f.x),f.y),\n"
        "             mix(mix(h13(i+vec3(0,0,1)),h13(i+vec3(1,0,1)),f.x), mix(h13(i+vec3(0,1,1)),h13(i+vec3(1,1,1)),f.x),f.y), f.z); }\n"
        "float cloudDens(vec3 p){\n"
        "  float cb=300.0, ct=470.0; float hf=(p.y-cb)/(ct-cb);\n"
        "  if(hf<0.0||hf>1.0) return 0.0;\n"
        "  hf = smoothstep(0.0,0.32,hf)*smoothstep(1.0,0.5,hf);\n"
        "  vec3 q=p*0.0013;\n"

        "  float base = vn3(q*4.0+3.1)*0.6 + vn3(q*8.0)*0.3 + vn3(q*16.0)*0.15;\n"
        "  return smoothstep(0.62,0.95, base) * hf;\n"
        "}\n"
        "vec4 cloudVolume(vec3 ro, vec3 rd, vec3 sun, float lift){\n"
        "  if(rd.y < 0.03) return vec4(0.0);\n"
        "  float cb=300.0, ct=470.0;\n"
        "  float t0=(cb-ro.y)/rd.y, t1=(ct-ro.y)/rd.y;\n"
        "  float tn=max(min(t0,t1),0.0), tf=max(t0,t1);\n"
        "  if(tf<=tn) return vec4(0.0); tf=min(tf,tn+1500.0);\n"
        "  const int N=14; float dt=(tf-tn)/float(N);\n"
        "  float t=tn+dt*h13(vec3(gl_FragCoord.xy,1.0));\n"
        "  float trans=1.0; vec3 acc=vec3(0.0);\n"
        "  vec3 sunC=mix(vec3(1.0,0.85,0.70), vec3(1.0,0.97,0.92), lift);\n"
        "  vec3 ambC=vec3(0.60,0.67,0.80);\n"
        "  for(int i=0;i<N;i++){\n"
        "    vec3 p=ro+rd*t; float d=cloudDens(p);\n"
        "    if(d>0.01){\n"
        "      float ld=cloudDens(p+sun*42.0)+cloudDens(p+sun*95.0)*0.6;\n"
        "      float sh=exp(-ld*1.5);\n"
        "      vec3 col=ambC + sunC*sh*1.6;\n"
        "      float a=clamp(d*0.9,0.0,1.0);\n"
        "      acc+=trans*a*col; trans*=(1.0-a);\n"
        "      if(trans<0.03) break;\n"
        "    }\n"
        "    t+=dt;\n"
        "  }\n"
        "  return vec4(acc, 1.0-trans);\n"
        "}\n"
        "vec3 skyCol(vec3 d, vec3 sun){\n"
        "  const vec3 ZEN=vec3(0.035,0.22,0.62), MID=vec3(0.22,0.50,0.86), HOR=vec3(0.72,0.86,1.0);\n"
        "  const vec3 HAZE=vec3(1.0,0.78,0.48), GND=vec3(0.30,0.38,0.47);\n"
        "  float h = clamp(d.y*0.5+0.5,0.0,1.0); float t = smoothstep(0.03,0.92,h);\n"
        "  vec3 c = mix(HOR,MID,smoothstep(0.0,0.55,t)); c = mix(c,ZEN,smoothstep(0.34,1.0,t));\n"

        "  float airMass = exp(-max(d.y,0.0)*2.6); c = mix(c, HOR, airMass*0.42);\n"
        "  float hz = exp(-abs(d.y)*4.2); float lift = smoothstep(-0.12,0.55,sun.y);\n"
        "  c += HOR*hz*0.20; c += HAZE*hz*(0.08+0.18*(1.0-lift));\n"
        "  c = mix(GND,c,smoothstep(-0.10,0.04,d.y));\n"
        "  float mu = clamp(dot(d,sun),-1.0,1.0); float fwd = max(mu,0.0);\n"
        "  c *= 0.55+0.45*mu*mu;\n"
        "  vec3 sunTint = mix(vec3(1.0,0.62,0.32),vec3(1.0,0.90,0.70),lift);\n"

        "  c += sunTint*pow(fwd,22.0)*(0.42+0.36*hz);\n"
        "  c += sunTint*pow(fwd,5.0)*(0.10+0.16*(1.0-lift));\n"
        "  c += sunTint*pow(fwd,8.0)*0.10*lift;\n"
        "  float disc = smoothstep(0.9994,0.99986,mu);\n"
        "  c += vec3(1.0,0.95,0.80)*disc*6.0*lift;\n"
        "  vec4 cl = cloudVolume(camPos, d, sun, lift);\n"
        "  c = c*(1.0-cl.a) + cl.rgb;\n"
        "  return c;\n"
        "}\n"
        "const vec3 SUN_RAD = vec3(3.6,3.15,2.55);\n"

        "vec3 shadeHit(vec3 hitP, vec3 n, vec3 albedo, vec3 viewRd, vec3 sun, float t){\n"
        "  float ndl = max(dot(n, sun), 0.0);\n"
        "  vec3 direct = vec3(0.0);\n"

        "  if(ndl > 0.0 && !occludedOpaque(hitP, sun)) direct = SUN_RAD * ndl * shadowMapVis(hitP, n, sun);\n"
        "  vec3 amb = mix(vec3(0.16,0.15,0.13), vec3(0.42,0.54,0.74), clamp(n.y*0.5+0.5,0.0,1.0));\n"

        "  float ao = 1.0;\n"
        "  if(t < 90.0){\n"
        "    vec3 uax = normalize(cross(abs(n.y)>0.9?vec3(1,0,0):vec3(0,1,0), n));\n"
        "    vec3 vax = cross(n, uax);\n"
        "    float occ = 0.0; ivec3 ac; vec3 an; float at;\n"
        "    vec3 dirs[4] = vec3[4]( n, normalize(n+uax*0.9), normalize(n-uax*0.9), normalize(n+vax*0.9) );\n"
        "    for(int k=0;k<4;k++){ if(trace(hitP, dirs[k], 6, 0.5, ac, an, at)) occ += 1.0; }\n"
        "    ao = 1.0 - 0.55*(occ/4.0);\n"
        "  }\n"
        "  vec3 surf = albedo * (direct + amb*ao);\n"

        "  float fog = clamp((t-80.0)/220.0, 0.0, 1.0); fog *= fog; fog = min(fog, 0.88);\n"
        "  surf = mix(surf, skyCol(viewRd, sun), fog);\n"
        "  return surf;\n"
        "}\n"
        "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"

        "void main(){\n"
        "  vec2 ndc = uv*2.0 - 1.0;\n"
        "  vec3 rd = normalize(camDir + camRight*ndc.x*tanHalfFovY*aspect + camUp*(-ndc.y)*tanHalfFovY);\n"
        "  vec3 ro = camPos; vec3 sun = normalize(sunDir);\n"
        "  vec3 col; ivec3 hc; vec3 n; float t;\n"
        "  float fragZ = 1.0;\n"
        "  if(!trace(ro, rd, 512, 0.5, hc, n, t)){\n"
        "    col = skyCol(rd, sun);\n"
        "  } else {\n"

        "    float dview = t * dot(rd, camDir);\n"
        "    const float NR = 0.01, FR = 1000.0;\n"
        "    float ndcz = (FR+NR)/(FR-NR) - (2.0*FR*NR/(FR-NR))/max(dview, NR);\n"
        "    fragZ = clamp(ndcz*0.5 + 0.5, 0.0, 1.0);\n"
        "    vec3 hitP = ro + rd*t + n*0.001;\n"
        "    vec4 vox = voxFetch(hc); vec3 albedo = vox.rgb; bool water = vox.a < 0.8;\n"
        "    if(water){\n"

        "      vec2 wp2 = hitP.xz;\n"
        "      vec3 wn = normalize(vec3(0.07*sin(wp2.x*0.65+wp2.y*0.42)+0.05*sin(wp2.y*1.13),\n"
        "                               1.0,\n"
        "                               0.07*sin(wp2.y*0.65+wp2.x*0.42)+0.05*sin(wp2.x*1.13)));\n"
        "      float fres = clamp(0.08 + 0.92*pow(1.0 - max(dot(-rd, wn),0.0), 5.0), 0.16, 0.98);\n"

        "      vec3 deep;\n"
        "      { ivec3 bc; vec3 bn; float bt;\n"
        "        if(trace(hitP + vec3(0.0,-0.05,0.0), vec3(0.0,-1.0,0.0), 64, 0.7, bc, bn, bt)){\n"
        "          vec4 bv = voxFetch(bc);\n"
        "          vec3 bed = bv.rgb * (0.30 + 0.30*max(dot(bn,sun),0.0));\n"
        "          float murk = clamp(1.0 - exp(-bt*0.42), 0.0, 1.0);\n"
        "          deep = mix(bed, vec3(0.010,0.05,0.09), murk);\n"
        "        } else deep = vec3(0.010,0.05,0.09);\n"
        "      }\n"

        "      vec3 rrd = reflect(rd, wn);\n"
        "      ivec3 rc; vec3 rn; float rt2; vec3 refl;\n"
        "      if(!trace(hitP + vec3(0.0,1.0,0.0)*0.02, rrd, 160, 0.3, rc, rn, rt2)) refl = skyCol(rrd, sun);\n"
        "      else { vec3 rp = hitP + rrd*rt2 + rn*0.001; vec4 rv = voxFetch(rc);\n"
        "             float rndl = max(dot(rn, sun), 0.0);\n"
        "             vec3 ramb = mix(vec3(0.16,0.15,0.13), vec3(0.42,0.54,0.74), clamp(rn.y*0.5+0.5,0.0,1.0));\n"
        "             refl = rv.rgb * (SUN_RAD*rndl*0.5*shadowMapVis(rp, rn, sun) + ramb); }\n"
        "      refl += vec3(1.0,0.95,0.82) * pow(max(dot(rrd, sun),0.0), 220.0) * 6.0;\n"
        "      col = mix(deep, refl, fres);\n"
        "    } else {\n"
        "      col = shadeHit(hitP, n, albedo, rd, sun, t);\n"
        "    }\n"
        "  }\n"
        "  vec3 c = aces(col*0.52);\n"
        "  c = pow(c, vec3(1.0/2.2));\n"
        "  float l = dot(c, vec3(0.299,0.587,0.114));\n"
        "  c = mix(vec3(l), c, 1.08);\n"
        "  finalColor = vec4(c, 1.0);\n"
        "  gl_FragDepth = fragZ;\n"
        "}\n";

    static const char *BLIT_FS =
        "#version 330\n"
        "in vec2 uv; out vec4 finalColor;\n"
        "uniform sampler2D texture0;\n"
        "uniform sampler2D depthTex;\n"
        "uniform vec2 invRes;\n"
        "float luma(vec3 c){ return dot(c, vec3(0.299,0.587,0.114)); }\n"
        "void main(){\n"
        "  vec3 m  = texture(texture0, uv).rgb;\n"
        "  vec3 nw = texture(texture0, uv + vec2(-1.0,-1.0)*invRes).rgb;\n"
        "  vec3 ne = texture(texture0, uv + vec2( 1.0,-1.0)*invRes).rgb;\n"
        "  vec3 sw = texture(texture0, uv + vec2(-1.0, 1.0)*invRes).rgb;\n"
        "  vec3 se = texture(texture0, uv + vec2( 1.0, 1.0)*invRes).rgb;\n"
        "  float lm=luma(m), lnw=luma(nw), lne=luma(ne), lsw=luma(sw), lse=luma(se);\n"
        "  float lmin = min(lm, min(min(lnw,lne), min(lsw,lse)));\n"
        "  float lmax = max(lm, max(max(lnw,lne), max(lsw,lse)));\n"
        "  vec2 dir = vec2(-((lnw+lne)-(lsw+lse)), ((lnw+lsw)-(lne+lse)));\n"
        "  float red = max((lnw+lne+lsw+lse)*0.25*0.2, 1.0/128.0);\n"
        "  float rcp = 1.0/(min(abs(dir.x),abs(dir.y)) + red);\n"
        "  dir = clamp(dir*rcp, -8.0, 8.0) * invRes;\n"
        "  vec3 rgbA = 0.5*(texture(texture0, uv + dir*(1.0/3.0-0.5)).rgb +\n"
        "                   texture(texture0, uv + dir*(2.0/3.0-0.5)).rgb);\n"
        "  vec3 rgbB = rgbA*0.5 + 0.25*(texture(texture0, uv + dir*-0.5).rgb +\n"
        "                               texture(texture0, uv + dir* 0.5).rgb);\n"
        "  float lB = luma(rgbB);\n"
        "  finalColor = vec4((lB<lmin||lB>lmax) ? rgbA : rgbB, 1.0);\n"
        "  gl_FragDepth = texture(depthTex, uv).r;\n"
        "}\n";

    trace   = LoadShaderFromMemory(PT_VS, PT_FS);
    resolve = LoadShaderFromMemory(RS_VS, RS_FS);
    rt      = LoadShaderFromMemory(RT_VS, RT_FS);
    rtBlit  = LoadShaderFromMemory(RS_VS, BLIT_FS);

    locCamPos   = GetShaderLocation(trace, "camPos");
    locCamDir   = GetShaderLocation(trace, "camDir");
    locCamRight = GetShaderLocation(trace, "camRight");
    locCamUp    = GetShaderLocation(trace, "camUp");
    locTan      = GetShaderLocation(trace, "tanHalfFovY");
    locAspect   = GetShaderLocation(trace, "aspect");
    locSunDir   = GetShaderLocation(trace, "sunDir");
    locRes      = GetShaderLocation(trace, "resolution");
    locFrame    = GetShaderLocation(trace, "frameIdx");
    locGridMin  = GetShaderLocation(trace, "gridMin");
    locGridN    = GetShaderLocation(trace, "gridN");
    locVoxSize  = GetShaderLocation(trace, "voxSize");
    locAtlasSize= GetShaderLocation(trace, "atlasSize");
    locTiles    = GetShaderLocation(trace, "tiles");
    locPrev     = GetShaderLocation(trace, "prevAccum");

    rCamPos   = GetShaderLocation(rt, "camPos");
    rCamDir   = GetShaderLocation(rt, "camDir");
    rCamRight = GetShaderLocation(rt, "camRight");
    rCamUp    = GetShaderLocation(rt, "camUp");
    rTan      = GetShaderLocation(rt, "tanHalfFovY");
    rAspect   = GetShaderLocation(rt, "aspect");
    rSunDir   = GetShaderLocation(rt, "sunDir");
    rGridMin  = GetShaderLocation(rt, "gridMin");
    rGridN    = GetShaderLocation(rt, "gridN");
    rVoxSize  = GetShaderLocation(rt, "voxSize");
    rAtlasSize= GetShaderLocation(rt, "atlasSize");
    rTiles    = GetShaderLocation(rt, "tiles");
    rLightVP    = GetShaderLocation(rt, "lightVP");
    rShadowMap  = GetShaderLocation(rt, "shadowMap");
    rShadowTexel= GetShaderLocation(rt, "shadowTexel");
    printf("RT LOCS: lightVP=%d shadowMap=%d shadowTexel=%d  (rt.id=%u)\n",
           rLightVP, rShadowMap, rShadowTexel, rt.id); fflush(stdout);

    bDepthTex = GetShaderLocation(rtBlit, "depthTex");
    bInvRes   = GetShaderLocation(rtBlit, "invRes");
}

void PTSys::initBuffers(int w, int h) {
    if (accum.id) UnloadRenderTexture(accum);
    if (ping.id) UnloadRenderTexture(ping);
    accum = {};
    ping = {};
    W = w; H = h;

    auto makeHDR = [&](RenderTexture2D &rt){
        rt.id = rlLoadFramebuffer();
        rlEnableFramebuffer(rt.id);
        rt.texture.id = rlLoadTexture(NULL, w, h, RL_PIXELFORMAT_UNCOMPRESSED_R32G32B32A32, 1);
        rt.texture.width = w; rt.texture.height = h; rt.texture.mipmaps = 1;
        rt.texture.format = PIXELFORMAT_UNCOMPRESSED_R32G32B32A32;
        rlFramebufferAttach(rt.id, rt.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
        rt.depth.id = 0;
        if(!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "PT: HDR framebuffer incomplete");
        rlDisableFramebuffer();
    };
    makeHDR(accum);
    makeHDR(ping);
    allocAtlas();
    setCoarseUniforms();
}

void PTSys::allocAtlas() {
    if (vox.id != 0) return;
    PT_TILES_X = (int)ceilf(sqrtf((float)PT_NY * (float)PT_NZ / (float)PT_NX));
    if (PT_TILES_X < 1) PT_TILES_X = 1;
    PT_TILES_Y = (PT_NY + PT_TILES_X - 1) / PT_TILES_X;
    PT_ATLAS_W = PT_TILES_X * PT_NX;
    int fineH = PT_TILES_Y * PT_NZ;

    PT_CNX = (PT_NX + PT_MK - 1) / PT_MK;
    PT_CNY = (PT_NY + PT_MK - 1) / PT_MK;
    PT_CNZ = (PT_NZ + PT_MK - 1) / PT_MK;
    PT_CTX = (int)ceilf(sqrtf((float)PT_CNY * (float)PT_CNZ / (float)PT_CNX));
    if (PT_CTX < 1) PT_CTX = 1;
    PT_CTY = (PT_CNY + PT_CTX - 1) / PT_CTX;
    int coarseW = PT_CTX * PT_CNX, coarseH = PT_CTY * PT_CNZ;
    if (coarseW > PT_ATLAS_W) PT_ATLAS_W = coarseW;
    PT_COARSE_Y0 = fineH;
    PT_ATLAS_H = fineH + coarseH;

    vox.id = rlLoadTexture(NULL, PT_ATLAS_W, PT_ATLAS_H,
                           RL_PIXELFORMAT_UNCOMPRESSED_R32G32B32A32, 1);
    vox.width = PT_ATLAS_W; vox.height = PT_ATLAS_H; vox.mipmaps = 1;
    vox.format = PIXELFORMAT_UNCOMPRESSED_R32G32B32A32;
    SetTextureFilter(vox, TEXTURE_FILTER_POINT);
}

void PTSys::setCoarseUniforms() {
    int cdims[3] = { PT_CNX, PT_CNY, PT_CNZ };
    int ctiles[2] = { PT_CTX, PT_CTY };
    int cy0 = PT_COARSE_Y0;
    int mk = PT_MK;
    struct { Shader sh; } shs[2] = { { rt }, { trace } };
    for (auto &e : shs) {
        if (e.sh.id == 0) continue;
        int lD = GetShaderLocation(e.sh, "coarseN");
        int lT = GetShaderLocation(e.sh, "coarseTiles");
        int lY = GetShaderLocation(e.sh, "coarseY0");
        int lM = GetShaderLocation(e.sh, "macroK");
        if (lD>=0) SetShaderValue(e.sh, lD, cdims, SHADER_UNIFORM_IVEC3);
        if (lT>=0) SetShaderValue(e.sh, lT, ctiles, SHADER_UNIFORM_IVEC2);
        if (lY>=0) SetShaderValue(e.sh, lY, &cy0, SHADER_UNIFORM_INT);
        if (lM>=0) SetShaderValue(e.sh, lM, &mk, SHADER_UNIFORM_INT);
    }
}

static int PT_LIVE_DIV = 1;

void PTSys::releaseLiveTarget() {
    if (rtBuf.id) UnloadRenderTexture(rtBuf);
    rtBuf = {};
    rtW = rtH = 0;
}

void PTSys::initLive(int rw, int rh) {
    releaseLiveTarget();
    allocAtlas();
    rtW = rw / PT_LIVE_DIV; rtH = rh / PT_LIVE_DIV;
    if (rtW < 1) rtW = 1; if (rtH < 1) rtH = 1;

    rtBuf = RenderTexture2D{};
    rtBuf.id = rlLoadFramebuffer();
    rlEnableFramebuffer(rtBuf.id);
    rtBuf.texture.id = rlLoadTexture(NULL, rtW, rtH, RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 1);
    rtBuf.texture.width = rtW; rtBuf.texture.height = rtH; rtBuf.texture.mipmaps = 1;
    rtBuf.texture.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    rtBuf.depth.id = rlLoadTextureDepth(rtW, rtH, false);
    rtBuf.depth.width = rtW; rtBuf.depth.height = rtH;
    rlFramebufferAttach(rtBuf.id, rtBuf.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    rlFramebufferAttach(rtBuf.id, rtBuf.depth.id,   RL_ATTACHMENT_DEPTH,          RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(rtBuf.id)) TraceLog(LOG_WARNING, "RT: live framebuffer incomplete");
    rlDisableFramebuffer();
    SetTextureFilter(rtBuf.texture, TEXTURE_FILTER_BILINEAR);
    setCoarseUniforms();
}

void PTSys::shutdown() {
    if (accum.id) UnloadRenderTexture(accum);
    if (ping.id) UnloadRenderTexture(ping);
    releaseLiveTarget();
    if (vox.id) UnloadTexture(vox);
    if (trace.id) UnloadShader(trace);
    if (resolve.id) UnloadShader(resolve);
    if (rt.id) UnloadShader(rt);
    if (rtBlit.id) UnloadShader(rtBlit);
    accum = {}; ping = {}; rtBuf = {}; vox = {};
    trace = {}; resolve = {}; rt = {}; rtBlit = {};
    W = H = 0;
}

static Vector3 g_ptGridMin;

static void pt_biomeColor(float wx, float wz, int h, Color &capC, Color &bodyC) {
    bool beach = (h + 1.0f) <= WATER_Y + 0.6f;
    float bio   = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
    float humid = fbm(wx * 0.0028f + 44.0f, wz * 0.0028f + 108.0f, 2);
    float temp  = fbm(wx * 0.0019f + 12.0f, wz * 0.0019f + 204.0f, 2);
    capC = GRASS; bodyC = DIRT;
    if (h >= 260)      { capC = {204,214,224,255}; bodyC = {132,140,154,255}; }
    else if (h >= 158) { capC = {128,138,146,255}; bodyC = {108,116,126,255}; }
    else if (beach)    { capC = SAND; }
    else if (humid < 0.23f && temp > 0.42f) { capC = {214,196,108,255}; bodyC = {162,126,72,255}; }
    else if (humid > 0.72f && bio < 0.72f)  { capC = { 76,176, 92,255}; bodyC = {118, 96, 72,255}; }
    else if (bio < 0.34f) { }
    else if (bio < 0.58f) { capC = {118,206,108,255}; }
    else if (bio < 0.78f) { capC = {210,202,132,255}; }
    else { capC = {112,150,112,255}; bodyC = {118,104,86,255}; }
    if (capC.r == GRASS.r && capC.g == GRASS.g && capC.b == GRASS.b) {
        float patch = vnoise(wx * 0.03f + 7.7f, wz * 0.03f + 4.2f);
        Color lush = {96,188,96,255}, dry = {196,206,120,255};
        capC = mixc(capC, mixc(lush, dry, patch), 0.35f);
    }

    int cx = (int)floorf(wx / CELL), cz = (int)floorf(wz / CELL);
    float s = 0.89f + 0.13f * hashf(cx * 5 + 1, cz * 5 + 2);
    capC = shade(capC, s); bodyC = shade(bodyC, s * 0.95f);
}

static void bakeVoxelsCPU(Vector3 camCtr, const Track &trk, float u,
                          std::vector<float> &buf, Vector3 &outGridMin) {

    const int finalN = trk.finalizedPointCount();
    const float finalMaxU = trk.maxFinalU();

    int baseY = (int)floorf(WATER_Y - 8.0f);
    outGridMin = Vector3{
        floorf(camCtr.x / PT_VOX) * PT_VOX - (PT_NX / 2) * PT_VOX,
        (float)baseY,
        floorf(camCtr.z / PT_VOX) * PT_VOX - (PT_NZ / 2) * PT_VOX
    };
    Vector3 &g_ptGridMin = outGridMin;

    buf.assign((size_t)PT_ATLAS_W * PT_ATLAS_H * 4, 0.0f);

    auto putMat = [&](int gx, int gy, int gz, float r, float g, float b, float mat) {
        if (gx < 0 || gy < 0 || gz < 0 || gx >= PT_NX || gy >= PT_NY || gz >= PT_NZ) return;
        int tx = gy % PT_TILES_X, ty = gy / PT_TILES_X;
        int ax = tx * PT_NX + gx, az = ty * PT_NZ + gz;
        size_t idx = ((size_t)az * PT_ATLAS_W + ax) * 4;
        buf[idx+0] = pt_srgb2lin(r); buf[idx+1] = pt_srgb2lin(g);
        buf[idx+2] = pt_srgb2lin(b); buf[idx+3] = mat;
    };
    auto put = [&](int gx, int gy, int gz, float r, float g, float b) {
        putMat(gx, gy, gz, r, g, b, 1.0f);
    };
    auto putWorld = [&](float wx, float wy, float wz, Color c) {
        int gx = (int)floorf((wx - g_ptGridMin.x) / PT_VOX);
        int gy = (int)floorf((wy - g_ptGridMin.y) / PT_VOX);
        int gz = (int)floorf((wz - g_ptGridMin.z) / PT_VOX);
        put(gx, gy, gz, c.r/255.0f, c.g/255.0f, c.b/255.0f);
    };
    auto putWorldMat = [&](Vector3 p, Color c, float mat) {
        int gx = (int)floorf((p.x - g_ptGridMin.x) / PT_VOX);
        int gy = (int)floorf((p.y - g_ptGridMin.y) / PT_VOX);
        int gz = (int)floorf((p.z - g_ptGridMin.z) / PT_VOX);
        putMat(gx, gy, gz, c.r/255.0f, c.g/255.0f, c.b/255.0f, mat);
    };

    std::vector<float> cvLo((size_t)PT_NX * PT_NZ, 1e9f), cvHi((size_t)PT_NX * PT_NZ, -1e9f),
                       cvDeep((size_t)PT_NX * PT_NZ, 1e9f);
    {
        const float BORE = 4.5f;
        const float DEEPR = BORE + 6.0f;
        int brad = (int)ceilf(DEEPR / PT_VOX) + 1;
        for (float su = fmaxf(u - 8.0f, 0.0f); su <= fminf(u + 16.0f, finalMaxU); su += 0.15f) {
            Vector3 ps = trk.pos(su);
            float lo = ps.y - 4.0f, hi = ps.y + 4.5f;
            int sgx = (int)floorf((ps.x - g_ptGridMin.x) / PT_VOX);
            int sgz = (int)floorf((ps.z - g_ptGridMin.z) / PT_VOX);
            for (int oz = -brad; oz <= brad; oz++)
            for (int ox = -brad; ox <= brad; ox++) {
                int gx = sgx + ox, gz = sgz + oz;
                if (gx < 0 || gz < 0 || gx >= PT_NX || gz >= PT_NZ) continue;
                float cwx = g_ptGridMin.x + (gx + 0.5f) * PT_VOX;
                float cwz = g_ptGridMin.z + (gz + 0.5f) * PT_VOX;
                float ex = cwx - ps.x, ez = cwz - ps.z;
                float d2 = ex * ex + ez * ez;
                if (d2 > DEEPR * DEEPR) continue;
                if (lo >= (float)terrainH(cwx, cwz) + 1.0f) continue;
                size_t ci = (size_t)gz * PT_NX + gx;
                float deepTo = lo - 8.0f;
                if (deepTo < cvDeep[ci]) cvDeep[ci] = deepTo;
                if (d2 > BORE * BORE) continue;
                if (lo < cvLo[ci]) cvLo[ci] = lo;
                if (hi > cvHi[ci]) cvHi[ci] = hi;
            }
        }
    }

    for (int gz = 0; gz < PT_NZ; gz++) {
        for (int gx = 0; gx < PT_NX; gx++) {
            float wx = g_ptGridMin.x + (gx + 0.5f) * PT_VOX;
            float wz = g_ptGridMin.z + (gz + 0.5f) * PT_VOX;
            int h = terrainH(wx, wz);
            Color cap, body; pt_biomeColor(wx, wz, h, cap, body);
            float top = (float)h + 1.0f;
            float colDepth = 42.0f;
            size_t ci = (size_t)gz * PT_NX + gx;
            float colBot = (float)h - colDepth;
            if (cvDeep[ci] < colBot) colBot = cvDeep[ci];
            int topG = (int)floorf((h + 0.5f - g_ptGridMin.y) / PT_VOX);
            int botG = (int)floorf((colBot - g_ptGridMin.y) / PT_VOX); if (botG < 0) botG = 0;
            float cLo = cvLo[ci], cHi = cvHi[ci]; bool carved = cHi > cLo;
            auto putBodySpan = [&](float y0, float y1) {
                if (y1 <= y0) return;
                for (int gy = botG; gy < topG; gy++) {
                    float wy = g_ptGridMin.y + (gy + 0.5f) * PT_VOX;
                    if (wy >= y0 && wy < y1) put(gx, gy, gz, body.r/255.0f, body.g/255.0f, body.b/255.0f);
                }
            };
            if (carved && cHi > colBot && cLo < top) {
                float loTop = fminf(cLo, top);
                if (loTop > colBot + 0.1f) putBodySpan(colBot, loTop);
                float roofBot = fmaxf(cHi, colBot);
                if (roofBot < top - 0.4f) {
                    if (roofBot < (float)h - 0.1f) putBodySpan(roofBot, (float)h);
                    if (topG >= 0 && topG < PT_NY) put(gx, topG, gz, cap.r/255.0f, cap.g/255.0f, cap.b/255.0f);
                }
            } else {
                putBodySpan(colBot, (float)h);
                if (topG >= 0 && topG < PT_NY) put(gx, topG, gz, cap.r/255.0f, cap.g/255.0f, cap.b/255.0f);
            }

            if (h + 1.0f < WATER_Y) {
                int wG = (int)floorf((WATER_Y - g_ptGridMin.y) / PT_VOX);
                if (wG >= 0 && wG < PT_NY) putMat(gx, wG, gz, 0.10f, 0.20f, 0.26f, 0.6f);
            }
        }
    }

    int ccx = (int)floorf(camCtr.x / CELL), ccz = (int)floorf(camCtr.z / CELL);
    auto treeOverlapsTrack = [&](float wx, float wz, float y0, float y1, float radius) {
        float r2 = radius * radius;
        for (float su = fmaxf(u - 8.0f, 0.0f); su <= fminf(u + 16.0f, finalMaxU); su += 0.18f) {
            Vector3 ps = trk.pos(su);
            float dx = ps.x - wx, dz = ps.z - wz;
            if (dx * dx + dz * dz > r2) continue;
            float lo = ps.y - 5.0f, hi = ps.y + 6.0f;
            if (hi >= y0 && lo <= y1) return true;
        }
        return false;
    };

    auto treeInPlatform = [&](float wx, float wz, Vector3 pos, float yaw) {
        float c = cosf(yaw), s = sinf(yaw);
        float dx = wx - pos.x, dz = wz - pos.z;
        float lx = dx * c - dz * s, lz = dx * s + dz * c;
        return (fabsf(lx) < 9.0f && lz > -30.0f && lz < 74.0f);
    };
    for (int dz = -TERRA_R; dz <= TERRA_R; dz++)
    for (int dx = -TERRA_R; dx <= TERRA_R; dx++) {
        int cx = ccx + dx, cz = ccz + dz;
        float wx = cx * CELL + CELL * 0.5f, wz = cz * CELL + CELL * 0.5f;
        int h = terrainH(wx, wz); float top = h + 1.0f;
        if (top <= WATER_Y + 0.6f) continue;
        float bio = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
        float humid = fbm(wx * 0.0028f + 44.0f, wz * 0.0028f + 108.0f, 2);
        float temp  = fbm(wx * 0.0019f + 12.0f, wz * 0.0019f + 204.0f, 2);
        int treeType = -1; float treeDen = 0;
        if      (h >= 158) {}
        else if (humid < 0.23f && temp > 0.42f) { treeType = 3; treeDen = 0.003f; }
        else if (humid > 0.72f && bio < 0.72f)  { treeType = 0; treeDen = 0.065f; }
        else if (bio < 0.34f) { treeType = 0; treeDen = 0.012f; }
        else if (bio < 0.58f) { treeType = 1; treeDen = 0.045f; }
        else if (bio < 0.78f) { treeType = 3; treeDen = 0.006f; }
        else { treeType = 2; treeDen = 0.018f; }
        if (treeType < 0) continue;
        if (hashf(cx * 9 + 7, cz * 9 + 3) >= treeDen) continue;

        bool clear = true;
        for (int q = (int)fmaxf(u - 2, 0); q <= (int)(u + 12) && q < finalN; q++) {
            float tx = trk.cp[q].x - wx, tz = trk.cp[q].z - wz;
            if (tx*tx + tz*tz < 56) { clear = false; break; }
        }
        if (!clear) continue;
        if (treeType == 1 && hashf(cx*9+7,cz*9+3) > treeDen*0.5f) treeType = 0;
        if (treeOverlapsTrack(wx, wz, top, top + 6.0f, 8.0f)) continue;

        if (treeInPlatform(wx, wz, trk.startPos, trk.startYaw)) continue;
        if (trk.stationActive && treeInPlatform(wx, wz, trk.stationPos, trk.stationYaw)) continue;
        Color trunk, leaf;
        switch (treeType) {
            case 0: trunk = WOOD; leaf = LEAF; break;
            case 1: trunk = {214,209,194,255}; leaf = {112,162,81,255}; break;
            case 2: trunk = {82,60,40,255}; leaf = {65,101,65,255}; break;
            default:trunk = {106,82,53,255}; leaf = {131,144,65,255}; break;
        }

        for (float ty = top; ty < top + 3.0f; ty += PT_VOX*0.6f) putWorld(wx, ty, wz, trunk);

        for (int ly = 0; ly <= 3; ly++)
        for (int lx = -2; lx <= 2; lx++)
        for (int lz = -2; lz <= 2; lz++) {
            float rad = 2.4f - ly*0.5f;
            if (lx*lx + lz*lz > rad*rad) continue;
            putWorld(wx + lx*PT_VOX*0.9f, top + 3.0f + ly*PT_VOX*0.9f, wz + lz*PT_VOX*0.9f, leaf);
        }
    }

    const float PROXY_MAT = 0.35f;
    auto stampBox = [&](Vector3 c, Vector3 ax, Vector3 ay, Vector3 az,
                        float sx, float sy, float sz, Color col) {
        int nx = (int)ceilf(sx / PT_VOX); if (nx < 1) nx = 1;
        int ny = (int)ceilf(sy / PT_VOX); if (ny < 1) ny = 1;
        int nz = (int)ceilf(sz / PT_VOX); if (nz < 1) nz = 1;
        for (int iz = 0; iz < nz; iz++)
        for (int iy = 0; iy < ny; iy++)
        for (int ix = 0; ix < nx; ix++) {
            float lx = (((float)ix + 0.5f) / (float)nx - 0.5f) * sx;
            float ly = (((float)iy + 0.5f) / (float)ny - 0.5f) * sy;
            float lz = (((float)iz + 0.5f) / (float)nz - 0.5f) * sz;
            Vector3 p = Vector3Add(c, Vector3Add(Vector3Scale(ax, lx),
                        Vector3Add(Vector3Scale(ay, ly), Vector3Scale(az, lz))));
            putWorldMat(p, col, PROXY_MAT);
        }
    };
    auto frameAt = [&](float uu, Vector3 &p, Vector3 &f, Vector3 &up, Vector3 &right) {
        p = trk.pos(uu);
        f = trk.tangent(uu);
        up = orthoUp(f, trk.upAt(uu));
        right = Vector3CrossProduct(up, f);
        float rl = Vector3Length(right);
        right = (rl < 1e-3f) ? Vector3{1,0,0} : Vector3Scale(right, 1.0f / rl);
    };

    int sk = (int)fmaxf(u - 8.0f, 0.0f);
    int ek = (int)fminf(u + 14.0f, finalMaxU);
    for (int k = sk; k <= ek; k++) {
        float segLen = fmaxf(trk.speedScale(k + 0.5f), 0.01f);
        int nSmp = (int)ceilf(segLen / 1.2f);
        if (nSmp < 1) nSmp = 1; else if (nSmp > 64) nSmp = 64;
        int ki = k < finalN ? k : finalN - 1;
        bool chain = ki >= 0 && trk.chainf[ki] != 0;
        unsigned char segTag = (ki >= 0 && ki < (int)trk.kind.size()) ? trk.kind[ki] : 0;
        bool poweredSpine = (segTag == M_LAUNCH || segTag == M_BOOST);
        for (int j = 0; j < nSmp; j++) {
            float uu = k + (j + 0.5f) / (float)nSmp;
            Vector3 p, f, up, right; frameAt(uu, p, f, up, right);
            float rl = segLen / nSmp + 0.22f;
            if (poweredSpine)
                stampBox(Vector3Add(p, Vector3Scale(up, -0.30f)), right, up, f,
                         0.42f, 0.58f, rl, trk.spineC);
            stampBox(Vector3Add(p, Vector3Scale(right, -0.55f)), right, up, f, 0.24f, 0.24f, rl, trk.railC);
            stampBox(Vector3Add(p, Vector3Scale(right,  0.55f)), right, up, f, 0.24f, 0.24f, rl, trk.railC);
            if ((j & 1) == 0)
                stampBox(Vector3Add(p, Vector3Scale(up, -0.13f)), right, up, f,
                         1.36f, 0.18f, 0.46f, Color{ 96, 99, 108, 255 });
            if (chain)
                stampBox(Vector3Add(p, Vector3Scale(up, -0.05f)), right, up, f,
                         0.18f, 0.18f, rl, CHAINC);
        }
    }

    int i0 = (int)fmaxf(1.0f, u - 8.0f), i1 = (int)(u + 14.0f);
    for (int i = i0; i <= i1 && i + 1 < finalN; i++) {
        Vector3 p = trk.cp[i];
        unsigned char tg = trk.kind[i];
        if ((tg == M_LOOP || tg == M_ROLL || tg == M_IMMEL) && trk.up[i].y < 0.35f) continue;
        float g = groundTopAt(p.x, p.z);
        if (p.y - g < 1.5f) continue;
        Vector3 t = Vector3Normalize(Vector3Subtract(trk.cp[i + 1], trk.cp[i - 1]));
        Vector3 lat = Vector3CrossProduct(Vector3{ t.x, 0, t.z }, Vector3{ 0, 1, 0 });
        float ll = Vector3Length(lat);
        lat = (ll < 1e-3f) ? Vector3{1,0,0} : Vector3Scale(lat, 1.0f / ll);
        Color steel = Color{ 118, 122, 130, 255 };
        float bankTilt = 1.0f - Clamp(trk.up[i].y, 0.0f, 1.0f);
        if (bankTilt > 0.12f) {
            float hgt = (p.y - 1.0f) - g;
            if (hgt > 0.5f)
                stampBox(Vector3{ p.x, g + hgt * 0.5f, p.z },
                         Vector3{1,0,0}, Vector3{0,1,0}, Vector3{0,0,1},
                         0.38f, hgt, 0.38f, steel);
        } else {
            for (float s : { -0.8f, 0.8f }) {
                float px = p.x + lat.x * s, pz = p.z + lat.z * s;
                float hgt = p.y - 0.4f - g;
                if (hgt > 0.5f)
                    stampBox(Vector3{ px, g + hgt * 0.5f, pz },
                             Vector3{1,0,0}, Vector3{0,1,0}, Vector3{0,0,1},
                             0.34f, hgt, 0.34f, steel);
            }
        }
    }

    auto backUProxy = [&](float from, float distAB) {
        float uu = from, rem = distAB;
        for (int it = 0; it < 2048 && rem > 1e-2f && uu > 0.06f; it++) {
            float ss = fmaxf(trk.speedScale(uu), 0.5f);
            float du = fminf(0.06f, rem / ss);
            if (du < 1e-4f) break;
            uu -= du; rem -= du * ss;
        }
        return uu < 0.06f ? 0.06f : uu;
    };
    for (int i = 0; i < 2; i++) {
        float ui = (i == 0) ? u : backUProxy(u, i * 4.2f);
        Vector3 p, f, up, right; frameAt(ui, p, f, up, right);
        stampBox(Vector3Add(p, Vector3Scale(up, 0.55f)), right, up, f,
                 2.0f, 1.2f, 3.4f, trk.trainBody);
        stampBox(Vector3Add(p, Vector3Scale(up, 1.25f)), right, up, f,
                 1.5f, 0.7f, 2.0f, Color{ 32, 34, 40, 255 });
    }

    auto fineIdx = [&](int gx, int gy, int gz) -> size_t {
        int tx = gy % PT_TILES_X, ty = gy / PT_TILES_X;
        int ax = tx * PT_NX + gx, az = ty * PT_NZ + gz;
        return ((size_t)az * PT_ATLAS_W + ax) * 4;
    };
    for (int cy = 0; cy < PT_CNY; cy++)
    for (int cz = 0; cz < PT_CNZ; cz++)
    for (int cx = 0; cx < PT_CNX; cx++) {
        bool occ = false;
        int gy0 = cy * PT_MK, gz0 = cz * PT_MK, gx0 = cx * PT_MK;
        for (int dy = 0; dy < PT_MK && !occ; dy++) {
            int gy = gy0 + dy; if (gy >= PT_NY) break;
            for (int dz = 0; dz < PT_MK && !occ; dz++) {
                int gz = gz0 + dz; if (gz >= PT_NZ) break;
                for (int dx = 0; dx < PT_MK; dx++) {
                    int gx = gx0 + dx; if (gx >= PT_NX) break;
                    size_t fi = fineIdx(gx, gy, gz);
                    if (buf[fi+3] > 0.5f && (buf[fi+0]+buf[fi+1]+buf[fi+2]) > 0.004f) { occ = true; break; }
                }
            }
        }
        int ctx = cy % PT_CTX, cty = cy / PT_CTX;
        int ax = ctx * PT_CNX + cx, az = PT_COARSE_Y0 + cty * PT_CNZ + cz;
        size_t ci = ((size_t)az * PT_ATLAS_W + ax) * 4;
        buf[ci+0] = buf[ci+1] = buf[ci+2] = 0.0f;
        buf[ci+3] = occ ? 1.0f : 0.0f;
    }
}

static inline void uploadVoxels(const std::vector<float> &buf) {
    rlUpdateTexture(gPT.vox.id, 0, 0, PT_ATLAS_W, PT_ATLAS_H,
                    RL_PIXELFORMAT_UNCOMPRESSED_R32G32B32A32, buf.data());
}

static void bakeVoxels(Vector3 camCtr, const Track &trk, float u,
                       std::vector<float> &buf) {
    bakeVoxelsCPU(camCtr, trk, u, buf, g_ptGridMin);
    uploadVoxels(buf);
}

struct AsyncBaker {
    std::thread       worker;
    std::mutex        m;
    std::condition_variable cv;
    bool   running   = false;
    bool   stop      = false;
    bool   hasJob    = false;
    bool   ready     = false;
    Vector3 jobCtr{}; Track jobTrk; float jobU = 0;
    std::vector<float> back; Vector3 backGridMin{};

    void start() {
        running = true;
        worker = std::thread([this]{
            std::vector<float> local;
            for (;;) {
                Vector3 ctr; Track trk; float u;
                {
                    std::unique_lock<std::mutex> lk(m);
                    cv.wait(lk, [this]{ return hasJob || stop; });
                    if (stop) return;
                    ctr = jobCtr; trk = jobTrk; u = jobU;
                }
                Vector3 gm; bakeVoxelsCPU(ctr, trk, u, local, gm);
                {
                    std::unique_lock<std::mutex> lk(m);
                    back.swap(local); backGridMin = gm;
                    ready = true; hasJob = false;
                }
            }
        });
    }

    bool request(Vector3 ctr, const Track &trk, float u) {
        std::unique_lock<std::mutex> lk(m);
        if (hasJob) return false;
        jobCtr = ctr; jobTrk = trk; jobU = u; hasJob = true;
        cv.notify_one();
        return true;
    }

    bool consume(std::vector<float> &outBuf, Vector3 &outGridMin) {
        std::unique_lock<std::mutex> lk(m);
        if (!ready) return false;
        outBuf.swap(back); outGridMin = backGridMin; ready = false;
        return true;
    }
    void shutdown() {
        if (!running) return;
        { std::unique_lock<std::mutex> lk(m); stop = true; cv.notify_one(); }
        worker.join(); running = false;
    }
    ~AsyncBaker() { shutdown(); }
};
static AsyncBaker gBaker;

#endif

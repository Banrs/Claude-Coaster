#ifndef MINECOASTER_RENDER_FX_CPP
#define MINECOASTER_RENDER_FX_CPP

static const char *SHADOW_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "out vec2 fragTexCoord; out vec4 fragColor; out vec3 fragNormal; out vec3 fragWorld;\n"
    "void main(){\n"
    "  vec4 wp = vec4(vertexPosition,1.0);\n"
    "  fragWorld = wp.xyz;\n"
    "  fragTexCoord = vertexTexCoord; fragColor = vertexColor;\n"
    "  fragNormal = normalize(vertexNormal);\n"
    "  gl_Position = mvp*vec4(vertexPosition,1.0);\n"
    "}\n";
static const char *SHADOW_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; in vec4 fragColor; in vec3 fragNormal; in vec3 fragWorld;\n"
    "uniform sampler2D texture0; uniform vec4 colDiffuse;\n"
    "uniform vec3 lightDir; uniform vec3 viewPos;\n"
    "uniform vec3 sunCol; uniform vec3 skyCol; uniform vec3 groundCol;\n"
    "uniform float uTime;\n"

    "uniform float fogEnd; uniform vec3 fogCol;\n"
    "out vec4 finalColor;\n"

    // Cascaded shadow maps: 3 cascades (near/mid/far), each its own ortho box +
    // texture, so near-field shadows get far more texel density than a single
    // map could afford at this render distance, while the far cascade still
    // reaches out to the edge of the generated terrain ring.
    "uniform mat4 lightVP0; uniform mat4 lightVP1; uniform mat4 lightVP2;\n"
    "uniform sampler2D shadowMap0; uniform sampler2D shadowMap1; uniform sampler2D shadowMap2;\n"
    "uniform vec2 shadowTexel0; uniform vec2 shadowTexel1; uniform vec2 shadowTexel2;\n"
    // Each cascade's normalized [0,1] depth-buffer range covers a different span
    // of world metres (near cascades are tight/shallow, far cascades deep) -- a
    // single normalized bias would be either too small (acne) on the far
    // cascade or too large (peter-panning) on the near one, so bias is computed
    // in world metres then converted per-cascade via its own inverse depth range.
    "uniform float invRange0; uniform float invRange1; uniform float invRange2;\n"
    "uniform float cascadeSplit0; uniform float cascadeSplit1;\n"
    // The cascades are centred on the train/focus point P (see ShadowSys::computeLightVP),
    // NOT the camera -- cascade SELECTION must use distance from that same point, or any
    // shot where the camera sits away from the train (orbit/free-look/elevated views) picks
    // the wrong cascade (or falls outside all of them, going flat/shadowless) even though
    // the fragment is well within a cascade's actual coverage box.
    "uniform vec3 shadowFocus;\n"

    "const vec2 PD12[12] = vec2[12](\n"
    "  vec2(-0.326,-0.406),vec2(-0.840,-0.074),vec2(-0.696, 0.457),vec2(-0.203, 0.621),\n"
    "  vec2( 0.962,-0.195),vec2( 0.473,-0.480),vec2( 0.519, 0.767),vec2( 0.185,-0.893),\n"
    "  vec2( 0.507, 0.064),vec2( 0.896, 0.412),vec2(-0.322,-0.933),vec2(-0.792,-0.598));\n"
    "float pcfTap(sampler2D sm, vec2 texel, vec3 p, float bias){\n"
    "  float ang = fract(sin(dot(fragWorld.xz, vec2(12.9898,78.233)))*43758.5453)*6.2831853;\n"
    "  float ca=cos(ang), sa=sin(ang); mat2 rot=mat2(ca,-sa,sa,ca);\n"
    "  vec2 o = texel*1.4;\n"
    "  float s=0.0;\n"
    "  for(int i=0; i<12; i++){\n"
    "    vec2 tap = p.xy + rot*PD12[i]*o;\n"
    "    if(tap.x<0.0||tap.x>1.0||tap.y<0.0||tap.y>1.0) s += 1.0;\n"
    "    else s += (p.z-bias > texture(sm, tap).r) ? 0.0 : 1.0;\n"
    "  }\n"
    "  return s*(1.0/12.0);\n"
    "}\n"
    // Bias tuned in world metres (matches the old single-map 1.4-3.2 m range,
    // now comfortably clearing the ~1 m terrain height steps regardless of
    // which cascade is sampled); converted to each cascade's normalized units
    // by the caller via invRangeN.
    "float worldBias(float NoL){ return clamp(1.4 + 1.75*(1.0-NoL), 1.4, 3.2); }\n"
    "float shadowCascade(int idx, vec3 N){\n"
    "  float NoL = max(dot(N,lightDir),0.0);\n"
    "  vec4 wp = vec4(fragWorld,1.0);\n"
    "  if(idx==0){\n"
    "    vec4 lp = lightVP0*wp; vec3 p = lp.xyz/lp.w; p = p*0.5+0.5;\n"
    "    if(p.z<=0.0||p.z>1.0||p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"
    "    return pcfTap(shadowMap0, shadowTexel0, p, worldBias(NoL)*invRange0);\n"
    "  } else if(idx==1){\n"
    "    vec4 lp = lightVP1*wp; vec3 p = lp.xyz/lp.w; p = p*0.5+0.5;\n"
    "    if(p.z<=0.0||p.z>1.0||p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"
    "    return pcfTap(shadowMap1, shadowTexel1, p, worldBias(NoL)*invRange1);\n"
    "  } else {\n"
    "    vec4 lp = lightVP2*wp; vec3 p = lp.xyz/lp.w; p = p*0.5+0.5;\n"
    "    if(p.z<=0.0||p.z>1.0||p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"
    "    return pcfTap(shadowMap2, shadowTexel2, p, worldBias(NoL)*invRange2);\n"
    "  }\n"
    "}\n"
    // Select cascade by camera-XZ distance (matches the fog/culling convention
    // used elsewhere in this shader), soft-blending across a band near each
    // split so the cascade seam is never a visible hard line.
    "float shadow(vec3 N){\n"
    "  float d = length(shadowFocus.xz - fragWorld.xz);\n"
    "  float band0 = cascadeSplit0*0.85, band1 = cascadeSplit1*0.85;\n"
    "  if(d < band0) return shadowCascade(0, N);\n"
    "  if(d < cascadeSplit0){\n"
    "    float t = (d-band0)/max(cascadeSplit0-band0,0.001);\n"
    "    return mix(shadowCascade(0,N), shadowCascade(1,N), t);\n"
    "  }\n"
    "  if(d < band1) return shadowCascade(1, N);\n"
    "  if(d < cascadeSplit1){\n"
    "    float t = (d-band1)/max(cascadeSplit1-band1,0.001);\n"
    "    return mix(shadowCascade(1,N), shadowCascade(2,N), t);\n"
    "  }\n"
    "  return shadowCascade(2, N);\n"
    "}\n"

    "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"
    "vec3 toLinear(vec3 c){ return pow(c, vec3(2.2)); }\n"

    "float wh2(vec2 p){ return fract(sin(dot(p,vec2(41.3,289.1)))*43758.5453); }\n"
    "float wvn(vec2 p){ vec2 i=floor(p),f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  return mix(mix(wh2(i),wh2(i+vec2(1,0)),f.x), mix(wh2(i+vec2(0,1)),wh2(i+vec2(1,1)),f.x), f.y); }\n"

    "vec3 waterShade(vec3 baseCol, float rawSh, float foam){\n"
    "  vec3 V = normalize(viewPos - fragWorld);\n"
    "  vec2 w = fragWorld.xz;\n"

    "  float t = uTime;\n"
    "  float wdist = length(viewPos.xz - w);\n"
    "  float amp = 1.0 / (1.0 + wdist*0.018);\n"
    "  float nx = 0.032*sin(w.x*0.55 + w.y*0.31 + t*1.3)\n"
    "           + 0.022*sin(w.y*1.07 - t*0.9)\n"
    "           + 0.014*sin((w.x+w.y)*0.83 + t*1.7);\n"
    "  float nz = 0.032*sin(w.y*0.55 + w.x*0.31 - t*1.1)\n"
    "           + 0.022*sin(w.x*1.07 + t*0.8)\n"
    "           + 0.014*sin((w.x-w.y)*0.83 - t*1.5);\n"
    "  nx += (wvn(w*1.6 + t*0.25)-0.5)*0.028;\n"
    "  nz += (wvn(w*1.6 - t*0.21)-0.5)*0.028;\n"
    "  vec3 N = normalize(vec3(nx*amp, 1.0, nz*amp));\n"
    "  float NoV = max(dot(N, V), 0.0);\n"

    "  float fres = clamp(0.02 + 0.98*pow(1.0 - NoV, 5.0), 0.02, 0.62);\n"
    "  fres *= (1.0 - foam*0.65);\n"

    "  vec3 body = toLinear(baseCol);\n"
    "  float ndlW = max(dot(N, lightDir), 0.0);\n"
    "  vec3 deep = body * (0.62 + 0.46*ndlW*mix(0.4,1.0,rawSh)) + groundCol*0.04;\n"

    "  vec3 R = reflect(-V, N);\n"
    "  float ry = clamp(R.y, 0.0, 1.0);\n"
    "  vec3 skyZen  = vec3(0.24, 0.46, 0.86);\n"
    "  vec3 skyHorz = vec3(0.52, 0.70, 0.92);\n"
    "  vec3 sky = mix(skyHorz, skyZen, pow(ry, 0.45));\n"
    "  float sunGlow = pow(max(dot(R, lightDir), 0.0), 8.0);\n"
    "  vec3 refl = mix(sky, sunCol*0.55, 0.30*sunGlow);\n"
    "  vec3 col = mix(deep, refl, fres);\n"

    "  vec3 H = normalize(lightDir + V);\n"
    "  float glint = pow(max(dot(N,H),0.0), 200.0)*rawSh;\n"
    "  col += sunCol*glint*1.6;\n"

    "  if(foam > 0.001){\n"
    "    float lace = wvn(w*2.3 + t*0.6)*0.6 + wvn(w*5.1 - t*0.4)*0.4;\n"
    "    float froth = smoothstep(0.45, 0.95, lace) * foam;\n"
    "    col = mix(col, vec3(0.92,0.96,0.98), clamp(froth*0.85, 0.0, 0.8));\n"
    "  }\n"
    "  return col;\n"
    "}\n"
    "void main(){\n"
    "  vec4 tex = texture(texture0, fragTexCoord);\n"
    "  vec3 base = tex.rgb*fragColor.rgb*colDiffuse.rgb;\n"

    "  if(fragColor.a < 0.72 && normalize(fragNormal).y > 0.80 && base.b > base.r*1.15){\n"
    "    float rawShW = shadow(vec3(0.0,1.0,0.0));\n"

    "    float foam = smoothstep(0.62, 0.70, fragColor.a);\n"
    "    vec3 wcol = waterShade(base, rawShW, foam);\n"
    "    wcol = aces(wcol*0.94);\n"
    "    wcol = pow(wcol, vec3(1.0/2.2));\n"
    "    float wa = mix(0.90, 0.96, foam);\n"

    "    if(fogEnd > 0.0){\n"
    "      float d = length(viewPos.xz - fragWorld.xz);\n"
    "      float fog = clamp((d - fogEnd*0.55)/(fogEnd*0.40), 0.0, 1.0);\n"
    "      wcol = mix(wcol, fogCol, fog);\n"
    "      wa = mix(wa, 0.0, fog);\n"
    "    }\n"
    "    finalColor = vec4(wcol, wa);\n"
    "    return;\n"
    "  }\n"
    "  vec3 albedo = toLinear(base);\n"
    "  float maxB = max(max(base.r, base.g), base.b);\n"
    "  float minB = min(min(base.r, base.g), base.b);\n"
    "  float maxA = max(max(albedo.r, albedo.g), albedo.b);\n"
    "  float minA = min(min(albedo.r, albedo.g), albedo.b);\n"
    "  float pale = smoothstep(0.50, 0.86, maxA) * (1.0-smoothstep(0.08, 0.26, maxA-minA));\n"
    "  float sheen = smoothstep(0.40, 0.82, maxB) * (1.0-smoothstep(0.10, 0.34, maxB-minB));\n"
    "  albedo = mix(albedo, albedo*vec3(0.76,0.83,0.91), pale*0.42);\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  float ndl = max(dot(N,lightDir),0.0);\n"
    "  float rawSh = shadow(N);\n"
    "  float sh = mix(0.18, 1.0, rawSh);\n"
    "  vec3 direct = sunCol*1.02*ndl*sh;\n"

    "  float up   = clamp( N.y*0.5+0.5, 0.0, 1.0);\n"
    "  float down = clamp(-N.y, 0.0, 1.0);\n"
    "  vec3 skyFill   = skyCol   * (0.62 + 0.55*up);\n"
    "  vec3 bounce    = groundCol* (0.50 + 1.40*down);\n"
    "  vec3 ambient   = (skyFill + bounce) * (0.52 + 0.22*rawSh);\n"
    "  vec3 V = normalize(viewPos-fragWorld);\n"
    "  vec3 H = normalize(lightDir+V);\n"
    "  float NoH = max(dot(N,H),0.0);\n"
    "  float spec = (pow(NoH, 56.0)*(0.18 + 0.36*sheen) + pow(NoH, 18.0)*0.045*sheen)*rawSh*ndl;\n"
    "  float rim = pow(1.0-clamp(dot(N,V),0.0,1.0),3.6)*sheen*0.055*mix(0.35,1.0,rawSh);\n"
    "  vec3 col = albedo*(ambient + direct) + sunCol*spec + skyCol*rim;\n"
    "  col = aces(col*0.94);\n"
    "  col = pow(col, vec3(1.0/2.2));\n"

    "  float lum = dot(col, vec3(0.299,0.587,0.114));\n"
    "  col = mix(vec3(lum), col, 1.10);\n"

    "  if(fogEnd > 0.0){\n"
    "    float d = length(viewPos.xz - fragWorld.xz);\n"
    "    float fog = clamp((d - fogEnd*0.55)/(fogEnd*0.40), 0.0, 1.0);\n"
    "    col = mix(col, fogCol, fog);\n"
    "  }\n"
    "  finalColor = vec4(col, tex.a*fragColor.a*colDiffuse.a);\n"
    "}\n";

static const char *DEPTH_VS =
    "#version 330\n"
    "in vec3 vertexPosition; uniform mat4 mvp;\n"
    "void main(){ gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
static const char *DEPTH_FS =
    "#version 330\n"
    "void main(){}\n";

static const int SHADOW_CASCADES = 3;
// Cascade half-extents (world m): near/mid/far. The terrain ring generates out
// to TERRA_R=256 m around the camera (main.cpp, 16 chunks @ 16 m/chunk), so
// the far cascade is sized to use nearly all of that footprint rather than
// reaching past ground that will never exist.
static const float SHADOW_CASCADE_R[SHADOW_CASCADES] = { 32.0f, 100.0f, 245.0f };
// Depth-pass draw-call cull radius per cascade: box half-diagonal (R*sqrt2)
// plus a safety margin, mirroring the old single-map SHADOW_CULL_R pattern.
static const float SHADOW_CASCADE_CULL_R[SHADOW_CASCADES] = {
    SHADOW_CASCADE_R[0] * 1.42f + 15.0f,
    SHADOW_CASCADE_R[1] * 1.42f + 15.0f,
    SHADOW_CASCADE_R[2] * 1.42f + 15.0f,
};

struct ShadowSys {
    Shader lit{}, depth{};
    unsigned int fbo[SHADOW_CASCADES] = {0,0,0};
    unsigned int depthTex[SHADOW_CASCADES] = {0,0,0};
    int SM[SHADOW_CASCADES] = { 2048, 1536, 1024 };   // near cascade keeps full res; far cascade doesn't need it
    int locLightVP[SHADOW_CASCADES]   = {-1,-1,-1};
    int locShadowMap[SHADOW_CASCADES] = {-1,-1,-1};
    int locShadowTexel[SHADOW_CASCADES] = {-1,-1,-1};
    int locInvRange[SHADOW_CASCADES]  = {-1,-1,-1};
    int locCascadeSplit0=-1, locCascadeSplit1=-1, locShadowFocus=-1;
    int locLightDir=-1, locViewPos=-1;
    int locSun=-1, locSky=-1, locGround=-1, locDepthMVP=-1, locTime=-1;
    int locFogEnd=-1, locFogCol=-1;
    Matrix lightVP[SHADOW_CASCADES]{};
    float invRange[SHADOW_CASCADES]{};
    Vector3 focus{};   // last point cascades were centred on -- shaders must select cascades by distance from THIS, not the camera

    void init() {
        lit   = LoadShaderFromMemory(SHADOW_VS, SHADOW_FS);
        depth = LoadShaderFromMemory(DEPTH_VS, DEPTH_FS);
        const char *vpNames[SHADOW_CASCADES]    = { "lightVP0", "lightVP1", "lightVP2" };
        const char *mapNames[SHADOW_CASCADES]   = { "shadowMap0", "shadowMap1", "shadowMap2" };
        const char *texelNames[SHADOW_CASCADES] = { "shadowTexel0", "shadowTexel1", "shadowTexel2" };
        const char *rangeNames[SHADOW_CASCADES] = { "invRange0", "invRange1", "invRange2" };
        for (int i = 0; i < SHADOW_CASCADES; i++) {
            locLightVP[i]     = GetShaderLocation(lit, vpNames[i]);
            locShadowMap[i]   = GetShaderLocation(lit, mapNames[i]);
            locShadowTexel[i] = GetShaderLocation(lit, texelNames[i]);
            locInvRange[i]    = GetShaderLocation(lit, rangeNames[i]);
        }
        locCascadeSplit0 = GetShaderLocation(lit, "cascadeSplit0");
        locCascadeSplit1 = GetShaderLocation(lit, "cascadeSplit1");
        locShadowFocus   = GetShaderLocation(lit, "shadowFocus");
        locLightDir    = GetShaderLocation(lit, "lightDir");
        locViewPos     = GetShaderLocation(lit, "viewPos");
        locSun         = GetShaderLocation(lit, "sunCol");
        locSky         = GetShaderLocation(lit, "skyCol");
        locGround      = GetShaderLocation(lit, "groundCol");
        locTime        = GetShaderLocation(lit, "uTime");
        locFogEnd      = GetShaderLocation(lit, "fogEnd");
        locFogCol      = GetShaderLocation(lit, "fogCol");
        for (int i = 0; i < SHADOW_CASCADES; i++) {
            fbo[i] = rlLoadFramebuffer();
            rlEnableFramebuffer(fbo[i]);
            depthTex[i] = rlLoadTextureDepth(SM[i], SM[i], false);
            rlFramebufferAttach(fbo[i], depthTex[i], RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
            if (!rlFramebufferComplete(fbo[i])) TraceLog(LOG_WARNING, "SHADOW: cascade %d framebuffer is incomplete, shadows may be disabled", i);
            rlDisableFramebuffer();
        }
    }

    // Fills lightVP[]/invRange[] for all cascades, centred on `focus` (the
    // camera/train focus point) along the fixed sun direction. Each cascade's
    // eye distance and far plane scale with its own half-extent so the box is
    // always fully contained regardless of sun elevation.
    void computeLightVP(Vector3 f) {
        focus = f;
        for (int i = 0; i < SHADOW_CASCADES; i++) {
            float R = SHADOW_CASCADE_R[i];
            float eyeDist = R * 2.2f + 40.0f;
            float nearP = 4.0f;
            float farP  = eyeDist + R * 1.8f;
            Vector3 eye = Vector3Add(focus, Vector3Scale(g_sunDir, eyeDist));
            Matrix view = MatrixLookAt(eye, focus, Vector3{ 0, 1, 0 });
            Matrix proj = MatrixOrtho(-R, R, -R, R, nearP, farP);
            lightVP[i] = MatrixMultiply(view, proj);
            invRange[i] = 1.0f / (farP - nearP);
        }
    }
};
static ShadowSys gShadow;

static const char *SKY_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
    "uniform mat4 mvp;\n"
    "uniform vec2 resolution;\n"
    "out vec2 uv;\n"
    "void main(){ uv = vertexPosition.xy/resolution; gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
static const char *SKY_FS =
    "#version 330\n"
    "in vec2 uv; out vec4 finalColor;\n"
    "uniform vec3 camDir; uniform vec3 camRight; uniform vec3 camUp;\n"
    "uniform float tanHalfFovY; uniform float aspect;\n"
    "uniform vec3 sunDir; uniform vec3 camPos;\n"
    "const vec3 ZENITH  = vec3(0.045, 0.26, 0.74);\n"
    "const vec3 MIDSKY  = vec3(0.16, 0.50, 0.95);\n"
    "const vec3 HORIZON = vec3(0.52, 0.74, 1.00);\n"
    "const vec3 HAZE    = vec3(1.00, 0.78, 0.50);\n"

    "const vec3 GROUND  = vec3(0.64, 0.72, 0.80);\n"
    "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"

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

    "  float cb=300.0, ct=470.0;\n"
    "  float t0=(cb-ro.y)/rd.y, t1=(ct-ro.y)/rd.y;\n"
    "  float tn=max(min(t0,t1),0.0), tf=max(t0,t1);\n"
    "  if(tf<=tn) return vec4(0.0);\n"
    "  const float DMAX=4200.0;\n"
    "  float distFade = 1.0 - smoothstep(DMAX*0.72, DMAX, tn);\n"
    "  if(distFade <= 0.0) return vec4(0.0);\n"
    "  tf=min(tf,tn+1500.0);\n"
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
    "      vec3 cc=ambC + sunC*sh*1.6;\n"
    "      float a=clamp(d*0.9,0.0,1.0);\n"
    "      acc+=trans*a*cc; trans*=(1.0-a);\n"
    "      if(trans<0.03) break;\n"
    "    }\n"
    "    t+=dt;\n"
    "  }\n"
    "  return vec4(acc, (1.0-trans)*distFade);\n"
    "}\n"
    "void main(){\n"
    "  vec2 ndc = uv*2.0-1.0;\n"
    "  vec3 dir = normalize(camDir + camRight*ndc.x*tanHalfFovY*aspect + camUp*(-ndc.y)*tanHalfFovY);\n"
    "  vec3 sun = normalize(sunDir);\n"
    "  float sunLift = smoothstep(-0.12, 0.55, sun.y);\n"

    "  float h = clamp(dir.y*0.5+0.5, 0.0, 1.0);\n"
    "  float skyT = smoothstep(0.03, 0.92, h);\n"
    "  vec3 col = mix(HORIZON, MIDSKY, smoothstep(0.0, 0.55, skyT));\n"
    "  col = mix(col, ZENITH, smoothstep(0.34, 1.0, skyT));\n"

    "  float airMass = exp(-max(dir.y,0.0)*2.6);\n"
    "  col = mix(col, HORIZON, airMass*0.22);\n"
    "  float horizon = exp(-abs(dir.y)*4.2);\n"
    "  col += HORIZON*horizon*0.22;\n"
    "  col += HAZE*horizon*(0.10 + 0.20*(1.0-sunLift));\n"

    "  float sunAz = clamp(dot(normalize(dir.xz), normalize(sun.xz)), 0.0, 1.0);\n"
    "  vec3 lowHaze = mix(GROUND, GROUND*vec3(1.05,1.0,0.93), sunAz*0.6);\n"
    "  col = mix(lowHaze, col, smoothstep(-0.16, 0.06, dir.y));\n"

    "  float mu = clamp(dot(dir, sun), -1.0, 1.0);\n"
    "  float forward = max(mu, 0.0);\n"

    "  float rayleigh = 0.55 + 0.45*mu*mu;\n"
    "  col *= rayleigh;\n"
    "  vec3 sunTint = mix(vec3(1.0,0.62,0.32), vec3(1.0,0.90,0.70), sunLift);\n"

    "  float mieSharp = pow(forward, 22.0);\n"
    "  float mieBroad = pow(forward, 5.0);\n"
    "  col += sunTint*mieSharp*(0.42 + 0.36*horizon);\n"
    "  col += sunTint*mieBroad*(0.10 + 0.16*(1.0-sunLift));\n"

    "  float viewSun = max(dot(camDir, sun), 0.0);\n"
    "  float lookGate = smoothstep(0.55, 0.96, viewSun);\n"

    "  vec3 ref = (abs(sun.y) > 0.94) ? vec3(1.0,0.0,0.0) : vec3(0.0,1.0,0.0);\n"
    "  vec3 sr = normalize(cross(ref, sun));\n"
    "  vec3 su = cross(sun, sr);\n"
    "  vec2 sunPlane = vec2(dot(dir, sr), dot(dir, su));\n"
    "  float a = atan(sunPlane.y, sunPlane.x);\n"
    "  float r = length(sunPlane);\n"
    "  float streaks = pow(0.5+0.5*sin(a*13.0 + sin(a*5.0)*1.7), 4.0);\n"
    "  float shafts = streaks*smoothstep(0.72, 0.995, forward)*smoothstep(0.68, 0.08, r);\n"
    "  col += sunTint*shafts*(0.26 + 0.30*horizon)*sunLift*lookGate;\n"

    "  float bar = exp(-pow(sunPlane.y*9.0,2.0)) * smoothstep(0.5,0.0,r);\n"
    "  col += sunTint*bar*0.22*lookGate;\n"
    "  float ghost = exp(-pow((r-0.16)*10.0,2.0)) + 0.6*exp(-pow((r-0.34)*7.0,2.0));\n"
    "  col += sunTint*ghost*0.05*lookGate;\n"

    "  float disc = smoothstep(0.99950, 0.99986, mu);\n"
    "  float corona = pow(forward, 96.0);\n"
    "  col += vec3(1.0,0.93,0.74)*corona*0.58*sunLift*mix(0.4,1.0,lookGate);\n"
    "  col = mix(col, vec3(1.0,0.94,0.78), disc*0.78*sunLift);\n"
    "  col += vec3(1.0,0.97,0.84)*disc*1.15*sunLift;\n"

    "  vec4 cl = cloudVolume(camPos, dir, sun, sunLift);\n"
    "  col = col*(1.0-cl.a) + cl.rgb;\n"
    "  col = aces(col*1.08);\n"
    "  finalColor = vec4(pow(clamp(col, 0.0, 1.0), vec3(1.0/2.2)), 1.0);\n"
    "}\n";

struct SkySys {
    Shader sh{};
    int locCamDir=-1, locCamRight=-1, locCamUp=-1, locTan=-1, locAspect=-1, locSun=-1, locRes=-1, locCamPos=-1;
    void init() {
        sh = LoadShaderFromMemory(SKY_VS, SKY_FS);
        locCamDir   = GetShaderLocation(sh, "camDir");
        locCamRight = GetShaderLocation(sh, "camRight");
        locCamUp    = GetShaderLocation(sh, "camUp");
        locTan      = GetShaderLocation(sh, "tanHalfFovY");
        locAspect   = GetShaderLocation(sh, "aspect");
        locSun      = GetShaderLocation(sh, "sunDir");
        locRes      = GetShaderLocation(sh, "resolution");
        locCamPos   = GetShaderLocation(sh, "camPos");
    }
};
static SkySys gSky;

#endif

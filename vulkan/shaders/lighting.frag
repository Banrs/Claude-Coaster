#version 450
// Deferred lighting + analytic sky. Reads the G-buffer + shadow map + AO and
// produces LINEAR HDR. Background texels (flag==0) become the analytic sky/clouds;
// geometry is Cook-Torrance lit and its distance fog blends toward that same sky.
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;     // xyz
    vec4 camPos;     // xyz, w = fog end
} u;
layout(set = 0, binding = 1) uniform sampler2D shadowMap;
layout(set = 0, binding = 2) uniform sampler2D gAlbedo;
layout(set = 0, binding = 3) uniform sampler2D gNormalRough;
layout(set = 0, binding = 4) uniform sampler2D gPosition;
layout(set = 0, binding = 5) uniform sampler2D ssaoTex;

layout(push_constant) uniform PC {
    vec4 camDir;    // xyz normalized forward
    vec4 camRight;  // xyz normalized right
    vec4 camUp;     // xyz normalized up
    vec4 params;    // x = tanHalfFovY, y = aspect, z = time, w = 0
} pc;

const float PI = 3.14159265359;

// ---- analytic sky + clouds (ported from src/pathtrace.cpp) ----
float h13(vec3 p){ p=fract(p*0.1031); p+=dot(p,p.yzx+33.33); return fract((p.x+p.y)*p.z); }
float vn3(vec3 x){ vec3 i=floor(x), f=fract(x); f=f*f*(3.0-2.0*f);
  return mix(mix(mix(h13(i+vec3(0,0,0)),h13(i+vec3(1,0,0)),f.x), mix(h13(i+vec3(0,1,0)),h13(i+vec3(1,1,0)),f.x),f.y),
             mix(mix(h13(i+vec3(0,0,1)),h13(i+vec3(1,0,1)),f.x), mix(h13(i+vec3(0,1,1)),h13(i+vec3(1,1,1)),f.x),f.y), f.z); }
float cloudDens(vec3 p){
  float cb=300.0, ct=470.0; float hf=(p.y-cb)/(ct-cb);
  if(hf<0.0||hf>1.0) return 0.0;
  hf = smoothstep(0.0,0.32,hf)*smoothstep(1.0,0.5,hf);
  vec3 q=p*0.0012; vec3 qb=floor(q*5.0)/5.0;
  float base=mix(vn3(q*5.0+3.1), vn3(qb*5.0+3.1), 0.5);
  float det=vn3(q*4.0)*0.5+vn3(q*9.5)*0.28+vn3(q*20.0)*0.14;
  return smoothstep(0.54,1.0, base*0.72+det*0.5) * hf;   // sparser, thinner clouds
}
vec4 cloudVolume(vec3 ro, vec3 rd, vec3 sun, float lift){
  if(rd.y < 0.03) return vec4(0.0);
  float cb=300.0, ct=470.0; float t0=(cb-ro.y)/rd.y, t1=(ct-ro.y)/rd.y;
  float tn=max(min(t0,t1),0.0), tf=max(t0,t1);
  if(tf<=tn) return vec4(0.0); tf=min(tf,tn+1500.0);
  const int N=14; float dt=(tf-tn)/float(N);
  float t=tn+dt*h13(vec3(gl_FragCoord.xy,1.0));
  float trans=1.0; vec3 acc=vec3(0.0);
  vec3 sunC=mix(vec3(1.0,0.85,0.70), vec3(1.0,0.97,0.92), lift); vec3 ambC=vec3(0.60,0.67,0.80);
  for(int i=0;i<N;i++){ vec3 p=ro+rd*t; float d=cloudDens(p);
    if(d>0.01){ float ld=cloudDens(p+sun*42.0)+cloudDens(p+sun*95.0)*0.6; float sh=exp(-ld*1.5);
      vec3 col=ambC+sunC*sh*1.35; float a=clamp(d*0.6,0.0,1.0);
      acc+=trans*a*col; trans*=(1.0-a); if(trans<0.03) break; }
    t+=dt; }
  return vec4(acc, 1.0-trans);
}
// cloud-free sky gradient — also serves as the IBL reflection probe
vec3 skyBase(vec3 d, vec3 sun){
  const vec3 ZEN=vec3(0.020,0.14,0.55), MID=vec3(0.10,0.34,0.80), HOR=vec3(0.30,0.54,0.88);
  const vec3 HAZE=vec3(1.0,0.80,0.52), GND=vec3(0.20,0.40,0.62);
  float h = clamp(d.y*0.5+0.5,0.0,1.0); float t = smoothstep(0.03,0.92,h);
  vec3 c = mix(HOR,MID,smoothstep(0.0,0.55,t)); c = mix(c,ZEN,smoothstep(0.34,1.0,t));
  float airMass = exp(-max(d.y,0.0)*2.6); c = mix(c, HOR, airMass*0.10);
  float hz = exp(-abs(d.y)*4.2); float lift = smoothstep(-0.12,0.55,sun.y);
  c += HOR*hz*0.05; c += HAZE*hz*(0.03+0.07*(1.0-lift));
  c = mix(GND,c,smoothstep(-0.28,-0.02,d.y));
  float mu = clamp(dot(d,sun),-1.0,1.0); float fwd = max(mu,0.0);
  c *= 0.55+0.45*mu*mu;
  vec3 sunTint = mix(vec3(1.0,0.62,0.32),vec3(1.0,0.90,0.70),lift);
  c += sunTint*pow(fwd,22.0)*(0.42+0.36*hz);
  c += sunTint*pow(fwd,5.0)*(0.10+0.16*(1.0-lift));
  c += sunTint*pow(fwd,8.0)*0.10*lift;
  float disc = smoothstep(0.9994,0.99986,mu);
  c += vec3(1.0,0.95,0.80)*disc*6.0*lift;
  return c;
}
vec3 skyCol(vec3 d, vec3 sun, vec3 ro){
  vec3 c = skyBase(d, sun);
  float lift = smoothstep(-0.12,0.55,sun.y);
  vec4 cl = cloudVolume(ro, d, sun, lift);
  return c*(1.0-cl.a) + cl.rgb;
}

// ---- PBR ----
float D_GGX(float NoH, float a){ float a2=a*a; float d=(NoH*NoH)*(a2-1.0)+1.0; return a2/(PI*d*d); }
float G_SchlickGGX(float NoX, float k){ return NoX/(NoX*(1.0-k)+k); }
float G_Smith(float NoV, float NoL, float r){ float k=(r+1.0); k=(k*k)/8.0; return G_SchlickGGX(NoV,k)*G_SchlickGGX(NoL,k); }
vec3  F_Schlick(float c, vec3 F0){ return F0 + (1.0-F0)*pow(clamp(1.0-c,0.0,1.0),5.0); }

float sampleShadow(vec4 lp, float ndl){
    vec3 p = lp.xyz / lp.w;
    vec2 suv = p.xy * 0.5 + 0.5;
    if(suv.x<0.0||suv.x>1.0||suv.y<0.0||suv.y>1.0||p.z>1.0) return 1.0;
    float bias = max(0.0030*(1.0-ndl), 0.0009);
    vec2 texel = 1.0 / vec2(textureSize(shadowMap,0));
    float vis = 0.0;
    for(int y=-1;y<=1;y++) for(int x=-1;x<=1;x++){
        float d = texture(shadowMap, suv + vec2(x,y)*texel).r;
        vis += (p.z - bias > d) ? 0.0 : 1.0;
    }
    return vis / 9.0;
}

// single-tap shadow test at a world point (for the volumetric march). Only air
// within the shadow frustum contributes in-scatter; elsewhere returns 0 so far
// air doesn't wash the whole frame.
float litAt(vec3 wp){
    vec4 lp = u.lightVP * vec4(wp,1.0); vec3 p = lp.xyz/lp.w;
    vec2 suv = p.xy*0.5+0.5;
    if(suv.x<0.0||suv.x>1.0||suv.y<0.0||suv.y>1.0||p.z>1.0) return 0.0;   // outside coverage: no contribution
    return (p.z - 0.0015 > texture(shadowMap, suv).r) ? 0.0 : 1.0;
}
float hash12(vec2 p){ return fract(sin(dot(p,vec2(41.3,289.1)))*43758.5453); }
// Volumetric sun in-scatter: march the view ray, accumulate where air is sunlit.
// Returns a scalar density already weighted by a Henyey-Greenstein phase toward
// the sun, so it blooms when looking into the light.
float godRays(vec3 ro, vec3 rd, float maxD, float mu){
    const int N = 28;
    float seg = maxD / float(N);
    float j = hash12(gl_FragCoord.xy) * seg;
    float acc = 0.0;
    for(int i=0;i<N;i++) acc += litAt(ro + rd*(j + seg*float(i)));
    acc /= float(N);
    const float g = 0.66;
    float denom = 1.0 + g*g - 2.0*g*max(mu,0.0);
    float phase = (1.0 - g*g) / (4.0*PI*pow(denom, 1.5));
    return acc * phase;
}

// Screen-space contact shadows: a short march toward the sun in screen space,
// catching fine near-contact occlusion the shadow map is too coarse to resolve.
float contactShadow(vec3 P, vec3 N, vec3 L){
    const int M = 12; const float stepLen = 0.40;
    vec3 sp = P + N*0.15;
    for(int i=0;i<M;i++){
        sp += L*stepLen;
        vec4 clip = u.viewProj * vec4(sp,1.0);
        if(clip.w <= 0.0) break;
        vec2 suv = (clip.xy/clip.w)*0.5 + 0.5;
        if(suv.x<0.0||suv.x>1.0||suv.y<0.0||suv.y>1.0) break;
        vec4 g = texture(gPosition, suv);
        if(g.a < 0.5) continue;
        float d = distance(u.camPos.xyz, sp) - distance(u.camPos.xyz, g.xyz);
        if(d > 0.04 && d < 1.4) return 0.0;            // a closer surface blocks the sun
    }
    return 1.0;
}

void main(){
    vec3 sun = normalize(u.sunDir.xyz);
    vec3 ro  = u.camPos.xyz;
    // view ray for this pixel (used for the sky background and fog target)
    vec2 ndc = uv*2.0 - 1.0;
    vec3 rd  = normalize(pc.camDir.xyz + pc.camRight.xyz*ndc.x*pc.params.x*pc.params.y + pc.camUp.xyz*(-ndc.y)*pc.params.x);

    float mu = dot(rd, sun);
    vec3 sunTintAir = vec3(1.0,0.92,0.78);
    const float SHAFT = 1.1;   // overall god-ray strength

    vec4 pf = texture(gPosition, uv);
    if(pf.a < 0.5){
        vec3 c = skyCol(rd, sun, ro);
        c += sunTintAir * godRays(ro, rd, 900.0, mu) * SHAFT;            // shafts against the sky
        outColor = vec4(c, 1.0); return;
    }

    vec3 wp = pf.xyz;
    vec4 nr = texture(gNormalRough, uv);
    vec3 N  = normalize(nr.xyz);
    float rough = nr.a;
    vec4 alb = texture(gAlbedo, uv);
    vec3 baseCol = alb.rgb; float metal = alb.a;
    float ao = texture(ssaoTex, uv).r;

    vec3 V = normalize(ro - wp);
    vec3 L = sun;
    vec3 H = normalize(V + L);

    vec3  albedo = pow(baseCol, vec3(2.2));
    vec3  F0 = mix(vec3(0.04), albedo, metal);

    float NoL=max(dot(N,L),0.0), NoV=max(dot(N,V),1e-4), NoH=max(dot(N,H),0.0), VoH=max(dot(V,H),0.0);
    float D=D_GGX(NoH,rough*rough), G=G_Smith(NoV,NoL,rough);
    vec3  F=F_Schlick(VoH,F0);
    vec3  spec=(D*G*F)/(4.0*NoV*NoL+1e-4);
    vec3  kd=(vec3(1.0)-F)*(1.0-metal);
    vec3  diffuse=kd*albedo/PI;

    vec4 lightPos = u.lightVP * vec4(wp, 1.0);
    float shadow = sampleShadow(lightPos, NoL);
    if(shadow > 0.5 && NoL > 0.0) shadow = min(shadow, contactShadow(wp, N, L));  // contact shadows
    vec3 sunRad = vec3(3.4,3.1,2.7);
    vec3 Lo = (diffuse + spec) * sunRad * NoL * shadow;

    vec3 skyA=vec3(0.34,0.45,0.66), ground=vec3(0.16,0.15,0.13);
    vec3 amb = mix(ground, skyA, clamp(N.y*0.5+0.5,0.0,1.0)) * albedo * ao * (1.0-metal);
    // image-based ambient specular: reflect the sky probe, Fresnel-weighted, blurred by roughness
    vec3 Rdir = reflect(-V, N);
    vec3 Fr = F_Schlick(NoV, F0);
    vec3 env = mix(skyBase(Rdir, L), skyA, rough);             // rough -> duller, sky-tinted
    vec3 iblSpec = env * Fr * ao * (1.0 - rough*0.6);
    vec3 c = Lo + amb + iblSpec;
    // cheap subsurface scattering for foliage: sun transmitted through thin leaves
    float foliage = clamp((albedo.g - max(albedo.r, albedo.b))*3.5, 0.0, 1.0);
    float back = pow(clamp(dot(V, -L), 0.0, 1.0), 3.0) + 0.15*clamp(dot(-N,L),0.0,1.0);
    c += foliage * back * albedo * sunRad * shadow * 0.6;

    float dist = length(wp - ro);
    float fogEnd = max(u.camPos.w, 1.0);
    float fog = clamp((dist - fogEnd*0.65)/(fogEnd*0.35), 0.0, 1.0); fog *= fog;
    vec3 fogCol = skyCol(normalize(wp - ro), sun, ro);
    c = mix(c, fogCol, fog*0.85);
    c += sunTintAir * godRays(ro, rd, min(dist,900.0), mu) * SHAFT;       // sunlit air in front of geometry

    outColor = vec4(c, 1.0);
}

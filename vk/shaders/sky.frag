#version 450
// Analytic sky + volumetric clouds, ported from the raylib path tracer (src/pathtrace.cpp).
// Outputs LINEAR HDR radiance (no tonemap/gamma); a later post pass applies exposure+ACES+gamma.

layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PC {
    vec4 camPos;   // xyz = camera world position
    vec4 camDir;   // xyz = normalized forward
    vec4 camRight; // xyz = normalized right
    vec4 camUp;    // xyz = normalized up
    vec4 sunDir;   // xyz = normalized sun direction (toward sun)
    vec4 params;   // x = tanHalfFovY, y = aspect, z = time, w = 0
} pc;

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
  return smoothstep(0.54,1.0, base*0.72+det*0.5) * hf;
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

vec3 skyCol(vec3 d, vec3 sun, vec3 ro){
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
  vec4 cl = cloudVolume(ro, d, sun, lift);
  c = c*(1.0-cl.a) + cl.rgb;
  return c;
}

void main(){
  vec2 ndc = uv*2.0 - 1.0;
  vec3 rd = normalize(pc.camDir.xyz + pc.camRight.xyz*ndc.x*pc.params.x*pc.params.y + pc.camUp.xyz*(-ndc.y)*pc.params.x);
  vec3 ro = pc.camPos.xyz;
  vec3 sun = normalize(pc.sunDir.xyz);
  outColor = vec4(skyCol(rd, sun, ro), 1.0);
}

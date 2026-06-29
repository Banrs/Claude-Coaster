#version 450
// Realistic water surface shading. Ported from the base game's waterShade()
// (../../src/render_fx.cpp) and skyCol() (../../src/pathtrace.cpp), adapted to
// the Vulkan scene UBO contract (matches mesh.frag). Outputs LINEAR HDR — the
// post pass owns exposure + ACES + gamma, so we do NOT tonemap here.
layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec4 vLightPos;

layout(set = 0, binding = 0) uniform U {
    mat4 viewProj;
    mat4 lightVP;
    vec4 sunDir;     // xyz
    vec4 camPos;     // xyz, w = fog end
} u;
layout(set = 0, binding = 1) uniform sampler2D shadowMap;
layout(set = 0, binding = 2) uniform sampler2D gPosition;   // world pos of the bed behind the water

layout(push_constant) uniform PC {
    vec4 misc;       // x = time (seconds)
} pc;

layout(location = 0) out vec4 outColor;

// --- value noise for ripple jitter (from render_fx.cpp wh2/wvn) ---
float wh2(vec2 p){ return fract(sin(dot(p, vec2(41.3, 289.1))) * 43758.5453); }
float wvn(vec2 p){
    vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
    return mix(mix(wh2(i),            wh2(i + vec2(1, 0)), f.x),
               mix(wh2(i + vec2(0,1)), wh2(i + vec2(1, 1)), f.x), f.y);
}

// --- compact sky for reflections (ported/simplified from pathtrace.cpp skyCol,
// clouds dropped). Keeps the horizon gradient + sun tint. ---
vec3 skyCol(vec3 d, vec3 sun){
    const vec3 ZEN = vec3(0.020, 0.14, 0.55);
    const vec3 MID = vec3(0.10, 0.34, 0.80);
    const vec3 HOR = vec3(0.30, 0.54, 0.88);
    const vec3 HAZE = vec3(1.0, 0.80, 0.52);
    const vec3 GND = vec3(0.20, 0.40, 0.62);
    float h = clamp(d.y * 0.5 + 0.5, 0.0, 1.0);
    float t = smoothstep(0.03, 0.92, h);
    vec3 c = mix(HOR, MID, smoothstep(0.0, 0.55, t));
    c = mix(c, ZEN, smoothstep(0.34, 1.0, t));
    float airMass = exp(-max(d.y, 0.0) * 2.6);
    c = mix(c, HOR, airMass * 0.42);
    float hz = exp(-abs(d.y) * 4.2);
    float lift = smoothstep(-0.12, 0.55, sun.y);
    c += HOR * hz * 0.20;
    c += HAZE * hz * (0.08 + 0.18 * (1.0 - lift));
    c = mix(GND, c, smoothstep(-0.10, 0.04, d.y));
    float mu = clamp(dot(d, sun), -1.0, 1.0);
    float fwd = max(mu, 0.0);
    c *= 0.55 + 0.45 * mu * mu;
    vec3 sunTint = mix(vec3(1.0, 0.62, 0.32), vec3(1.0, 0.90, 0.70), lift);
    c += sunTint * pow(fwd, 22.0) * (0.42 + 0.36 * hz);
    c += sunTint * pow(fwd, 5.0) * (0.10 + 0.16 * (1.0 - lift));
    c += sunTint * pow(fwd, 8.0) * 0.10 * lift;
    return c;
}

// --- 3x3 PCF shadow (from mesh.frag sampleShadow) ---
float sampleShadow(vec4 lp, float ndl){
    vec3 p = lp.xyz / lp.w;
    vec2 uv = p.xy * 0.5 + 0.5;
    if(uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || p.z > 1.0) return 1.0;
    float bias = max(0.0030 * (1.0 - ndl), 0.0009);
    vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));
    float vis = 0.0;
    for(int y = -1; y <= 1; y++) for(int x = -1; x <= 1; x++){
        float d = texture(shadowMap, uv + vec2(x, y) * texel).r;
        vis += (p.z - bias > d) ? 0.0 : 1.0;
    }
    return vis / 9.0;
}

void main(){
    float t = pc.misc.x;
    vec3 V = normalize(u.camPos.xyz - vWorldPos);
    vec3 L = normalize(u.sunDir.xyz);
    vec2 w = vWorldPos.xz;

    // Animated surface normal: sum of a few sines + value-noise jitter, damped by
    // distance so far water stays calm (no aliasing). From waterShade().
    float wdist = length(u.camPos.xz - w);
    float amp = 1.0 / (1.0 + wdist * 0.018);
    float nx = 0.032 * sin(w.x * 0.55 + w.y * 0.31 + t * 1.3)
             + 0.022 * sin(w.y * 1.07 - t * 0.9)
             + 0.014 * sin((w.x + w.y) * 0.83 + t * 1.7);
    float nz = 0.032 * sin(w.y * 0.55 + w.x * 0.31 - t * 1.1)
             + 0.022 * sin(w.x * 1.07 + t * 0.8)
             + 0.014 * sin((w.x - w.y) * 0.83 - t * 1.5);
    nx += (wvn(w * 1.6 + t * 0.25) - 0.5) * 0.028;
    nz += (wvn(w * 1.6 - t * 0.21) - 0.5) * 0.028;
    vec3 N = normalize(vec3(nx * amp, 1.0, nz * amp));

    float NoV = max(dot(N, V), 0.0);

    // Fresnel-Schlick, F0 ~ 0.02 for water, clamped lower so the deep blue body
    // dominates (less pale sky reflection at grazing angles).
    float fres = clamp(0.02 + 0.98 * pow(1.0 - NoV, 5.0), 0.02, 0.58);

    // Shadowing (flat up-facing surface uses N.y ~ 1).
    float ndl = max(dot(N, L), 0.0);
    float rawSh = sampleShadow(vLightPos, ndl);

    // Refracted body: deep teal-blue, lightening toward grazing angles. A simple
    // depth proxy (how head-on we look) darkens the centre, lightens the edges.
    // REAL water depth: distance from the surface down to the lit bed already in
    // the G-buffer behind this pixel. Drives colour AND opacity (Beer-Lambert-ish),
    // so shallows are clear turquoise over the sand and deep water is opaque blue.
    vec2 suv = gl_FragCoord.xy / vec2(textureSize(gPosition, 0));
    vec4 bed = texture(gPosition, suv);
    float wdepth = (bed.a > 0.5) ? max(0.0, vWorldPos.y - bed.y) : 40.0;  // no bed -> open ocean (deep)
    float dN = 1.0 - exp(-wdepth * 0.14);                // 0 at the shore, ->1 in deep water
    vec3 deepCol    = vec3(0.008, 0.05, 0.16);           // deep ocean blue
    vec3 shallowCol = vec3(0.07,  0.34, 0.45);           // clear near-shore turquoise
    vec3 body = mix(shallowCol, deepCol, dN);
    body *= (0.55 + 0.55 * ndl * mix(0.4, 1.0, rawSh));  // sun-lit body
    body += vec3(0.30, 0.38, 0.47) * 0.04;               // tiny ground/ambient bounce

    // Sky reflection along the mirrored view direction.
    vec3 R = reflect(-V, N);
    vec3 refl = skyCol(R, L);
    refl = mix(refl, deepCol*2.2, 0.30);                 // tint reflection toward the water colour
    float sunGlow = pow(max(dot(R, L), 0.0), 8.0);
    refl = mix(refl, vec3(1.0, 0.92, 0.78), 0.25 * sunGlow);

    vec3 col = mix(body, refl, fres);

    // Sun specular glint: tight Blinn-Phong lobe, gated by shadow.
    vec3 H = normalize(L + V);
    float glint = pow(max(dot(N, H), 0.0), 200.0) * rawSh;
    col += vec3(1.0, 0.96, 0.86) * glint * 1.6;

    // Transparency: looking straight down (low fresnel) the water is clear and the
    // lit bed already in the HDR target shows through; at grazing angles it turns
    // reflective/opaque. Alpha blending composites over the terrain behind it.
    // Opacity by depth: shallows are see-through (reveal the bed in the HDR target),
    // deep water opaque; grazing reflection and glint always stay visible.
    float alpha = mix(0.16, 0.97, dN);
    alpha = max(alpha, fres*0.92);
    alpha = clamp(alpha + glint, 0.0, 1.0);
    outColor = vec4(col, alpha);
}

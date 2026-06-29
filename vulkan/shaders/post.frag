#version 450
// Post: radial god-ray shafts + composite HDR + bloom, exposure, ACES, gamma.
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;
layout(set = 0, binding = 0) uniform sampler2D hdr;
layout(set = 0, binding = 1) uniform sampler2D bloom;
layout(push_constant) uniform PC {
    float exposure; float bloomStrength;
    vec2  sunUV;        // sun position in screen UV
    float grStrength;   // god-ray strength
    float sunVis;       // 1 if the sun is in front of the camera
    vec2  pad;
} pc;

vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14), 0.0, 1.0); }
float luma(vec3 c){ return dot(c, vec3(0.2126,0.7152,0.0722)); }

// Auto-exposure / eye adaptation: geometric-mean scene luminance from a sparse
// grid drives the exposure toward a mid-grey key. (Instant; a temporal history
// would smooth the adaptation further.)
float autoExposure(){
    float logSum = 0.0; const int G = 6;
    for(int y=0;y<G;y++) for(int x=0;x<G;x++){
        vec2 guv = (vec2(x,y)+0.5)/float(G);
        logSum += log(luma(texture(hdr, guv).rgb) + 1e-4);
    }
    float avg = exp(logSum / float(G*G));
    return clamp(0.16 / max(avg, 1e-3), 0.35, 2.6);
}

void main(){
    vec3 c = texture(hdr, uv).rgb;
    c += texture(bloom, uv).rgb * pc.bloomStrength;

    // Screen-space crepuscular rays: march from this pixel toward the sun and
    // accumulate bright (sky/sun) radiance with exponential decay. Terrain along
    // the ray is dark, so it carves visible shafts radiating from the sun.
    if(pc.sunVis > 0.5){
        const int N = 64;
        vec2 delta = (pc.sunUV - uv) / float(N) * 0.92;
        vec2 p = uv;
        float decay = 0.95, w = 0.92, illum = 0.0;
        for(int i=0;i<N;i++){
            p += delta;
            if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) break;
            float l = luma(texture(hdr, p).rgb);
            illum += clamp(l-0.95, 0.0, 6.0) * w;   // only the brightest near-sun sky streaks
            w *= decay;
        }
        illum /= float(N);
        float aim = clamp(1.0 - length(pc.sunUV - uv), 0.0, 1.0); // stronger near the sun
        c += vec3(1.0,0.93,0.78) * illum * pc.grStrength * (0.4 + 0.6*aim);
    }

    c = aces(c * pc.exposure * autoExposure());
    c = pow(c, vec3(1.0/2.2));
    outColor = vec4(c, 1.0);
}

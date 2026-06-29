#version 450
// Temporal anti-aliasing resolve. Reprojection-based: the camera is jittered by a
// sub-pixel Halton offset each frame (in main.cpp), and this pass blends the
// current tonemapped frame with the reprojected history, clamped to the local
// neighbourhood to suppress ghosting. This is exactly the plumbing a DLSS backend
// consumes (jitter + motion via reprojection + history), so swapping in DLSS later
// is a drop-in for this resolve.
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D curTex;     // current tonemapped frame (jittered)
layout(set = 0, binding = 1) uniform sampler2D histTex;    // previous resolved frame
layout(set = 0, binding = 2) uniform sampler2D gPosition;  // world position + flag (.w)

layout(push_constant) uniform PC {
    mat4 prevVP;       // previous-frame UNJITTERED view-projection (for reprojection)
    vec4 camPos;       // xyz = camera position, w = feedback (0 => history invalid)
    vec4 camDir;       // xyz = forward,  w = tanHalfFovY
    vec4 camRight;     // xyz = right,    w = aspect
    vec4 camUp;        // xyz = up,       w = unused
} pc;

void main(){
    vec2 res = vec2(textureSize(curTex, 0));
    vec3 cur = texture(curTex, uv).rgb;
    float feedback = pc.camPos.w;

    // World position behind this pixel. Geometry stores it in the G-buffer; sky
    // (flag==0) has none, so reconstruct a far point along the view ray so the
    // background reprojects correctly under camera rotation.
    vec4 g = texture(gPosition, uv);
    vec3 wp;
    if(g.w > 0.5){
        wp = g.xyz;
    } else {
        vec2 ndc = uv * 2.0 - 1.0;
        vec3 rd = normalize(pc.camDir.xyz
                          + pc.camRight.xyz * ndc.x * pc.camDir.w * pc.camRight.w
                          + pc.camUp.xyz    * (-ndc.y) * pc.camDir.w);
        wp = pc.camPos.xyz + rd * 3000.0;
    }

    // Reproject into the previous frame.
    vec4 clip = pc.prevVP * vec4(wp, 1.0);
    vec2 prevUV = (clip.xy / clip.w) * 0.5 + 0.5;

    // No usable history (first frame / offscreen / reprojected off-screen) -> current.
    if(feedback < 0.001 || clip.w <= 0.0 ||
       prevUV.x < 0.0 || prevUV.x > 1.0 || prevUV.y < 0.0 || prevUV.y > 1.0){
        outColor = vec4(cur, 1.0);
        return;
    }

    vec3 hist = texture(histTex, prevUV).rgb;

    // Neighbourhood colour clamp (3x3 AABB of the current frame) — rejects stale
    // history at disocclusions/edges so moving geometry doesn't smear.
    vec3 lo = cur, hi = cur;
    vec2 t = 1.0 / res;
    for(int y = -1; y <= 1; y++)
    for(int x = -1; x <= 1; x++){
        vec3 c = texture(curTex, uv + vec2(x, y) * t).rgb;
        lo = min(lo, c); hi = max(hi, c);
    }
    hist = clamp(hist, lo, hi);

    outColor = vec4(mix(cur, hist, feedback), 1.0);
}

#version 450
// Bloom: soft-knee bright-pass + a wide cross Gaussian, rendered at half res.
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;
layout(set = 0, binding = 0) uniform sampler2D hdr;
layout(push_constant) uniform PC { vec2 texel; float threshold; float knee; } pc;

vec3 prefilter(vec3 c){
    float l = max(max(c.r,c.g), c.b);
    // soft knee around the threshold
    float soft = clamp(l - pc.threshold + pc.knee, 0.0, 2.0*pc.knee);
    soft = soft*soft / (4.0*pc.knee + 1e-4);
    float contrib = max(soft, l - pc.threshold) / max(l, 1e-4);
    return c * contrib;
}
void main(){
    vec2 t = pc.texel;
    float w[5] = float[](0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);
    vec3 sum = prefilter(texture(hdr, uv).rgb) * w[0];
    for(int i=1;i<5;i++){
        sum += prefilter(texture(hdr, uv + vec2(t.x*i, 0)).rgb) * w[i];
        sum += prefilter(texture(hdr, uv - vec2(t.x*i, 0)).rgb) * w[i];
        sum += prefilter(texture(hdr, uv + vec2(0, t.y*i)).rgb) * w[i];
        sum += prefilter(texture(hdr, uv - vec2(0, t.y*i)).rgb) * w[i];
    }
    outColor = vec4(sum, 1.0);
}

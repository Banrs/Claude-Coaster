#version 450
// G-buffer write: three render targets sampled by the deferred lighting + SS passes.
//   0  albedo            (RGBA8)    rgb = surface colour, a = unused
//   1  normal+roughness  (RGBA16F)  rgb = world normal,  a = roughness
//   2  position+flag     (RGBA16F)  rgb = world position, a = 1 (geometry present)
layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vColor;

layout(location = 0) out vec4 outAlbedo;        // rgb albedo, a = metalness
layout(location = 1) out vec4 outNormalRough;   // rgb world normal, a = roughness
layout(location = 2) out vec4 outPositionFlag;  // rgb world pos, a = 1 (geometry)

// Classify material from vertex colour: bright desaturated greys are the steel
// rails/supports (metallic, smooth); everything else is matte dielectric. This
// is what makes the coaster actually reflect the sky/sun.
void main(){
    vec3 col = vColor;
    float mx = max(max(col.r,col.g),col.b);
    float mn = min(min(col.r,col.g),col.b);
    float sat = (mx > 1e-3) ? (mx-mn)/mx : 0.0;
    float metal = 0.0, rough = 0.85;
    if(sat < 0.16 && mx > 0.58 && mx < 0.90){       // polished steel
        metal = 1.0; rough = 0.26;
    } else if(sat < 0.22 && mx >= 0.42 && mx <= 0.58){ // darker brushed metal (posts)
        metal = 0.85; rough = 0.45;
    }
    outAlbedo       = vec4(col, metal);
    outNormalRough  = vec4(normalize(vNormal), rough);
    outPositionFlag = vec4(vWorldPos, 1.0);
}

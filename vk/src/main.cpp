// MINECOASTER — Vulkan renderer.
//
// HDR(fp16) forward PBR geometry -> bloom -> ACES tonemap/post, presented to an
// SDL2 swapchain (interactive) or copied to a PPM (--shot, headless). This post
// chain + descriptor infrastructure is the base for the deferred G-buffer and the
// screen-space effects (SSAO/SSR/CSM/TAA) that follow.
#include <vulkan/vulkan.h>
#define SDL_MAIN_HANDLED          // we provide our own main() (no SDL2main / WinMain)
#include <SDL.h>                  // portable across vcpkg / pkg-config include dirs
#include <SDL_vulkan.h>

#include "Math.h"
#include "Terrain.h"
#include "Water.h"
#include "Track.h"
#include "CoasterTrack.h"
#include "Props.h"
#include "Physics.h"
#include "Hud.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <functional>

#define VK_CHECK(x) do { VkResult _r=(x); if(_r!=VK_SUCCESS){ \
    fprintf(stderr,"VK error %d at %s:%d\n",_r,__FILE__,__LINE__); exit(1);} } while(0)

static const VkFormat HDR_FMT     = VK_FORMAT_R16G16B16A16_SFLOAT;
static const VkFormat DEPTH_FMT   = VK_FORMAT_D32_SFLOAT;
static const VkFormat OUT_FMT     = VK_FORMAT_R8G8B8A8_UNORM;
static const VkFormat ALBEDO_FMT  = VK_FORMAT_R8G8B8A8_UNORM;        // G-buffer albedo
static const VkFormat NORMRGH_FMT = VK_FORMAT_R16G16B16A16_SFLOAT;   // world normal + roughness
static const VkFormat POS_FMT     = VK_FORMAT_R16G16B16A16_SFLOAT;   // world position + flag
static const VkFormat AO_FMT      = VK_FORMAT_R8_UNORM;              // SSAO visibility
static const uint32_t SHADOW_DIM  = 2048;

struct SceneUBO { Mat4 viewProj; Mat4 lightVP; float sunDir[4]; float camPos[4]; };
struct ShadowPC { Mat4 lightVP; };
struct BloomPC { float texel[2]; float threshold; float knee; };
struct PostPC  { float exposure; float bloomStrength; float sunUV[2]; float grStrength; float sunVis; float pad[2]; };
struct LightPC { float camDir[4]; float camRight[4]; float camUp[4]; float params[4]; }; // params: tanHalfFovY, aspect, time, 0
struct WaterPC { float misc[4]; }; // misc.x = time
struct HudPC   { float screen[2]; float pad[2]; }; // framebuffer size in pixels
// optional --cam override for framing headless verification shots
static bool g_camSet=false; static Vec3 g_camPos; static float g_camYaw=0, g_camPitch=0;

static std::vector<char> readFile(const std::string& p){
    FILE* f=fopen(p.c_str(),"rb"); if(!f){ fprintf(stderr,"cannot open %s\n",p.c_str()); exit(1);}
    fseek(f,0,SEEK_END); long n=ftell(f); fseek(f,0,SEEK_SET);
    std::vector<char> b(n); size_t rd=fread(b.data(),1,n,f); (void)rd; fclose(f); return b;
}
static uint32_t findMemType(VkPhysicalDevice pd, uint32_t bits, VkMemoryPropertyFlags want){
    VkPhysicalDeviceMemoryProperties mp; vkGetPhysicalDeviceMemoryProperties(pd,&mp);
    for(uint32_t i=0;i<mp.memoryTypeCount;i++)
        if((bits&(1u<<i)) && (mp.memoryTypes[i].propertyFlags&want)==want) return i;
    fprintf(stderr,"no memory type\n"); exit(1);
}
struct Buffer { VkBuffer buf=VK_NULL_HANDLE; VkDeviceMemory mem=VK_NULL_HANDLE; };
static Buffer makeBuffer(VkPhysicalDevice pd, VkDevice dev, VkDeviceSize size,
                         VkBufferUsageFlags usage, VkMemoryPropertyFlags props){
    Buffer b; VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size=size; bi.usage=usage; bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
    VK_CHECK(vkCreateBuffer(dev,&bi,nullptr,&b.buf));
    VkMemoryRequirements mr; vkGetBufferMemoryRequirements(dev,b.buf,&mr);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize=mr.size; ai.memoryTypeIndex=findMemType(pd,mr.memoryTypeBits,props);
    VK_CHECK(vkAllocateMemory(dev,&ai,nullptr,&b.mem));
    VK_CHECK(vkBindBufferMemory(dev,b.buf,b.mem,0)); return b;
}
static VkShaderModule makeShader(VkDevice dev, const std::string& path){
    auto code=readFile(path); VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize=code.size(); ci.pCode=reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule m; VK_CHECK(vkCreateShaderModule(dev,&ci,nullptr,&m)); return m;
}

struct Img { VkImage img=VK_NULL_HANDLE; VkDeviceMemory mem=VK_NULL_HANDLE; VkImageView view=VK_NULL_HANDLE; };
static Img makeImg(VkPhysicalDevice pd, VkDevice dev, uint32_t W, uint32_t H,
                   VkFormat fmt, VkImageUsageFlags usage, VkImageAspectFlags aspect){
    Img r{}; VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ii.imageType=VK_IMAGE_TYPE_2D; ii.format=fmt; ii.extent={W,H,1}; ii.mipLevels=1; ii.arrayLayers=1;
    ii.samples=VK_SAMPLE_COUNT_1_BIT; ii.tiling=VK_IMAGE_TILING_OPTIMAL; ii.usage=usage;
    VK_CHECK(vkCreateImage(dev,&ii,nullptr,&r.img));
    VkMemoryRequirements mr; vkGetImageMemoryRequirements(dev,r.img,&mr);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; ai.allocationSize=mr.size;
    ai.memoryTypeIndex=findMemType(pd,mr.memoryTypeBits,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK(vkAllocateMemory(dev,&ai,nullptr,&r.mem)); VK_CHECK(vkBindImageMemory(dev,r.img,r.mem,0));
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO}; vi.image=r.img; vi.viewType=VK_IMAGE_VIEW_TYPE_2D;
    vi.format=fmt; vi.subresourceRange={aspect,0,1,0,1}; VK_CHECK(vkCreateImageView(dev,&vi,nullptr,&r.view)); return r;
}
static void destroyImg(VkDevice dev, Img& i){
    if(i.view) vkDestroyImageView(dev,i.view,nullptr);
    if(i.img)  vkDestroyImage(dev,i.img,nullptr);
    if(i.mem)  vkFreeMemory(dev,i.mem,nullptr);
    i={};
}

// render pass: one color attachment (+ optional depth), with begin/end external
// dependencies so a following fullscreen pass can sample the result.
static VkRenderPass makeRP(VkDevice dev, VkFormat color, VkImageLayout finalColor, bool withDepth){
    VkAttachmentDescription a[2]{}; uint32_t na=1;
    a[0].format=color; a[0].samples=VK_SAMPLE_COUNT_1_BIT;
    a[0].loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; a[0].storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a[0].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[0].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a[0].initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; a[0].finalLayout=finalColor;
    VkAttachmentReference cr{0,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference dr{1,VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription s{}; s.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS;
    s.colorAttachmentCount=1; s.pColorAttachments=&cr;
    if(withDepth){
        a[1].format=DEPTH_FMT; a[1].samples=VK_SAMPLE_COUNT_1_BIT;
        a[1].loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; a[1].storeOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
        a[1].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[1].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
        a[1].initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; a[1].finalLayout=VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        s.pDepthStencilAttachment=&dr; na=2;
    }
    VkSubpassDependency dep[2]{};
    dep[0].srcSubpass=VK_SUBPASS_EXTERNAL; dep[0].dstSubpass=0;
    dep[0].srcStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[0].srcAccessMask=VK_ACCESS_SHADER_READ_BIT;
    dep[0].dstStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT|VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep[0].dstAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].srcSubpass=0; dep[1].dstSubpass=VK_SUBPASS_EXTERNAL;
    dep[1].srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; dep[1].srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dep[1].dstStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT;
    dep[1].dstAccessMask=VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_TRANSFER_READ_BIT;
    VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rp.attachmentCount=na; rp.pAttachments=a; rp.subpassCount=1; rp.pSubpasses=&s; rp.dependencyCount=2; rp.pDependencies=dep;
    VkRenderPass out; VK_CHECK(vkCreateRenderPass(dev,&rp,nullptr,&out)); return out;
}

static VkPipeline makeFsPipe(VkDevice dev, VkRenderPass rp, VkPipelineLayout layout, VkShaderModule vs, VkShaderModule fs){
    VkPipelineShaderStageCreateInfo st[2]{};
    st[0]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st[0].module=vs; st[0].pName="main";
    st[1]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module=fs; st[1].pName="main";
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
    VkDynamicState dyn[2]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; ds.dynamicStateCount=2; ds.pDynamicStates=dyn;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.lineWidth=1.0f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState cba{}; cba.colorWriteMask=0xF;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&cba;
    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount=2; gp.pStages=st; gp.pVertexInputState=&vin; gp.pInputAssemblyState=&ia; gp.pViewportState=&vp;
    gp.pRasterizationState=&rs; gp.pMultisampleState=&ms; gp.pColorBlendState=&cb; gp.pDynamicState=&ds;
    gp.layout=layout; gp.renderPass=rp; gp.subpass=0;
    VkPipeline pipe; VK_CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&gp,nullptr,&pipe)); return pipe;
}

static VkRenderPass makeShadowRP(VkDevice dev){
    VkAttachmentDescription a{}; a.format=DEPTH_FMT; a.samples=VK_SAMPLE_COUNT_1_BIT;
    a.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; a.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a.stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a.stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a.initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; a.finalLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkAttachmentReference dr{0,VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription s{}; s.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS; s.pDepthStencilAttachment=&dr;
    VkSubpassDependency dep[2]{};
    dep[0].srcSubpass=VK_SUBPASS_EXTERNAL; dep[0].dstSubpass=0;
    dep[0].srcStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[0].srcAccessMask=VK_ACCESS_SHADER_READ_BIT;
    dep[0].dstStageMask=VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT; dep[0].dstAccessMask=VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].srcSubpass=0; dep[1].dstSubpass=VK_SUBPASS_EXTERNAL;
    dep[1].srcStageMask=VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT; dep[1].srcAccessMask=VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].dstStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[1].dstAccessMask=VK_ACCESS_SHADER_READ_BIT;
    VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO}; rp.attachmentCount=1; rp.pAttachments=&a; rp.subpassCount=1; rp.pSubpasses=&s; rp.dependencyCount=2; rp.pDependencies=dep;
    VkRenderPass out; VK_CHECK(vkCreateRenderPass(dev,&rp,nullptr,&out)); return out;
}
static VkPipeline makeShadowPipe(VkDevice dev, VkRenderPass rp, VkPipelineLayout layout, VkShaderModule vs){
    VkPipelineShaderStageCreateInfo st{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st.stage=VK_SHADER_STAGE_VERTEX_BIT; st.module=vs; st.pName="main";
    VkVertexInputBindingDescription bind{0,sizeof(Vertex),VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription at{0,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,pos)};
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vin.vertexBindingDescriptionCount=1; vin.pVertexBindingDescriptions=&bind; vin.vertexAttributeDescriptionCount=1; vin.pVertexAttributeDescriptions=&at;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
    VkDynamicState dyn[2]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; ds.dynamicStateCount=2; ds.pDynamicStates=dyn;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.lineWidth=1.0f;
    rs.depthBiasEnable=VK_TRUE; rs.depthBiasConstantFactor=1.25f; rs.depthBiasSlopeFactor=1.75f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depth.depthTestEnable=VK_TRUE; depth.depthWriteEnable=VK_TRUE; depth.depthCompareOp=VK_COMPARE_OP_LESS;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=0;
    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount=1; gp.pStages=&st; gp.pVertexInputState=&vin; gp.pInputAssemblyState=&ia; gp.pViewportState=&vp;
    gp.pRasterizationState=&rs; gp.pMultisampleState=&ms; gp.pDepthStencilState=&depth; gp.pColorBlendState=&cb; gp.pDynamicState=&ds;
    gp.layout=layout; gp.renderPass=rp; gp.subpass=0;
    VkPipeline pipe; VK_CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&gp,nullptr,&pipe)); return pipe;
}

// G-buffer render pass: 3 colour MRT (albedo, normal+rough, position+flag) + depth.
// All colours end SHADER_READ_ONLY (sampled by SSAO + lighting); depth is stored
// (DEPTH_STENCIL_ATTACHMENT_OPTIMAL) so the forward water pass can depth-test later.
static VkRenderPass makeGBufRP(VkDevice dev){
    VkAttachmentDescription a[4]{};
    VkFormat cf[3]={ALBEDO_FMT,NORMRGH_FMT,POS_FMT};
    for(int i=0;i<3;i++){
        a[i].format=cf[i]; a[i].samples=VK_SAMPLE_COUNT_1_BIT;
        a[i].loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; a[i].storeOp=VK_ATTACHMENT_STORE_OP_STORE;
        a[i].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[i].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
        a[i].initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; a[i].finalLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }
    a[3].format=DEPTH_FMT; a[3].samples=VK_SAMPLE_COUNT_1_BIT;
    a[3].loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; a[3].storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a[3].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[3].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a[3].initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; a[3].finalLayout=VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    VkAttachmentReference cr[3]={{0,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL},
        {1,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL},{2,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL}};
    VkAttachmentReference dr{3,VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription s{}; s.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS;
    s.colorAttachmentCount=3; s.pColorAttachments=cr; s.pDepthStencilAttachment=&dr;
    VkSubpassDependency dep[2]{};
    dep[0].srcSubpass=VK_SUBPASS_EXTERNAL; dep[0].dstSubpass=0;
    dep[0].srcStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[0].srcAccessMask=VK_ACCESS_SHADER_READ_BIT;
    dep[0].dstStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT|VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep[0].dstAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].srcSubpass=0; dep[1].dstSubpass=VK_SUBPASS_EXTERNAL;
    dep[1].srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT|VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dep[1].srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].dstStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[1].dstAccessMask=VK_ACCESS_SHADER_READ_BIT;
    VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rp.attachmentCount=4; rp.pAttachments=a; rp.subpassCount=1; rp.pSubpasses=&s; rp.dependencyCount=2; rp.pDependencies=dep;
    VkRenderPass out; VK_CHECK(vkCreateRenderPass(dev,&rp,nullptr,&out)); return out;
}
// MRT geometry pipeline: full Vertex input, depth test+write, 3 colour blend states.
static VkPipeline makeGBufPipe(VkDevice dev, VkRenderPass rp, VkPipelineLayout layout, VkShaderModule vs, VkShaderModule fs){
    VkPipelineShaderStageCreateInfo st[2]{};
    st[0]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st[0].module=vs; st[0].pName="main";
    st[1]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module=fs; st[1].pName="main";
    VkVertexInputBindingDescription bind{0,sizeof(Vertex),VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription at[3]={{0,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,pos)},
        {1,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,nrm)},{2,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,col)}};
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vin.vertexBindingDescriptionCount=1; vin.pVertexBindingDescriptions=&bind; vin.vertexAttributeDescriptionCount=3; vin.pVertexAttributeDescriptions=at;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
    VkDynamicState dyn[2]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; ds.dynamicStateCount=2; ds.pDynamicStates=dyn;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth=1.0f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depth.depthTestEnable=VK_TRUE; depth.depthWriteEnable=VK_TRUE; depth.depthCompareOp=VK_COMPARE_OP_LESS;
    VkPipelineColorBlendAttachmentState cba[3]{}; for(int i=0;i<3;i++) cba[i].colorWriteMask=0xF;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=3; cb.pAttachments=cba;
    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount=2; gp.pStages=st; gp.pVertexInputState=&vin; gp.pInputAssemblyState=&ia; gp.pViewportState=&vp;
    gp.pRasterizationState=&rs; gp.pMultisampleState=&ms; gp.pDepthStencilState=&depth; gp.pColorBlendState=&cb; gp.pDynamicState=&ds;
    gp.layout=layout; gp.renderPass=rp; gp.subpass=0;
    VkPipeline pipe; VK_CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&gp,nullptr,&pipe)); return pipe;
}

// Forward water render pass: composites over the lit HDR (LOAD) and depth-tests
// against the G-buffer depth (LOAD) so water is occluded by terrain/props.
static VkRenderPass makeWaterRP(VkDevice dev){
    VkAttachmentDescription a[2]{};
    a[0].format=HDR_FMT; a[0].samples=VK_SAMPLE_COUNT_1_BIT;
    a[0].loadOp=VK_ATTACHMENT_LOAD_OP_LOAD; a[0].storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a[0].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[0].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a[0].initialLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; a[0].finalLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    a[1].format=DEPTH_FMT; a[1].samples=VK_SAMPLE_COUNT_1_BIT;
    a[1].loadOp=VK_ATTACHMENT_LOAD_OP_LOAD; a[1].storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a[1].stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a[1].stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a[1].initialLayout=VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL; a[1].finalLayout=VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    VkAttachmentReference cr{0,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference dr{1,VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription s{}; s.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS; s.colorAttachmentCount=1; s.pColorAttachments=&cr; s.pDepthStencilAttachment=&dr;
    VkSubpassDependency dep[2]{};
    dep[0].srcSubpass=VK_SUBPASS_EXTERNAL; dep[0].dstSubpass=0;
    dep[0].srcStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT|VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dep[0].srcAccessMask=VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[0].dstStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT|VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep[0].dstAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT|VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep[1].srcSubpass=0; dep[1].dstSubpass=VK_SUBPASS_EXTERNAL;
    dep[1].srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; dep[1].srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dep[1].dstStageMask=VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; dep[1].dstAccessMask=VK_ACCESS_SHADER_READ_BIT;
    VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO}; rp.attachmentCount=2; rp.pAttachments=a; rp.subpassCount=1; rp.pSubpasses=&s; rp.dependencyCount=2; rp.pDependencies=dep;
    VkRenderPass out; VK_CHECK(vkCreateRenderPass(dev,&rp,nullptr,&out)); return out;
}
// Forward water pipeline: full Vertex input, depth test (no write), one HDR target.
static VkPipeline makeWaterPipe(VkDevice dev, VkRenderPass rp, VkPipelineLayout layout, VkShaderModule vs, VkShaderModule fs){
    VkPipelineShaderStageCreateInfo st[2]{};
    st[0]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st[0].module=vs; st[0].pName="main";
    st[1]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module=fs; st[1].pName="main";
    VkVertexInputBindingDescription bind{0,sizeof(Vertex),VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription at[3]={{0,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,pos)},
        {1,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,nrm)},{2,0,VK_FORMAT_R32G32B32_SFLOAT,offsetof(Vertex,col)}};
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vin.vertexBindingDescriptionCount=1; vin.pVertexBindingDescriptions=&bind; vin.vertexAttributeDescriptionCount=3; vin.pVertexAttributeDescriptions=at;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
    VkDynamicState dyn[2]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; ds.dynamicStateCount=2; ds.pDynamicStates=dyn;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth=1.0f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depth.depthTestEnable=VK_TRUE; depth.depthWriteEnable=VK_FALSE; depth.depthCompareOp=VK_COMPARE_OP_LESS;
    VkPipelineColorBlendAttachmentState cba{}; cba.colorWriteMask=0xF;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&cba;
    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount=2; gp.pStages=st; gp.pVertexInputState=&vin; gp.pInputAssemblyState=&ia; gp.pViewportState=&vp;
    gp.pRasterizationState=&rs; gp.pMultisampleState=&ms; gp.pDepthStencilState=&depth; gp.pColorBlendState=&cb; gp.pDynamicState=&ds;
    gp.layout=layout; gp.renderPass=rp; gp.subpass=0;
    VkPipeline pipe; VK_CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&gp,nullptr,&pipe)); return pipe;
}

// HUD overlay render pass: alpha-blend text onto the final image (LOAD), then
// hand it to the blit/copy as TRANSFER_SRC.
static VkRenderPass makeHudRP(VkDevice dev){
    VkAttachmentDescription a{}; a.format=OUT_FMT; a.samples=VK_SAMPLE_COUNT_1_BIT;
    a.loadOp=VK_ATTACHMENT_LOAD_OP_LOAD; a.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
    a.stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE; a.stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE;
    a.initialLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; a.finalLayout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    VkAttachmentReference cr{0,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription s{}; s.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS; s.colorAttachmentCount=1; s.pColorAttachments=&cr;
    VkSubpassDependency dep[2]{};
    dep[0].srcSubpass=VK_SUBPASS_EXTERNAL; dep[0].dstSubpass=0;
    dep[0].srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; dep[0].srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dep[0].dstStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; dep[0].dstAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT|VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
    dep[1].srcSubpass=0; dep[1].dstSubpass=VK_SUBPASS_EXTERNAL;
    dep[1].srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; dep[1].srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dep[1].dstStageMask=VK_PIPELINE_STAGE_TRANSFER_BIT; dep[1].dstAccessMask=VK_ACCESS_TRANSFER_READ_BIT;
    VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO}; rp.attachmentCount=1; rp.pAttachments=&a; rp.subpassCount=1; rp.pSubpasses=&s; rp.dependencyCount=2; rp.pDependencies=dep;
    VkRenderPass out; VK_CHECK(vkCreateRenderPass(dev,&rp,nullptr,&out)); return out;
}
// Textured-quad pipeline for HUD glyphs: pos(vec2)+uv(vec2)+col(vec3), alpha blend.
static VkPipeline makeHudPipe(VkDevice dev, VkRenderPass rp, VkPipelineLayout layout, VkShaderModule vs, VkShaderModule fs){
    VkPipelineShaderStageCreateInfo st[2]{};
    st[0]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st[0].module=vs; st[0].pName="main";
    st[1]={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}; st[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module=fs; st[1].pName="main";
    VkVertexInputBindingDescription bind{0,sizeof(hud::Vtx),VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription at[3]={{0,0,VK_FORMAT_R32G32_SFLOAT,offsetof(hud::Vtx,x)},
        {1,0,VK_FORMAT_R32G32_SFLOAT,offsetof(hud::Vtx,u)},{2,0,VK_FORMAT_R32G32B32A32_SFLOAT,offsetof(hud::Vtx,r)}};
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vin.vertexBindingDescriptionCount=1; vin.pVertexBindingDescriptions=&bind; vin.vertexAttributeDescriptionCount=3; vin.pVertexAttributeDescriptions=at;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
    VkDynamicState dyn[2]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; ds.dynamicStateCount=2; ds.pDynamicStates=dyn;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.lineWidth=1.0f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState cba{}; cba.colorWriteMask=0xF; cba.blendEnable=VK_TRUE;
    cba.srcColorBlendFactor=VK_BLEND_FACTOR_SRC_ALPHA; cba.dstColorBlendFactor=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA; cba.colorBlendOp=VK_BLEND_OP_ADD;
    cba.srcAlphaBlendFactor=VK_BLEND_FACTOR_ONE; cba.dstAlphaBlendFactor=VK_BLEND_FACTOR_ZERO; cba.alphaBlendOp=VK_BLEND_OP_ADD;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&cba;
    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount=2; gp.pStages=st; gp.pVertexInputState=&vin; gp.pInputAssemblyState=&ia; gp.pViewportState=&vp;
    gp.pRasterizationState=&rs; gp.pMultisampleState=&ms; gp.pColorBlendState=&cb; gp.pDynamicState=&ds;
    gp.layout=layout; gp.renderPass=rp; gp.subpass=0;
    VkPipeline pipe; VK_CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&gp,nullptr,&pipe)); return pipe;
}

// ---- world ----
struct World { Mesh mesh; Mesh water; Vec3 focus; ::Track trk; };
static const float PATCH_HALF = 175.0f;   // terrain patch radius around the active centre
// (Re)build the terrain/trees/track/water meshes around an arbitrary centre.
// terrainH(x,z) is a pure function of world position, so re-centring just slides
// the rendered window over the same infinite world — this is what makes terrain
// (and the on-demand-generated coaster) effectively endless / streaming.
static void buildWorldMeshes(World& w, Vec3 center){
    w.mesh.verts.clear(); w.mesh.idx.clear();
    float half=PATCH_HALF; w.focus=center;
    world::buildTerrain(center.x,center.z,half,1.0f,w.mesh);
    world::buildTrees(center.x,center.z,half,w.mesh);
    world::buildTrackMesh(w.trk,w.mesh,center.x,center.z,half);
    world::buildStation(w.trk,w.mesh,center.x,center.z,half);
    world::buildCoins(w.trk,w.mesh,center.x,center.z,half);
    w.water=world::buildWaterMesh(center.x,center.z,1600.0f,120);     // ocean follows the centre
}
static World buildWorld(){
    World w; world::genLongTrack(w.trk,2200);
    buildWorldMeshes(w, world::trackFocus(w.trk,260));
    printf("[vk] world mesh: %zu verts, %zu tris; water: %zu tris\n",
           w.mesh.verts.size(), w.mesh.idx.size()/3, w.water.idx.size()/3);
    printf("[vk] focus: %.1f %.1f %.1f\n", w.focus.x, w.focus.y, w.focus.z);
    return w;
}
// A concrete camera (position + orthonormal basis) the renderer consumes. Both
// the free-fly camera and the on-rails ride camera resolve to one of these.
struct CamView {
    Vec3 pos{0,0,0}, fwd{0,0,1}, up{0,1,0}; float fovy=1.05f;
    Mat4 viewProj(float a) const { return mul(perspectiveVk(fovy,a,0.4f,4000.0f), lookAt(pos,pos+fwd,up)); }
};
struct FlyCam {
    Vec3 pos; float yaw=-2.2f, pitch=-0.32f;
    Vec3 fwd() const { float cp=cosf(pitch),sp=sinf(pitch),cy=cosf(yaw),sy=sinf(yaw); return normalize(Vec3{cp*sy,sp,cp*cy}); }
    CamView view() const { CamView v; v.pos=pos; v.fwd=fwd(); v.up={0,1,0}; return v; }
};

// ---- on-rails ride camera + telemetry (drives the HUD) ----
// Friendly element names matching the base game's OpenGL HUD (src/main.cpp).
static const char* RIDE_NM[M_COUNT] = {"","TOP HAT","DROP","AIRTIME HILL","OVERBANKED TURN",
    "VERTICAL LOOP","CORKSCREW","STATION","SPLASHDOWN","LAUNCH","HELIX","BOOSTER","IMMELMANN",
    "S-CURVE","DIVE TURN","BANKED AIRTIME","WAVE TURN","ZERO-G STALL","DIVE LOOP","COBRA ROLL",
    "WING-OVER","HEARTLINE ROLL","PRETZEL LOOP","STENGEL DIVE","BANANA ROLL"};
static bool isSpecialElem(int k){
    return k==M_LOOP||k==M_ROLL||k==M_IMMEL||k==M_STALL||k==M_DIVELOOP||k==M_COBRA||
           k==M_HEARTLINE||k==M_WINGOVER||k==M_PRETZEL||k==M_STENGEL||k==M_BANANA;
}
struct RideTelemetry { float speedKmh=0, altitude=0, gForce=1; const char* element=""; bool special=false; };
static CamView rideCamView(const ::Track& trk, const world::RideSim& rs, RideTelemetry& tel){
    Vector3 P,fwd,up,right; rs.frame(trk,P,fwd,up,right);
    Vec3 vP=world::v3(P), vF=normalize(world::v3(fwd)), vU=normalize(world::v3(up));
    CamView c; c.pos = vP + vU*2.3f - vF*0.6f;     // seat just above & behind the rail point
    c.fwd = vF; c.up = vU; c.fovy = 1.18f;          // a touch wider for the sense of speed
    tel.speedKmh = rs.v*3.6f; tel.altitude = P.y - groundTopAt(P.x,P.z); tel.gForce = rs.feltG(trk);
    int k=(int)rs.u; int n=(int)trk.kind.size();
    if(n>0){ if(k<0)k=0; if(k>=n)k=n-1; int kind=trk.kind[k];
        tel.element=(kind>=0&&kind<M_COUNT)?RIDE_NM[kind]:""; tel.special=isSpecialElem(kind); }
    return c;
}
// optional --ride <seconds> for headless ride-camera verification
static bool g_rideSet=false; static float g_rideSec=0.0f;

// =====================================================================
// Renderer: HDR scene -> bloom -> post, into outImg (TRANSFER_SRC)
// =====================================================================
struct Renderer {
    VkPhysicalDevice pd; VkDevice dev; VkQueue queue; uint32_t gfx;
    VkExtent2D ext{}, halfExt{};
    VkSampler samp;
    VkRenderPass gbufRP, ssaoRP, lightRP, bloomRP, postRP;
    VkPipelineLayout gbufLayout, ssaoLayout, lightLayout, bloomLayout, postLayout;
    VkPipeline gbufPipe, ssaoPipe, lightPipe, bloomPipe, postPipe;
    VkDescriptorSetLayout dsl1, dsl2, ssaoDSL, lightDSL; VkDescriptorPool dpool;
    VkDescriptorSet bloomSet, postSet, ssaoSet, lightSet;
    Img gAlbedo, gNormal, gPosition, ssaoImg, hdr, hdr2, bloom, out;
    // SSR (reflect the lit scene in metallic surfaces) — ping-pong hdr -> hdr2
    VkRenderPass ssrRP; VkPipelineLayout ssrLayout; VkPipeline ssrPipe; VkFramebuffer ssrFB;
    VkDescriptorSetLayout ssrDSL; VkDescriptorSet ssrSet;
    VkImage depthImg; VkDeviceMemory depthMem; VkImageView depthView;
    VkFramebuffer gbufFB, ssaoFB, lightFB, bloomFB, postFB;
    Buffer vb, ib; uint32_t indexCount;
    // forward water pass (separate mesh, depth-tested against the G-buffer depth)
    VkRenderPass waterRP; VkPipelineLayout waterLayout; VkPipeline waterPipe; VkFramebuffer waterFB;
    Buffer wvb, wib; uint32_t waterIndexCount=0;
    // HUD overlay (alpha-blended text on the final image)
    VkRenderPass hudRP; VkPipelineLayout hudLayout; VkPipeline hudPipe; VkFramebuffer hudFB;
    VkDescriptorSetLayout hudDSL; VkDescriptorSet hudSet; Img fontImg;
    Buffer hudVB; void* hudVBmap=nullptr; uint32_t hudVtxCount=0, hudVtxCap=0;
    RideTelemetry hudTel; bool hudRide=false;
    // shadows + scene UBO
    VkRenderPass shadowRP; VkPipelineLayout shadowLayout; VkPipeline shadowPipe; VkFramebuffer shadowFB;
    Img shadow; VkSampler shadowSamp;
    VkDescriptorSetLayout sceneDSL; VkDescriptorSet sceneSet; Buffer sceneUBO; void* sceneUBOmap=nullptr;
    Mat4 lightVP; Vec3 center{}; float timeSec=0.0f;

    void setCenter(Vec3 c){
        center=c;
        Vec3 sun=normalize(Vec3{-0.48f,0.60f,0.64f});
        Vec3 eye=center + sun*400.0f;
        Mat4 lview=lookAt(eye, center, Vec3{0,1,0});
        lightVP = mul(orthoVk(260.0f,260.0f,1.0f,800.0f), lview);
    }

    void initPipelines(const std::string& sd){
        VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        si.magFilter=si.minFilter=VK_FILTER_LINEAR; si.addressModeU=si.addressModeV=si.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        VK_CHECK(vkCreateSampler(dev,&si,nullptr,&samp));
        VkSamplerCreateInfo ssi{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        ssi.magFilter=ssi.minFilter=VK_FILTER_LINEAR; ssi.addressModeU=ssi.addressModeV=ssi.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        ssi.borderColor=VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;     // outside the map = lit
        VK_CHECK(vkCreateSampler(dev,&ssi,nullptr,&shadowSamp));
        gbufRP =makeGBufRP(dev);
        ssaoRP =makeRP(dev,AO_FMT,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,false);
        lightRP=makeRP(dev,HDR_FMT,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,false);
        ssrRP  =makeRP(dev,HDR_FMT,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,false);
        waterRP=makeWaterRP(dev);
        bloomRP=makeRP(dev,HDR_FMT,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,false);
        postRP =makeRP(dev,OUT_FMT,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,false);  // HUD pass draws on top next
        hudRP  =makeHudRP(dev);
        shadowRP=makeShadowRP(dev);
        auto mkDSL=[&](uint32_t n){ std::vector<VkDescriptorSetLayoutBinding> b(n);
            for(uint32_t i=0;i<n;i++){ b[i]={i,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr}; }
            VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO}; ci.bindingCount=n; ci.pBindings=b.data();
            VkDescriptorSetLayout l; VK_CHECK(vkCreateDescriptorSetLayout(dev,&ci,nullptr,&l)); return l; };
        dsl1=mkDSL(1); dsl2=mkDSL(2);
        // UBO at binding 0 (vert+frag) followed by (n-1) fragment samplers at 1..n-1
        auto mkUboDSL=[&](uint32_t nSamp){ std::vector<VkDescriptorSetLayoutBinding> b(nSamp+1);
            b[0]={0,VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr};
            for(uint32_t i=0;i<nSamp;i++) b[i+1]={i+1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr};
            VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO}; ci.bindingCount=nSamp+1; ci.pBindings=b.data();
            VkDescriptorSetLayout l; VK_CHECK(vkCreateDescriptorSetLayout(dev,&ci,nullptr,&l)); return l; };
        sceneDSL=mkUboDSL(1);   // UBO + shadow                 (G-buffer + water passes)
        ssaoDSL =mkUboDSL(2);   // UBO + gPosition + gNormal    (SSAO pass)
        lightDSL=mkUboDSL(5);   // UBO + shadow + albedo+normal+position + ssao (lighting)
        ssrDSL  =mkUboDSL(4);   // UBO + litColor + position + normal + albedo  (SSR)
        VkDescriptorPoolSize ps[2]={{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,32},{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,8}};
        VkDescriptorPoolCreateInfo pci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO}; pci.maxSets=12; pci.poolSizeCount=2; pci.pPoolSizes=ps;
        VK_CHECK(vkCreateDescriptorPool(dev,&pci,nullptr,&dpool));
        auto mkPL=[&](VkDescriptorSetLayout dsl, uint32_t pcSize, VkShaderStageFlags pcStage){
            VkPushConstantRange pcr{pcStage,0,pcSize};
            VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            if(dsl){ pli.setLayoutCount=1; pli.pSetLayouts=&dsl; }
            if(pcSize){ pli.pushConstantRangeCount=1; pli.pPushConstantRanges=&pcr; }
            VkPipelineLayout l; VK_CHECK(vkCreatePipelineLayout(dev,&pli,nullptr,&l)); return l; };
        gbufLayout  =mkPL(sceneDSL,0,0);
        ssaoLayout  =mkPL(ssaoDSL,0,0);
        waterLayout =mkPL(sceneDSL,sizeof(WaterPC),VK_SHADER_STAGE_FRAGMENT_BIT);
        ssrLayout   =mkPL(ssrDSL,0,0);
        lightLayout =mkPL(lightDSL,sizeof(LightPC),VK_SHADER_STAGE_FRAGMENT_BIT);
        shadowLayout=mkPL(VK_NULL_HANDLE,sizeof(ShadowPC),VK_SHADER_STAGE_VERTEX_BIT);
        bloomLayout =mkPL(dsl1,sizeof(BloomPC),VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT);
        postLayout  =mkPL(dsl2,sizeof(PostPC),VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT);
        VkShaderModule gv=makeShader(dev,sd+"/gbuffer.vert.spv"), gf=makeShader(dev,sd+"/gbuffer.frag.spv");
        VkShaderModule fv=makeShader(dev,sd+"/fullscreen.vert.spv");
        VkShaderModule af=makeShader(dev,sd+"/ssao.frag.spv"), lf=makeShader(dev,sd+"/lighting.frag.spv");
        VkShaderModule bf=makeShader(dev,sd+"/bloom.frag.spv"), pf=makeShader(dev,sd+"/post.frag.spv");
        VkShaderModule sv=makeShader(dev,sd+"/shadow.vert.spv");
        VkShaderModule wv=makeShader(dev,sd+"/water.vert.spv"), wf=makeShader(dev,sd+"/water.frag.spv");
        VkShaderModule rf=makeShader(dev,sd+"/ssr.frag.spv");
        gbufPipe  =makeGBufPipe(dev,gbufRP,gbufLayout,gv,gf);
        ssaoPipe  =makeFsPipe(dev,ssaoRP,ssaoLayout,fv,af);
        lightPipe =makeFsPipe(dev,lightRP,lightLayout,fv,lf);
        ssrPipe   =makeFsPipe(dev,ssrRP,ssrLayout,fv,rf);
        waterPipe =makeWaterPipe(dev,waterRP,waterLayout,wv,wf);
        shadowPipe=makeShadowPipe(dev,shadowRP,shadowLayout,sv);
        bloomPipe =makeFsPipe(dev,bloomRP,bloomLayout,fv,bf);
        postPipe  =makeFsPipe(dev,postRP,postLayout,fv,pf);
        for(auto m:{gv,gf,fv,af,lf,bf,pf,sv,wv,wf,rf}) vkDestroyShaderModule(dev,m,nullptr);
        // scene UBO (persistently mapped) + shadow map image/framebuffer (fixed size)
        sceneUBO=makeBuffer(pd,dev,sizeof(SceneUBO),VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkMapMemory(dev,sceneUBO.mem,0,sizeof(SceneUBO),0,&sceneUBOmap);
        shadow=makeImg(pd,dev,SHADOW_DIM,SHADOW_DIM,DEPTH_FMT,VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT|VK_IMAGE_USAGE_SAMPLED_BIT,VK_IMAGE_ASPECT_DEPTH_BIT);
        { VkFramebufferCreateInfo fbi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO}; fbi.renderPass=shadowRP; fbi.attachmentCount=1; fbi.pAttachments=&shadow.view; fbi.width=SHADOW_DIM; fbi.height=SHADOW_DIM; fbi.layers=1;
          VK_CHECK(vkCreateFramebuffer(dev,&fbi,nullptr,&shadowFB)); }
        // descriptor sets
        auto alloc=[&](VkDescriptorSetLayout l){ VkDescriptorSet s; VkDescriptorSetAllocateInfo a{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO}; a.descriptorPool=dpool; a.descriptorSetCount=1; a.pSetLayouts=&l; VK_CHECK(vkAllocateDescriptorSets(dev,&a,&s)); return s; };
        bloomSet=alloc(dsl1); postSet=alloc(dsl2);
        sceneSet=alloc(sceneDSL); ssaoSet=alloc(ssaoDSL); lightSet=alloc(lightDSL); ssrSet=alloc(ssrDSL);
        // fixed bindings that never change on resize: the scene UBO (all three sets)
        // and the shadow map (scene + lighting sets). G-buffer/AO image bindings are
        // (re)written in buildTargets.
        VkDescriptorBufferInfo ubi{sceneUBO.buf,0,sizeof(SceneUBO)};
        VkDescriptorImageInfo smi{shadowSamp,shadow.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        auto ubo=[&](VkDescriptorSet set){ VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            w.dstSet=set; w.dstBinding=0; w.descriptorCount=1; w.descriptorType=VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; w.pBufferInfo=&ubi; return w; };
        auto img=[&](VkDescriptorSet set,uint32_t b,VkDescriptorImageInfo* ii){ VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            w.dstSet=set; w.dstBinding=b; w.descriptorCount=1; w.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; w.pImageInfo=ii; return w; };
        VkWriteDescriptorSet sw[6]={ ubo(sceneSet), img(sceneSet,1,&smi), ubo(ssaoSet), ubo(lightSet), img(lightSet,1,&smi), ubo(ssrSet) };
        vkUpdateDescriptorSets(dev,6,sw,0,nullptr);
        // ---- HUD: pipeline, font atlas texture, dynamic vertex buffer ----
        hudLayout=mkPL(dsl1,sizeof(HudPC),VK_SHADER_STAGE_VERTEX_BIT);
        { VkShaderModule hv=makeShader(dev,sd+"/hud.vert.spv"), hf=makeShader(dev,sd+"/hud.frag.spv");
          hudPipe=makeHudPipe(dev,hudRP,hudLayout,hv,hf); vkDestroyShaderModule(dev,hv,nullptr); vkDestroyShaderModule(dev,hf,nullptr); }
        hud::Atlas atlas=hud::fontAtlas();
        fontImg=makeImg(pd,dev,(uint32_t)atlas.w,(uint32_t)atlas.h,VK_FORMAT_R8_UNORM,
                        VK_IMAGE_USAGE_SAMPLED_BIT|VK_IMAGE_USAGE_TRANSFER_DST_BIT,VK_IMAGE_ASPECT_COLOR_BIT);
        { VkDeviceSize n=(VkDeviceSize)atlas.w*atlas.h;
          Buffer stg=makeBuffer(pd,dev,n,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
          void* mp; vkMapMemory(dev,stg.mem,0,n,0,&mp); memcpy(mp,atlas.pixels.data(),n); vkUnmapMemory(dev,stg.mem);
          oneTimeSubmit([&](VkCommandBuffer c){
            VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER}; b.oldLayout=VK_IMAGE_LAYOUT_UNDEFINED; b.newLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            b.srcQueueFamilyIndex=b.dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED; b.image=fontImg.img; b.subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1}; b.dstAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;
            vkCmdPipelineBarrier(c,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,0,0,nullptr,0,nullptr,1,&b);
            VkBufferImageCopy r{}; r.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1}; r.imageExtent={(uint32_t)atlas.w,(uint32_t)atlas.h,1};
            vkCmdCopyBufferToImage(c,stg.buf,fontImg.img,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,1,&r);
            b.oldLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.newLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; b.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask=VK_ACCESS_SHADER_READ_BIT;
            vkCmdPipelineBarrier(c,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,0,0,nullptr,0,nullptr,1,&b);
          });
          vkDestroyBuffer(dev,stg.buf,nullptr); vkFreeMemory(dev,stg.mem,nullptr); }
        hudSet=alloc(dsl1);
        VkDescriptorImageInfo fi{samp,fontImg.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet fw=img(hudSet,0,&fi); vkUpdateDescriptorSets(dev,1,&fw,0,nullptr);
        hudVtxCap=8192;
        hudVB=makeBuffer(pd,dev,(VkDeviceSize)hudVtxCap*sizeof(hud::Vtx),VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkMapMemory(dev,hudVB.mem,0,(VkDeviceSize)hudVtxCap*sizeof(hud::Vtx),0,&hudVBmap);
    }
    // run a short transfer/transition command synchronously (one-time setup)
    void oneTimeSubmit(const std::function<void(VkCommandBuffer)>& fn){
        VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; cpi.queueFamilyIndex=gfx; cpi.flags=VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        VkCommandPool p; VK_CHECK(vkCreateCommandPool(dev,&cpi,nullptr,&p));
        VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO}; ai.commandPool=p; ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; ai.commandBufferCount=1;
        VkCommandBuffer c; VK_CHECK(vkAllocateCommandBuffers(dev,&ai,&c));
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; bi.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(c,&bi); fn(c); vkEndCommandBuffer(c);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO}; si.commandBufferCount=1; si.pCommandBuffers=&c;
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; VkFence f; vkCreateFence(dev,&fci,nullptr,&f);
        vkQueueSubmit(queue,1,&si,f); vkWaitForFences(dev,1,&f,VK_TRUE,UINT64_MAX);
        vkDestroyFence(dev,f,nullptr); vkDestroyCommandPool(dev,p,nullptr);
    }
    void uploadMesh(const World& w){
        indexCount=(uint32_t)w.mesh.idx.size();
        VkDeviceSize vbS=w.mesh.verts.size()*sizeof(Vertex), ibS=w.mesh.idx.size()*sizeof(uint32_t);
        vb=makeBuffer(pd,dev,vbS,VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        ib=makeBuffer(pd,dev,ibS,VK_BUFFER_USAGE_INDEX_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        void* p; vkMapMemory(dev,vb.mem,0,vbS,0,&p); memcpy(p,w.mesh.verts.data(),vbS); vkUnmapMemory(dev,vb.mem);
        vkMapMemory(dev,ib.mem,0,ibS,0,&p); memcpy(p,w.mesh.idx.data(),ibS); vkUnmapMemory(dev,ib.mem);
        // separate water mesh
        waterIndexCount=(uint32_t)w.water.idx.size();
        if(waterIndexCount){
            VkDeviceSize wvbS=w.water.verts.size()*sizeof(Vertex), wibS=w.water.idx.size()*sizeof(uint32_t);
            wvb=makeBuffer(pd,dev,wvbS,VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            wib=makeBuffer(pd,dev,wibS,VK_BUFFER_USAGE_INDEX_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            vkMapMemory(dev,wvb.mem,0,wvbS,0,&p); memcpy(p,w.water.verts.data(),wvbS); vkUnmapMemory(dev,wvb.mem);
            vkMapMemory(dev,wib.mem,0,wibS,0,&p); memcpy(p,w.water.idx.data(),wibS); vkUnmapMemory(dev,wib.mem);
        }
    }
    // Re-upload after the world re-bakes around a new centre (streaming).
    void reuploadMesh(const World& w){
        vkDeviceWaitIdle(dev);
        vkDestroyBuffer(dev,vb.buf,nullptr); vkFreeMemory(dev,vb.mem,nullptr);
        vkDestroyBuffer(dev,ib.buf,nullptr); vkFreeMemory(dev,ib.mem,nullptr);
        if(waterIndexCount){ vkDestroyBuffer(dev,wvb.buf,nullptr); vkFreeMemory(dev,wvb.mem,nullptr);
                             vkDestroyBuffer(dev,wib.buf,nullptr); vkFreeMemory(dev,wib.mem,nullptr); }
        uploadMesh(w);
    }
    void buildTargets(VkExtent2D e){
        ext=e; halfExt={ (e.width+1)/2, (e.height+1)/2 };
        const VkImageUsageFlags CA_S=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT|VK_IMAGE_USAGE_SAMPLED_BIT;
        gAlbedo  = makeImg(pd,dev,e.width,e.height,ALBEDO_FMT, CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        gNormal  = makeImg(pd,dev,e.width,e.height,NORMRGH_FMT,CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        gPosition= makeImg(pd,dev,e.width,e.height,POS_FMT,    CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        ssaoImg  = makeImg(pd,dev,e.width,e.height,AO_FMT,     CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        hdr  = makeImg(pd,dev,e.width,e.height,HDR_FMT,CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        hdr2 = makeImg(pd,dev,e.width,e.height,HDR_FMT,CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        bloom= makeImg(pd,dev,halfExt.width,halfExt.height,HDR_FMT,CA_S,VK_IMAGE_ASPECT_COLOR_BIT);
        out  = makeImg(pd,dev,e.width,e.height,OUT_FMT,VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT|VK_IMAGE_USAGE_TRANSFER_SRC_BIT,VK_IMAGE_ASPECT_COLOR_BIT);
        Img d= makeImg(pd,dev,e.width,e.height,DEPTH_FMT,VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,VK_IMAGE_ASPECT_DEPTH_BIT);
        depthImg=d.img; depthMem=d.mem; depthView=d.view;
        auto mkFB=[&](VkRenderPass rp, std::vector<VkImageView> views, VkExtent2D ex){
            VkFramebufferCreateInfo fbi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO}; fbi.renderPass=rp;
            fbi.attachmentCount=(uint32_t)views.size(); fbi.pAttachments=views.data(); fbi.width=ex.width; fbi.height=ex.height; fbi.layers=1;
            VkFramebuffer fb; VK_CHECK(vkCreateFramebuffer(dev,&fbi,nullptr,&fb)); return fb; };
        gbufFB =mkFB(gbufRP,{gAlbedo.view,gNormal.view,gPosition.view,depthView},ext);
        ssaoFB =mkFB(ssaoRP,{ssaoImg.view},ext);
        lightFB=mkFB(lightRP,{hdr.view},ext);
        waterFB=mkFB(waterRP,{hdr.view,depthView},ext);
        ssrFB  =mkFB(ssrRP,{hdr2.view},ext);
        bloomFB=mkFB(bloomRP,{bloom.view},halfExt);
        postFB =mkFB(postRP,{out.view},ext);
        hudFB  =mkFB(hudRP,{out.view},ext);
        // (re)point the image-sampling descriptors at the current targets
        VkDescriptorImageInfo gAi{samp,gAlbedo.view,  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo gNi{samp,gNormal.view,  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo gPi{samp,gPosition.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo aoi{samp,ssaoImg.view,  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo hi {samp,hdr.view,      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo h2i{samp,hdr2.view,     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo bi {samp,bloom.view,    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        auto img=[&](VkDescriptorSet set,uint32_t b,VkDescriptorImageInfo* ii){ VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            w.dstSet=set; w.dstBinding=b; w.descriptorCount=1; w.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; w.pImageInfo=ii; return w; };
        VkWriteDescriptorSet w[]={
            img(ssaoSet,1,&gPi), img(ssaoSet,2,&gNi),
            img(lightSet,2,&gAi), img(lightSet,3,&gNi), img(lightSet,4,&gPi), img(lightSet,5,&aoi),
            img(ssrSet,1,&hi), img(ssrSet,2,&gPi), img(ssrSet,3,&gNi), img(ssrSet,4,&gAi),
            img(bloomSet,0,&h2i), img(postSet,0,&h2i), img(postSet,1,&bi) };   // bloom/post read SSR output
        vkUpdateDescriptorSets(dev,(uint32_t)(sizeof(w)/sizeof(w[0])),w,0,nullptr);
    }
    void destroyTargets(){
        vkDestroyFramebuffer(dev,gbufFB,nullptr); vkDestroyFramebuffer(dev,ssaoFB,nullptr); vkDestroyFramebuffer(dev,lightFB,nullptr);
        vkDestroyFramebuffer(dev,waterFB,nullptr); vkDestroyFramebuffer(dev,bloomFB,nullptr); vkDestroyFramebuffer(dev,postFB,nullptr); vkDestroyFramebuffer(dev,hudFB,nullptr);
        vkDestroyImageView(dev,depthView,nullptr); vkDestroyImage(dev,depthImg,nullptr); vkFreeMemory(dev,depthMem,nullptr);
        vkDestroyFramebuffer(dev,ssrFB,nullptr);
        destroyImg(dev,gAlbedo); destroyImg(dev,gNormal); destroyImg(dev,gPosition); destroyImg(dev,ssaoImg);
        destroyImg(dev,hdr); destroyImg(dev,hdr2); destroyImg(dev,bloom); destroyImg(dev,out);
    }
    void resize(VkExtent2D e){ vkDeviceWaitIdle(dev); destroyTargets(); buildTargets(e); }

    void record(VkCommandBuffer cmd, const CamView& cam){
        auto setVP=[&](VkExtent2D e){ VkViewport v{0,0,(float)e.width,(float)e.height,0,1}; VkRect2D s{{0,0},e};
            vkCmdSetViewport(cmd,0,1,&v); vkCmdSetScissor(cmd,0,1,&s); };
        VkDeviceSize off=0;
        // --- shadow pass: depth from the sun ---
        VkClearValue sc; sc.depthStencil={1,0};
        VkExtent2D sext{SHADOW_DIM,SHADOW_DIM};
        VkRenderPassBeginInfo rs{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; rs.renderPass=shadowRP; rs.framebuffer=shadowFB; rs.renderArea={{0,0},sext}; rs.clearValueCount=1; rs.pClearValues=&sc;
        vkCmdBeginRenderPass(cmd,&rs,VK_SUBPASS_CONTENTS_INLINE); setVP(sext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,shadowPipe);
        ShadowPC spc{lightVP}; vkCmdPushConstants(cmd,shadowLayout,VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(spc),&spc);
        vkCmdBindVertexBuffers(cmd,0,1,&vb.buf,&off); vkCmdBindIndexBuffer(cmd,ib.buf,0,VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd,indexCount,1,0,0,0); vkCmdEndRenderPass(cmd);
        // --- update scene UBO ---
        SceneUBO ubo{}; ubo.viewProj=cam.viewProj((float)ext.width/ext.height); ubo.lightVP=lightVP;
        Vec3 sun=normalize(Vec3{-0.48f,0.60f,0.64f}); ubo.sunDir[0]=sun.x;ubo.sunDir[1]=sun.y;ubo.sunDir[2]=sun.z;
        ubo.camPos[0]=cam.pos.x;ubo.camPos[1]=cam.pos.y;ubo.camPos[2]=cam.pos.z;ubo.camPos[3]=620.0f;
        memcpy(sceneUBOmap,&ubo,sizeof(ubo));
        // --- G-buffer: geometry -> albedo / normal+rough / position+flag (+depth) ---
        VkClearValue gcl[4]; gcl[0].color={{0,0,0,0}}; gcl[1].color={{0,0,0,0}}; gcl[2].color={{0,0,0,0}}; gcl[3].depthStencil={1,0};
        VkRenderPassBeginInfo rg{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; rg.renderPass=gbufRP; rg.framebuffer=gbufFB; rg.renderArea={{0,0},ext}; rg.clearValueCount=4; rg.pClearValues=gcl;
        vkCmdBeginRenderPass(cmd,&rg,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,gbufPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,gbufLayout,0,1,&sceneSet,0,nullptr);
        vkCmdBindVertexBuffers(cmd,0,1,&vb.buf,&off); vkCmdBindIndexBuffer(cmd,ib.buf,0,VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd,indexCount,1,0,0,0); vkCmdEndRenderPass(cmd);
        // --- SSAO: hemisphere occlusion from gPosition + gNormal ---
        VkClearValue acl; acl.color={{1,1,1,1}};
        VkRenderPassBeginInfo ra{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; ra.renderPass=ssaoRP; ra.framebuffer=ssaoFB; ra.renderArea={{0,0},ext}; ra.clearValueCount=1; ra.pClearValues=&acl;
        vkCmdBeginRenderPass(cmd,&ra,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,ssaoPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,ssaoLayout,0,1,&ssaoSet,0,nullptr);
        vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
        // --- deferred lighting -> HDR (background cleared to linear sky; lit geometry over it) ---
        VkClearValue lcl; lcl.color={{0.18f,0.34f,0.85f,1}};
        VkRenderPassBeginInfo r1{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; r1.renderPass=lightRP; r1.framebuffer=lightFB; r1.renderArea={{0,0},ext}; r1.clearValueCount=1; r1.pClearValues=&lcl;
        vkCmdBeginRenderPass(cmd,&r1,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,lightPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,lightLayout,0,1,&lightSet,0,nullptr);
        { Vec3 f=cam.fwd; Vec3 r=normalize(cross(f,cam.up)); Vec3 up=normalize(cross(r,f));
          LightPC lpc{}; lpc.camDir[0]=f.x;lpc.camDir[1]=f.y;lpc.camDir[2]=f.z;
          lpc.camRight[0]=r.x;lpc.camRight[1]=r.y;lpc.camRight[2]=r.z; lpc.camUp[0]=up.x;lpc.camUp[1]=up.y;lpc.camUp[2]=up.z;
          lpc.params[0]=tanf(cam.fovy*0.5f); lpc.params[1]=(float)ext.width/ext.height; lpc.params[2]=timeSec;
          vkCmdPushConstants(cmd,lightLayout,VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(lpc),&lpc); }
        vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
        // --- forward water over the lit HDR, depth-tested against terrain ---
        if(waterIndexCount){
            VkRenderPassBeginInfo rw{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; rw.renderPass=waterRP; rw.framebuffer=waterFB; rw.renderArea={{0,0},ext}; rw.clearValueCount=0;
            vkCmdBeginRenderPass(cmd,&rw,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
            vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,waterPipe);
            vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,waterLayout,0,1,&sceneSet,0,nullptr);
            WaterPC wpc{}; wpc.misc[0]=timeSec; vkCmdPushConstants(cmd,waterLayout,VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(wpc),&wpc);
            vkCmdBindVertexBuffers(cmd,0,1,&wvb.buf,&off); vkCmdBindIndexBuffer(cmd,wib.buf,0,VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd,waterIndexCount,1,0,0,0); vkCmdEndRenderPass(cmd);
        }
        // --- SSR: reflect the lit scene (hdr) into metallic surfaces -> hdr2 ---
        VkClearValue rcl; rcl.color={{0,0,0,1}};
        VkRenderPassBeginInfo rr{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; rr.renderPass=ssrRP; rr.framebuffer=ssrFB; rr.renderArea={{0,0},ext}; rr.clearValueCount=1; rr.pClearValues=&rcl;
        vkCmdBeginRenderPass(cmd,&rr,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,ssrPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,ssrLayout,0,1,&ssrSet,0,nullptr);
        vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
        // bloom -> half-res
        VkClearValue cb{}; cb.color={{0,0,0,1}};
        VkRenderPassBeginInfo r2{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; r2.renderPass=bloomRP; r2.framebuffer=bloomFB; r2.renderArea={{0,0},halfExt}; r2.clearValueCount=1; r2.pClearValues=&cb;
        vkCmdBeginRenderPass(cmd,&r2,VK_SUBPASS_CONTENTS_INLINE); setVP(halfExt);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,bloomPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,bloomLayout,0,1,&bloomSet,0,nullptr);
        BloomPC bpc{}; bpc.texel[0]=1.0f/halfExt.width; bpc.texel[1]=1.0f/halfExt.height; bpc.threshold=1.1f; bpc.knee=0.6f;
        vkCmdPushConstants(cmd,bloomLayout,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(bpc),&bpc);
        vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
        // post -> out
        VkClearValue co{}; co.color={{0,0,0,1}};
        VkRenderPassBeginInfo r3{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; r3.renderPass=postRP; r3.framebuffer=postFB; r3.renderArea={{0,0},ext}; r3.clearValueCount=1; r3.pClearValues=&co;
        vkCmdBeginRenderPass(cmd,&r3,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,postPipe);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,postLayout,0,1,&postSet,0,nullptr);
        PostPC ppc{}; ppc.exposure=1.0f; ppc.bloomStrength=0.55f; ppc.grStrength=0.55f;
        { // project the sun direction to screen for radial god-ray shafts
          Vec3 f=cam.fwd; Vec3 r=normalize(cross(f,cam.up)); Vec3 up=normalize(cross(r,f));
          Vec3 s=normalize(Vec3{-0.48f,0.60f,0.64f});
          float z=dot(s,f), sx=dot(s,r), sy=dot(s,up);
          float th=tanf(cam.fovy*0.5f), asp=(float)ext.width/ext.height;
          ppc.sunVis = (z>0.15f)?1.0f:0.0f;
          if(z>0.15f){ float nx=(sx/z)/(th*asp), ny=-(sy/z)/th;
            ppc.sunUV[0]=nx*0.5f+0.5f; ppc.sunUV[1]=ny*0.5f+0.5f; } }
        vkCmdPushConstants(cmd,postLayout,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(ppc),&ppc);
        vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
        // --- HUD overlay: alpha-blend text on the final image (always runs to
        //     transition `out` COLOR_ATTACHMENT -> TRANSFER_SRC for the blit/copy) ---
        {
            std::vector<hud::Vtx> v; v.reserve(2048);
            float W=(float)ext.width, Hh=(float)ext.height;
            auto panel=[&](float x,float y,float w,float h,float a){ hud::addRect(v,x,y,w,h, 0.07f,0.085f,0.13f, a); };
            auto txt=[&](const char* s,float x,float y,float sc,float r,float g,float b){
                hud::addText(v,s,x+1.5f,y+1.5f,sc, 0.05f,0.06f,0.10f,0.85f);   // drop shadow
                hud::addText(v,s,x,y,sc, r,g,b,1.0f); };
            // brand panel (top-left)
            panel(18,14,232,40,0.60f); txt("MINECOASTER",30,20,22, 1.0f,1.0f,1.0f);
            if(hudRide){
                // speed card (top-right): big colour-coded KM/H + altitude
                char num[16]; snprintf(num,sizeof num,"%d",(int)hudTel.speedKmh);
                float sc=40.0f, nw=hud::textWidth(num,sc);
                float cardW=nw+152.0f, cardX=W-cardW-18.0f;
                panel(cardX,14,cardW,64,0.60f);
                float sr=1,sg=1,sb=1;
                if(hudTel.speedKmh>250){ sr=1.0f;sg=0.47f;sb=0.35f; } else if(hudTel.speedKmh>150){ sr=0.47f;sg=0.90f;sb=0.66f; }
                txt(num,cardX+18,20,sc, sr,sg,sb);
                txt("KM/H",cardX+30+nw,34,18, 0.66f,0.72f,0.84f);
                char alt[24]; snprintf(alt,sizeof alt,"ALT %dM",(int)hudTel.altitude);
                float aw=hud::textWidth(alt,15.0f); txt(alt,cardX+cardW-aw-14,57,15, 0.59f,0.66f,0.78f);
                // element name panel with accent bar (below the speed card)
                if(hudTel.element && hudTel.element[0]){
                    float esc=18.0f, ew=hud::textWidth(hudTel.element,esc), pw=ew+30.0f, px=W-pw-18.0f, py=88.0f;
                    panel(px,py,pw,30,0.60f);
                    float ar=0.59f,ag=0.72f,ab=0.90f; if(hudTel.special){ ar=1.0f;ag=0.78f;ab=0.43f; }
                    hud::addRect(v,px+8,py+8,4,14, ar,ag,ab,1.0f);
                    txt(hudTel.element,px+18,py+7,esc, hudTel.special?ar:0.84f, hudTel.special?ag:0.88f, hudTel.special?ab:0.94f);
                }
                // g-meter (bottom-left)
                char gs[24]; snprintf(gs,sizeof gs,"%+.1f G",hudTel.gForce);
                float gr=1.0f,gg=0.80f,gb=0.45f; if(hudTel.gForce>5.0f||hudTel.gForce<-1.5f){ gr=1.0f;gg=0.40f;gb=0.35f; }
                panel(18,Hh-50,150,34,0.55f); txt(gs,30,Hh-44,20, gr,gg,gb);
            } else {
                txt("FREE-FLY   PRESS C TO RIDE",30,62,15, 0.80f,0.86f,0.95f);
            }
            hudVtxCount=(uint32_t)v.size(); if(hudVtxCount>hudVtxCap) hudVtxCount=hudVtxCap;
            if(hudVtxCount) memcpy(hudVBmap,v.data(),(size_t)hudVtxCount*sizeof(hud::Vtx));
            VkRenderPassBeginInfo rh{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; rh.renderPass=hudRP; rh.framebuffer=hudFB; rh.renderArea={{0,0},ext}; rh.clearValueCount=0;
            vkCmdBeginRenderPass(cmd,&rh,VK_SUBPASS_CONTENTS_INLINE); setVP(ext);
            if(hudVtxCount){
                vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,hudPipe);
                vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,hudLayout,0,1,&hudSet,0,nullptr);
                HudPC hpc{}; hpc.screen[0]=(float)ext.width; hpc.screen[1]=(float)ext.height;
                vkCmdPushConstants(cmd,hudLayout,VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(hpc),&hpc);
                vkCmdBindVertexBuffers(cmd,0,1,&hudVB.buf,&off);
                vkCmdDraw(cmd,hudVtxCount,1,0,0);
            }
            vkCmdEndRenderPass(cmd);
        }
        // out is now TRANSFER_SRC_OPTIMAL
    }
};

static void pickDevice(VkInstance inst, VkSurfaceKHR surf, VkPhysicalDevice& pd, uint32_t& gfx){
    uint32_t n=0; vkEnumeratePhysicalDevices(inst,&n,nullptr); std::vector<VkPhysicalDevice> pds(n); vkEnumeratePhysicalDevices(inst,&n,pds.data());
    pd=VK_NULL_HANDLE; gfx=0;
    for(auto c:pds){ uint32_t nq=0; vkGetPhysicalDeviceQueueFamilyProperties(c,&nq,nullptr); std::vector<VkQueueFamilyProperties> qf(nq);
        vkGetPhysicalDeviceQueueFamilyProperties(c,&nq,qf.data());
        for(uint32_t i=0;i<nq;i++){ VkBool32 pr=VK_TRUE; if(surf) vkGetPhysicalDeviceSurfaceSupportKHR(c,i,surf,&pr);
            if((qf[i].queueFlags&VK_QUEUE_GRAPHICS_BIT)&&pr){ pd=c; gfx=i; break; } }
        if(pd) break; }
    if(!pd){ fprintf(stderr,"no suitable device\n"); exit(1); }
    VkPhysicalDeviceProperties p; vkGetPhysicalDeviceProperties(pd,&p); printf("[vk] device: %s\n",p.deviceName);
}

// =====================================================================
static int runOffscreen(const World& world, const std::string& sd, const std::string& outPath){
    const uint32_t W=1280,H=720;
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO}; app.apiVersion=VK_API_VERSION_1_1;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO}; ici.pApplicationInfo=&app;
    VkInstance inst; VK_CHECK(vkCreateInstance(&ici,nullptr,&inst));
    Renderer R{}; pickDevice(inst,VK_NULL_HANDLE,R.pd,R.gfx);
    float pr=1; VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO}; qci.queueFamilyIndex=R.gfx; qci.queueCount=1; qci.pQueuePriorities=&pr;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO}; dci.queueCreateInfoCount=1; dci.pQueueCreateInfos=&qci;
    VK_CHECK(vkCreateDevice(R.pd,&dci,nullptr,&R.dev)); vkGetDeviceQueue(R.dev,R.gfx,0,&R.queue);
    R.initPipelines(sd); R.uploadMesh(world); R.buildTargets({W,H}); R.setCenter(world.focus);

    FlyCam cam; cam.pos=world.focus+Vec3{135,98,165};
    if(g_camSet){ cam.pos=g_camPos; cam.yaw=g_camYaw; cam.pitch=g_camPitch; }
    CamView view=cam.view(); R.hudRide=g_rideSet;
    if(g_rideSet){ world::RideSim rs; rs.reset(world.trk); rs.advance(world.trk,g_rideSec);
        RideTelemetry tel; view=rideCamView(world.trk,rs,tel); R.hudTel=tel;
        printf("[ride] t=%.1fs  %.0f km/h  alt %.0fm  %.2fg  %s\n",g_rideSec,tel.speedKmh,tel.altitude,tel.gForce,tel.element); }
    Buffer rb=makeBuffer(R.pd,R.dev,(VkDeviceSize)W*H*4,VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; cpi.queueFamilyIndex=R.gfx;
    VkCommandPool pool; VK_CHECK(vkCreateCommandPool(R.dev,&cpi,nullptr,&pool));
    VkCommandBufferAllocateInfo cbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO}; cbi.commandPool=pool; cbi.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; cbi.commandBufferCount=1;
    VkCommandBuffer cmd; VK_CHECK(vkAllocateCommandBuffers(R.dev,&cbi,&cmd));
    VkCommandBufferBeginInfo bbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; bbi.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd,&bbi));
    R.record(cmd,view);
    VkBufferImageCopy reg{}; reg.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1}; reg.imageExtent={W,H,1};
    vkCmdCopyImageToBuffer(cmd,R.out.img,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,rb.buf,1,&reg);
    VK_CHECK(vkEndCommandBuffer(cmd));
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; VkFence fence; VK_CHECK(vkCreateFence(R.dev,&fci,nullptr,&fence));
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO}; si.commandBufferCount=1; si.pCommandBuffers=&cmd;
    VK_CHECK(vkQueueSubmit(R.queue,1,&si,fence)); VK_CHECK(vkWaitForFences(R.dev,1,&fence,VK_TRUE,UINT64_MAX));
    void* mp; vkMapMemory(R.dev,rb.mem,0,(VkDeviceSize)W*H*4,0,&mp); const uint8_t* px=(const uint8_t*)mp;
    FILE* of=fopen(outPath.c_str(),"wb"); fprintf(of,"P6\n%u %u\n255\n",W,H); std::vector<uint8_t> row(W*3);
    for(uint32_t y=0;y<H;y++){ for(uint32_t x=0;x<W;x++){ const uint8_t* q=px+((size_t)y*W+x)*4; row[x*3]=q[0]; row[x*3+1]=q[1]; row[x*3+2]=q[2]; } fwrite(row.data(),1,row.size(),of); }
    fclose(of); vkUnmapMemory(R.dev,rb.mem); printf("[vk] wrote %s\n",outPath.c_str());
    vkDeviceWaitIdle(R.dev); return 0;
}

struct Swap { VkSwapchainKHR chain=VK_NULL_HANDLE; VkFormat fmt; VkExtent2D ext; std::vector<VkImage> images; };
static void makeSwap(VkPhysicalDevice pd, VkDevice dev, VkSurfaceKHR surf, Swap& s, uint32_t W, uint32_t H){
    VkSurfaceCapabilitiesKHR caps; vkGetPhysicalDeviceSurfaceCapabilitiesKHR(pd,surf,&caps);
    uint32_t nf=0; vkGetPhysicalDeviceSurfaceFormatsKHR(pd,surf,&nf,nullptr); std::vector<VkSurfaceFormatKHR> fmts(nf); vkGetPhysicalDeviceSurfaceFormatsKHR(pd,surf,&nf,fmts.data());
    VkSurfaceFormatKHR sf=fmts[0]; for(auto&f:fmts) if(f.format==VK_FORMAT_B8G8R8A8_UNORM){ sf=f; break; }
    s.fmt=sf.format; VkExtent2D e=caps.currentExtent; if(e.width==0xFFFFFFFF){ e.width=W; e.height=H; } s.ext=e;
    uint32_t ic=caps.minImageCount+1; if(caps.maxImageCount&&ic>caps.maxImageCount) ic=caps.maxImageCount;
    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR}; ci.surface=surf; ci.minImageCount=ic; ci.imageFormat=sf.format; ci.imageColorSpace=sf.colorSpace;
    ci.imageExtent=e; ci.imageArrayLayers=1; ci.imageUsage=VK_IMAGE_USAGE_TRANSFER_DST_BIT; ci.imageSharingMode=VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform=caps.currentTransform; ci.compositeAlpha=VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; ci.presentMode=VK_PRESENT_MODE_FIFO_KHR; ci.clipped=VK_TRUE;
    VK_CHECK(vkCreateSwapchainKHR(dev,&ci,nullptr,&s.chain));
    uint32_t ni=0; vkGetSwapchainImagesKHR(dev,s.chain,&ni,nullptr); s.images.resize(ni); vkGetSwapchainImagesKHR(dev,s.chain,&ni,s.images.data());
}

static int runWindowed(World& world, const std::string& sd, int maxFrames){
    if(SDL_Init(SDL_INIT_VIDEO)!=0){ fprintf(stderr,"SDL_Init: %s\n",SDL_GetError()); return 1; }
    uint32_t W=1280,H=720;
    SDL_Window* win=SDL_CreateWindow("MINECOASTER (Vulkan)",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,W,H,SDL_WINDOW_VULKAN|SDL_WINDOW_RESIZABLE);
    if(!win){ fprintf(stderr,"SDL_CreateWindow: %s\n",SDL_GetError()); return 1; }
    uint32_t nE=0; SDL_Vulkan_GetInstanceExtensions(win,&nE,nullptr); std::vector<const char*> exts(nE); SDL_Vulkan_GetInstanceExtensions(win,&nE,exts.data());
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO}; app.apiVersion=VK_API_VERSION_1_1;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO}; ici.pApplicationInfo=&app; ici.enabledExtensionCount=nE; ici.ppEnabledExtensionNames=exts.data();
    VkInstance inst; VK_CHECK(vkCreateInstance(&ici,nullptr,&inst));
    VkSurfaceKHR surf; if(!SDL_Vulkan_CreateSurface(win,inst,&surf)){ fprintf(stderr,"CreateSurface: %s\n",SDL_GetError()); return 1; }
    Renderer R{}; pickDevice(inst,surf,R.pd,R.gfx);
    const char* devExt[]={VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    float pr=1; VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO}; qci.queueFamilyIndex=R.gfx; qci.queueCount=1; qci.pQueuePriorities=&pr;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO}; dci.queueCreateInfoCount=1; dci.pQueueCreateInfos=&qci; dci.enabledExtensionCount=1; dci.ppEnabledExtensionNames=devExt;
    VK_CHECK(vkCreateDevice(R.pd,&dci,nullptr,&R.dev)); vkGetDeviceQueue(R.dev,R.gfx,0,&R.queue);

    Swap swap{}; makeSwap(R.pd,R.dev,surf,swap,W,H);
    R.initPipelines(sd); R.uploadMesh(world); R.buildTargets(swap.ext); R.setCenter(world.focus);

    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; cpi.queueFamilyIndex=R.gfx; cpi.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    VkCommandPool pool; VK_CHECK(vkCreateCommandPool(R.dev,&cpi,nullptr,&pool));
    VkCommandBufferAllocateInfo cbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO}; cbi.commandPool=pool; cbi.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; cbi.commandBufferCount=1;
    VkCommandBuffer cmd; VK_CHECK(vkAllocateCommandBuffers(R.dev,&cbi,&cmd));
    VkSemaphoreCreateInfo sci{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO}; VkSemaphore semAcq,semDone; VK_CHECK(vkCreateSemaphore(R.dev,&sci,nullptr,&semAcq)); VK_CHECK(vkCreateSemaphore(R.dev,&sci,nullptr,&semDone));
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; fci.flags=VK_FENCE_CREATE_SIGNALED_BIT; VkFence fence; VK_CHECK(vkCreateFence(R.dev,&fci,nullptr,&fence));

    FlyCam cam; cam.pos=world.focus+Vec3{120,86,150};
    world::RideSim rs; rs.reset(world.trk); bool rideMode=false; RideTelemetry tel; float telAcc=0;
    Vec3 streamCenter=world.focus;   // re-bake the patch around the viewer as they roam (endless world)
    SDL_SetRelativeMouseMode(SDL_TRUE);
    printf("[vk] controls: WASD move, Q/E down/up, mouse look, Shift fast, C ride coaster, Esc quit\n");
    auto recreate=[&](){ vkDeviceWaitIdle(R.dev); vkDestroySwapchainKHR(R.dev,swap.chain,nullptr); makeSwap(R.pd,R.dev,surf,swap,W,H); R.resize(swap.ext); };

    bool run=true; uint32_t last=SDL_GetTicks(); int frame=0;
    while(run){
        uint32_t now=SDL_GetTicks(); float dt=(now-last)/1000.0f; last=now; if(dt>0.1f)dt=0.1f;
        R.timeSec += dt;
        SDL_Event e;
        while(SDL_PollEvent(&e)){
            if(e.type==SDL_QUIT) run=false;
            else if(e.type==SDL_KEYDOWN&&e.key.keysym.sym==SDLK_ESCAPE) run=false;
            else if(e.type==SDL_KEYDOWN&&e.key.keysym.sym==SDLK_c){ rideMode=!rideMode; if(rideMode) rs.reset(world.trk);
                printf("[vk] %s\n", rideMode?"ride camera ON (C to exit)":"free-fly camera"); }
            else if(e.type==SDL_MOUSEMOTION&&!rideMode){ cam.yaw-=e.motion.xrel*0.0025f; cam.pitch-=e.motion.yrel*0.0025f;
                if(cam.pitch>1.5f)cam.pitch=1.5f; if(cam.pitch<-1.5f)cam.pitch=-1.5f; }
            else if(e.type==SDL_WINDOWEVENT&&e.window.event==SDL_WINDOWEVENT_SIZE_CHANGED) recreate();
        }
        CamView view; R.hudRide=rideMode;
        if(rideMode){
            rs.advance(world.trk,dt); view=rideCamView(world.trk,rs,tel); R.hudTel=tel;
            telAcc+=dt; if(telAcc>0.5f){ telAcc=0; printf("[ride] %.0f km/h  alt %.0fm  %.2fg  %s\n",tel.speedKmh,tel.altitude,tel.gForce,tel.element); }
        } else {
            const Uint8* ks=SDL_GetKeyboardState(nullptr); Vec3 f=cam.fwd(), r=normalize(cross(f,Vec3{0,1,0}));
            float sp=(ks[SDL_SCANCODE_LSHIFT]?120.0f:38.0f)*dt;
            if(ks[SDL_SCANCODE_W])cam.pos=cam.pos+f*sp; if(ks[SDL_SCANCODE_S])cam.pos=cam.pos-f*sp;
            if(ks[SDL_SCANCODE_D])cam.pos=cam.pos+r*sp; if(ks[SDL_SCANCODE_A])cam.pos=cam.pos-r*sp;
            if(ks[SDL_SCANCODE_E])cam.pos.y+=sp; if(ks[SDL_SCANCODE_Q])cam.pos.y-=sp;
            view=cam.view();
        }
        // --- streaming: re-bake terrain/track/water around the viewer when they
        //     leave the current patch, so the world is effectively endless ---
        { Vec3 here=view.pos; float dx=here.x-streamCenter.x, dz=here.z-streamCenter.z;
          if(dx*dx+dz*dz > (PATCH_HALF*0.6f)*(PATCH_HALF*0.6f)){
            buildWorldMeshes(world, Vec3{here.x, world.focus.y, here.z});
            R.reuploadMesh(world); R.setCenter(Vec3{here.x, groundTopAt(here.x,here.z), here.z});
            streamCenter=here; last=SDL_GetTicks();   // skip the rebuild stall in dt
          } }

        VK_CHECK(vkWaitForFences(R.dev,1,&fence,VK_TRUE,UINT64_MAX));
        uint32_t idx; VkResult acq=vkAcquireNextImageKHR(R.dev,swap.chain,UINT64_MAX,semAcq,VK_NULL_HANDLE,&idx);
        if(acq==VK_ERROR_OUT_OF_DATE_KHR){ recreate(); continue; }
        VK_CHECK(vkResetFences(R.dev,1,&fence)); vkResetCommandBuffer(cmd,0);
        VkCommandBufferBeginInfo bbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; VK_CHECK(vkBeginCommandBuffer(cmd,&bbi));
        R.record(cmd,view);
        // blit out -> swapchain image
        VkImageMemoryBarrier toDst{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER}; toDst.oldLayout=VK_IMAGE_LAYOUT_UNDEFINED; toDst.newLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        toDst.srcQueueFamilyIndex=toDst.dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED; toDst.image=swap.images[idx]; toDst.subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        toDst.dstAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,0,0,nullptr,0,nullptr,1,&toDst);
        VkImageBlit blit{}; blit.srcSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1}; blit.dstSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};
        blit.srcOffsets[1]={(int)R.ext.width,(int)R.ext.height,1}; blit.dstOffsets[1]={(int)swap.ext.width,(int)swap.ext.height,1};
        vkCmdBlitImage(cmd,R.out.img,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,swap.images[idx],VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,1,&blit,VK_FILTER_LINEAR);
        VkImageMemoryBarrier toPres{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER}; toPres.oldLayout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toPres.newLayout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        toPres.srcQueueFamilyIndex=toPres.dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED; toPres.image=swap.images[idx]; toPres.subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        toPres.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,0,0,nullptr,0,nullptr,1,&toPres);
        VK_CHECK(vkEndCommandBuffer(cmd));
        VkPipelineStageFlags wait=VK_PIPELINE_STAGE_TRANSFER_BIT;
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO}; si.waitSemaphoreCount=1; si.pWaitSemaphores=&semAcq; si.pWaitDstStageMask=&wait;
        si.commandBufferCount=1; si.pCommandBuffers=&cmd; si.signalSemaphoreCount=1; si.pSignalSemaphores=&semDone;
        VK_CHECK(vkQueueSubmit(R.queue,1,&si,fence));
        VkPresentInfoKHR pp{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR}; pp.waitSemaphoreCount=1; pp.pWaitSemaphores=&semDone; pp.swapchainCount=1; pp.pSwapchains=&swap.chain; pp.pImageIndices=&idx;
        VkResult pres=vkQueuePresentKHR(R.queue,&pp);
        if(pres==VK_ERROR_OUT_OF_DATE_KHR||pres==VK_SUBOPTIMAL_KHR) recreate();
        if(maxFrames>0 && ++frame>=maxFrames) run=false;
    }
    vkDeviceWaitIdle(R.dev); SDL_DestroyWindow(win); SDL_Quit(); printf("[vk] windowed session ended\n"); return 0;
}

int main(int argc, char** argv){
    SDL_SetMainReady();           // required when SDL_MAIN_HANDLED is defined
    // default shaders/ resolved next to the executable (CMake copies them there)
    std::string sd="shaders";
    if(char* base=SDL_GetBasePath()){ sd=std::string(base)+"shaders"; SDL_free(base); }
    std::string outPath="minecoaster_vk.ppm"; bool shot=false; int maxFrames=0;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--shaders")&&i+1<argc) sd=argv[++i];
        else if(!strcmp(argv[i],"-o")&&i+1<argc){ outPath=argv[++i]; shot=true; }
        else if(!strcmp(argv[i],"--shot")) shot=true;
        else if(!strcmp(argv[i],"--frames")&&i+1<argc) maxFrames=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--cam")&&i+1<argc){ // x,y,z,yaw,pitch
            float x,y,z,yw,pt; if(sscanf(argv[++i],"%f,%f,%f,%f,%f",&x,&y,&z,&yw,&pt)==5){ g_camSet=true; g_camPos={x,y,z}; g_camYaw=yw; g_camPitch=pt; } }
        else if(!strcmp(argv[i],"--ride")&&i+1<argc){ g_rideSet=true; g_rideSec=(float)atof(argv[++i]); }
    }
    World world=buildWorld();
    return shot ? runOffscreen(world,sd,outPath) : runWindowed(world,sd,maxFrames);
}

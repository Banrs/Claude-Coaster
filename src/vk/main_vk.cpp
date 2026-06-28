// MINECOASTER Vulkan renderer -- MILESTONE 1: window + swapchain + clear.
// Cross-platform (macOS via MoltenVK, Windows/Linux native). This does NOT touch the GL
// build; it is a separate target (see src/vk/build_vk.sh, docs/VULKAN_PORT.md).
// Subsequent milestones (textured quad, terrain, shadows, ...) build on this skeleton.
//
// Run on macOS: the loader needs the MoltenVK ICD; build_vk.sh exports VK_ICD_FILENAMES.
#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <optional>

#define VKCHECK(x) do { VkResult _r = (x); if (_r != VK_SUCCESS) { \
    fprintf(stderr, "Vulkan error %d at %s:%d\n", _r, __FILE__, __LINE__); exit(1); } } while (0)

static const int    WIN_W = 1280, WIN_H = 720;
static const int    MAX_FRAMES = 2;
static const float  SKY[4] = { 186/255.f, 205/255.f, 232/255.f, 1.0f };

struct VkApp {
    GLFWwindow *win = nullptr;
    VkInstance instance{};
    VkSurfaceKHR surface{};
    VkPhysicalDevice phys{};
    VkDevice device{};
    uint32_t gfxFamily = 0, presentFamily = 0;
    VkQueue gfxQueue{}, presentQueue{};

    VkSwapchainKHR swapchain{};
    VkFormat scFormat{};
    VkExtent2D scExtent{};
    std::vector<VkImage> scImages;
    std::vector<VkImageView> scViews;
    std::vector<VkFramebuffer> scFbos;
    VkRenderPass renderPass{};

    VkCommandPool cmdPool{};
    std::vector<VkCommandBuffer> cmds;
    VkSemaphore imgAvail[MAX_FRAMES]{}, renderDone[MAX_FRAMES]{};
    VkFence inFlight[MAX_FRAMES]{};
    uint32_t frame = 0;
    bool resized = false;

    void run() { initWindow(); initVulkan(); loop(); cleanup(); }

    static void onResize(GLFWwindow *w, int, int) {
        ((VkApp *)glfwGetWindowUserPointer(w))->resized = true;
    }

    void initWindow() {
        if (!glfwInit()) { fprintf(stderr, "glfwInit failed\n"); exit(1); }
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        win = glfwCreateWindow(WIN_W, WIN_H, "MINECOASTER (Vulkan)", nullptr, nullptr);
        glfwSetWindowUserPointer(win, this);
        glfwSetFramebufferSizeCallback(win, onResize);
    }

    void initVulkan() {
        createInstance();
        VKCHECK(glfwCreateWindowSurface(instance, win, nullptr, &surface));
        pickPhysical();
        createDevice();
        createSwapchain();
        createRenderPass();
        createFramebuffers();
        createCommands();
        createSync();
    }

    void createInstance() {
        VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
        app.pApplicationName = "MINECOASTER";
        app.apiVersion = VK_API_VERSION_1_2;

        uint32_t glfwN = 0;
        const char **glfwExt = glfwGetRequiredInstanceExtensions(&glfwN);
        std::vector<const char *> exts(glfwExt, glfwExt + glfwN);
        // MoltenVK / portability: required to enumerate the Metal-backed device.
        exts.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
        exts.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);

        VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
        ci.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
        ci.pApplicationInfo = &app;
        ci.enabledExtensionCount = (uint32_t)exts.size();
        ci.ppEnabledExtensionNames = exts.data();
        VKCHECK(vkCreateInstance(&ci, nullptr, &instance));
    }

    void pickPhysical() {
        uint32_t n = 0; vkEnumeratePhysicalDevices(instance, &n, nullptr);
        if (!n) { fprintf(stderr, "no Vulkan devices\n"); exit(1); }
        std::vector<VkPhysicalDevice> devs(n);
        vkEnumeratePhysicalDevices(instance, &n, devs.data());
        phys = devs[0];                 // milestone 1: first device is fine
        for (auto d : devs) {           // prefer a discrete GPU if present
            VkPhysicalDeviceProperties p; vkGetPhysicalDeviceProperties(d, &p);
            if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) { phys = d; break; }
        }
        // find queue families (graphics + present)
        uint32_t qn = 0; vkGetPhysicalDeviceQueueFamilyProperties(phys, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qs(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(phys, &qn, qs.data());
        std::optional<uint32_t> g, pr;
        for (uint32_t i = 0; i < qn; i++) {
            if (qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) g = i;
            VkBool32 sup = 0; vkGetPhysicalDeviceSurfaceSupportKHR(phys, i, surface, &sup);
            if (sup) pr = i;
            if (g && pr) break;
        }
        if (!g || !pr) { fprintf(stderr, "no graphics/present queue\n"); exit(1); }
        gfxFamily = *g; presentFamily = *pr;
    }

    void createDevice() {
        float prio = 1.0f;
        std::vector<VkDeviceQueueCreateInfo> qcis;
        std::vector<uint32_t> fams = { gfxFamily };
        if (presentFamily != gfxFamily) fams.push_back(presentFamily);
        for (uint32_t f : fams) {
            VkDeviceQueueCreateInfo q{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
            q.queueFamilyIndex = f; q.queueCount = 1; q.pQueuePriorities = &prio;
            qcis.push_back(q);
        }
        // swapchain + (MoltenVK) portability_subset if the device advertises it
        std::vector<const char *> devExt = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
        uint32_t en = 0; vkEnumerateDeviceExtensionProperties(phys, nullptr, &en, nullptr);
        std::vector<VkExtensionProperties> avail(en);
        vkEnumerateDeviceExtensionProperties(phys, nullptr, &en, avail.data());
        for (auto &e : avail)
            if (!strcmp(e.extensionName, "VK_KHR_portability_subset"))
                devExt.push_back("VK_KHR_portability_subset");

        VkDeviceCreateInfo ci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
        ci.queueCreateInfoCount = (uint32_t)qcis.size();
        ci.pQueueCreateInfos = qcis.data();
        ci.enabledExtensionCount = (uint32_t)devExt.size();
        ci.ppEnabledExtensionNames = devExt.data();
        VKCHECK(vkCreateDevice(phys, &ci, nullptr, &device));
        vkGetDeviceQueue(device, gfxFamily, 0, &gfxQueue);
        vkGetDeviceQueue(device, presentFamily, 0, &presentQueue);
    }

    void createSwapchain() {
        VkSurfaceCapabilitiesKHR caps;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys, surface, &caps);
        uint32_t fn = 0; vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &fn, nullptr);
        std::vector<VkSurfaceFormatKHR> fmts(fn);
        vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &fn, fmts.data());
        VkSurfaceFormatKHR fmt = fmts[0];
        for (auto &f : fmts)
            if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) fmt = f;
        scFormat = fmt.format;

        if (caps.currentExtent.width != UINT32_MAX) scExtent = caps.currentExtent;
        else {
            int w, h; glfwGetFramebufferSize(win, &w, &h);
            scExtent.width  = std::clamp((uint32_t)w, caps.minImageExtent.width,  caps.maxImageExtent.width);
            scExtent.height = std::clamp((uint32_t)h, caps.minImageExtent.height, caps.maxImageExtent.height);
        }
        uint32_t imgCount = caps.minImageCount + 1;
        if (caps.maxImageCount && imgCount > caps.maxImageCount) imgCount = caps.maxImageCount;

        VkSwapchainCreateInfoKHR ci{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
        ci.surface = surface;
        ci.minImageCount = imgCount;
        ci.imageFormat = fmt.format;
        ci.imageColorSpace = fmt.colorSpace;
        ci.imageExtent = scExtent;
        ci.imageArrayLayers = 1;
        ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        uint32_t fams[2] = { gfxFamily, presentFamily };
        if (gfxFamily != presentFamily) {
            ci.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            ci.queueFamilyIndexCount = 2; ci.pQueueFamilyIndices = fams;
        } else ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        ci.preTransform = caps.currentTransform;
        ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;     // vsync; always supported
        ci.clipped = VK_TRUE;
        VKCHECK(vkCreateSwapchainKHR(device, &ci, nullptr, &swapchain));

        vkGetSwapchainImagesKHR(device, swapchain, &imgCount, nullptr);
        scImages.resize(imgCount);
        vkGetSwapchainImagesKHR(device, swapchain, &imgCount, scImages.data());
        scViews.resize(imgCount);
        for (uint32_t i = 0; i < imgCount; i++) {
            VkImageViewCreateInfo v{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            v.image = scImages[i]; v.viewType = VK_IMAGE_VIEW_TYPE_2D; v.format = scFormat;
            v.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            VKCHECK(vkCreateImageView(device, &v, nullptr, &scViews[i]));
        }
    }

    void createRenderPass() {
        VkAttachmentDescription color{};
        color.format = scFormat;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        VkAttachmentReference ref{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1; sub.pColorAttachments = &ref;
        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL; dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        VkRenderPassCreateInfo ci{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        ci.attachmentCount = 1; ci.pAttachments = &color;
        ci.subpassCount = 1; ci.pSubpasses = &sub;
        ci.dependencyCount = 1; ci.pDependencies = &dep;
        VKCHECK(vkCreateRenderPass(device, &ci, nullptr, &renderPass));
    }

    void createFramebuffers() {
        scFbos.resize(scViews.size());
        for (size_t i = 0; i < scViews.size(); i++) {
            VkFramebufferCreateInfo f{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
            f.renderPass = renderPass; f.attachmentCount = 1; f.pAttachments = &scViews[i];
            f.width = scExtent.width; f.height = scExtent.height; f.layers = 1;
            VKCHECK(vkCreateFramebuffer(device, &f, nullptr, &scFbos[i]));
        }
    }

    void createCommands() {
        VkCommandPoolCreateInfo p{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        p.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        p.queueFamilyIndex = gfxFamily;
        VKCHECK(vkCreateCommandPool(device, &p, nullptr, &cmdPool));
        cmds.resize(MAX_FRAMES);
        VkCommandBufferAllocateInfo a{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        a.commandPool = cmdPool; a.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        a.commandBufferCount = MAX_FRAMES;
        VKCHECK(vkAllocateCommandBuffers(device, &a, cmds.data()));
    }

    void createSync() {
        VkSemaphoreCreateInfo s{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkFenceCreateInfo f{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        f.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (int i = 0; i < MAX_FRAMES; i++) {
            VKCHECK(vkCreateSemaphore(device, &s, nullptr, &imgAvail[i]));
            VKCHECK(vkCreateSemaphore(device, &s, nullptr, &renderDone[i]));
            VKCHECK(vkCreateFence(device, &f, nullptr, &inFlight[i]));
        }
    }

    void recreateSwapchain() {
        int w = 0, h = 0;
        glfwGetFramebufferSize(win, &w, &h);
        while (w == 0 || h == 0) { glfwGetFramebufferSize(win, &w, &h); glfwWaitEvents(); }
        vkDeviceWaitIdle(device);
        for (auto fb : scFbos) vkDestroyFramebuffer(device, fb, nullptr);
        for (auto v : scViews) vkDestroyImageView(device, v, nullptr);
        vkDestroySwapchainKHR(device, swapchain, nullptr);
        createSwapchain(); createFramebuffers();
    }

    void recordClear(VkCommandBuffer cb, uint32_t img) {
        VkCommandBufferBeginInfo b{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        VKCHECK(vkBeginCommandBuffer(cb, &b));
        VkClearValue clear{};
        memcpy(clear.color.float32, SKY, sizeof(SKY));
        VkRenderPassBeginInfo rp{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
        rp.renderPass = renderPass; rp.framebuffer = scFbos[img];
        rp.renderArea.extent = scExtent;
        rp.clearValueCount = 1; rp.pClearValues = &clear;
        vkCmdBeginRenderPass(cb, &rp, VK_SUBPASS_CONTENTS_INLINE);
        // milestone >=2: bind pipelines + draw terrain/coaster/HUD here.
        vkCmdEndRenderPass(cb);
        VKCHECK(vkEndCommandBuffer(cb));
    }

    void drawFrame() {
        vkWaitForFences(device, 1, &inFlight[frame], VK_TRUE, UINT64_MAX);
        uint32_t img;
        VkResult r = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
                                           imgAvail[frame], VK_NULL_HANDLE, &img);
        if (r == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapchain(); return; }
        if (r != VK_SUCCESS && r != VK_SUBOPTIMAL_KHR) { fprintf(stderr, "acquire %d\n", r); exit(1); }
        vkResetFences(device, 1, &inFlight[frame]);
        vkResetCommandBuffer(cmds[frame], 0);
        recordClear(cmds[frame], img);

        VkPipelineStageFlags wait = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
        si.waitSemaphoreCount = 1; si.pWaitSemaphores = &imgAvail[frame]; si.pWaitDstStageMask = &wait;
        si.commandBufferCount = 1; si.pCommandBuffers = &cmds[frame];
        si.signalSemaphoreCount = 1; si.pSignalSemaphores = &renderDone[frame];
        VKCHECK(vkQueueSubmit(gfxQueue, 1, &si, inFlight[frame]));

        VkPresentInfoKHR pi{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
        pi.waitSemaphoreCount = 1; pi.pWaitSemaphores = &renderDone[frame];
        pi.swapchainCount = 1; pi.pSwapchains = &swapchain; pi.pImageIndices = &img;
        r = vkQueuePresentKHR(presentQueue, &pi);
        if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR || resized) {
            resized = false; recreateSwapchain();
        } else if (r != VK_SUCCESS) { fprintf(stderr, "present %d\n", r); exit(1); }
        frame = (frame + 1) % MAX_FRAMES;
    }

    void loop() {
        int frames = 0;
        bool headless = getenv("VK_HEADLESS_FRAMES") != nullptr;
        int maxFrames = headless ? atoi(getenv("VK_HEADLESS_FRAMES")) : -1;
        while (!glfwWindowShouldClose(win)) {
            glfwPollEvents();
            drawFrame();
            if (maxFrames > 0 && ++frames >= maxFrames) break;
        }
        vkDeviceWaitIdle(device);
        printf("Vulkan milestone-1 OK: instance+device+swapchain+clear ran (%d frames).\n",
               frames ? frames : -1);
    }

    void cleanup() {
        for (int i = 0; i < MAX_FRAMES; i++) {
            vkDestroySemaphore(device, imgAvail[i], nullptr);
            vkDestroySemaphore(device, renderDone[i], nullptr);
            vkDestroyFence(device, inFlight[i], nullptr);
        }
        vkDestroyCommandPool(device, cmdPool, nullptr);
        for (auto fb : scFbos) vkDestroyFramebuffer(device, fb, nullptr);
        vkDestroyRenderPass(device, renderPass, nullptr);
        for (auto v : scViews) vkDestroyImageView(device, v, nullptr);
        vkDestroySwapchainKHR(device, swapchain, nullptr);
        vkDestroyDevice(device, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr);
        vkDestroyInstance(instance, nullptr);
        glfwDestroyWindow(win);
        glfwTerminate();
    }
};

int main() {
    VkApp app;
    app.run();
    return 0;
}

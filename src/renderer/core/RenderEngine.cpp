#include "RenderEngine.hpp"
#include "VulkanInternals.hpp"

#include "renderer/geometries/RendererCurve.hpp"
#include "renderer/geometries/RendererObject.hpp"

#include <nfd.h>

#include <fstream>
#include <cstring>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <set>

using namespace renderer;

std::vector<char> readShaderFile(const std::string& relativePath) {
    const std::vector<std::filesystem::path> searchPaths = {
        std::filesystem::current_path() / "../" / relativePath,
        std::filesystem::current_path() / "../shaders" / relativePath
    };

    for (const auto& path : searchPaths) {
        if (std::filesystem::exists(path)) {
            std::ifstream file(path, std::ios::ate | std::ios::binary);
            size_t fileSize = (size_t)file.tellg();
            std::vector<char> buffer(fileSize);
            file.seekg(0);
            file.read(buffer.data(), fileSize);
            file.close();
            return buffer;
        }
    }
    throw std::runtime_error("Failed to find shader: " + relativePath + " : " + std::filesystem::current_path().generic_string());
}



// RenderEngine::VulkanInternals implementation
// ================================================================

RenderEngine::VulkanInternals::VulkanInternals() {
    cameraPosition = glm::vec3{ 0.f, -2.f, 0.f };
	cameraFront = glm::vec3{ 0.f, 1.f, 0.f };
	cameraRight = glm::vec3{ 1.f, 0.f, 0.f };
    
    currentFrame = 0;
    frameBufferResized = false;
}

bool RenderEngine::VulkanInternals::QueueFamilyIndices::isComplete() {
    return graphicsFamily.has_value() && presentFamily.has_value();
}

void RenderEngine::VulkanInternals::initWindow() {
    glfwInit();

    // Tell GLFW to not create a OpenGL context, since vulkan is used here
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    window = glfwCreateWindow(WIDTH, HEIGHT, "Vulkan Test", nullptr, nullptr);
    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
}

void RenderEngine::VulkanInternals::initVulkan() {
    createInstance();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapChain();
    createImageViews();
    createRenderPass();
    createDescriptorSetLayouts();
    createObjectPipelines();
    createLineBasedPipeline();
    createCommandPool();
    createDepthResources();
    createExtraRenderBuffers();
    createFrameBuffers();
    createUniformBuffers();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();
}

void RenderEngine::VulkanInternals::initImgui() {
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsClassic();
    ImGui_ImplGlfw_InitForVulkan(window, true);

    ImGui_ImplVulkan_InitInfo info;
    info.DescriptorPool = vulkanContext.descriptorPool;
    info.RenderPass = renderPass;
    info.Device = vulkanContext.device;
    info.PhysicalDevice = vulkanContext.physicalDevice;
    info.ImageCount = MAX_FRAMES_IN_FLIGHT;
    info.MsaaSamples = (VkSampleCountFlagBits)0x00000001;
    info.Subpass = 2;
    ImGui_ImplVulkan_Init(&info);

    VkCommandBuffer ImCommandBuffer = beginSingleTimeCommands(vulkanContext);
    ImGui_ImplVulkan_CreateFontsTexture(ImCommandBuffer);
    endSingleTimeCommands(vulkanContext, ImCommandBuffer);

    vkDeviceWaitIdle(vulkanContext.device);
    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void RenderEngine::VulkanInternals::pickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

    if (deviceCount == 0) {
        throw std::runtime_error("failed to find GPUs with Vulkan support!");
    }

    std::vector<VkPhysicalDevice> devices{ deviceCount };
    vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

    for (const auto& device : devices) {
        if (isDeviceSuitable(device)) {
            vulkanContext.physicalDevice = device;
            break;
        }
    }

    if (vulkanContext.physicalDevice == VK_NULL_HANDLE) {
        throw std::runtime_error("failed to find a suitable GPU!");
    }
}

void RenderEngine::VulkanInternals::createLogicalDevice() {
    QueueFamilyIndices indices = findQueueFamilies(vulkanContext.physicalDevice);

    // Define the queues we want to create
    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
    std::set<uint32_t> uniqueQueueFamilies = { indices.graphicsFamily.value(), indices.presentFamily.value() };

    float queuePriority = 1.0f;
    for (uint32_t queueFamily : uniqueQueueFamilies) {
        VkDeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo.queueFamilyIndex = queueFamily;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        queueCreateInfos.push_back(queueCreateInfo);
    }

    // Define what device features we want to use
    VkPhysicalDeviceFeatures deviceFeatures{};
    deviceFeatures.samplerAnisotropy = VK_TRUE;
    deviceFeatures.wideLines = VK_TRUE;
    deviceFeatures.independentBlend = VK_TRUE;

    // Create the logical device
    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pEnabledFeatures = &deviceFeatures;

    createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
    createInfo.pQueueCreateInfos = queueCreateInfos.data();
    createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    // Set validation layers (depracated in modern vulkan releases, but added for backwards compatability
    if (enableValidationLayers) {
        createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
        createInfo.ppEnabledLayerNames = validationLayers.data();
    }
    else {
        createInfo.enabledLayerCount = 0;
    }

    // Finally actually create the logical device
    if (vkCreateDevice(vulkanContext.physicalDevice, &createInfo, nullptr, &vulkanContext.device) != VK_SUCCESS) {
        throw std::runtime_error("failed to create logical device!");
    }

    // Retrieve the queue implicitly created by the logical device
    vkGetDeviceQueue(vulkanContext.device, indices.graphicsFamily.value(), 0, &vulkanContext.graphicsQueue);
    vkGetDeviceQueue(vulkanContext.device, indices.presentFamily.value(), 0, &vulkanContext.presentQueue);
}

bool RenderEngine::VulkanInternals::isDeviceSuitable(VkPhysicalDevice device) {
    VkPhysicalDeviceProperties deviceProperties;
    vkGetPhysicalDeviceProperties(device, &deviceProperties);

    VkPhysicalDeviceFeatures deviceFeatures;
    vkGetPhysicalDeviceFeatures(device, &deviceFeatures);

    QueueFamilyIndices indices = findQueueFamilies(device);

    bool extensionsSupported = checkDeviceExtensionSupport(device);

    bool swapChainAdequate = false;
    if (extensionsSupported) {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);
        swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
    }

    return
        deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU &&
        deviceFeatures.geometryShader &&
        deviceFeatures.samplerAnisotropy &&
        extensionsSupported &&
        swapChainAdequate &&
        indices.isComplete();
}

bool RenderEngine::VulkanInternals::checkDeviceExtensionSupport(VkPhysicalDevice device) {
    uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

    std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

    for (const auto& extension : availableExtensions) {
        requiredExtensions.erase(extension.extensionName);
    }

    return requiredExtensions.empty();
}

RenderEngine::VulkanInternals::QueueFamilyIndices RenderEngine::VulkanInternals::findQueueFamilies(VkPhysicalDevice device) {
    QueueFamilyIndices indices;

    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

    std::vector<VkQueueFamilyProperties> queueFamilies{ queueFamilyCount };
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

    int i = 0;
    for (const auto& queueFamily : queueFamilies) {
        if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            indices.graphicsFamily = i;
        }

        VkBool32 presentSupport = false;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);
        if (presentSupport) {
            indices.presentFamily = i;
        }

        if (indices.isComplete()) {
            break;
        }

        i++;
    }

    return indices;
}

RenderEngine::VulkanInternals::SwapChainSupportDetails RenderEngine::VulkanInternals::querySwapChainSupport(VkPhysicalDevice device) {
    SwapChainSupportDetails details;

    // Load supported surface capabilities
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);

    // Load supported surface formats
    uint32_t formatCount;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);

    if (formatCount != 0) {
        details.formats.resize(formatCount);
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, details.formats.data());
    }

    // Load supported present modes
    uint32_t presentModeCount;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);

    if (presentModeCount != 0) {
        details.presentModes.resize(presentModeCount);
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
    }

    return details;
}

void RenderEngine::VulkanInternals::createSwapChain() {
    SwapChainSupportDetails swapChainSupport = querySwapChainSupport(vulkanContext.physicalDevice);

    VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
    VkPresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
    VkExtent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

    // Define the amount of images in the queue of the swapchain. Recommended to slightly increase the minimum to reduce chances of having to wait on the driver
    uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;

    // Ensure by increasing the imagecount by one, the maximum allowable isn't exceeded
    if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount) {
        imageCount = swapChainSupport.capabilities.maxImageCount;
    }

    // create and fill in the creation struct for the swap chain
    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = surface;

    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    QueueFamilyIndices indices = findQueueFamilies(vulkanContext.physicalDevice);
    uint32_t queueFamilyIndices[] = { indices.graphicsFamily.value(), indices.presentFamily.value() };

    // Depending on if both the graphics queue, on which we'll be 'drawing', and the present queue, on which we'll 'submit' them, are the same queue, set the image sharing settings
    if (indices.graphicsFamily != indices.presentFamily) {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    }
    else {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.queueFamilyIndexCount = 0;
        createInfo.pQueueFamilyIndices = nullptr;
    }

    createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    // Actually create the swapchain 
    if (vkCreateSwapchainKHR(vulkanContext.device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) {
        throw std::runtime_error("failed to create swap chain!");
    }

    // Retrieve the handles for the images in the swapchain
    vkGetSwapchainImagesKHR(vulkanContext.device, swapChain, &imageCount, nullptr);
    swapChainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(vulkanContext.device, swapChain, &imageCount, swapChainImages.data());

    // Set private variables, so swapchain settings can be used elsewhere
    swapChainImageFormat = surfaceFormat.format;
    swapChainExtent = extent;
}

void RenderEngine::VulkanInternals::cleanupSwapChain() {
    vkDestroyImageView(vulkanContext.device, accumulationBufferView, nullptr);
    vkDestroyImage(vulkanContext.device, accumulationBuffer, nullptr);
    vkFreeMemory(vulkanContext.device, accumulationBufferMemory, nullptr);

    vkDestroyImageView(vulkanContext.device, revealageBufferView, nullptr);
    vkDestroyImage(vulkanContext.device, revealageBuffer, nullptr);
    vkFreeMemory(vulkanContext.device, revealageBufferMemory, nullptr);

    vkDestroyImageView(vulkanContext.device, depthImageView, nullptr);
    vkDestroyImage(vulkanContext.device, depthImage, nullptr);
    vkFreeMemory(vulkanContext.device, depthImageMemory, nullptr);

    for (auto framebuffer : swapChainFramebuffers) {
        vkDestroyFramebuffer(vulkanContext.device, framebuffer, nullptr);
    }
    for (auto imageView : swapChainImageViews) {
        vkDestroyImageView(vulkanContext.device, imageView, nullptr);
    }
    vkDestroySwapchainKHR(vulkanContext.device, swapChain, nullptr);
}

void RenderEngine::VulkanInternals::recreateSwapChain() {
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    while (width == 0 || height == 0) {
        glfwGetFramebufferSize(window, &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(vulkanContext.device);

    cleanupSwapChain();

    createSwapChain();
    createImageViews();
    createDepthResources();

    createExtraRenderBuffers();
    updateCompositeDescriptorSet();

    createFrameBuffers();
}

void RenderEngine::VulkanInternals::createImageViews() {
    swapChainImageViews.resize(swapChainImages.size());
    for (size_t i = 0; i < swapChainImages.size(); i++) {
        swapChainImageViews[i] = createImageView(vulkanContext, swapChainImages[i], swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
    }
}

void RenderEngine::VulkanInternals::createRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapChainImageFormat;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = findDepthFormat();
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription accumulationAttachment{};
    accumulationAttachment.format = VK_FORMAT_R16G16B16A16_SFLOAT;
    accumulationAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    accumulationAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    accumulationAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    accumulationAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    accumulationAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    accumulationAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription revealageAttachment{};
    revealageAttachment.format = VK_FORMAT_R8_UNORM;
    revealageAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    revealageAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    revealageAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    revealageAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    revealageAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    revealageAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    
    std::array<VkAttachmentDescription, 4> attachments = { colorAttachment, depthAttachment, accumulationAttachment, revealageAttachment };

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference accumulationWriteAttachmentRef{};
    accumulationWriteAttachmentRef.attachment = 2;
    accumulationWriteAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference accumulationReadAttachmentRef{};
    accumulationReadAttachmentRef.attachment = 2;
    accumulationReadAttachmentRef.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentReference revealageWriteAttachmentRef{};
    revealageWriteAttachmentRef.attachment = 3;
    revealageWriteAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference revealageReadAttachmentRef{};
    revealageReadAttachmentRef.attachment = 3;
    revealageReadAttachmentRef.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkSubpassDescription subpassOpaque {};
    subpassOpaque.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpassOpaque.colorAttachmentCount = 1;
    subpassOpaque.pColorAttachments = &colorAttachmentRef;
    subpassOpaque.pDepthStencilAttachment = &depthAttachmentRef;

    std::array<VkAttachmentReference, 2> transparentAttachmentRefs = { accumulationWriteAttachmentRef, revealageWriteAttachmentRef };
    VkSubpassDescription subpassTransparent {};
    subpassTransparent.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpassTransparent.colorAttachmentCount = 2;
    subpassTransparent.pColorAttachments = transparentAttachmentRefs.data();
    subpassTransparent.pDepthStencilAttachment = &depthAttachmentRef;

    std::array<VkAttachmentReference, 2> compositeAttachmentRefs = { accumulationReadAttachmentRef, revealageReadAttachmentRef };
    VkSubpassDescription subpassComposite {};
    subpassComposite.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpassComposite.colorAttachmentCount = 1;
    subpassComposite.pColorAttachments = &colorAttachmentRef;
    subpassComposite.inputAttachmentCount = 2;
    subpassComposite.pInputAttachments = compositeAttachmentRefs.data();
    subpassComposite.pDepthStencilAttachment = nullptr;

    std::array<VkSubpassDescription, 3> subpasses = { subpassOpaque, subpassTransparent, subpassComposite };

    // Transition from external to subpass 0 (opaque)
    VkSubpassDependency dependency0 {};
    dependency0.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency0.dstSubpass = 0;
    dependency0.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency0.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency0.srcAccessMask = 0;
    dependency0.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependency0.dependencyFlags = 0;

    // Transition from subpass 0 (opaque) to subpass 1 (transparent)
    VkSubpassDependency dependency1 {};
    dependency1.srcSubpass = 0;
    dependency1.dstSubpass = 1;
    dependency1.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependency1.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependency1.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependency1.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    dependency1.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    // Transition from subpass 1 (transparent) to subpass 2 (composite)
    VkSubpassDependency dependency2 {};
    dependency2.srcSubpass = 1;
    dependency2.dstSubpass = 2;
    dependency2.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency2.dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependency2.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependency2.dstAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_SHADER_READ_BIT;
    dependency2.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    /*
    // Transition from subpass 2 (composite) to external (presentation)
    VkSubpassDependency dependency3 {};
    dependency2.srcSubpass = 2;
    dependency2.dstSubpass = VK_SUBPASS_EXTERNAL;
    dependency2.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency2.dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependency2.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependency2.dstAccessMask = 0;
    dependency2.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    std::array<VkSubpassDependency, 4> dependencies = { dependency0, dependency1, dependency2, dependency3 };*/
    std::array<VkSubpassDependency, 3> dependencies = { dependency0, dependency1, dependency2 };

    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<uint32_t>(subpasses.size());
    renderPassInfo.pSubpasses = subpasses.data();
    renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();

    if (vkCreateRenderPass(vulkanContext.device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) {
        throw std::runtime_error("failed to create render pass!");
    }
}

void RenderEngine::VulkanInternals::createDescriptorSetLayouts() {
    VkDescriptorSetLayoutBinding uboLayoutBinding{};
    uboLayoutBinding.binding = 0;
    uboLayoutBinding.descriptorCount = 1;
    uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    VkDescriptorSetLayoutBinding samplerLayoutBinding{};
    samplerLayoutBinding.binding = 0;
    samplerLayoutBinding.descriptorCount = 1;
    samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    samplerLayoutBinding.pImmutableSamplers = nullptr;
    samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    
    VkDescriptorSetLayoutBinding accumulationLayoutBinding{};
    accumulationLayoutBinding.binding = 0;
    accumulationLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    accumulationLayoutBinding.descriptorCount = 1;
    accumulationLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding revealageLayoutBinding{};
    revealageLayoutBinding.binding = 1;
    revealageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    revealageLayoutBinding.descriptorCount = 1;
    revealageLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::array<VkDescriptorSetLayoutBinding, 2> inputAttachmentBindings = { accumulationLayoutBinding, revealageLayoutBinding };

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &uboLayoutBinding;

    if (vkCreateDescriptorSetLayout(vulkanContext.device, &layoutInfo, nullptr, &vulkanContext.uniformDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create uniform descriptor set layout!");
    }

    layoutInfo.pBindings = &samplerLayoutBinding;

    if (vkCreateDescriptorSetLayout(vulkanContext.device, &layoutInfo, nullptr, &vulkanContext.textureDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture descriptor set layout!");
    }

    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = inputAttachmentBindings.data();

    if (vkCreateDescriptorSetLayout(vulkanContext.device, &layoutInfo, nullptr, &vulkanContext.compositeDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create composite descriptor set layout!");
    }
}

void RenderEngine::VulkanInternals::createObjectPipelines() {
    createObjectOpaquePipeline();
    createObjectTransparentPipeline();
    createObjectCompositePipeline();
}

void RenderEngine::VulkanInternals::createObjectOpaquePipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("opaqueObjectShaderVert.spv");
    auto fragShaderCode = readShaderFile("opaqueObjectShaderFrag.spv");

    // wrap them in the appropriate Vulkan struct
    VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

    // assign the vertex shader to its spot in the pipeline
    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule;
    fragShaderStageInfo.pName = "main";

    // put the two info structs in a basic array
    VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

    // Enable the dynamic bit of the pipeline state
    std::vector<VkDynamicState> dynamicStates = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };

    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    // Declare the format of the vertex data sent to the vertex shader
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    auto bindingDescription = RendererVertex::getBindingDescription();
    auto attributeDescriptions = RendererVertex::getAttributeDescriptions();

    vertexInputInfo.vertexBindingDescriptionCount = 1;
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());

    vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    // Declare the way and what type of primitives are made from the vertex data
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // Declare the viewport settings
    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    // Declare rasterizer settings
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;

    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    // Declare multisampling settings
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0f; // Optional
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    // Declare color blending settings
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    VkDescriptorSetLayout setLayouts[] = { vulkanContext.uniformDescriptorSetLayout, vulkanContext.textureDescriptorSetLayout };

    VkPushConstantRange psRange;
    psRange.offset = 0;
    psRange.size = sizeof(ObjectShaderPushConstant);
    psRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    // Declare the pipeline layout (mainly what uniforms are sent to the vertex shader)
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 2;
    pipelineLayoutInfo.pSetLayouts = setLayouts;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &psRange;

    if (vkCreatePipelineLayout(vulkanContext.device, &pipelineLayoutInfo, nullptr, &objectOpaquePipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create object opaque pipeline layout!");
    }

    // Finally declare the actual create info struct
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;

    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = objectOpaquePipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Actually create the VkPipeline
    if (vkCreateGraphicsPipelines(vulkanContext.device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &objectOpaquePipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create object opaque graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(vulkanContext.device, vertShaderModule, nullptr);
    vkDestroyShaderModule(vulkanContext.device, fragShaderModule, nullptr);
}

void RenderEngine::VulkanInternals::createObjectTransparentPipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("transparentObjectShaderVert.spv");
    auto fragShaderCode = readShaderFile("transparentObjectShaderFrag.spv");

    // wrap them in the appropriate Vulkan struct
    VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

    // assign the vertex shader to its spot in the pipeline
    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule;
    fragShaderStageInfo.pName = "main";

    // put the two info structs in a basic array
    VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

    // Enable the dynamic bit of the pipeline state
    std::vector<VkDynamicState> dynamicStates = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };

    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    // Declare the format of the vertex data sent to the vertex shader
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    auto bindingDescription = RendererVertex::getBindingDescription();
    auto attributeDescriptions = RendererVertex::getAttributeDescriptions();

    vertexInputInfo.vertexBindingDescriptionCount = 1;
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());

    vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    // Declare the way and what type of primitives are made from the vertex data
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // Declare the viewport settings
    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    // Declare rasterizer settings
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;

    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    // Declare multisampling settings
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0f; // Optional
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    // Declare color blending settings
    // since this pipeline will be writing to 2 buffers, we need 2 blendAttachments
    VkPipelineColorBlendAttachmentState colorBlendAttachmentAccumulation{};
    colorBlendAttachmentAccumulation.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachmentAccumulation.blendEnable = VK_TRUE;
    colorBlendAttachmentAccumulation.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachmentAccumulation.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachmentAccumulation.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachmentAccumulation.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachmentAccumulation.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachmentAccumulation.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendAttachmentState colorBlendAttachmentRevealage{};
    colorBlendAttachmentRevealage.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    colorBlendAttachmentRevealage.blendEnable = VK_TRUE;
    colorBlendAttachmentRevealage.srcColorBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachmentRevealage.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
    colorBlendAttachmentRevealage.colorBlendOp = VK_BLEND_OP_ADD;

    std::array<VkPipelineColorBlendAttachmentState, 2> colorBlendAttachments = { colorBlendAttachmentAccumulation, colorBlendAttachmentRevealage };

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 2;
    colorBlending.pAttachments = colorBlendAttachments.data();

    VkDescriptorSetLayout setLayouts[] = { vulkanContext.uniformDescriptorSetLayout, vulkanContext.textureDescriptorSetLayout };

    VkPushConstantRange psRange;
    psRange.offset = 0;
    psRange.size = sizeof(ObjectShaderPushConstant);
    psRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    // Declare the pipeline layout (mainly what uniforms are sent to the vertex shader)
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 2;
    pipelineLayoutInfo.pSetLayouts = setLayouts;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &psRange;

    if (vkCreatePipelineLayout(vulkanContext.device, &pipelineLayoutInfo, nullptr, &objectTransparentPipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create object transparent pipeline layout!");
    }

    // Finally declare the actual create info struct
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;

    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = objectTransparentPipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 1;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Actually create the VkPipeline
    if (vkCreateGraphicsPipelines(vulkanContext.device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &objectTransparentPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create object transparent graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(vulkanContext.device, vertShaderModule, nullptr);
    vkDestroyShaderModule(vulkanContext.device, fragShaderModule, nullptr);
}

void RenderEngine::VulkanInternals::createObjectCompositePipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("compositeObjectShaderVert.spv");
    auto fragShaderCode = readShaderFile("compositeObjectShaderFrag.spv");

    // wrap them in the appropriate Vulkan struct
    VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

    // assign the vertex shader to its spot in the pipeline
    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule;
    fragShaderStageInfo.pName = "main";

    // put the two info structs in a basic array
    VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

    // Enable the dynamic bit of the pipeline state
    std::vector<VkDynamicState> dynamicStates = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };

    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    // Since the composite shader only combines the results from earlier subpasses, no vertex data is needed
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = 0;
    vertexInputInfo.vertexAttributeDescriptionCount = 0;

    // Declare the way and what type of primitives are made from the vertex data
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // Declare the viewport settings
    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    // Declare rasterizer settings
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;

    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    // Declare multisampling settings
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0f; // Optional
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    // Declare color blending settings
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    // Declare the pipeline layout (mainly what uniforms are sent to the vertex shader)
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &vulkanContext.compositeDescriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 0;

    if (vkCreatePipelineLayout(vulkanContext.device, &pipelineLayoutInfo, nullptr, &objectCompositePipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }

    // Finally declare the actual create info struct
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;

    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = objectCompositePipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 2;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Actually create the VkPipeline
    if (vkCreateGraphicsPipelines(vulkanContext.device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &objectCompositePipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(vulkanContext.device, vertShaderModule, nullptr);
    vkDestroyShaderModule(vulkanContext.device, fragShaderModule, nullptr);
}

void RenderEngine::VulkanInternals::createLineBasedPipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("shaders/lineShaderVert.spv");
    auto fragShaderCode = readShaderFile("shaders/lineShaderFrag.spv");

    // wrap them in the appropriate Vulkan struct
    VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

    // assign the vertex shader to its spot in the pipeline
    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule;
    fragShaderStageInfo.pName = "main";

    // put the two info structs in a basic array
    VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

    // Enable the dynamic bit of the pipeline state
    std::vector<VkDynamicState> dynamicStates = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR,
        VK_DYNAMIC_STATE_LINE_WIDTH
    };

    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    // Declare the format of the vertex data sent to the vertex shader
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    auto bindingDescription = RendererVertex::getBindingDescription();
    auto attributeDescriptions = RendererVertex::getAttributeDescriptions();

    vertexInputInfo.vertexBindingDescriptionCount = 1;
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());

    vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    // Declare the way and what type of primitives are made from the vertex data
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // Declare the viewport settings
    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    // Declare rasterizer settings
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;

    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    // Declare multisampling settings
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0f; // Optional
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    // Declare color blending settings
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    // the layouts declare the uniform buffers that will be sent to the shaders
    VkDescriptorSetLayout setLayouts[] = { vulkanContext.uniformDescriptorSetLayout };

    // define the settings for the push constants used, in this case stuff like transparency, default color, etc...
    VkPushConstantRange psRange;
    psRange.offset = 0;
    psRange.size = sizeof(LineShaderPushConstant);
    psRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    // Declare the pipeline layout (mainly what uniforms are sent to the vertex shader)
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = setLayouts;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &psRange;

    if (vkCreatePipelineLayout(vulkanContext.device, &pipelineLayoutInfo, nullptr, &lineBasedPipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create line based pipeline layout!");
    }

    // Finally declare the actual create info struct
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;

    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = lineBasedPipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Actually create the VkPipeline
    if (vkCreateGraphicsPipelines(vulkanContext.device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &lineBasedPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(vulkanContext.device, vertShaderModule, nullptr);
    vkDestroyShaderModule(vulkanContext.device, fragShaderModule, nullptr);
}

void RenderEngine::VulkanInternals::createFrameBuffers() {
    swapChainFramebuffers.resize(swapChainImageViews.size());

    for (size_t i = 0; i < swapChainImageViews.size(); i++) {
        std::array<VkImageView, 4> attachments = {
                swapChainImageViews[i],
                depthImageView,
                accumulationBufferView,
                revealageBufferView
        };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = swapChainExtent.width;
        framebufferInfo.height = swapChainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(vulkanContext.device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
    }
}

void RenderEngine::VulkanInternals::createCommandPool() {
    QueueFamilyIndices queueFamilyIndices = findQueueFamilies(vulkanContext.physicalDevice);

    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

    if (vkCreateCommandPool(vulkanContext.device, &poolInfo, nullptr, &vulkanContext.commandPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create command pool!");
    }
}

VkFormat RenderEngine::VulkanInternals::findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features) {
    for (VkFormat format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(vulkanContext.physicalDevice, format, &props);

        if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features) {
            return format;
        }
        else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features) {
            return format;
        }
    }

    throw std::runtime_error("failed to find supported format!");
}

VkFormat RenderEngine::VulkanInternals::findDepthFormat() {
    return findSupportedFormat(
        { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
    );
}

void RenderEngine::VulkanInternals::createDepthResources() {
    VkFormat depthFormat = findDepthFormat();

    createImage(vulkanContext, swapChainExtent.width, swapChainExtent.height, depthFormat, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depthImage, depthImageMemory);
    depthImageView = createImageView(vulkanContext, depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

    transitionImageLayout(vulkanContext, depthImage, depthFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT);
}

void RenderEngine::VulkanInternals::createExtraRenderBuffers() {
    createImage(vulkanContext, swapChainExtent.width, swapChainExtent.height, VK_FORMAT_R16G16B16A16_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accumulationBuffer, accumulationBufferMemory);
    accumulationBufferView = createImageView(vulkanContext, accumulationBuffer, VK_FORMAT_R16G16B16A16_SFLOAT, VK_IMAGE_ASPECT_COLOR_BIT);

    createImage(vulkanContext, swapChainExtent.width, swapChainExtent.height, VK_FORMAT_R8_UNORM, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, revealageBuffer, revealageBufferMemory);
    revealageBufferView = createImageView(vulkanContext, revealageBuffer, VK_FORMAT_R8_UNORM, VK_IMAGE_ASPECT_COLOR_BIT);
}

void RenderEngine::VulkanInternals::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);

    uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        createBuffer(vulkanContext, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, uniformBuffers[i], uniformBuffersMemory[i]);

        vkMapMemory(vulkanContext.device, uniformBuffersMemory[i], 0, bufferSize, 0, &uniformBuffersMapped[i]);
    }
}

void RenderEngine::VulkanInternals::updateUniformBuffer(uint32_t currentImage) {

    auto newCurrentTime = std::chrono::high_resolution_clock::now();
    deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(newCurrentTime - oldCurrentTime);
    oldCurrentTime = newCurrentTime;

    UniformBufferObject ubo{};
    ubo.model = glm::mat4(1.0); // this model matrix has become obsolete, since the model matrix is now updated for every object in a single frame through a uniform buffer
    ubo.view = glm::lookAt(cameraPosition, cameraPosition + cameraFront, glm::vec3{ 0.f, 0.f, 1.f });
    ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float)swapChainExtent.height, 0.001f, 10000.0f);
    ubo.proj[1][1] *= -1;

    memcpy(uniformBuffersMapped[currentImage], &ubo, sizeof(ubo));
}

void RenderEngine::VulkanInternals::createDescriptorPool() {
    std::array<VkDescriptorPoolSize, 3> poolSizes{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = renderer::MAX_OBJECTS;
    poolSizes[2].type = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    poolSizes[2].descriptorCount = 2;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT) + renderer::MAX_OBJECTS + 2;

    if (vkCreateDescriptorPool(vulkanContext.device, &poolInfo, nullptr, &vulkanContext.descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}

void RenderEngine::VulkanInternals::createDescriptorSets() {
    std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, vulkanContext.uniformDescriptorSetLayout);
    VkDescriptorSetAllocateInfo UBOAllocInfo{};
    UBOAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    UBOAllocInfo.descriptorPool = vulkanContext.descriptorPool;
    UBOAllocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    UBOAllocInfo.pSetLayouts = layouts.data();

    UBODescriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
    if (vkAllocateDescriptorSets(vulkanContext.device, &UBOAllocInfo, UBODescriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate UBO descriptor sets!");
    }

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = uniformBuffers[i];
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(UniformBufferObject);

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = UBODescriptorSets[i];
        descriptorWrite.dstBinding = 0;
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pBufferInfo = &bufferInfo;

        vkUpdateDescriptorSets(vulkanContext.device, 1, &descriptorWrite, 0, nullptr);
    }

    VkDescriptorSetAllocateInfo compositeAllocInfo{};
    compositeAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    compositeAllocInfo.descriptorPool = vulkanContext.descriptorPool;
    compositeAllocInfo.descriptorSetCount = 1;
    compositeAllocInfo.pSetLayouts = &vulkanContext.compositeDescriptorSetLayout;

    if (vkAllocateDescriptorSets(vulkanContext.device, &compositeAllocInfo, &compositeDescriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate composite descriptor set!");
    }

    updateCompositeDescriptorSet();
}

void RenderEngine::VulkanInternals::updateCompositeDescriptorSet() {
    VkDescriptorImageInfo accumulationImageInfo{};
    accumulationImageInfo.sampler = VK_NULL_HANDLE;
    accumulationImageInfo.imageView = accumulationBufferView;
    accumulationImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo revealageImageInfo{};
    revealageImageInfo.sampler = VK_NULL_HANDLE;
    revealageImageInfo.imageView = revealageBufferView;
    revealageImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet accumulationDescriptorWrite{};
    accumulationDescriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    accumulationDescriptorWrite.dstSet = compositeDescriptorSet;
    accumulationDescriptorWrite.dstBinding = 0;
    accumulationDescriptorWrite.descriptorCount = 1;
    accumulationDescriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    accumulationDescriptorWrite.pImageInfo = &accumulationImageInfo;

    VkWriteDescriptorSet revealageDescriptorWrite{};
    revealageDescriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    revealageDescriptorWrite.dstSet = compositeDescriptorSet;
    revealageDescriptorWrite.dstBinding = 1;
    revealageDescriptorWrite.descriptorCount = 1;
    revealageDescriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    revealageDescriptorWrite.pImageInfo = &revealageImageInfo;

    std::array<VkWriteDescriptorSet, 2> descriptorWrites = { accumulationDescriptorWrite, revealageDescriptorWrite };

    vkUpdateDescriptorSets(vulkanContext.device, 2, descriptorWrites.data(), 0, nullptr);
}

void RenderEngine::VulkanInternals::createCommandBuffers() {
    commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = vulkanContext.commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = (uint32_t)MAX_FRAMES_IN_FLIGHT;

    if (vkAllocateCommandBuffers(vulkanContext.device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate command buffer!");
    }
}

void RenderEngine::VulkanInternals::createSyncObjects() {
    imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        if (vkCreateSemaphore(vulkanContext.device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
            vkCreateSemaphore(vulkanContext.device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
            vkCreateFence(vulkanContext.device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) {

            throw std::runtime_error("failed to create synchronization objects for a frame!");
        }
    }
}

void RenderEngine::VulkanInternals::recordCommandBuffer(std::list<std::shared_ptr<renderer::Curve>>& curves, std::list<std::shared_ptr<renderer::Object>>& objects, std::function<void()> drawGUIFunction, VkCommandBuffer commandBuffer, uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    beginInfo.pInheritanceInfo = nullptr; // Optional

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("failed to begin recording command buffer!");
    }

    std::array<VkClearValue, 4> clearValues{};
    clearValues[0].color = { {CLEAR_COLOR.x, CLEAR_COLOR.y, CLEAR_COLOR.z, 1.f} };  // swapchain
    clearValues[1].depthStencil = { 1.0f, 0 };                                      // depth
    clearValues[2].color = { {0.f, 0.f, 0.f, 0.f} };                                // accumulation
    clearValues[3].color = { {1.f, 0.f, 0.f, 0.f} };                                // revealage

    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = renderPass;
    renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];
    renderPassInfo.renderArea.offset = { 0, 0 };
    renderPassInfo.renderArea.extent = swapChainExtent;
    renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    // Scissor and viewport were set to dynamic, so we set them here now
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(swapChainExtent.width);
    viewport.height = static_cast<float>(swapChainExtent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{};
    scissor.offset = { 0, 0 };
    scissor.extent = swapChainExtent;

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    // ========================================
    // Render lines 
    // ========================================
    /* TODO for now, during development of the Weight blended, order-independent transparency, I'll keep the lines off, and focus on the objects
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, lineBasedPipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    // bind the UBO
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, lineBasedPipelineLayout, 0, 1, &UBODescriptorSets[currentFrame], 0, nullptr);

    for (auto& curve : curves) {
        if (curve->isCurveRendered)
            curve->render(commandBuffer, lineBasedPipelineLayout);
    }
    */
    // ========================================
    // Render objects
    // ========================================
    // Subpass 0: opaque objects
    // ========================================
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectOpaquePipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectOpaquePipelineLayout, 0, 1, &UBODescriptorSets[currentFrame], 0, nullptr);

    for (auto& object : objects)
    {
        if (object->isObjectRendered && object->getTransparency() >= 0.99f)
            object->render(commandBuffer, objectOpaquePipelineLayout);
    }
    
    // Subpass 1: transparent objects
    // ========================================
    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectTransparentPipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectTransparentPipelineLayout, 0, 1, &UBODescriptorSets[currentFrame], 0, nullptr);

    for (auto& object : objects)
    {
        if (object->isObjectRendered && object->getTransparency() < 0.99f)
            object->render(commandBuffer, objectTransparentPipelineLayout);
    }

    // Subpass 2: composite
    // ========================================
    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectCompositePipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, objectCompositePipelineLayout, 0, 1, &compositeDescriptorSet, 0, nullptr);
    
    vkCmdDraw(commandBuffer, 3, 1, 0, 0);

    // ========================================
    // Render UI
    // ========================================
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSizeConstraints(ImVec2(400, -1), ImVec2(FLT_MAX, -1));

    ImGui::Begin("Configuration", nullptr, 
        ImGuiWindowFlags_NoMove | 
        ImGuiWindowFlags_NoCollapse | 
        ImGuiWindowFlags_NoBringToFrontOnFocus | 
        ImGuiWindowFlags_NoTitleBar);
    {
        drawGUIFunction();

        // ensure that the window fills the entire height
        ImVec2 windowSize = ImGui::GetWindowSize();
        ImVec2 windowPos = ImGui::GetWindowPos();
        float availableHeight = ImGui::GetIO().DisplaySize.y - windowPos.y;
        if (windowSize.y != availableHeight)
            ImGui::SetWindowSize(ImVec2(windowSize.x, availableHeight));

    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer, 0, NULL);
    
    // ========================================

    vkCmdEndRenderPass(commandBuffer);

    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to record command buffer!");
    }
}

VkShaderModule RenderEngine::VulkanInternals::createShaderModule(const std::vector<char>& code) {
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    VkShaderModule shaderModule;
    if (vkCreateShaderModule(vulkanContext.device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
        throw std::runtime_error("failed to create shader module!");
    }

    return shaderModule;
}

VkSurfaceFormatKHR RenderEngine::VulkanInternals::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats) {
    for (const auto& availableFormat : availableFormats) {
        if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return availableFormat;
        }
    }

    // if the desired format isn't available, just use the first one
    return availableFormats[0];
}

VkPresentModeKHR RenderEngine::VulkanInternals::chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes) {
    for (const auto& availablePresentMode : availablePresentModes) {
        if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return availablePresentMode;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D RenderEngine::VulkanInternals::chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities) {
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        return capabilities.currentExtent;
    }
    else {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        VkExtent2D actualExtent = {
            static_cast<uint32_t>(width),
            static_cast<uint32_t>(height),
        };

        actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

        return actualExtent;
    }
}

void RenderEngine::VulkanInternals::createInstance() {
    if (enableValidationLayers && !checkValidationLayerSupport()) {
        throw std::runtime_error("validation layers requested, but not available!");
    }

    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Hello Triangle";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "No Engine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_0;

    // Gather the necessary vulkan extensions for the OS it is being compiled for
    uint32_t glfwExtensionsCount = 0;
    const char** glfwExtensions;

    glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionsCount);

    // Set vulkan driver global settings
    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = glfwExtensionsCount;
    createInfo.ppEnabledExtensionNames = glfwExtensions;

    if (enableValidationLayers) {
        createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
        createInfo.ppEnabledLayerNames = validationLayers.data();
    }
    else {
        createInfo.enabledLayerCount = 0;
    }

    // Finally create the instance :D
    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
        throw std::runtime_error("failed to create instance!");
    }
}

bool RenderEngine::VulkanInternals::checkValidationLayerSupport() {
    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

    std::vector<VkLayerProperties> availableLayers{ layerCount };
    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

    for (const char* layerName : validationLayers) {
        bool layerFound = false;

        for (const auto& layerProperties : availableLayers) {
            if (strcmp(layerName, layerProperties.layerName) == 0) {
                layerFound = true;
                break;
            }
        }

        if (!layerFound) {
            return false;
        }
    }

    return true;
}

void RenderEngine::VulkanInternals::createSurface() {
    if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
        throw std::runtime_error("failed to create window surface!");
    }
}

void RenderEngine::VulkanInternals::framebufferResizeCallback(GLFWwindow* window, int width, int height) {
    auto self = static_cast<VulkanInternals*>(glfwGetWindowUserPointer(window));
    self->frameBufferResized = true;
}

void RenderEngine::VulkanInternals::handleUserInput() {
    // this forwards vector is in the xy plane of the world, but rotated around z to line up with the camera front vector
    glm::vec3 worldUp = glm::vec3{ 0.f, 0.f, 1.f };
    glm::vec3 cameraForward = glm::normalize(glm::cross(worldUp, cameraRight));
    glm::vec3 cameraUp = glm::normalize(glm::cross(cameraRight, cameraFront));

    float cameraVelocity = DEFAULT_CAMERA_MOVE_VELOCITY;
    
    // convert deltaTime to seconds
    float timeStep = deltaTime.count() * 0.000001f;

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        cameraVelocity *= 5.f;
    if (glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS)
        cameraVelocity *= 0.2f;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        cameraPosition.z += timeStep * cameraVelocity;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        cameraPosition.z -= timeStep * cameraVelocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPosition -= timeStep * cameraVelocity * cameraRight;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPosition += timeStep * cameraVelocity * cameraRight;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPosition += timeStep * cameraVelocity * cameraForward;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPosition -= timeStep * cameraVelocity * cameraForward;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        cameraFront = glm::normalize(glm::rotate(cameraFront, timeStep * DEFAULT_CAMERA_ROTATE_VELOCITY, cameraRight));
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        cameraFront = glm::normalize(glm::rotate(cameraFront, -timeStep * DEFAULT_CAMERA_ROTATE_VELOCITY, cameraRight));
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        cameraRight = glm::normalize(glm::rotate(cameraRight, timeStep * DEFAULT_CAMERA_ROTATE_VELOCITY, worldUp));
        cameraFront = glm::normalize(glm::cross(cameraUp, cameraRight));
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        cameraRight = glm::normalize(glm::rotate(cameraRight, -timeStep * DEFAULT_CAMERA_ROTATE_VELOCITY, worldUp));
        cameraFront = glm::normalize(glm::cross(cameraUp, cameraRight));
    }
}

void RenderEngine::VulkanInternals::cleanup(std::list<std::shared_ptr<renderer::Curve>>& curves, std::list<std::shared_ptr<renderer::Object>>& objects) {
	vkDeviceWaitIdle(vulkanContext.device);

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    cleanupSwapChain(); // the other buffers from createExtraBuffers() are already cleaned up in the cleanupSwapchain() function, because they need to always have identical size as the frame buffer

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkDestroyBuffer(vulkanContext.device, uniformBuffers[i], nullptr);
        vkFreeMemory(vulkanContext.device, uniformBuffersMemory[i], nullptr);
    }

    for (auto& object : objects) {
        object->cleanup();
    }
    objects.clear();

    for (auto& curve : curves) {
        curve->cleanup();
    }
    curves.clear();

    vkDestroyDescriptorPool(vulkanContext.device, vulkanContext.descriptorPool, nullptr);

    vkDestroyDescriptorSetLayout(vulkanContext.device, vulkanContext.compositeDescriptorSetLayout, nullptr);
    vkDestroyDescriptorSetLayout(vulkanContext.device, vulkanContext.uniformDescriptorSetLayout, nullptr);
    vkDestroyDescriptorSetLayout(vulkanContext.device, vulkanContext.textureDescriptorSetLayout, nullptr);

    vkDestroyPipeline(vulkanContext.device, objectOpaquePipeline, nullptr);
    vkDestroyPipeline(vulkanContext.device, objectTransparentPipeline, nullptr);
    vkDestroyPipeline(vulkanContext.device, objectCompositePipeline, nullptr);
    vkDestroyPipelineLayout(vulkanContext.device, objectOpaquePipelineLayout, nullptr);
    vkDestroyPipelineLayout(vulkanContext.device, objectTransparentPipelineLayout, nullptr);
    vkDestroyPipelineLayout(vulkanContext.device, objectCompositePipelineLayout, nullptr);

    vkDestroyPipeline(vulkanContext.device, lineBasedPipeline, nullptr);
    vkDestroyPipelineLayout(vulkanContext.device, lineBasedPipelineLayout, nullptr);

    vkDestroyRenderPass(vulkanContext.device, renderPass, nullptr);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkDestroySemaphore(vulkanContext.device, renderFinishedSemaphores[i], nullptr);
        vkDestroySemaphore(vulkanContext.device, imageAvailableSemaphores[i], nullptr);
        vkDestroyFence(vulkanContext.device, inFlightFences[i], nullptr);
    }

    vkDestroyCommandPool(vulkanContext.device, vulkanContext.commandPool, nullptr);

    vkDestroyDevice(vulkanContext.device, nullptr);
    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyInstance(instance, nullptr);

    glfwDestroyWindow(window);

    glfwTerminate();

    vulkanContext.device = VK_NULL_HANDLE;
}



// RenderEngine implementation
// ================================================================

RenderEngine::~RenderEngine() = default;

RenderEngine::RenderEngine() : vulkanInternals(std::make_unique<VulkanInternals>()) { }

void RenderEngine::initialize() {
    vulkanInternals->initWindow();
    vulkanInternals->initVulkan();
    vulkanInternals->initImgui();
}

void RenderEngine::handleFrame() {
    glfwPollEvents();

    // Wait for the previous frame to finish rendering
    vkWaitForFences(vulkanInternals->vulkanContext.device, 1, &vulkanInternals->inFlightFences[vulkanInternals->currentFrame], VK_TRUE, UINT64_MAX);

    processRemovals();

    // Retrieve a new image from the swap chain
    uint32_t imageIndex;
    VkResult acquireResult = vkAcquireNextImageKHR(vulkanInternals->vulkanContext.device, vulkanInternals->swapChain, UINT64_MAX, vulkanInternals->imageAvailableSemaphores[vulkanInternals->currentFrame], VK_NULL_HANDLE, &imageIndex);

    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        vulkanInternals->recreateSwapChain();
        return;
    }
    else if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("failed to acquire swap chain image!");
    }
    
    // Only reset the fence if work will be submitted to the GPU
    vkResetFences(vulkanInternals->vulkanContext.device, 1, &vulkanInternals->inFlightFences[vulkanInternals->currentFrame]);

    // Update the uniform buffers
    vulkanInternals->updateUniformBuffer(vulkanInternals->currentFrame);
    vulkanInternals->recordCommandBuffer(curves, objects, [this](){ recordGUI(); }, vulkanInternals->commandBuffers[vulkanInternals->currentFrame], imageIndex);

    // Submit the command buffer.. i.e. actually do the graphics computations
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = { vulkanInternals->imageAvailableSemaphores[vulkanInternals->currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &vulkanInternals->commandBuffers[vulkanInternals->currentFrame];
    VkSemaphore signalSemaphores[] = { vulkanInternals->renderFinishedSemaphores[vulkanInternals->currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    if (vkQueueSubmit(vulkanInternals->vulkanContext.graphicsQueue, 1, &submitInfo, vulkanInternals->inFlightFences[vulkanInternals->currentFrame]) != VK_SUCCESS) {
        throw std::runtime_error("failed to submit draw command buffer!");
    }

    // Submit the results back to the swapchain, to be presented to the screen
    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    VkSwapchainKHR swapChains[] = { vulkanInternals->swapChain };
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &imageIndex;
    presentInfo.pResults = nullptr; // Optional

    VkResult presentResult = vkQueuePresentKHR(vulkanInternals->vulkanContext.presentQueue, &presentInfo);

    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR || vulkanInternals->frameBufferResized) {
        vulkanInternals->frameBufferResized = false;
        vulkanInternals->recreateSwapChain();
    }
    else if (presentResult != VK_SUCCESS) {
        throw std::runtime_error("failed to present swap chain image!");
    }

    // Only handle inputs if the user isn't using the GUI
    if (!ImGui::GetIO().WantCaptureKeyboard)
        vulkanInternals->handleUserInput();

    vulkanInternals->currentFrame = (vulkanInternals->currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

bool RenderEngine::shouldWindowClose() {
    return glfwWindowShouldClose(vulkanInternals->window);
}

void RenderEngine::cleanup() {
    vulkanInternals->cleanup(curves, objects);
}

void RenderEngine::registerGuiModule(std::function<void(RenderEngine&)> callback) {
    guiCallbacks.emplace_back(std::move(callback));
}

renderer::VulkanContext& RenderEngine::getContext() const {
    return vulkanInternals->vulkanContext;
}

std::shared_ptr<renderer::Object> RenderEngine::createObject(
    const char* modelPath,
    const char* texturePath,
    std::string name,
    Vector3d basePosition,
    Vector3d baseScale,
    Vector3d baseRotation,
    float transparency,
    bool isObjectShownInGui,
    bool isObjectRendered
) {
    if (objects.size() + objectsToRemove.size() >= renderer::MAX_OBJECTS)
        throw std::runtime_error("Max object count has been reached, can't create a new object!\n");
    
    std::shared_ptr<renderer::Object> newObject = std::make_shared<renderer::Object>(*this, name);
    newObject->load(modelPath, texturePath, basePosition, baseScale, baseRotation, transparency);
    newObject->isObjectShownInGui = isObjectShownInGui;
    newObject->isObjectRendered = isObjectRendered;

    objects.push_back(newObject);
    return newObject;
}

std::shared_ptr<renderer::Object> RenderEngine::createObject(
    const char* modelPath,
    std::string name,
    Vector3f color,
    Vector3d basePosition,
    Vector3d baseScale,
    Vector3d baseRotation,
    float transparency,
    bool isObjectShownInGui,
    bool isObjectRendered
) {
    if (objects.size() + objectsToRemove.size() >= renderer::MAX_OBJECTS)
        throw std::runtime_error("Max object count has been reached, can't create a new object!\n");
    
    std::shared_ptr<renderer::Object> newObject = std::make_shared<renderer::Object>(*this, name);
    newObject->load(modelPath, color, basePosition, baseScale, baseRotation, transparency);
    newObject->isObjectShownInGui = isObjectShownInGui;
    newObject->isObjectRendered = isObjectRendered;

    objects.push_back(newObject);
    return newObject;
}

std::shared_ptr<renderer::Object> RenderEngine::createDefaultCube(
    std::string name,
    Vector3f color,
    Vector3d basePosition,
    Vector3d baseScale,
    Vector3d baseRotation,
    float transparency,
    bool isObjectShownInGui,
    bool isObjectRendered
) {
    std::shared_ptr<renderer::Object> cube = createObject("../../resources/assets/cube.obj", name, color, basePosition, baseScale, baseRotation, transparency, isObjectShownInGui, isObjectRendered);
    return cube;
}

std::shared_ptr<renderer::Object> RenderEngine::createDefaultPlane(
    std::string name,
    Vector3f color,
    Vector3d basePosition,
    Vector3d baseScale,
    Vector3d baseRotation,
    float transparency,
    bool isObjectShownInGui,
    bool isObjectRendered
) {
    std::shared_ptr<renderer::Object> plane = createObject("../../resources/assets/plane.obj", name, color, basePosition, baseScale, baseRotation, transparency, isObjectShownInGui, isObjectRendered);
    return plane;
}

std::shared_ptr<renderer::Curve> RenderEngine::createCurve(
    core::Polyline2_5D& polyline, 
    std::string name,
    Vector3f color,
    float transparency,
    bool isCurveShownInGui,
    bool isCurveRendered
) {
    if (curves.size() + curvesToRemove.size() >= renderer::MAX_CURVES)
        throw std::runtime_error("Max curve count has been reached, can't create a new curve!\n");

    std::shared_ptr<renderer::Curve> newCurve = std::make_shared<renderer::Curve>(*this, name);
    newCurve->load(polyline, color, transparency);
    newCurve->isCurveShownInGui = isCurveShownInGui;
    newCurve->isCurveRendered = isCurveRendered;

    curves.push_back(newCurve);
    return newCurve;
}

std::shared_ptr<renderer::Curve> RenderEngine::createCurve(
    core::Polyline2D& polyline, 
    core::Plane& plane,
    std::string name,
    Vector3f color,
    float transparency,
    bool isCurveShownInGui,
    bool isCurveRendered
) {
    core::Polyline2_5D newPolyline(polyline, plane);
    return createCurve(newPolyline, name, color, transparency, isCurveShownInGui, isCurveRendered);
}

void RenderEngine::removeObject(std::shared_ptr<renderer::Object>& object) {
    if (!object) return;
    objectsToRemove.push_back(object);
}

void RenderEngine::removeObject(renderer::Object* object) {
    if (!object) return;
    auto it = std::find_if(objects.begin(), objects.end(), [object](const std::shared_ptr<renderer::Object>& ptr) {
        return ptr.get() == object;
    });

    if (it != objects.end()) {
        objectsToRemove.push_back(*it);
    }
}

void RenderEngine::removeCurve(std::shared_ptr<renderer::Curve>& curve) {
    if (!curve) return;
    curvesToRemove.push_back(curve);
}

void RenderEngine::removeCurve(renderer::Curve* curve) {
    if (!curve) return;
    auto it = std::find_if(curves.begin(), curves.end(), [curve](const std::shared_ptr<renderer::Curve>& ptr) {
        return ptr.get() == curve;
    });

    if (it != curves.end()) {
        curvesToRemove.push_back(*it);
    }
}

std::chrono::microseconds RenderEngine::getDeltaTime() {
    return vulkanInternals->deltaTime;
}

std::list<std::shared_ptr<renderer::Curve>>& RenderEngine::getCurves() {
    return curves;
}

std::list<std::shared_ptr<renderer::Object>>& RenderEngine::getObjects() {
    return objects;
}

void RenderEngine::recordGUI() {
    if(ImGui::Button("Generate Cube")) {
        createDefaultCube(
                "cube",                     // name
                Vector3f{ .1f, .2f, .8f },  // color
                Vector3d{ 0.f, 0.f, 0.f },  // position
                Vector3d{ .5f, .5f, .5f }   // scale
            );
    }
    ImGui::SameLine();
    if(ImGui::Button("Generate Plane")) {
        createDefaultPlane(
                "plane",                    // name
                Vector3f{ 1.f, 0.9f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f },  // position
                Vector3d{ 1.f, 1.f, 1.f }   // scale
            );
    }
    ImGui::SameLine();
    if(ImGui::Button("Import Object")) {
        char* outPath = NULL;
        nfdresult_t result = NFD_OpenDialog( "obj", NULL, &outPath );
            
        if ( result == NFD_OKAY ) {
            std::shared_ptr<renderer::Object> object = createObject(
                    outPath,
                    std::string(outPath),
                    Vector3f{ 0.1f, 0.3f, 0.5f } // color
                );
            free(outPath);
        }
    }

    ImGui::SeparatorText("");
    // properties of loaded Objects
    if (ImGui::CollapsingHeader("Objects")) {
        int i = 0;
        for (auto it = objects.begin(); it != objects.end(); ++it, ++i) {
            auto& object = *it;

            if (!object->isObjectShownInGui)
                continue;

            ImGui::PushID(("Objects_" + std::to_string(i)).c_str());

            // Get starting position for this row
            const float rowStartX = ImGui::GetCursorPosX();
            
            // TreeNode with standard behavior
            ImGui::AlignTextToFramePadding();
            bool isOpen = ImGui::TreeNodeEx("##object_node", 
                ImGuiTreeNodeFlags_SpanAvailWidth | 
                ImGuiTreeNodeFlags_AllowItemOverlap);
            
            // Name input (fixed position relative to row start)
            ImGui::SameLine(rowStartX + ImGui::GetTreeNodeToLabelSpacing());
            ImGui::SetNextItemWidth(120);
            char nameBuffer[256];
            strncpy(nameBuffer, object->getName().c_str(), sizeof(nameBuffer));
            if (ImGui::InputText("##NameEdit", nameBuffer, sizeof(nameBuffer))) {
                object->setName(nameBuffer);
            }

            // Calculate positions for right-aligned controls
            const float checkboxWidth = ImGui::GetFrameHeight();
            const float buttonWidth = 25.0f;
            const float spacing = ImGui::GetStyle().ItemSpacing.x;
            const float totalRightWidth = checkboxWidth + buttonWidth + spacing;

            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - totalRightWidth);
            ImGui::Checkbox("##RenderToggle", &object->isObjectRendered);

            // Delete button (fixed position relative to window edge)
            ImGui::SameLine();
            if (ImGui::Button("X##CloseObject", ImVec2(25, ImGui::GetFrameHeight()))) {
                objectsToRemove.push_back(object);
            }

            // Contents when expanded
            if (isOpen) {
                ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Vertices: %d | Faces: %d", object->getNumberOfVertices(), object->getNumberOfIndices() / 3);
                object->drawGUI();
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
    }        
    // properties of loaded Lines
    if(ImGui::CollapsingHeader("Lines")) {
        int i = 0;
        for (auto it = curves.begin(); it != curves.end(); ++it, ++i) {
            auto& curve = *it;

            if (!curve->isCurveShownInGui)
                continue;

            ImGui::PushID(("Lines_" + std::to_string(i)).c_str());

            // Get starting position for this row
            const float rowStartX = ImGui::GetCursorPosX();
            
            // TreeNode with standard behavior
            ImGui::AlignTextToFramePadding();
            bool isOpen = ImGui::TreeNodeEx("##object_node", 
                ImGuiTreeNodeFlags_SpanAvailWidth | 
                ImGuiTreeNodeFlags_AllowItemOverlap);
            
            // Name input (fixed position relative to row start)
            ImGui::SameLine(rowStartX + ImGui::GetTreeNodeToLabelSpacing());
            ImGui::SetNextItemWidth(120);
            char nameBuffer[256];
            strncpy(nameBuffer, curve->getName().c_str(), sizeof(nameBuffer));
            if (ImGui::InputText("##NameEdit", nameBuffer, sizeof(nameBuffer))) {
                curve->setName(nameBuffer);
            }

            // Calculate positions for right-aligned controls
            const float checkboxWidth = ImGui::GetFrameHeight();
            const float buttonWidth = 25.f;
            const float spacing = ImGui::GetStyle().ItemSpacing.x;
            const float totalRightWidth = checkboxWidth + buttonWidth + spacing;

            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - totalRightWidth);
            ImGui::Checkbox("##RenderToggle", &curve->isCurveRendered);

            // Delete button (fixed position relative to window edge)
            ImGui::SameLine();
            if (ImGui::Button("X##CloseObject", ImVec2(25, ImGui::GetFrameHeight()))) {
                curvesToRemove.push_back(curve);
            }

            if (isOpen) {
                ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Vertices: %d", curve->getNumberOfVertices());
                curve->drawGUI();

                ImGui::TreePop();
            }
            ImGui::PopID();
        }
    }

    ImGui::SeparatorText("");
    // add the GUI for all registered modules (i.e. stuff like MeshIntersect, PolylineOffsets, etc)
    for (auto& drawGUI : guiCallbacks) {
        drawGUI(*this);
    }
}

void RenderEngine::processRemovals() {
    vkDeviceWaitIdle(getContext().device);

    for (auto& object : objectsToRemove) {
        object->cleanup();
        object->alive = false;
        objects.remove(object);
    }

    for (auto& curve : curvesToRemove) {
        curve->cleanup();
        curve->alive = false;
        curves.remove(curve);
    }

    objectsToRemove.clear();
    curvesToRemove.clear();
}

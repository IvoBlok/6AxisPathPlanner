#ifndef RENDER_CORE_TYPES_HPP
#define RENDER_CORE_TYPES_HPP

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_SWIZZLE
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLMF_FORCE_DEFAULT_ALIGNED_GENTYPES
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/hash.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <array>

namespace renderer {
    // define constants that define initial behaviour of or limits on the RenderEngine class
    const uint32_t WIDTH = 800;
    const uint32_t HEIGHT = 600;
    const int MAX_FRAMES_IN_FLIGHT = 2;
    const int MAX_OBJECTS = 64;
    const int MAX_CURVES = 1000;

    const float DEFAULT_CAMERA_MOVE_VELOCITY = 0.7f;
    const float DEFAULT_CAMERA_ROTATE_VELOCITY = 0.7f;
    const glm::vec4 CLEAR_COLOR = glm::vec4{ 0.7f, 0.7f, 0.7f, 1.f };

    const std::vector<const char*> validationLayers = {
        "VK_LAYER_KHRONOS_validation"
    };

    const std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    const bool enableValidationLayers = true;

    // define a struct containing all vulkan related context required for rendering
    struct VulkanContext {
        VkDevice device = VK_NULL_HANDLE;
        VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
        VkQueue graphicsQueue = VK_NULL_HANDLE;
        VkQueue presentQueue = VK_NULL_HANDLE;
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
        VkDescriptorSetLayout uniformDescriptorSetLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout textureDescriptorSetLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout compositeDescriptorSetLayout = VK_NULL_HANDLE;
    };

    // define crucial structs that hold data that is exchanged with the shaders
    struct UniformBufferObject {
        alignas(16) glm::mat4 model;
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
    };

    struct ObjectShaderPushConstant {
        glm::mat4 modelMatrix;
        glm::vec3 color;
        bool isOneColor;
        float transparency;
    };

    struct CurveShadersPushConstant {
        glm::vec3 color;
        bool isOneColor;
        float transparency;
    };

    struct RendererVertex {
        glm::vec3 pos;
        glm::vec3 color;
        glm::vec3 normal;
        glm::vec2 texCoord;

        static VkVertexInputBindingDescription getBindingDescription();
        static std::array<VkVertexInputAttributeDescription, 4> getAttributeDescriptions();

        bool operator==(const RendererVertex& other) const;
    };

    // define useful functions for handling vulkan data types
    void createBuffer(VulkanContext& context, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory);
    void copyBuffer(VulkanContext& context, VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
    void copyBufferToImage(VulkanContext& context, VkBuffer buffer, VkImage image, uint32_t width, uint32_t height);

	VkCommandBuffer beginSingleTimeCommands(VulkanContext& context);
    void endSingleTimeCommands(VulkanContext& context, VkCommandBuffer commandBuffer);

	uint32_t findMemoryType(VulkanContext& context, uint32_t typeFilter, VkMemoryPropertyFlags properties);

	void createImage(VulkanContext& context, uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory);
    VkImageView createImageView(VulkanContext& context, VkImage image, VkFormat format, VkImageAspectFlags aspectFlags);
    void transitionImageLayout(VulkanContext& context, VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectFlags = VK_IMAGE_ASPECT_COLOR_BIT);
}

namespace std {
	template<> struct hash<renderer::RendererVertex> {
		size_t operator()(renderer::RendererVertex const& vertex) const;
	};
}

#endif
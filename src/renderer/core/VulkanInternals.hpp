#ifndef VULKAN_INTERNALS_HPP
#define VULKAN_INTERNALS_HPP

#include "RenderCoreTypes.hpp"

#include <chrono>
#include <vector>
#include <optional>

struct RenderEngine::VulkanInternals {
    renderer::VulkanContext vulkanContext;

    GLFWwindow* window;

    glm::vec3 cameraPosition;
    glm::vec3 cameraFront;
    glm::vec3 cameraRight;
    std::chrono::microseconds deltaTime;

	VkInstance instance;
	VkSurfaceKHR surface;

	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;

	VkSwapchainKHR swapChain;
	std::vector<VkImage> swapChainImages;
	std::vector<VkImageView> swapChainImageViews;
	std::vector<VkFramebuffer> swapChainFramebuffers;

	VkRenderPass renderPass;

	std::vector<VkDescriptorSet> descriptorSets;

	VkPipelineLayout triangleBasedPipelineLayout;
	VkPipeline triangleBasedPipeline;

	VkPipelineLayout lineBasedPipelineLayout;
	VkPipeline lineBasedPipeline;

	VkImage depthImage;
	VkDeviceMemory depthImageMemory;
	VkImageView depthImageView;

	std::vector<VkBuffer> uniformBuffers;
	std::vector<VkDeviceMemory> uniformBuffersMemory;
	std::vector<void*> uniformBuffersMapped;

	std::vector<VkCommandBuffer> commandBuffers;

	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;

	uint32_t currentFrame;
	std::chrono::time_point<std::chrono::high_resolution_clock> oldCurrentTime;

	bool frameBufferResized;

	struct QueueFamilyIndices {
		std::optional<uint32_t> graphicsFamily;
		std::optional<uint32_t> presentFamily;

		bool isComplete();
	};

	struct SwapChainSupportDetails {
		VkSurfaceCapabilitiesKHR capabilities;
		std::vector<VkSurfaceFormatKHR> formats;
		std::vector<VkPresentModeKHR> presentModes;
	};

	VulkanInternals();

    void initWindow();
    void initVulkan();
    void initImgui();

    void pickPhysicalDevice();
    void createLogicalDevice();
	void createSwapChain();
    void recreateSwapChain();
    void cleanupSwapChain();
    void createImageViews();
    void createRenderPass();
    void createDescriptorSetLayout();
    void createTriangleBasedPipeline();
    void createLineBasedPipeline();

	void createFrameBuffers();
	void createCommandPool();

	VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);
	VkFormat findDepthFormat();
	void createDepthResources();
	void createUniformBuffers();
	void updateUniformBuffer(uint32_t currentImage);
	void createDescriptorPool();
	void createDescriptorSets();
	void createCommandBuffers();
	void createSyncObjects();
	void recordCommandBuffer(std::list<std::shared_ptr<renderer::Curve>>& curves, std::list<std::shared_ptr<renderer::Object>>& objects, std::function<void()> drawGUIFunction, VkCommandBuffer commandBuffer, uint32_t imageIndex);
	VkShaderModule createShaderModule(const std::vector<char>& code);
	VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
	VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
	VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
	void createInstance();
	bool checkValidationLayerSupport();
	void createSurface();

	bool isDeviceSuitable(VkPhysicalDevice device);
	bool checkDeviceExtensionSupport(VkPhysicalDevice device);

	QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
	SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);

    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
    void handleUserInput();
	void cleanup(std::list<std::shared_ptr<renderer::Curve>>& curves, std::list<std::shared_ptr<renderer::Object>>& objects);
};

#endif
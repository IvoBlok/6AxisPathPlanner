#ifndef VULKAN_INTERNALS_HPP
#define VULKAN_INTERNALS_HPP

#include "RenderCoreTypes.hpp"

#include <chrono>
#include <vector>
#include <optional>

struct RenderEngine::VulkanInternals {
    renderer::VulkanContext vulkanContext;

    GLFWwindow* window;

	VkInstance instance;
	VkSurfaceKHR surface;

	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;

	VkSwapchainKHR swapChain;
	std::vector<VkImage> swapChainImages;
	std::vector<VkImageView> swapChainImageViews;
	std::vector<VkFramebuffer> swapChainFramebuffers;

	VkRenderPass renderPass;

	VkPipelineLayout objectOpaquePipelineLayout;
	VkPipelineLayout objectTransparentPipelineLayout;
	VkPipeline objectOpaquePipeline;
	VkPipeline objectTransparentPipeline;

	VkPipelineLayout curveOpaquePipelineLayout;
	VkPipelineLayout curveTransparentPipelineLayout;
	VkPipeline curveOpaquePipeline;
	VkPipeline curveTransparentPipeline;

	VkPipelineLayout compositePipelineLayout;
	VkPipeline objectCompositePipeline;

	// buffer for usage by a pipeline as its depth buffer, for depth testing etc
	VkImage depthImage;
	VkDeviceMemory depthImageMemory;
	VkImageView depthImageView;

	// buffer for usage in subpasses for order-independent transparency
	VkImage accumulationBuffer;
	VkImageView accumulationBufferView;
	VkDeviceMemory accumulationBufferMemory;

	// buffer for usage in subpasses for order-independent transparency
	VkImage revealageBuffer;
	VkImageView revealageBufferView;
	VkDeviceMemory revealageBufferMemory;

	// holds the uniform buffer for each 'frame in flight'
	std::vector<VkBuffer> uniformBuffers;
	std::vector<VkDeviceMemory> uniformBuffersMemory;
	std::vector<void*> uniformBuffersMapped;

	std::vector<VkDescriptorSet> UBODescriptorSets;
	VkDescriptorSet compositeDescriptorSet;

	// holds the command buffer for each 'frame in flight'
	std::vector<VkCommandBuffer> commandBuffers;

	// various variables for syncing when memory is safe to be used
	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;

	// 'frameBufferResized' describes if the user resized the window, triggering an update of internal buffers
	bool frameBufferResized;

	// 'currentFrame' stores which of the 'frames in flight' is currently being used
	uint32_t currentFrame;
	// 'oldCurrentTime' is used to calculate the delta time
	std::chrono::time_point<std::chrono::high_resolution_clock> oldCurrentTime;

    glm::vec3 cameraPosition;
    glm::vec3 cameraFront;
    glm::vec3 cameraRight;
    std::chrono::microseconds deltaTime;


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
    void createDescriptorSetLayouts();

	void createPipelines();
	void createObjectOpaquePipeline();
	void createObjectTransparentPipeline();
    void createCurveOpaquePipeline();
	void createCurveTransparentPipeline();
	void createCompositePipeline();

	void createFrameBuffers();
	void createCommandPool();

	VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);
	VkFormat findDepthFormat();
	void createDepthResources();
	void createExtraRenderBuffers();
	void createUniformBuffers();
	void updateUniformBuffer(uint32_t currentImage);
	void createDescriptorPool();
	void createDescriptorSets();
	void updateCompositeDescriptorSet();
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
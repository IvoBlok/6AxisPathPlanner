/*
This file defines the general renderer and its required structures/classes. 
Note that all objects/lines/points class instances in the scene live in the Renderer class. 
*/
#ifndef RENDERER_HPP
#define RENDERER_HPP

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

#include "stb_image.h"
#include "tiny_obj_loader.h"

//#include "OPTICK/optick.h"

#include "imconfig.h"
#include "imgui_internal.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan_but_better.hpp"	
#include "implot.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <set>
#include <optional>
#include <vector>
#include <map>
#include <unordered_map>
#include <filesystem>
#include <functional>

#include "core/objectShape.hpp"
#include "core/polyline.hpp"
#include "core/plane.hpp"

static std::vector<char> readShaderFile(const std::string& relativePath);

const uint32_t WIDTH = 800;
const uint32_t HEIGHT = 600;
const int MAX_FRAMES_IN_FLIGHT = 2;
const int MAX_OBJECTS = 64;
const int MAX_LINES = 1000;

const float DEFAULT_CAMERA_MOVE_VELOCITY = 0.7f;
const float DEFAULT_CAMERA_ROTATE_VELOCITY = 0.7f;
const glm::vec4 CLEAR_COLOR = glm::vec4{ 0.7f, 0.7f, 0.7f, 1.f };

const std::vector<const char*> validationLayers = {
	"VK_LAYER_KHRONOS_validation"
};

const std::vector<const char*> deviceExtensions = {
	VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

const bool enableValidationLayers = false;

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

struct LineShaderPushConstant {
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

namespace std {
	template<> struct hash<RendererVertex> {
		size_t operator()(RendererVertex const& vertex) const;
	};
}



namespace VulkanHelper {
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory);
    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height);

	VkCommandBuffer beginSingleTimeCommands();
    void endSingleTimeCommands(VkCommandBuffer commandBuffer);

	uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

	void createImage(uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory);
    VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags);
    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectFlags = VK_IMAGE_ASPECT_COLOR_BIT);
};

/* 
LoadedObject stores a certain triangle-based geometry. This geometry itself is not intended to be modified. 
The geometry can be used in read-only fashion for other operations. For these applications a separate geometry/mesh format is used, known as ObjectShape.
*/
class LoadedObject {
private:
	class LoadedTexture {
	public:
		VkDescriptorSet descriptorSet;

		void load(const char* path);
		void free();
		void destroy();

	private:
		VkImage textureImage;
		VkDeviceMemory textureImageMemory;
		VkImageView textureImageView;
		VkSampler textureSampler;

		void createTextureImage(const char* path);
		void createTextureImageView();
		void createTextureSampler();
		void createTextureDescriptorSet();
	};

	class LoadedModel {
	public:
		std::vector<RendererVertex> vertices;
		std::vector<uint32_t> indices;
		float transparency;

		void load(const char* path, float modelTransparency = 1.f);
		void destroy();
		void render(VkCommandBuffer commandBuffer);

	private:
		VkBuffer vertexBuffer;
		VkDeviceMemory vertexBufferMemory;
		VkBuffer indexBuffer;
		VkDeviceMemory indexBufferMemory;

		void loadModel(const char* path);
		void createVertexBuffer();
		void createIndexBuffer();
	};

public:
	bool renderObj;
	std::string name;

    glm::vec3 position;
    glm::vec3 scale;
    glm::mat4 rotationMatrix;

    bool isOneColor;
    glm::vec3 color;

	LoadedModel model;
    LoadedTexture texture;

	LoadedObject();
	LoadedObject(std::string objectName, bool visible = true);

	void locateWithMatrix(Matrix4d matrix);

	// getObjectShape converts the object geometry from vertex data in one format and the position,scale, rotation data together into a new vertex data format, where the transformations have been applied.
	core::ObjectShape getObjectShape();

	core::Plane getPlane();

	void load(const char* modelPath, const char* texturePath, Vector3d basePosition = { 0.f, 0.f, 0.f }, Vector3d baseScale = { 1.f, 1.f, 1.f }, glm::mat4 baseRotationMatrix = glm::mat4{ 1.0f }, float modelTransparency = 1.f);
    void load(const char* modelPath, Vector3d objectColor, Vector3d basePosition = { 0.f, 0.f, 0.f }, Vector3d baseScale = { 1.f, 1.f, 1.f }, glm::mat4 baseRotationMatrix = glm::mat4{ 1.0f }, float modelTransparency = 1.f);
    void destroy();
    glm::mat4 getTransformationMatrix();
    void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
};

/*
LoadedLine stores a certain polyline. It stores both the polyline it represents, and a renderer version of the same curve. 
the polyline can undergo modifications. To update the rendering representation of it, a call to updateRendererLine() is required.
*/
class LoadedLine {
public:
	bool renderLine;
	std::string name;

	float transparency;
	float lineWidth;
	
	glm::vec3 defaultColor;
	bool isOneColor;

	LoadedLine();
	void load(core::Polyline2_5D& polylineIn, float lineTransparency = 1.f, glm::vec3 lineColor = glm::vec3{ 1.f, 0.f, 0.f });
	
	void destroyRenderData();
	void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);

	core::Polyline2_5D& getPolyline();

	// updateRendererLine updates the renderer version of the polyline. Without calling this function the renderer will  not show changes to the polyline
	void updateRendererLine();

private:
	// 'polyline' generally contains the actual curve of interest. Math is done specifically to 'polyline' and not 'vertices'. 'vertices'/'indices' is merely the visual representation of the actual data.
	// Exceptions to this might be purely visual indications, like gizmos. There polyline is irrelevant, and it is only the 'vertices'/'indices' that matters.
	// currently this is a2.5D polyline. This doesn't support lines that don't fully lay in a single plane. 
	// TODO when a proper 3D polyline is added, replace the 2.5D with the 3D version
	core::Polyline2_5D polyline;

	std::vector<RendererVertex> vertices;
    std::vector<uint32_t> indices;

	VkBuffer vertexBuffer;
	VkDeviceMemory vertexBufferMemory;
	VkBuffer indexBuffer;
	VkDeviceMemory indexBufferMemory;


	void createVertexBuffer();
	void createIndexBuffer();
	void loadPolylineIntoRendererFormat();
};

class VulkanRenderEngine {
public:
    GLFWwindow* window;

    glm::vec3 cameraPosition;
    glm::vec3 cameraFront;
    glm::vec3 cameraRight;
    std::chrono::microseconds deltaTime;

	
	std::list<LoadedObject> loadedObjects;
	std::list<LoadedLine> loadedLines;


	VulkanRenderEngine();

    void initialize();
    void drawFrame();
    void cleanup();

    LoadedObject& createObject(const char* modelPath, const char* texturePath, Vector3d basePosition = { 0.f, 0.f, 0.f }, Vector3d baseScale = { 1.f, 1.f, 1.f }, glm::mat4 rotationMatrix = glm::mat4{ 1.f }, float modelTransparency = 1.f);
    LoadedObject& createObject(const char* modelPath, Vector3d objectColor = { 0.f, 0.f, 0.f }, Vector3d basePosition = { 0.f, 0.f, 0.f }, Vector3d baseScale = { 1.f, 1.f, 1.f }, glm::mat4 rotationMatrix = glm::mat4{ 1.f }, float modelTransparency = 1.f);
    
	LoadedLine& createLine(core::Polyline2_5D& polyline, float lineTransparency = 1.f, glm::vec3 lineColor = glm::vec3{ 1.f, 0.f, 0.f });
	LoadedLine& createLine(core::Polyline2D& polyline, core::Plane plane, float lineTransparency = 1.f, glm::vec3 lineColor = glm::vec3{ 1.f, 0.f, 0.f });

	void handleUserInput();

	void registerGuiModule(std::function<void(VulkanRenderEngine&)> callback);

private:
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

	std::vector<std::function<void(VulkanRenderEngine&)>> guiCallbacks;

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
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
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
};

#endif // RENDERER_HPP
#ifndef RENDERER_GEOMETRY_BASE_HPP
#define RENDERER_GEOMETRY_BASE_HPP

#include "renderer/core/RenderEngine.hpp"
#include "CustomEigen.hpp"
#include "core/polyline.hpp"

#include <glm/glm.hpp>

namespace renderer {
	class Texture {
	public:
        Texture(RenderEngine& renderer);
		~Texture();

		// disable copying to avoid vulkan buffers getting freed twice
		Texture(const Texture&) = delete;
		Texture& operator=(const Texture&) = delete;

		Texture(Texture&& other) noexcept;
		Texture& operator=(Texture&& other) noexcept;

		void load(const char* path);
		void loadDummy();
		void destroy();
		const VkDescriptorSet& getDescriptorSet() const;
		
	private:
		VkImage textureImage;
		VkDeviceMemory textureImageMemory;
		VkImageView textureImageView;
		VkSampler textureSampler;

		VkDescriptorSet descriptorSet;

        VulkanContext& context;

		void createTextureImage(const char* path);
		void createTextureImageView();
		void createTextureSampler();
		void createTextureDescriptorSet();
	};

	class Model {
	public:
		std::vector<RendererVertex> vertices;
		std::vector<uint32_t> indices;
		float transparency;

        Model(RenderEngine& renderer);
		~Model();

		// disable copying to avoid vulkan buffers getting freed twice
		Model(const Model&) = delete;
		Model& operator=(const Model&) = delete;

		Model(Model&& other) noexcept;
		Model& operator=(Model&& other) noexcept;

		void load(const char* path, float modelTransparency = 1.f);
		void destroy();
		void render(VkCommandBuffer commandBuffer);

	private:
		VkBuffer vertexBuffer;
		VkDeviceMemory vertexBufferMemory;
		VkBuffer indexBuffer;
		VkDeviceMemory indexBufferMemory;

        VulkanContext& context;

		void loadModel(const char* path);
		void createVertexBuffer();
		void createIndexBuffer();
	};

	class CurveBuffer {
	public:
		std::vector<RendererVertex> vertices;
		float transparency;

		CurveBuffer(RenderEngine& renderer);
		~CurveBuffer();

		// disable copying to avoid vulkan buffers getting freed twice
		CurveBuffer(const CurveBuffer&) = delete;
		CurveBuffer& operator=(const CurveBuffer&) = delete;

		CurveBuffer(CurveBuffer&& other) noexcept;
		CurveBuffer& operator=(CurveBuffer&& other) noexcept;

		void load(core::Polyline2_5D& polyline, float curveTransparency = 1.f);

		void destroy();
		void render(VkCommandBuffer commandBuffer);

	private:
		VkBuffer vertexBuffer;
		VkDeviceMemory vertexBufferMemory;

        VulkanContext& context;

		void loadPolyline(core::Polyline2_5D& polyline);
		void createVertexBuffer();
	};

    class Rotation {
    public:
        Rotation();

        Rotation& operator=(const Vector3d& euler);
        Rotation& operator=(const Matrix3d& matrix);

        glm::mat4 glmMatrix() const;
        Matrix4d matrix4d() const;
        Matrix3d matrix3d() const;

        Vector3d angles() const;

    private:
        Vector3d eulerAngles;
        Matrix3d rotationMatrix;

        void updateEulerAngles();
        void updateRotationMatrix();
    };
}   

#endif
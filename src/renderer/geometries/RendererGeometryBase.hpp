#ifndef RENDERER_GEOMETRY_BASE_HPP
#define RENDERER_GEOMETRY_BASE_HPP

#include "renderer/core/RenderEngine.hpp"
#include "CustomEigen.hpp"
#include "core/polyline.hpp"

#include <glm/glm.hpp>

namespace renderer {
	class Texture {
	public:
		VkDescriptorSet descriptorSet;

        Texture(RenderEngine& renderer);

		void load(const char* path);
		void loadDummy();
		void free();
		void destroy();

	private:
        VulkanContext& context;

		VkImage textureImage;
		VkDeviceMemory textureImageMemory;
		VkImageView textureImageView;
		VkSampler textureSampler;

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

		void load(const char* path, float modelTransparency = 1.f);
		void destroy();
		void render(VkCommandBuffer commandBuffer);

	private:
        VulkanContext& context;

		VkBuffer vertexBuffer;
		VkDeviceMemory vertexBufferMemory;
		VkBuffer indexBuffer;
		VkDeviceMemory indexBufferMemory;

		void loadModel(const char* path);
		void createVertexBuffer();
		void createIndexBuffer();
	};

	class CurveBuffer {
	public:
		std::vector<RendererVertex> vertices;
		std::vector<uint32_t> indices;
		float transparency;

		CurveBuffer(RenderEngine& renderer);

		void load(core::Polyline2_5D& polyline, float curveTransparency = 1.f);
		
		void destroy();
		void render(VkCommandBuffer commandBuffer);

	private:
        VulkanContext& context;

		VkBuffer vertexBuffer;
		VkDeviceMemory vertexBufferMemory;
		VkBuffer indexBuffer;
		VkDeviceMemory indexBufferMemory;

		void loadPolyline(core::Polyline2_5D& polyline);
		void createVertexBuffer();
		void createIndexBuffer();
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
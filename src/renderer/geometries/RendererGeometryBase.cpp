#include "RendererGeometryBase.hpp"

#include "stb_image.h"
#include "tiny_obj_loader.h"

#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include <cstring>

namespace renderer {
    // Texture class implementation
    // ======================================================================================

    Texture::Texture(RenderEngine& renderer) : context(renderer.getContext()) {
        textureImage = VK_NULL_HANDLE;
        textureImageMemory = VK_NULL_HANDLE;
        textureImageView = VK_NULL_HANDLE;
        textureSampler = VK_NULL_HANDLE;

        descriptorSet = VK_NULL_HANDLE;
    }

    Texture::~Texture() {
        destroy();
    }

    Texture::Texture(Texture&& other) noexcept
        : context(other.context), 
        descriptorSet(other.descriptorSet),
        textureImage(other.textureImage),
        textureImageMemory(other.textureImageMemory),
        textureImageView(other.textureImageView),
        textureSampler(other.textureSampler) 
    {
        other.textureImage = VK_NULL_HANDLE;
        other.textureImageMemory = VK_NULL_HANDLE;
        other.textureImageView = VK_NULL_HANDLE;
        other.textureSampler = VK_NULL_HANDLE;

        other.descriptorSet = VK_NULL_HANDLE;
    }

    Texture& Texture::operator=(Texture&& other) noexcept {
        if (this != &other) {
            vkDeviceWaitIdle(context.device);
            destroy();

            context = other.context;
            descriptorSet = other.descriptorSet;
            textureImage = other.textureImage;
            textureImageMemory = other.textureImageMemory;
            textureImageView = other.textureImageView;
            textureSampler = other.textureSampler;

            other.textureImage = VK_NULL_HANDLE;
            other.textureImageMemory = VK_NULL_HANDLE;
            other.textureImageView = VK_NULL_HANDLE;
            other.textureSampler = VK_NULL_HANDLE;

            other.descriptorSet = VK_NULL_HANDLE;
        }
        return *this;
    }

    void Texture::load(const char* path) {
        createTextureImage(path);
        createTextureImageView();
        createTextureSampler();
        createTextureDescriptorSet();
    }

    void Texture::loadDummy() {
        // Create a 1x1 white pixel
        const uint32_t texWidth = 1;
        const uint32_t texHeight = 1;
        const uint32_t whitePixel = 0xFFFFFFFF; // RGBA white
        
        VkDeviceSize imageSize = sizeof(whitePixel);

        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        
        createBuffer(context, imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, imageSize, 0, &data);
        memcpy(data, &whitePixel, static_cast<size_t>(imageSize));
        vkUnmapMemory(context.device, stagingBufferMemory);

        createImage(context, texWidth, texHeight, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, textureImage, textureImageMemory);

        transitionImageLayout(context, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        copyBufferToImage(context, stagingBuffer, textureImage, texWidth, texHeight);
        transitionImageLayout(context, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);

        // Create image view and sampler as usual
        createTextureImageView();
        createTextureSampler();
        createTextureDescriptorSet();
    }

    void Texture::destroy() {
        if (textureSampler != VK_NULL_HANDLE) {
            vkDestroySampler(context.device, textureSampler, nullptr);
            textureSampler = VK_NULL_HANDLE;
        }
        if (textureImageView != VK_NULL_HANDLE) {
            vkDestroyImageView(context.device, textureImageView, nullptr);
            textureImageView = VK_NULL_HANDLE;
        }
        if (textureImage != VK_NULL_HANDLE) {
            vkDestroyImage(context.device, textureImage, nullptr);
            textureImage = VK_NULL_HANDLE;        
        }
        if (textureImageMemory != VK_NULL_HANDLE) {
            vkFreeMemory(context.device, textureImageMemory, nullptr);
            textureImageMemory = VK_NULL_HANDLE;
        }
        
        if (descriptorSet != VK_NULL_HANDLE && context.descriptorPool != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(context.device, context.descriptorPool, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }

    const VkDescriptorSet& Texture::getDescriptorSet() const {
        return descriptorSet;
    }

    void Texture::createTextureImage(const char* path) {
        int texWidth, texHeight, texChannels;
        stbi_uc* pixels = stbi_load(path, &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
        VkDeviceSize imageSize = texWidth * texHeight * 4;

        if (!pixels) {
            throw std::runtime_error("failed to load texture image!");
        }

        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        
        createBuffer(context, imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, imageSize, 0, &data);
        memcpy(data, pixels, static_cast<size_t>(imageSize));
        vkUnmapMemory(context.device, stagingBufferMemory);

        stbi_image_free(pixels);

        createImage(context, texWidth, texHeight, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, textureImage, textureImageMemory);

        transitionImageLayout(context, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        copyBufferToImage(context, stagingBuffer, textureImage, static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight));
        transitionImageLayout(context, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);
    }

    void Texture::createTextureImageView() {
        textureImageView = createImageView(context, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT);
    }

    void Texture::createTextureSampler() {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(context.physicalDevice, &properties);

        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.anisotropyEnable = VK_TRUE;
        samplerInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;

        if (vkCreateSampler(context.device, &samplerInfo, nullptr, &textureSampler) != VK_SUCCESS) {
            throw std::runtime_error("failed to create object texture sampler!");
        }
    }

    void Texture::createTextureDescriptorSet() {
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = context.descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &context.textureDescriptorSetLayout;

        if (vkAllocateDescriptorSets(context.device, &allocInfo, &descriptorSet) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets for textures!");
        }

        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imageInfo.imageView = textureImageView;
        imageInfo.sampler = textureSampler;

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = descriptorSet;
        descriptorWrite.dstBinding = 0;
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pImageInfo = &imageInfo;

        vkUpdateDescriptorSets(context.device, 1, &descriptorWrite, 0, nullptr);
    }

    

    // Model class implementation
    // ======================================================================================

    Model::Model(RenderEngine& renderer) : context(renderer.getContext()) {
        vertexBuffer = VK_NULL_HANDLE;
        vertexBufferMemory = VK_NULL_HANDLE;
        indexBuffer = VK_NULL_HANDLE;
        indexBufferMemory = VK_NULL_HANDLE;
    }

    Model::~Model() {
        destroy();
    }

    Model::Model(Model&& other) noexcept 
        : context(other.context),
        vertices(std::move(other.vertices)),
        indices(std::move(other.indices)),
        transparency(other.transparency),
        vertexBuffer(other.vertexBuffer),
        vertexBufferMemory(other.vertexBufferMemory),
        indexBuffer(other.indexBuffer),
        indexBufferMemory(other.indexBufferMemory)
    {
        other.vertexBuffer = VK_NULL_HANDLE;
        other.vertexBufferMemory = VK_NULL_HANDLE;
        other.indexBuffer = VK_NULL_HANDLE;
        other.indexBufferMemory = VK_NULL_HANDLE;
    }

    Model& Model::operator=(Model&& other) noexcept {
        if (this != &other) {
            vkDeviceWaitIdle(context.device);
            destroy();

            vertices = std::move(other.vertices);
            indices = std::move(other.indices);
            transparency = other.transparency;
            vertexBuffer = other.vertexBuffer;
            vertexBufferMemory = other.vertexBufferMemory;
            indexBuffer = other.indexBuffer;
            indexBufferMemory = other.indexBufferMemory;

            other.vertexBuffer = VK_NULL_HANDLE;
            other.vertexBufferMemory = VK_NULL_HANDLE;
            other.indexBuffer = VK_NULL_HANDLE;
            other.indexBufferMemory = VK_NULL_HANDLE;
        }
        return *this;
    }

    void Model::load(const char* path, float modelTransparency) {
        loadModel(path);
        createVertexBuffer();
        createIndexBuffer();

        transparency = modelTransparency;
    }

    void Model::destroy() {
        if (indexBuffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(context.device, indexBuffer, nullptr);
            indexBuffer = VK_NULL_HANDLE;
        }
        if (indexBufferMemory != VK_NULL_HANDLE) {
            vkFreeMemory(context.device, indexBufferMemory, nullptr);
            indexBufferMemory = VK_NULL_HANDLE;
        }
        if (vertexBuffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(context.device, vertexBuffer, nullptr);
            vertexBuffer = VK_NULL_HANDLE;
        }
        if (vertexBufferMemory != VK_NULL_HANDLE) {
            vkFreeMemory(context.device, vertexBufferMemory, nullptr);
            vertexBufferMemory = VK_NULL_HANDLE;
        }

        vertices.clear();
        indices.clear();
    }

    void Model::render(VkCommandBuffer commandBuffer) {
        VkBuffer vertexBuffers[] = { vertexBuffer };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);

        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

        vkCmdDrawIndexed(commandBuffer, static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);
    }

    void Model::loadModel(const char* path) {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path)) {
            throw std::runtime_error(warn + err);
        }

        std::unordered_map<RendererVertex, uint32_t> uniqueVertices{};


        for (const auto& shape : shapes) {
            for (const auto& index : shape.mesh.indices) {
                RendererVertex vertex{};

                vertex.pos = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2]
                };

                if (index.normal_index >= 0) {
                    vertex.normal = {
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2]
                    };
                }

                if (index.texcoord_index >= 0) {
                    vertex.texCoord = {
                        attrib.texcoords[2 * index.texcoord_index + 0],
                        1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                    };
                }

                vertex.color = { 1.0f, 1.0f, 1.0f };

                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }

                indices.push_back(uniqueVertices[vertex]);
            }
        }
    }

    void Model::createVertexBuffer() {
        VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

        // Create the staging buffer
        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        // Load the data into the staging buffer
        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, bufferSize, 0, &data);
        memcpy(data, vertices.data(), (size_t)bufferSize);
        vkUnmapMemory(context.device, stagingBufferMemory);

        // Create the vertex buffer locally on the GPU
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

        // send the copy buffer command buffer to the GPU
        copyBuffer(context, stagingBuffer, vertexBuffer, bufferSize);

        // Clean up used local resources
        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);
    }

    void Model::createIndexBuffer() {
        VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

        // Create the staging buffer
        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        // Load the data into the staging buffer
        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, bufferSize, 0, &data);
        memcpy(data, indices.data(), (size_t)bufferSize);
        vkUnmapMemory(context.device, stagingBufferMemory);

        // Create the index buffer locally on the GPU
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

        // send the copy buffer command buffer to the GPU
        copyBuffer(context, stagingBuffer, indexBuffer, bufferSize);

        // Clean up used local resources
        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);
    }

    

    // CurveBuffer class implementation
    // ======================================================================================

    CurveBuffer::CurveBuffer(RenderEngine& renderer) : context(renderer.getContext()) {
        vertexBuffer = VK_NULL_HANDLE;
        vertexBufferMemory = VK_NULL_HANDLE;
        indexBuffer = VK_NULL_HANDLE;
        indexBufferMemory = VK_NULL_HANDLE;
    }

    CurveBuffer::~CurveBuffer() {
        destroy();
    }

    CurveBuffer::CurveBuffer(CurveBuffer&& other) noexcept 
        : context(other.context),
        vertices(std::move(other.vertices)),
        indices(std::move(other.indices)),
        transparency(other.transparency),
        vertexBuffer(other.vertexBuffer),
        vertexBufferMemory(other.vertexBufferMemory),
        indexBuffer(other.indexBuffer),
        indexBufferMemory(other.indexBufferMemory)
    {
        other.vertexBuffer = VK_NULL_HANDLE;
        other.vertexBufferMemory = VK_NULL_HANDLE;
        other.indexBuffer = VK_NULL_HANDLE;
        other.indexBufferMemory = VK_NULL_HANDLE;
    }

    CurveBuffer& CurveBuffer::operator=(CurveBuffer&& other) noexcept {
        if (this != &other) {
            vkDeviceWaitIdle(context.device);
            destroy();

            vertices = std::move(other.vertices);
            indices = std::move(other.indices);
            transparency = other.transparency;
            vertexBuffer = other.vertexBuffer;
            vertexBufferMemory = other.vertexBufferMemory;
            indexBuffer = other.indexBuffer;
            indexBufferMemory = other.indexBufferMemory;

            other.vertexBuffer = VK_NULL_HANDLE;
            other.vertexBufferMemory = VK_NULL_HANDLE;
            other.indexBuffer = VK_NULL_HANDLE;
            other.indexBufferMemory = VK_NULL_HANDLE;
        }
        return *this;
    }

    void CurveBuffer::load(core::Polyline2_5D& polyline, float curveTransparency) {
        loadPolyline(polyline);
        createVertexBuffer();
        createIndexBuffer();

        transparency = curveTransparency;
    }

    void CurveBuffer::destroy() {
        if (indexBuffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(context.device, indexBuffer, nullptr);
            indexBuffer = VK_NULL_HANDLE;
        }
        if (indexBufferMemory != VK_NULL_HANDLE) {
            vkFreeMemory(context.device, indexBufferMemory, nullptr);
            indexBufferMemory = VK_NULL_HANDLE;
        }
        if (vertexBuffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(context.device, vertexBuffer, nullptr);
            vertexBuffer = VK_NULL_HANDLE;
        }
        if (vertexBufferMemory != VK_NULL_HANDLE) {
            vkFreeMemory(context.device, vertexBufferMemory, nullptr);
            vertexBufferMemory = VK_NULL_HANDLE;
        }

        vertices.clear();
        indices.clear();
    }

    void CurveBuffer::render(VkCommandBuffer commandBuffer) {
        VkBuffer vertexBuffers[] = { vertexBuffer };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);

        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

        vkCmdDraw(commandBuffer, static_cast<uint32_t>(vertices.size()), 1, 0, 0);
    }

    void CurveBuffer::loadPolyline(core::Polyline2_5D& polyline) {
        std::vector<core::PlineVertex2_5D>& pathVertices = polyline.vertexes();

        // by default, each polyline segment (be it a line-segment, or an arc), gets a random color assigned
        glm::vec3 randomPolyVertexColor;

        auto addLineVertex = [&](core::PlineVertex2_5D& plineVertex) {
            RendererVertex vertex;

            vertex.pos = glm::vec3{ plineVertex.point.x(), plineVertex.point.y(),plineVertex.point.z() };
            vertex.color = randomPolyVertexColor;
            vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

            vertices.push_back(vertex);
            indices.push_back(indices.size());
        };

        auto addArcVertex = [&](core::PlineVertex2_5D& currentVertex, core::PlineVertex2_5D& nextVertex) {
            Vector2d localv1Coords = currentVertex.getPointInPlaneCoords();
            Vector2d localv2Coords = nextVertex.getPointInPlaneCoords();
            core::ArcRadiusAndCenter arcInfo = core::arcRadiusAndCenter(currentVertex.getVertexInPlaneCoords(), nextVertex.getVertexInPlaneCoords());
            
            float startAngle = angle(arcInfo.center, localv1Coords);
            float endAngle = angle(arcInfo.center, localv2Coords);

            float deltaAngle = core::utils::deltaAngle(startAngle, endAngle);
            
            for (size_t k = 0; k < 10; k++)
            {
                Vector2d localPosition;
                localPosition.x() = arcInfo.center.x() + arcInfo.radius * std::cos(startAngle + (deltaAngle / 10.f) * k);
                localPosition.y() = arcInfo.center.y() + arcInfo.radius * std::sin(startAngle + (deltaAngle / 10.f) * k);
                Vector3d vertexPosition = currentVertex.plane.getGlobalCoords(localPosition);

                RendererVertex vertex;

                vertex.pos = glm::vec3{ vertexPosition.x(), vertexPosition.y(), vertexPosition.z() };
                vertex.color = randomPolyVertexColor;
                vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

                vertices.push_back(vertex);
                indices.push_back(indices.size());
            }
        };

        for (size_t i = 0; i < pathVertices.size(); i++)
        {
            randomPolyVertexColor = core::utils::randomVec3();
            if (!pathVertices[i].bulgeIsZero()) {

                // if the polyline is open, the last vertex merely indicates the destination point of the previous polyline vertex, and hence in this case we should always just add the last one as a point
                if (!polyline.isClosed() && i == pathVertices.size() - 1) {
                    addLineVertex(pathVertices[i]);
                    continue;                    
                }

                // in all other cases, we need to know the next vertex to convert an arc into a line-based approximation so it can be rendered
                // since we already took out the case with a last vertex and an open polyline, here we can just find the next using a modulo operator
                core::PlineVertex2_5D& nextVertex = pathVertices[(i + 1) % pathVertices.size()];

                addArcVertex(pathVertices[i], nextVertex);
            }
            else {
                addLineVertex(pathVertices[i]);
            }
        }

        if (polyline.isClosed()) {
            randomPolyVertexColor = core::utils::randomVec3();
            addLineVertex(pathVertices[0]);
        }
    }

    void CurveBuffer::createVertexBuffer() {
        VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

        // Create the staging buffer
        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        // Load the data into the staging buffer
        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, bufferSize, 0, &data);
        memcpy(data, vertices.data(), (size_t)bufferSize);
        vkUnmapMemory(context.device, stagingBufferMemory);

        // Create the vertex buffer locally on the GPU
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

        // send the copy buffer command buffer to the GPU
        copyBuffer(context, stagingBuffer, vertexBuffer, bufferSize);

        // Clean up used local resources
        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);
    }

    void CurveBuffer::createIndexBuffer() {
        VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

        // Create the staging buffer
        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        // Load the data into the staging buffer
        void* data;
        vkMapMemory(context.device, stagingBufferMemory, 0, bufferSize, 0, &data);
        memcpy(data, indices.data(), (size_t)bufferSize);
        vkUnmapMemory(context.device, stagingBufferMemory);

        // Create the index buffer locally on the GPU
        createBuffer(context, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

        // send the copy buffer command buffer to the GPU
        copyBuffer(context, stagingBuffer, indexBuffer, bufferSize);

        // Clean up used local resources
        vkDestroyBuffer(context.device, stagingBuffer, nullptr);
        vkFreeMemory(context.device, stagingBufferMemory, nullptr);
    }



    // Rotation class implementation
    // ======================================================================================

    Rotation::Rotation() {
        eulerAngles = Vector3d::Zero();
        rotationMatrix = Matrix3d::Identity();
    }

    Rotation& Rotation::operator=(const Vector3d& euler) {
        eulerAngles = euler;
        updateRotationMatrix();

        return *this;
    }

    Rotation& Rotation::operator=(const Matrix3d& matrix) {
        rotationMatrix = matrix;
        updateEulerAngles();

        return *this;
    }

    glm::mat4 Rotation::glmMatrix() const {
        glm::mat4 result{1.f};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result[j][i] = rotationMatrix(i, j); // Both eigen and glm are column-major in memory, their access is reversed; i.e. glm::mat[i] gives the i'th column, where Eigen::Matrix(i) gives the i'th row

        return result;
    }

    Matrix4d Rotation::matrix4d() const {
        Matrix4d result = Matrix4d::Identity();
        result.topLeftCorner<3,3>() = rotationMatrix;
        return result;
    }

    Matrix3d Rotation::matrix3d() const {
        return rotationMatrix;
    }

    Vector3d Rotation::angles() const {
        return eulerAngles;
    }

    void Rotation::updateEulerAngles() {
        auto angles = Eigen::EulerAnglesXYZd{rotationMatrix};
        eulerAngles = angles.angles() * (360.0 / (2 * M_PI));
    }

    void Rotation::updateRotationMatrix() {
        auto angles = Eigen::EulerAnglesXYZd{eulerAngles * ((2 * M_PI) / 360.0)};
        rotationMatrix = angles.matrix();
    }

}
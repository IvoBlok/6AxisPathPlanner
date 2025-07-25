#include "renderer.hpp"

#include "core/polylineExport.hpp"
#include "core/mathUtils.hpp"

#include <fstream>
#include <stdexcept>
#include <cstring>
#include <iostream>

#include <nfd.h>

VkDevice device;
VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
VkQueue graphicsQueue;
VkQueue presentQueue;
VkCommandPool commandPool;
VkDescriptorPool descriptorPool;
VkDescriptorSetLayout uniformDescriptorSetLayout;
VkDescriptorSetLayout textureDescriptorSetLayout;

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

VkVertexInputBindingDescription RendererVertex::getBindingDescription() {
    VkVertexInputBindingDescription bindingDescription{};
    bindingDescription.binding = 0;
    bindingDescription.stride = sizeof(RendererVertex);
    bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bindingDescription;
}

std::array<VkVertexInputAttributeDescription, 4> RendererVertex::getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 4> attributeDescriptions{};

    attributeDescriptions[0].binding = 0;
    attributeDescriptions[0].location = 0;
    attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[0].offset = offsetof(RendererVertex, pos);

    attributeDescriptions[1].binding = 0;
    attributeDescriptions[1].location = 1;
    attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[1].offset = offsetof(RendererVertex, color);

    attributeDescriptions[2].binding = 0;
    attributeDescriptions[2].location = 2;
    attributeDescriptions[2].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[2].offset = offsetof(RendererVertex, normal);

    attributeDescriptions[3].binding = 0;
    attributeDescriptions[3].location = 3;
    attributeDescriptions[3].format = VK_FORMAT_R32G32_SFLOAT;
    attributeDescriptions[3].offset = offsetof(RendererVertex, texCoord);

    return attributeDescriptions;
}

bool RendererVertex::operator==(const RendererVertex& other) const {
    return pos == other.pos && color == other.color && texCoord == other.texCoord;
}

namespace std {
    size_t hash<RendererVertex>::operator()(RendererVertex const& vertex) const {
        return ((hash<glm::vec3>()(vertex.pos) ^ (hash<glm::vec3>()(vertex.color) << 1)) >> 1) ^ (hash<glm::vec2>()(vertex.texCoord) << 1);
    }
}



// VulkanHelper definitions
// =====================================================
namespace VulkanHelper {
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory) {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to create vertex buffer!");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate buffer memory!");
        }

        vkBindBufferMemory(device, buffer, bufferMemory, 0);
    }

    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

        endSingleTimeCommands(commandBuffer);
    }

    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkBufferImageCopy region{};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;

        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;

        region.imageOffset = { 0, 0, 0 };
        region.imageExtent = {
                width,
                height,
                1
        };

        vkCmdCopyBufferToImage(
            commandBuffer,
            buffer,
            image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &region
        );

        endSingleTimeCommands(commandBuffer);
    }

    VkCommandBuffer beginSingleTimeCommands() {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);
        return commandBuffer;
    }

    void endSingleTimeCommands(VkCommandBuffer commandBuffer) {
        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    }

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("failed to find suitable memory type!");
    }

    void createImage(uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage,
        VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory) {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = width;
        imageInfo.extent.height = height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = format;
        imageInfo.tiling = tiling;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(device, &imageInfo, nullptr, &image) != VK_SUCCESS) {
            throw std::runtime_error("failed to create image!");
        }

        VkMemoryRequirements memRequirements;
        vkGetImageMemoryRequirements(device, image, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate image memory!");
        }

        vkBindImageMemory(device, image, imageMemory, 0);
    }

    VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspectFlags;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        VkImageView imageView;
        if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
            throw std::runtime_error("failed to create image view!");
        }

        return imageView;
    }

    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectFlags) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = aspectFlags;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage;
        VkPipelineStageFlags destinationStage;

        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        }
        else {
            throw std::invalid_argument("unsupported layout transition!");
        }


        vkCmdPipelineBarrier(
            commandBuffer,
            sourceStage, destinationStage,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier
        );

        endSingleTimeCommands(commandBuffer);
    }
}



// LoadedTexture method definitions
// =====================================================
// public
void LoadedObject::LoadedTexture::load(const char* path) {
    createTextureImage(path);
    createTextureImageView();
    createTextureSampler();
    createTextureDescriptorSet();
}

void LoadedObject::LoadedTexture::free() {
    vkFreeDescriptorSets(device, descriptorPool, 1, &descriptorSet);
}

void LoadedObject::LoadedTexture::destroy() {
    vkDestroySampler(device, textureSampler, nullptr);
    vkDestroyImageView(device, textureImageView, nullptr);
    vkDestroyImage(device, textureImage, nullptr);
    vkFreeMemory(device, textureImageMemory, nullptr);
}

// private
void LoadedObject::LoadedTexture::createTextureImage(const char* path) {
    int texWidth, texHeight, texChannels;
    stbi_uc* pixels = stbi_load(path, &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
    VkDeviceSize imageSize = texWidth * texHeight * 4;

    if (!pixels) {
        throw std::runtime_error("failed to load texture image!");
    }

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;

    VulkanHelper::createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
    memcpy(data, pixels, static_cast<size_t>(imageSize));
    vkUnmapMemory(device, stagingBufferMemory);

    stbi_image_free(pixels);

    VulkanHelper::createImage(texWidth, texHeight, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, textureImage, textureImageMemory);

    VulkanHelper::transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VulkanHelper::copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight));
    VulkanHelper::transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void LoadedObject::LoadedTexture::createTextureImageView() {
    textureImageView = VulkanHelper::createImageView(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT);
}

void LoadedObject::LoadedTexture::createTextureSampler() {
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physicalDevice, &properties);

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

    if (vkCreateSampler(device, &samplerInfo, nullptr, &textureSampler) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture sampler!");
    }
}

void LoadedObject::LoadedTexture::createTextureDescriptorSet() {
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &textureDescriptorSetLayout;

    if (vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet) != VK_SUCCESS) {
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

    vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);
}



// LoadedModel method definitions
// =====================================================
// public
void LoadedObject::LoadedModel::load(const char* path, float modelTransparency) {
    loadModel(path);
    createVertexBuffer();
    createIndexBuffer();

    transparency = modelTransparency;
}

void LoadedObject::LoadedModel::destroy() {
    vkDestroyBuffer(device, indexBuffer, nullptr);
    vkFreeMemory(device, indexBufferMemory, nullptr);

    vkDestroyBuffer(device, vertexBuffer, nullptr);
    vkFreeMemory(device, vertexBufferMemory, nullptr);
}

void LoadedObject::LoadedModel::render(VkCommandBuffer commandBuffer) {
    VkBuffer vertexBuffers[] = { vertexBuffer };
    VkDeviceSize offsets[] = { 0 };
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);

    vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

    vkCmdDrawIndexed(commandBuffer, static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);
}

// private
void LoadedObject::LoadedModel::loadModel(const char* path) {
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

void LoadedObject::LoadedModel::createVertexBuffer() {
    VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

    // Create the staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

    // Load the data into the staging buffer
    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    // Create the vertex buffer locally on the GPU
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

    // send the copy buffer command buffer to the GPU
    VulkanHelper::copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

    // Clean up used local resources
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void LoadedObject::LoadedModel::createIndexBuffer() {
    VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

    // Create the staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

    // Load the data into the staging buffer
    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    // Create the index buffer locally on the GPU
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

    // send the copy buffer command buffer to the GPU
    VulkanHelper::copyBuffer(stagingBuffer, indexBuffer, bufferSize);

    // Clean up used local resources
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}



// LoadedObject method definitions
// =====================================================
// public
LoadedObject::LoadedObject() {
    renderObj = true;
    hideInGUI = false;
    name = "obj";

    position = glm::vec3{0.f};
    scale = glm::vec3{1.f};
    rotation = Vector3f{0.f, 0.f, 0.f};
    rotationMatrix = glm::mat4{1.f};

    bool isOneColor = true;
    color = glm::vec3{0.f};

    model = LoadedModel{};
    texture = LoadedTexture{};
}

LoadedObject::LoadedObject(std::string objectName, bool visible, bool hide) {
    renderObj = visible;
    hideInGUI = hide;
    name = objectName;

    position = glm::vec3{0.f};
    scale = glm::vec3{1.f};
    rotationMatrix = glm::mat4{1.f};

    bool isOneColor = true;
    color = glm::vec3{0.f};

    model = LoadedModel{};
    texture = LoadedTexture{};
}

void LoadedObject::locateWithMatrix(Matrix4d matrix) {
    position = glm::vec3{ matrix(0, 3), matrix(1, 3), matrix(2, 3) };

    rotationMatrix = glm::mat4{1.f};

    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            rotationMatrix[col][row] = static_cast<float>(matrix(row, col)); 
            // Eigen: (row,col) | GLM: [col][row]
        }
    }
    updateRotation();
}

void LoadedObject::updateRotation() {
    Matrix3d matrix;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            matrix(i, j) = rotationMatrix[j][i];

    auto eulerAngles = Eigen::EulerAnglesXYZd{matrix};
    rotation = eulerAngles.angles().cast<float>() * (360.f / (2 * core::PI));
}

void LoadedObject::updateRotationMatrix() {
    auto eulerAngles = Eigen::EulerAnglesXYZf{rotation * ((2 * core::PI) / 360.f)};
    auto matrix = eulerAngles.matrix();

    rotationMatrix = glm::mat4{1.f};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationMatrix[j][i] = matrix(i, j); // Both eigen and glm are column-major in memory, their access is reversed; i.e. glm::mat[i] gives the i'th column, where Eigen::Matrix(i) gives the i'th row
}


core::ObjectShape LoadedObject::getObjectShape() {
    glm::mat4 transformationMatrix = getTransformationMatrix();

    core::ObjectShape objectShape{};
    
    objectShape.vertices.reserve(model.vertices.size());
    objectShape.indices.reserve(model.indices.size());
    objectShape.normals.reserve(model.vertices.size());

    for (size_t i = 0; i < model.vertices.size(); i++)
    {
        glm::vec3 vertexWithAppliedMatrix = transformationMatrix * glm::vec4{ model.vertices[i].pos, 1.f };
        glm::vec3 normalWithAppliedMatrix = transformationMatrix * glm::vec4{ model.vertices[i].normal, 0.f };
        objectShape.vertices.push_back(Vector3d{ vertexWithAppliedMatrix.x, vertexWithAppliedMatrix.y, vertexWithAppliedMatrix.z });
        objectShape.normals.push_back(Vector3d{ normalWithAppliedMatrix.x, normalWithAppliedMatrix.y, normalWithAppliedMatrix.z });
    }

    for (size_t i = 0; i < model.indices.size(); i++)
    {
        objectShape.indices.push_back(model.indices[i]);
    }
    return objectShape;
}

core::Plane LoadedObject::getPlane() {
    glm::vec4 normal{0.f, 0.f, 1.f, 0.f};
    normal = rotationMatrix * normal;

    core::Plane objectPlane{Vector3d{position.x, position.y, position.z}, Vector3d{normal.x, normal.y, normal.z}};
    return objectPlane;
}

void LoadedObject::load(const char* modelPath, const char* texturePath, Vector3d basePosition, Vector3d baseScale, glm::mat4 baseRotationMatrix, float modelTransparency) {
    model.load(modelPath, modelTransparency);
    texture.load(texturePath);

    isOneColor = false;
    position = glm::vec3{ basePosition.x(), basePosition.y(), basePosition.z() };
    scale = glm::vec3{ baseScale.x(), baseScale.y(), baseScale.z() };
    rotationMatrix = baseRotationMatrix;
}

void LoadedObject::load(const char* modelPath, Vector3d objectColor, Vector3d basePosition, Vector3d baseScale, glm::mat4 baseRotationMatrix, float modelTransparency) {
    model.load(modelPath, modelTransparency);

    isOneColor = true;
    color = glm::vec3{ objectColor.x(), objectColor.y(), objectColor.z() };
    position = glm::vec3{ basePosition.x(), basePosition.y(), basePosition.z() };
    scale = glm::vec3{ baseScale.x(), baseScale.y(), baseScale.z() };
    rotationMatrix = baseRotationMatrix;
}

void LoadedObject::destroyRenderData() {
    model.destroy();
    texture.destroy();
}

glm::mat4 LoadedObject::getTransformationMatrix() {
    updateRotationMatrix();

    glm::mat4 transformationMatrix = glm::mat4{1.0f};
    transformationMatrix = glm::translate(transformationMatrix, position);  // 1. Translate first
    transformationMatrix = transformationMatrix * rotationMatrix;  // 2. Then rotate
    transformationMatrix = glm::scale(transformationMatrix, scale);  // 3. Scale last
    return transformationMatrix;
}

void LoadedObject::render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
    if(!renderObj)
        return;

    // get the model matrix
    glm::mat4 transformationMatrix = getTransformationMatrix();

    ObjectShaderPushConstant pushConstant{};
    pushConstant.modelMatrix = transformationMatrix;
    pushConstant.transparency = model.transparency;
    pushConstant.color = color;
    pushConstant.isOneColor = isOneColor;

    // bind the right texture
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &texture.descriptorSet, 0, nullptr);

    // bind the model matrix, and render the object
    vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(ObjectShaderPushConstant), &pushConstant);
    model.render(commandBuffer);
}



// LoadedLine method definitions
// =====================================================
// public
LoadedLine::LoadedLine() {
    renderLine = true;
    hideInGUI = false;
    name = "line";
    lineWidth = 3.f;
    defaultColor = glm::vec3{ 1.f, 0.f, 0.f };
    isOneColor = true;
}

void LoadedLine::load(core::Polyline2_5D& polylineIn, float lineTransparency, glm::vec3 lineColor) {
    defaultColor = lineColor;
    isOneColor = true;

    if(polyline.vertexes().size() != 0)
        throw std::runtime_error("load() called for non-empty LoadedLine!\n");

    polyline.insertPolyLine2_5D(polylineIn);
    loadPolylineIntoRendererFormat();

    createVertexBuffer();
    createIndexBuffer();

    transparency = lineTransparency;
}

void LoadedLine::destroyRenderData() {
    vkDestroyBuffer(device, indexBuffer, nullptr);
    vkFreeMemory(device, indexBufferMemory, nullptr);

    vkDestroyBuffer(device, vertexBuffer, nullptr);
    vkFreeMemory(device, vertexBufferMemory, nullptr);

    vertices.clear();
    indices.clear();
}

void LoadedLine::render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
    if (!renderLine)
        return;

    // set additional values to be sent to the shaders (anything that isn't the mesh itself, like push constants, ubo's etc...)
    // =======================================
    vkCmdSetLineWidth(commandBuffer, lineWidth);

    LineShaderPushConstant pushConstant{};
    pushConstant.color = defaultColor;
    pushConstant.isOneColor = isOneColor;
    pushConstant.transparency = transparency;

    // bind the color vec4, and render the line
    vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(LineShaderPushConstant), &pushConstant);

    // set the vertex and index buffers and call a draw cmd
    // =======================================
    VkBuffer vertexBuffers[] = { vertexBuffer };
    VkDeviceSize offsets[] = { 0 };
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);

    vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

    vkCmdDraw(commandBuffer, static_cast<uint32_t>(vertices.size()), 1, 0, 0);
}

core::Polyline2_5D& LoadedLine::getPolyline() {
    return polyline;
}

void LoadedLine::updateRendererLine() {
    destroyRenderData();

    loadPolylineIntoRendererFormat();

    createVertexBuffer();
    createIndexBuffer();
}

//private
void LoadedLine::createVertexBuffer() {
    VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

    // Create the staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

    // Load the data into the staging buffer
    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    // Create the vertex buffer locally on the GPU
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

    // send the copy buffer command buffer to the GPU
    VulkanHelper::copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

    // Clean up used local resources
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void LoadedLine::createIndexBuffer() {
    VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

    // Create the staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

    // Load the data into the staging buffer
    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    // Create the index buffer locally on the GPU
    VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

    // send the copy buffer command buffer to the GPU
    VulkanHelper::copyBuffer(stagingBuffer, indexBuffer, bufferSize);

    // Clean up used local resources
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void LoadedLine::loadPolylineIntoRendererFormat() {

    std::vector<core::PlineVertex2_5D>& pathVertices = polyline.vertexes();

    glm::vec3 randomPolyVertexColor;
    for (size_t i = 0; i < pathVertices.size(); i++)
    {
        randomPolyVertexColor = core::utils::randomVec3();
        if (!pathVertices[i].bulgeIsZero()) {

            core::PlineVertex2_5D nextVertex;
            if (polyline.isClosed() && i == pathVertices.size() - 1) {
                nextVertex = pathVertices[0];
            }
            else if (i == vertices.size() - 1) {
                RendererVertex vertex;

                vertex.pos = glm::vec3{ pathVertices[i].point.x(), pathVertices[i].point.y(), pathVertices[i].point.z() };
                vertex.color = randomPolyVertexColor;
                vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

                vertices.push_back(vertex);
                indices.push_back(indices.size());
                continue;
            }
            else {
                nextVertex = pathVertices[i + 1];
            }

            Vector2d localv1Coords = pathVertices[i].getPointInPlaneCoords();
            Vector2d localv2Coords = nextVertex.getPointInPlaneCoords();
            core::ArcRadiusAndCenter arcInfo = core::arcRadiusAndCenter(pathVertices[i].getVertexInPlaneCoords(), nextVertex.getVertexInPlaneCoords());
            
            float startAngle = angle(arcInfo.center, localv1Coords);
            float endAngle = angle(arcInfo.center, localv2Coords);

            float deltaAngle = core::utils::deltaAngle(startAngle, endAngle);
            


            for (size_t k = 0; k < 10; k++)
            {
                Vector2d localPosition;
                localPosition.x() = arcInfo.center.x() + arcInfo.radius * std::cos(startAngle + (deltaAngle / 10.f) * k);
                localPosition.y() = arcInfo.center.y() + arcInfo.radius * std::sin(startAngle + (deltaAngle / 10.f) * k);

                RendererVertex vertex;

                Vector3d vertexPosition = pathVertices[i].plane.getGlobalCoords(localPosition);
                vertex.pos = glm::vec3{ vertexPosition.x(), vertexPosition.y(), vertexPosition.z() };
                vertex.color = randomPolyVertexColor;
                vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

                vertices.push_back(vertex);
                indices.push_back(indices.size());
            }
        }
        else {
            RendererVertex vertex;

            vertex.pos = glm::vec3{ pathVertices[i].point.x(), pathVertices[i].point.y(), pathVertices[i].point.z() };
            vertex.color = randomPolyVertexColor;
            vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

            vertices.push_back(vertex);
            indices.push_back(indices.size());
        }
    }

    if (polyline.isClosed()) {
        RendererVertex vertex;

        vertex.pos = glm::vec3{ pathVertices[0].point.x(), pathVertices[0].point.y(), pathVertices[0].point.z() };
        vertex.color = core::utils::randomVec3();
        vertex.texCoord = glm::vec2{ 0.0f, 0.0f };

        vertices.push_back(vertex);
        indices.push_back(indices.size());
    }
}


// VulkanRenderEngine method definitions
// =====================================================
// public
VulkanRenderEngine::VulkanRenderEngine() {
    cameraPosition = glm::vec3{ 0.f };
	cameraFront = glm::vec3{ 0.f, 1.f, 0.f };
	cameraRight = glm::vec3{ 1.f, 0.f, 0.f };
    
    currentFrame = 0;
    frameBufferResized = false;
}

void VulkanRenderEngine::initialize() {
    initWindow();
    initVulkan();
    initImgui();
};

void VulkanRenderEngine::drawFrame() {
    glfwPollEvents();

    // Wait for the previous frame to finish rendering
    vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

    // Retrieve a new image from the swap chain
    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapChain();
        return;
    }
    else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("failed to acquire swap chain image!");
    }

    // Update the uniform buffers
    updateUniformBuffer(currentFrame);

    recordCommandBuffer(commandBuffers[currentFrame], imageIndex);

    // Only reset the fence if work will be submitted to the GPU
    vkResetFences(device, 1, &inFlightFences[currentFrame]);

    // Submit the command buffer.. i.e. actually do the graphics computations
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = { imageAvailableSemaphores[currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffers[currentFrame];
    VkSemaphore signalSemaphores[] = { renderFinishedSemaphores[currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) {
        throw std::runtime_error("failed to submit draw command buffer!");
    }

    // Submit the results back to the swapchain, to be presented to the screen
    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    VkSwapchainKHR swapChains[] = { swapChain };
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &imageIndex;
    presentInfo.pResults = nullptr; // Optional

    VkResult result2 = vkQueuePresentKHR(presentQueue, &presentInfo);

    if (result2 == VK_ERROR_OUT_OF_DATE_KHR || result2 == VK_SUBOPTIMAL_KHR || frameBufferResized) {
        frameBufferResized = false;
        recreateSwapChain();
    }
    else if (result != VK_SUCCESS) {
        throw std::runtime_error("failed to present swap chain image!");
    }

    // Only handle inputs if the user isn't using the GUI
    if (!ImGui::GetIO().WantCaptureKeyboard)
        handleUserInput();

    currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void VulkanRenderEngine::cleanup() {
	vkDeviceWaitIdle(device);

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    cleanupSwapChain();

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkDestroyBuffer(device, uniformBuffers[i], nullptr);
        vkFreeMemory(device, uniformBuffersMemory[i], nullptr);
    }

    vkDestroyDescriptorPool(device, descriptorPool, nullptr);

    for (auto& object : loadedObjects) {
        object->destroyRenderData();
    }

    for (auto& line : loadedLines) {
        line->destroyRenderData();
    }

    vkDestroyDescriptorSetLayout(device, uniformDescriptorSetLayout, nullptr);
    vkDestroyDescriptorSetLayout(device, textureDescriptorSetLayout, nullptr);

    vkDestroyPipeline(device, triangleBasedPipeline, nullptr);
    vkDestroyPipelineLayout(device, triangleBasedPipelineLayout, nullptr);

    vkDestroyPipeline(device, lineBasedPipeline, nullptr);
    vkDestroyPipelineLayout(device, lineBasedPipelineLayout, nullptr);

    vkDestroyRenderPass(device, renderPass, nullptr);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
        vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
        vkDestroyFence(device, inFlightFences[i], nullptr);
    }

    vkDestroyCommandPool(device, commandPool, nullptr);

    vkDestroyDevice(device, nullptr);
    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyInstance(instance, nullptr);

    glfwDestroyWindow(window);

    glfwTerminate();
}

shared_ptr<LoadedObject> VulkanRenderEngine::createObject(
    const char* modelPath, 
    const char* texturePath, 
    Vector3d basePosition, 
    Vector3d baseScale, 
    glm::mat4 rotationMatrix, 
    float modelTransparency) 
{
    if (loadedObjects.size() >= MAX_OBJECTS)
        throw std::runtime_error("Max object count has been reached, can't create a new object!\n");

    shared_ptr<LoadedObject> newObject = std::make_shared<LoadedObject>();
    newObject->load(modelPath, texturePath, basePosition, baseScale, rotationMatrix, modelTransparency);
    loadedObjects.push_back(newObject);

    return newObject;
}

shared_ptr<LoadedObject> VulkanRenderEngine::createObject(
    const char* modelPath, 
    Vector3d objectColor,
    Vector3d basePosition,
    Vector3d baseScale, 
    glm::mat4 rotationMatrix, 
    float modelTransparency) 
{
    if (loadedObjects.size() >= MAX_OBJECTS)
        throw std::runtime_error("Max object count has been reached, can't create a new object!\n");

    shared_ptr<LoadedObject> newObject = std::make_shared<LoadedObject>();

    newObject->load(modelPath, objectColor, basePosition, baseScale, rotationMatrix, modelTransparency);
    loadedObjects.push_back(newObject);

    return newObject;
}

shared_ptr<LoadedLine> VulkanRenderEngine::createLine(
    core::Polyline2_5D& polyline, 
    float lineTransparency, 
    glm::vec3 lineColor) 
{
    if (loadedLines.size() >= MAX_LINES)
        throw std::runtime_error("Max lines count has been reached, can't create a new line!\n");

    shared_ptr<LoadedLine> newLine = std::make_shared<LoadedLine>();

    newLine->load(polyline, lineTransparency, lineColor);
    loadedLines.push_back(newLine);

    return newLine;
}

shared_ptr<LoadedLine> VulkanRenderEngine::createLine(
    core::Polyline2D& polyline, 
    core::Plane plane,
    float lineTransparency, 
    glm::vec3 lineColor) 
{
    if (loadedLines.size() >= MAX_LINES)
        throw std::runtime_error("Max lines count has been reached, can't create a new line!\n");

    core::Polyline2_5D newPolyline{polyline, plane};

    shared_ptr<LoadedLine> newLine = std::make_shared<LoadedLine>();

    newLine->load(newPolyline, lineTransparency, lineColor);
    loadedLines.push_back(newLine);
    
    return newLine;
}

void VulkanRenderEngine::deleteObject(shared_ptr<LoadedObject> object) {
    if (!object) return;
    loadedObjects.remove(object);
}

void VulkanRenderEngine::deleteLine(shared_ptr<LoadedLine> line) {
    if (!line) return;
    loadedLines.remove(line);
}

void VulkanRenderEngine::handleUserInput() {
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

void VulkanRenderEngine::registerGuiModule(std::function<void(VulkanRenderEngine&)> callback) {
    guiCallbacks.emplace_back(std::move(callback));
}

// private 
void VulkanRenderEngine::initWindow() {
    glfwInit();

    // Tell GLFW to not create a OpenGL context, since vulkan is used here
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    window = glfwCreateWindow(WIDTH, HEIGHT, "Vulkan Test", nullptr, nullptr);
    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
}

void VulkanRenderEngine::initVulkan() {
    createInstance();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapChain();
    createImageViews();
    createRenderPass();
    createDescriptorSetLayout();
    createTriangleBasedPipeline();
    createLineBasedPipeline();
    createCommandPool();
    createDepthResources();
    createFrameBuffers();
    createUniformBuffers();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();
}

void VulkanRenderEngine::initImgui() {
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsClassic();
    ImGui_ImplGlfw_InitForVulkan(window, true);

    ImGui_ImplVulkan_InitInfo info;
    info.DescriptorPool = descriptorPool;
    info.RenderPass = renderPass;
    info.Device = device;
    info.PhysicalDevice = physicalDevice;
    info.ImageCount = MAX_FRAMES_IN_FLIGHT;
    info.MsaaSamples = (VkSampleCountFlagBits)0x00000001;
    ImGui_ImplVulkan_Init(&info);

    VkCommandBuffer ImCommandBuffer = VulkanHelper::beginSingleTimeCommands();
    ImGui_ImplVulkan_CreateFontsTexture(ImCommandBuffer);
    VulkanHelper::endSingleTimeCommands(ImCommandBuffer);

    vkDeviceWaitIdle(device);
    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

bool VulkanRenderEngine::QueueFamilyIndices::isComplete() {
    return graphicsFamily.has_value() && presentFamily.has_value();
}

void VulkanRenderEngine::pickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

    if (deviceCount == 0) {
        throw std::runtime_error("failed to find GPUs with Vulkan support!");
    }

    std::vector<VkPhysicalDevice> devices{ deviceCount };
    vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

    for (const auto& device : devices) {
        if (isDeviceSuitable(device)) {
            physicalDevice = device;
            break;
        }
    }

    if (physicalDevice == VK_NULL_HANDLE) {
        throw std::runtime_error("failed to find a suitable GPU!");
    }
}

void VulkanRenderEngine::createLogicalDevice() {
    QueueFamilyIndices indices = findQueueFamilies(physicalDevice);

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
    if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS) {
        throw std::runtime_error("failed to create logical device!");
    }

    // Retrieve the queue implicitly created by the logical device
    vkGetDeviceQueue(device, indices.graphicsFamily.value(), 0, &graphicsQueue);
    vkGetDeviceQueue(device, indices.presentFamily.value(), 0, &presentQueue);
}

bool VulkanRenderEngine::isDeviceSuitable(VkPhysicalDevice device) {
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

bool VulkanRenderEngine::checkDeviceExtensionSupport(VkPhysicalDevice device) {
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

VulkanRenderEngine::QueueFamilyIndices VulkanRenderEngine::findQueueFamilies(VkPhysicalDevice device) {
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

VulkanRenderEngine::SwapChainSupportDetails VulkanRenderEngine::querySwapChainSupport(VkPhysicalDevice device) {
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

void VulkanRenderEngine::createSwapChain() {
    SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

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

    QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
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
    if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) {
        throw std::runtime_error("failed to create swap chain!");
    }

    // Retrieve the handles for the images in the swapchain
    vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
    swapChainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

    // Set private variables, so swapchain settings can be used elsewhere
    swapChainImageFormat = surfaceFormat.format;
    swapChainExtent = extent;
}

void VulkanRenderEngine::cleanupSwapChain() {
    vkDestroyImageView(device, depthImageView, nullptr);
    vkDestroyImage(device, depthImage, nullptr);
    vkFreeMemory(device, depthImageMemory, nullptr);

    for (auto framebuffer : swapChainFramebuffers) {
        vkDestroyFramebuffer(device, framebuffer, nullptr);
    }
    for (auto imageView : swapChainImageViews) {
        vkDestroyImageView(device, imageView, nullptr);
    }
    vkDestroySwapchainKHR(device, swapChain, nullptr);
}

void VulkanRenderEngine::recreateSwapChain() {
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    while (width == 0 || height == 0) {
        glfwGetFramebufferSize(window, &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(device);

    cleanupSwapChain();

    createSwapChain();
    createImageViews();
    createDepthResources();
    createFrameBuffers();
}

void VulkanRenderEngine::createImageViews() {
    swapChainImageViews.resize(swapChainImages.size());
    for (size_t i = 0; i < swapChainImages.size(); i++) {
        swapChainImageViews[i] = VulkanHelper::createImageView(swapChainImages[i], swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
    }
}

void VulkanRenderEngine::createRenderPass() {
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

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;
    subpass.pDepthStencilAttachment = &depthAttachmentRef;

    std::array<VkAttachmentDescription, 2> attachments = { colorAttachment, depthAttachment };
    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) {
        throw std::runtime_error("failed to create render pass!");
    }
}

void VulkanRenderEngine::createDescriptorSetLayout() {
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

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &uboLayoutBinding;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &uniformDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create uniform descriptor set layout!");
    }

    layoutInfo.pBindings = &samplerLayoutBinding;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &textureDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture descriptor set layout!");
    }
}

void VulkanRenderEngine::createTriangleBasedPipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("vert.spv");
    auto fragShaderCode = readShaderFile("frag.spv");

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
    rasterizer.cullMode = VK_CULL_MODE_NONE; // required so that planes render from both sides
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

    VkDescriptorSetLayout setLayouts[] = { uniformDescriptorSetLayout, textureDescriptorSetLayout };

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

    if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &triangleBasedPipelineLayout) != VK_SUCCESS) {
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
    pipelineInfo.layout = triangleBasedPipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    // Actually create the VkPipeline
    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &triangleBasedPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(device, vertShaderModule, nullptr);
    vkDestroyShaderModule(device, fragShaderModule, nullptr);
}

void VulkanRenderEngine::createLineBasedPipeline() {
    // read the shaders into their respective buffers
    auto vertShaderCode = readShaderFile("shaders/lineVert.spv");
    auto fragShaderCode = readShaderFile("shaders/lineFrag.spv");

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
    VkDescriptorSetLayout setLayouts[] = { uniformDescriptorSetLayout };

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

    if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &lineBasedPipelineLayout) != VK_SUCCESS) {
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
    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &lineBasedPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }

    // clean up the local Vulkan variables
    vkDestroyShaderModule(device, vertShaderModule, nullptr);
    vkDestroyShaderModule(device, fragShaderModule, nullptr);
}

void VulkanRenderEngine::createFrameBuffers() {
    swapChainFramebuffers.resize(swapChainImageViews.size());

    for (size_t i = 0; i < swapChainImageViews.size(); i++) {
        std::array<VkImageView, 2> attachments = {
                swapChainImageViews[i],
                depthImageView
        };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = swapChainExtent.width;
        framebufferInfo.height = swapChainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
    }
}

void VulkanRenderEngine::createCommandPool() {
    QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

    if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create command pool!");
    }
}

VkFormat VulkanRenderEngine::findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features) {
    for (VkFormat format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

        if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features) {
            return format;
        }
        else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features) {
            return format;
        }
    }

    throw std::runtime_error("failed to find supported format!");
}

VkFormat VulkanRenderEngine::findDepthFormat() {
    return findSupportedFormat(
        { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
    );
}

void VulkanRenderEngine::createDepthResources() {
    VkFormat depthFormat = findDepthFormat();

    VulkanHelper::createImage(swapChainExtent.width, swapChainExtent.height, depthFormat, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depthImage, depthImageMemory);
    depthImageView = VulkanHelper::createImageView(depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

    VulkanHelper::transitionImageLayout(depthImage, depthFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT);
}

void VulkanRenderEngine::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);

    uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VulkanHelper::createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, uniformBuffers[i], uniformBuffersMemory[i]);

        vkMapMemory(device, uniformBuffersMemory[i], 0, bufferSize, 0, &uniformBuffersMapped[i]);
    }
}

void VulkanRenderEngine::updateUniformBuffer(uint32_t currentImage) {

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

void VulkanRenderEngine::createDescriptorPool() {
    std::array<VkDescriptorPoolSize, 2> poolSizes{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = 3;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT) + 3;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}

void VulkanRenderEngine::createDescriptorSets() {
    std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, uniformDescriptorSetLayout);
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    allocInfo.pSetLayouts = layouts.data();

    descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
    if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = uniformBuffers[i];
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(UniformBufferObject);

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = descriptorSets[i];
        descriptorWrite.dstBinding = 0;
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pBufferInfo = &bufferInfo;

        vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);
    }
}

void VulkanRenderEngine::createCommandBuffers() {
    commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = (uint32_t)MAX_FRAMES_IN_FLIGHT;

    if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate command buffer!");
    }
}

void VulkanRenderEngine::createSyncObjects() {
    imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
            vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
            vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) {

            throw std::runtime_error("failed to create synchronization objects for a frame!");
        }
    }
}

void VulkanRenderEngine::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    beginInfo.pInheritanceInfo = nullptr; // Optional

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("failed to begin recording command buffer!");
    }

    std::array<VkClearValue, 2> clearValues{};
    clearValues[0].color = { {CLEAR_COLOR.x, CLEAR_COLOR.y, CLEAR_COLOR.z, 1.f} };
    clearValues[1].depthStencil = { 1.0f, 0 };

    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = renderPass;
    renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];
    renderPassInfo.renderArea.offset = { 0, 0 };
    renderPassInfo.renderArea.extent = swapChainExtent;
    VkClearValue clearColor = { {{0.0f, 0.0f, 0.0f, 1.0f}} };
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
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, lineBasedPipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    // bind the UBO
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, lineBasedPipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);

    for (auto& line : loadedLines) {
        line->render(commandBuffer, lineBasedPipelineLayout);
    }

    // ========================================
    // Render triangles 
    // ========================================
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, triangleBasedPipeline);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    // bind the UBO
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, triangleBasedPipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);

    // render all objects one after the other...
    for (auto& object : loadedObjects)
    {
        object->render(commandBuffer, triangleBasedPipelineLayout);
    }

    // ========================================
    // Render UI
    // ========================================
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSizeConstraints(ImVec2(400, -1), ImVec2(FLT_MAX, -1));

    std::vector<std::list<shared_ptr<LoadedObject>>::iterator> objectsToDelete;
    std::vector<std::list<shared_ptr<LoadedLine>>::iterator> linesToDelete;

    ImGui::Begin("Configuration", nullptr, 
        ImGuiWindowFlags_NoMove | 
        ImGuiWindowFlags_NoCollapse | 
        ImGuiWindowFlags_NoBringToFrontOnFocus | 
        ImGuiWindowFlags_NoTitleBar);
    {
        if(ImGui::Button("Generate Cube")) {
            shared_ptr<LoadedObject> object = createObject(
                    "../../resources/assets/cube.obj",
                    Vector3d{ 0.f, 0.f, 1.f }, // color
                    Vector3d{ 0.f, 0.f, 0.f }, // position
                    Vector3d{ .25f, .25f, .25f }  // scale
                );
            
            object->name = "cube";
        }
        ImGui::SameLine();
        if(ImGui::Button("Generate Plane")) {
            shared_ptr<LoadedObject> object = createObject(
                    "../../resources/assets/plane.obj",
                    Vector3d{ 1.f, 0.9f, 0.f }, // color
                    Vector3d{ 0.f, 0.f, 0.f },  // position
                    Vector3d{ .5f, .5f, .5f }  // scale
                );
            
            object->name = "plane";
        }
        ImGui::SameLine();
        if(ImGui::Button("Import Object")) {
            char* outPath = NULL;
            nfdresult_t result = NFD_OpenDialog( "obj", NULL, &outPath );
                
            if ( result == NFD_OKAY ) {
                shared_ptr<LoadedObject> object = createObject(
                        outPath,
                        Vector3d{ 0.1f, 0.3f, 0.5f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }  // position
                    );
                
                object->name = outPath;
                free(outPath);
            }
        }


        ImGui::SeparatorText("");
        // properties of loaded Objects
        if (ImGui::CollapsingHeader("Objects")) {
            int i = 0;
            for (auto it = loadedObjects.begin(); it != loadedObjects.end(); ++it, ++i) {
                auto& object = *it;

                if (object->hideInGUI)
                    continue;

                ImGui::PushID(("Objects_" + std::to_string(i)).c_str());

                // Get starting position for this row
                const float row_start_x = ImGui::GetCursorPosX();
                
                // TreeNode with standard behavior
                ImGui::AlignTextToFramePadding();
                bool isOpen = ImGui::TreeNodeEx("##object_node", 
                    ImGuiTreeNodeFlags_SpanAvailWidth | 
                    ImGuiTreeNodeFlags_AllowItemOverlap);
                
                // Name input (fixed position relative to row start)
                ImGui::SameLine(row_start_x + ImGui::GetTreeNodeToLabelSpacing());
                ImGui::SetNextItemWidth(120);
                char nameBuffer[256];
                strncpy(nameBuffer, object->name.c_str(), sizeof(nameBuffer));
                if (ImGui::InputText("##NameEdit", nameBuffer, sizeof(nameBuffer))) {
                    object->name = nameBuffer;
                }

                // Calculate positions for right-aligned controls
                const float checkbox_width = ImGui::GetFrameHeight();
                const float button_width = 25.0f;
                const float spacing = ImGui::GetStyle().ItemSpacing.x;
                const float total_right_width = checkbox_width + button_width + spacing;

                ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - total_right_width);
                ImGui::Checkbox("##RenderToggle", &object->renderObj);

                // Delete button (fixed position relative to window edge)
                ImGui::SameLine();
                if (ImGui::Button("X##CloseObject", ImVec2(25, ImGui::GetFrameHeight()))) {
                    objectsToDelete.push_back(it);
                }

                // Contents when expanded
                if (isOpen) {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Vertices: %d | Faces: %d", object->model.vertices.size(), object->model.indices.size() / 3);
                    float positionArray[3] = { object->position.x, object->position.y, object->position.z };
                    if (ImGui::InputFloat3("Position", positionArray)) {
                        object->position.x = positionArray[0];
                        object->position.y = positionArray[1];
                        object->position.z = positionArray[2];
                    }

                    float scaleArray[3] = { object->scale.x, object->scale.y, object->scale.z };
                    if (ImGui::InputFloat3("Scale", scaleArray)) {
                        object->scale.x = scaleArray[0];
                        object->scale.y = scaleArray[1];
                        object->scale.z = scaleArray[2];
                    }

                    // TODO add some way of controlling the rotation matrix
                    float rotationArray[3] = { object->rotation.x(), object->rotation.y(), object->rotation.z() };
                    if (ImGui::InputFloat3("Rotation", rotationArray)) {
                        object->rotation.x() = rotationArray[0];
                        object->rotation.y() = rotationArray[1];
                        object->rotation.z() = rotationArray[2];
                    }
                    
                    float colorArray[3] = { object->color.x, object->color.y, object->color.z };
                    ImGui::Text("Color ");
                    ImGui::SameLine();
                    ImVec4 colVec4 = ImVec4(colorArray[0], colorArray[1], colorArray[2], 1.0f);
                    if (ImGui::ColorButton("MyColor##3", colVec4, ImGuiColorEditFlags_NoTooltip)) {
                        ImGui::OpenPopup("colorPicker");
                    }
                    if (ImGui::BeginPopup("colorPicker")) {
                        if (ImGui::ColorPicker3("##picker", colorArray, 
                            ImGuiColorEditFlags_DisplayRGB | 
                            ImGuiColorEditFlags_NoSidePreview |
                            ImGuiColorEditFlags_NoSmallPreview)) 
                        {
                            object->color.x = colorArray[0];
                            object->color.y = colorArray[1];
                            object->color.z = colorArray[2];
                        }
                        ImGui::EndPopup();
                    }
                    
                    ImGui::SliderFloat("Transparency", &object->model.transparency, 0.0f, 1.0f);

                    ImGui::TreePop(); // This must be called for each TreeNodeEx
                }
                ImGui::PopID();
            }
        }        
        // properties of loaded Lines
        if(ImGui::CollapsingHeader("Lines")) {
            int i = 0;
            for (auto it = loadedLines.begin(); it != loadedLines.end(); ++it, ++i) {
                auto& line = *it;

                if (line->hideInGUI)
                    continue;

                ImGui::PushID(("Lines_" + std::to_string(i)).c_str());

                // Get starting position for this row
                const float row_start_x = ImGui::GetCursorPosX();
                
                // TreeNode with standard behavior
                ImGui::AlignTextToFramePadding();
                bool isOpen = ImGui::TreeNodeEx("##object_node", 
                    ImGuiTreeNodeFlags_SpanAvailWidth | 
                    ImGuiTreeNodeFlags_AllowItemOverlap);
                
                // Name input (fixed position relative to row start)
                ImGui::SameLine(row_start_x + ImGui::GetTreeNodeToLabelSpacing());
                ImGui::SetNextItemWidth(120);
                char nameBuffer[256];
                strncpy(nameBuffer, line->name.c_str(), sizeof(nameBuffer));
                if (ImGui::InputText("##NameEdit", nameBuffer, sizeof(nameBuffer))) {
                    line->name = nameBuffer;
                }

                // Calculate positions for right-aligned controls
                const float checkbox_width = ImGui::GetFrameHeight();
                const float button_width = 25.0f;
                const float spacing = ImGui::GetStyle().ItemSpacing.x;
                const float total_right_width = checkbox_width + button_width + spacing;

                ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - total_right_width);
                ImGui::Checkbox("##RenderToggle", &line->renderLine);

                // Delete button (fixed position relative to window edge)
                ImGui::SameLine();
                if (ImGui::Button("X##CloseObject", ImVec2(25, ImGui::GetFrameHeight()))) {
                    linesToDelete.push_back(it);
                }

                if (isOpen) {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Vertices: %d | Closed: %d", line->getPolyline().vertexes().size(), line->getPolyline().isClosed());
                    
                    float colorArray[3] = { line->defaultColor.x, line->defaultColor.y, line->defaultColor.z };
                    ImGui::Text("Color ");
                    ImGui::SameLine();
                    ImVec4 colVec4 = ImVec4(colorArray[0], colorArray[1], colorArray[2], 1.0f);
                    if (ImGui::ColorButton("MyColor##3", colVec4, ImGuiColorEditFlags_NoTooltip)) {
                        ImGui::OpenPopup("colorPicker");
                    }
                    if (ImGui::BeginPopup("colorPicker")) {
                        if (ImGui::ColorPicker3("##picker", colorArray, 
                            ImGuiColorEditFlags_DisplayRGB | 
                            ImGuiColorEditFlags_NoSidePreview |
                            ImGuiColorEditFlags_NoSmallPreview)) 
                        {
                            line->defaultColor.x = colorArray[0];
                            line->defaultColor.y = colorArray[1];
                            line->defaultColor.z = colorArray[2];
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - total_right_width);
                    ImGui::Checkbox("##OneColorToggle", &line->isOneColor);
                    
                    ImGui::SliderFloat("Transparency", &line->transparency, 0.0f, 1.0f);
                    ImGui::SliderFloat("Line Width", &line->lineWidth, 0.0f, 10.0f);

                    if (ImGui::Button("Export##LinePolylineExport")) {
                        core::exportSettings settings;
                        settings.useArcs = false;
                        settings.arcReplacementMaxLength = 10.0;
                        settings.fileName = line->name + ".txt";
                        core::exportPath(line->getPolyline(), settings);
                    }

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

    for (auto& object : objectsToDelete)
        loadedObjects.erase(object);

    for (auto& line : linesToDelete)
        loadedLines.erase(line);

    // ========================================

    vkCmdEndRenderPass(commandBuffer);

    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to record command buffer!");
    }
}

VkShaderModule VulkanRenderEngine::createShaderModule(const std::vector<char>& code) {
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    VkShaderModule shaderModule;
    if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
        throw std::runtime_error("failed to create shader module!");
    }

    return shaderModule;
}

VkSurfaceFormatKHR VulkanRenderEngine::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats) {
    for (const auto& availableFormat : availableFormats) {
        if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return availableFormat;
        }
    }

    // if the desired format isn't available, just use the first one
    return availableFormats[0];
}

VkPresentModeKHR VulkanRenderEngine::chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes) {
    for (const auto& availablePresentMode : availablePresentModes) {
        if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return availablePresentMode;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D VulkanRenderEngine::chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities) {
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

void VulkanRenderEngine::createInstance() {
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

bool VulkanRenderEngine::checkValidationLayerSupport() {
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

void VulkanRenderEngine::createSurface() {
    if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
        throw std::runtime_error("failed to create window surface!");
    }
}

void VulkanRenderEngine::framebufferResizeCallback(GLFWwindow* window, int width, int height) {
    auto app = reinterpret_cast<VulkanRenderEngine*>(glfwGetWindowUserPointer(window));
    app->frameBufferResized = true;
}


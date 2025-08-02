#ifndef RENDERER_OBJECT_HPP
#define RENDERER_OBJECT_HPP

#include "RendererGeometryBase.hpp"
#include "renderer/core/RenderEngine.hpp"

#include <optional>
#include <utility>

namespace renderer {
    class Object {
    public:
        bool isObjectRendered;
        bool isObjectShownInGui;

        friend class RenderEngine;
        
        Object(RenderEngine& renderer);
        Object(RenderEngine& renderer, std::string name, bool isObjectRendered = true, bool isObjectShownInGui = true);

        void remove();
        void setPose(Matrix4d matrix);

        void setName(std::string nameIn);
        std::string getName() const;

        int getNumberOfVertices() const;
        int getNumberOfIndices() const;
        
    private:
        // variables associated with the renderer and how it is rendered and interacts with it
        // ======================================================================================
        
        RenderEngine& renderer;

        std::string name;

        // variables associated with the objects' shape, position, orientation, etc...
        // ======================================================================================

        Vector3d position;
        Vector3d scale;
        Rotation rotation;

        Vector3f color;
        std::pair<std::optional<Texture>, bool> texture;
        bool useTexture;

        Model model;

	    void load(const char* modelPath, const char* texturePath, Vector3d basePosition = Vector3d::Zero(), Vector3d baseScale = Vector3d::Ones(), Vector3d baseRotation = Vector3d::Zero(), float modelTransparency = 1.f);
        void load(const char* modelPath, Vector3f objectColor, Vector3d basePosition = Vector3d::Zero(), Vector3d baseScale = Vector3d::Ones(), Vector3d baseRotation = Vector3d::Zero(), float modelTransparency = 1.f);
        
        glm::mat4 getPoseGLM();

        void drawGUI();
        void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);

        void cleanup();
    };
}

#endif
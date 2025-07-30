#ifndef RENDERER_OBJECT_HPP
#define RENDERER_OBJECT_HPP

#include "RendererGeometryBase.hpp"
#include "renderer/core/RenderEngine.hpp"

namespace renderer {
    class Object {
    public:
        void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
        
        
    private:
        friend class RenderEngine;
        // variables associated with the renderer and how it is rendered and interacts with it
        // ======================================================================================
        
        RenderEngine& engine;
        
        bool isObjectRendered;
        bool isObjectShownInGui;

        std::string name;

        // variables associated with the objects' shape, orientation, etc...
        // ======================================================================================
        Vector3d position;
        Vector3d scale;
        Rotation rotation;
        // color (either texture, or 1 color)

        Model model;
        Texture texture; // ? maybe in some smart new format, so that it doesn't exist if 1 singular color was given

        Object(RenderEngine& engine);

    };
}

#endif
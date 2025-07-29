#ifndef RENDERER_CURVE_HPP
#define RENDERER_CURVE_HPP

#include "renderer/core/RenderEngine.hpp"
#include "CustomEigen.hpp"

#include <glm/glm.hpp>

namespace renderer {
    class Curve {
    public:
        Curve();

        void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
        
        
        
    private:
        // variables associated with the renderer and how it is rendered and interacts with it
        // RenderEngine& engine;
        
        bool isObjectRendered;
        bool isObjectShownInGui;

        // variables associated with the objects' shape, orientation, etc...
        Vector3d position;
        Vector3d scale;
        // rotation (some smart thing where both euler angles and matrix is allowed)
        // color (either 1 color or a varying color along the curve)



    };
}

#endif
#ifndef RENDERER_CURVE_HPP
#define RENDERER_CURVE_HPP

#include "RendererGeometryBase.hpp"
#include "renderer/core/RenderEngine.hpp"

namespace renderer {
    class Curve {
    public:
        void remove();

        friend class RenderEngine;

        Curve(RenderEngine& renderer);        
        Curve(RenderEngine& renderer, std::string name, bool isCurveRendered = true, bool isCurveShownInGui = true);
	    Curve(const Curve&) = default;
        
        
    private:
        // variables associated with the renderer and how it is rendered and interacts with it
        // ======================================================================================

        RenderEngine& renderer;
        
        bool isCurveRendered;
        bool isCurveShownInGui;

        std::string name;
        float lineWidth;

        // variables associated with the curve's shape, color, etc ...
        // ======================================================================================

        Vector3f defaultColor;
        bool useDefaultColor;

        CurveBuffer curveBuffer;

        void load(core::Polyline2_5D& polyline, Vector3f defaultColorIn, float transparency);

        void drawGUI();
        void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);

        void cleanup();
    };
}

#endif
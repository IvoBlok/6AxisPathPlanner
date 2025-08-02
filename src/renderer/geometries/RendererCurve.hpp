#ifndef RENDERER_CURVE_HPP
#define RENDERER_CURVE_HPP

#include "RendererGeometryBase.hpp"
#include "renderer/core/RenderEngine.hpp"

namespace renderer {
    class Curve {
    public:
        bool isCurveRendered;
        bool isCurveShownInGui;
        
        friend class RenderEngine;

        Curve(RenderEngine& renderer);        
        Curve(RenderEngine& renderer, std::string name, bool isCurveRendered = true, bool isCurveShownInGui = true);
	    Curve(const Curve&) = default;

        ~Curve();

        void remove();
        
        void setName(std::string nameIn);
        std::string getName() const;

        int getNumberOfVertices() const;

        const core::Polyline2_5D& getPolyline() const;
        
        void drawGUI();
        
    private:
        // variables associated with the renderer and how it is rendered and interacts with it
        // ======================================================================================

        RenderEngine& renderer;

        std::string name;
        float lineWidth;

        // variables associated with the curve's shape, color, etc ...
        // ======================================================================================

        Vector3f defaultColor;
        bool useDefaultColor;

        CurveBuffer curveBuffer;
        core::Polyline2_5D polyline;
        
        void load(core::Polyline2_5D& polylineIn, Vector3f defaultColorIn, float transparency);

        void render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);

        void cleanup();
    };
}

#endif
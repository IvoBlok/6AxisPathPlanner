#ifndef PATH_PLANNER_GUI_HPP
#define PATH_PLANNER_GUI_HPP

#include "renderer.hpp"
#include "ImGuiAdditions.hpp"

#include "toolPath2_5D.hpp"
#include "toolPath2_5DGUI.hpp"

class PathPlannerGUI {
public:
    toolPath2_5D::FacePassGUI facePassGUI;

    PathPlannerGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Test Thingie##thingie")) {

            facePassGUI.drawFacePass(renderer);
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }    


private:
    enum class toolPathOptions {
        facePass2_5D,
        surfacePass2_5D,
        clearingPass2_5D
    };

};

#endif
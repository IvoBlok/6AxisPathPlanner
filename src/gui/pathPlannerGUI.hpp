#ifndef PATH_PLANNER_GUI_HPP
#define PATH_PLANNER_GUI_HPP

#include <memory>

#include "renderer.hpp"
#include "toolPathGUIBase.hpp"

#include "toolPath2_5D.hpp"
#include "toolPath2_5DGUI.hpp"

class PathPlannerGUI {
public:
    PathPlannerGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);

        toolPathOptions.push_back(std::make_unique<toolPath2_5D::FacePassGUI>());
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::SurfacePassGUI>());
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::ClearingPassGUI>());
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("path planner##pathPlanner")) {
            std::vector<std::string> toolPathNames;
            for (const auto& option : toolPathOptions)
                toolPathNames.push_back(option->name());
            
            static int toolPathSelectionIndex = -1;

            handleDropdown("toolPathSelectionDropdown", "toolpath ...", toolPathNames, toolPathSelectionIndex);
            ImGui::SameLine();
            if (toolPathSelectionIndex >= 0 && toolPathSelectionIndex < toolPathOptions.size()) {
                if (ImGui::Button("Add Tool Path")) {
                    toolPaths.push_back(toolPathOptions[toolPathSelectionIndex]->clone());
                }
            }

            for (auto& toolPath : toolPaths) {
                toolPath->draw(renderer);
            }
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }    


private:
    std::vector<std::unique_ptr<ToolPathGUIBase>> toolPaths;
    std::vector<std::unique_ptr<ToolPathGUIBase>> toolPathOptions;

};

#endif
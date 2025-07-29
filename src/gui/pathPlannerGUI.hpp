#ifndef PATH_PLANNER_GUI_HPP
#define PATH_PLANNER_GUI_HPP

#include <memory>

#include "renderer.hpp"

#include "toolPath2_5D.hpp"
#include "toolPath2_5DGUI.hpp"

class PathPlannerGUI {
public:
    PathPlannerGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);

        //toolPathOptions.push_back(std::make_unique<toolPath2_5D::FacePassGUI>(renderer));
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::SurfacePassGUI>(renderer));
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::ClearingPassGUI>(renderer));

        toolPathSelectionIndex = -1;
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("path planner##pathPlanner")) {
            std::vector<std::string> toolPathNames;
            for (const auto& option : toolPathOptions)
                toolPathNames.push_back(option->name());
            
            handleDropdown("toolPathSelectionDropdown", "", "choose tool path type ...", toolPathNames, toolPathSelectionIndex);
            ImGui::SameLine();
            if (ImGui::Button("Add Tool Path")) {
                if (toolPathSelectionIndex >= 0 && toolPathSelectionIndex < toolPathOptions.size()) {
                    toolPaths.push_back(toolPathOptions[toolPathSelectionIndex]->clone());
                }
            }

            for (int i = 0; i < toolPaths.size(); i++) {
                toolPaths[i]->draw(std::to_string(i));
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

    int toolPathSelectionIndex;
};

#endif
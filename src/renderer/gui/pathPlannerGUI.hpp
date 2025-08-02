#ifndef PATH_PLANNER_GUI_HPP
#define PATH_PLANNER_GUI_HPP

#include "toolPath2_5D.hpp"
#include "toolPath2_5DGUI.hpp"

#include <memory>
#include <list>

class PathPlannerGUI {
public:
    PathPlannerGUI(RenderEngine& renderer) {
        registerWithRenderer(renderer);

        toolPathOptions.push_back(std::make_unique<toolPath2_5D::FacePassGUI>(renderer));
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::SurfacePassGUI>(renderer));
        toolPathOptions.push_back(std::make_unique<toolPath2_5D::ClearingPassGUI>(renderer));

        toolPathSelectionIndex = -1;
    }

    void drawGUI(RenderEngine& renderer) {
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
            

            decltype(toolPaths)::iterator toDelete = toolPaths.end();
            int i = 0;
            for (auto it = toolPaths.begin(); it != toolPaths.end(); ++it, ++i) {
                auto& toolPath = *it;
                std::string treeNodeName = std::to_string(i) + ": " + toolPath->name();

                ImGui::PushID(std::to_string(i).c_str());
                ImGui::AlignTextToFramePadding();
                bool isOpen = ImGui::TreeNodeEx(treeNodeName.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_AllowItemOverlap);

                const float buttonWidth = 25.f;
                ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - buttonWidth);
                if (ImGui::Button("X##RemoveToolPathOption", ImVec2((int)buttonWidth, ImGui::GetFrameHeight()))) {
                    toDelete = it;
                }

                toolPath->draw(isOpen);

                if (isOpen)
                    ImGui::TreePop();

                ImGui::PopID();
                i++;
            }

            if (toDelete != toolPaths.end())
                toolPaths.erase(toDelete);
        }
    }

    void registerWithRenderer(RenderEngine& renderer) {
        renderer.registerGuiModule([this](RenderEngine& r) {
            this->drawGUI(r); 
        });
    }    


private:
    std::list<std::unique_ptr<ToolPathGUIBase>> toolPaths;
    std::vector<std::unique_ptr<ToolPathGUIBase>> toolPathOptions;

    int toolPathSelectionIndex;
};

#endif
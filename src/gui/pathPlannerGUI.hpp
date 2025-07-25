#ifndef PATH_PLANNER_GUI_HPP
#define PATH_PLANNER_GUI_HPP

#include "renderer.hpp"

#include "meshIntersect.hpp"
#include "toolPath2_5D.hpp"
#include "robotKinematics.hpp"

class PathPlannerGUI {
public:



    PathPlannerGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Test Thingie##thingie")) {

            auto& objects = renderer.loadedObjects;
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            static int selectedIndex = -1;

            handleDropdown("testDropDown", "wow test dropdown!", objectNames, selectedIndex);
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }    


private:
    void handleDropdown(std::string ID, std::string dropdownText, std::vector<std::string>& objectNames, int& selectedIndex) {
        ImGui::PushID(ID.c_str());
        ImGui::Text(dropdownText.c_str());
        const char* preview = (selectedIndex != -1) ? objectNames[selectedIndex].c_str() : "-";
        std::string comboLabel = "##" + ID + "_combo";
        if (ImGui::BeginCombo(comboLabel.c_str(), preview)) {
            for (int i = 0; i < (int)objectNames.size(); ++i) {
                ImGui::PushID(i);
                bool isSelected = (selectedIndex == i);
                if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                    selectedIndex = i;
                }
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
    }

};

#endif
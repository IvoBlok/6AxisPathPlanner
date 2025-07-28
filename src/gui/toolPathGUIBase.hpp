#ifndef TOOLPATH_GUI_BASE_HPP
#define TOOLPATH_GUI_BASE_HPP

#include "renderer.hpp"

inline void handleDropdown(std::string ID, std::string dropdownText, std::vector<std::string>& objectNames, int& selectedIndex) {
    ImGui::PushID(ID.c_str());
    ImGui::Text(dropdownText.c_str());

    selectedIndex = (selectedIndex >= 0 && selectedIndex < (int)objectNames.size()) ? selectedIndex : -1;
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

class ToolPathGUIBase {
public:
    virtual void draw(VulkanRenderEngine& renderer, std::string ID) = 0;
    virtual std::string name() const = 0;
    virtual std::unique_ptr<ToolPathGUIBase> clone() const = 0;
};  

#endif
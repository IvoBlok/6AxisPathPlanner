#ifndef IMGUI_ADDITIONS_HPP
#define IMGUI_ADDITIONS_HPP

#include "renderer.hpp"

inline void handleDropdown(std::string ID, std::string dropdownText, std::vector<std::string>& objectNames, int& selectedIndex) {
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

#endif
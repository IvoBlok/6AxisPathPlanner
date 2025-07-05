#ifndef MESH_INTERSECT_GUI_HPP
#define MESH_INTERSECT_GUI_HPP

#include "renderer.hpp"
#include "meshIntersect.hpp"

namespace meshIntersect {
class MeshIntersectGUI {
public:
    MeshIntersectGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Mesh Intersection##mesh_intersect_header")) 
        {
            auto& objects = renderer.loadedObjects;
            
            // Safe static storage with proper scoping
            static int selectedIdx1 = -1;
            static int selectedIdx2 = -1;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            // Validate indices
            selectedIdx1 = (selectedIdx1 >= 0 && selectedIdx1 < (int)objects.size()) ? selectedIdx1 : -1;
            selectedIdx2 = (selectedIdx2 >= 0 && selectedIdx2 < (int)objects.size()) ? selectedIdx2 : -1;

            // First dropdown - Plane selection
            ImGui::PushID("plane_select_group");
            ImGui::Text("Select Plane:");
            const char* preview1 = (selectedIdx1 != -1) ? objectNames[selectedIdx1].c_str() : "-";
            if (ImGui::BeginCombo("##plane_select_combo", preview1)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedIdx1 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIdx1 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Second dropdown - Shape selection
            ImGui::PushID("shape_select_group");
            ImGui::Text("Select Shape:");
            const char* preview2 = (selectedIdx2 != -1) ? objectNames[selectedIdx2].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + objects.size()); // Different ID range
                    bool isSelected = (selectedIdx2 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIdx2 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Intersection button
            if (ImGui::Button("Calculate Intersection")) {
                if (selectedIdx1 != -1 && selectedIdx2 != -1) {

                    auto obj1 = objects.begin();
                    std::advance(obj1, selectedIdx1);
                    auto obj2 = objects.begin();
                    std::advance(obj2, selectedIdx2);
                    
                    core::Plane plane = (*obj1)->getPlane();
                    core::ObjectShape shape = (*obj2)->getObjectShape();

                    std::vector<core::Polyline2D> meshPlaneIntersect = getMeshPlaneIntersection(plane, shape);
                    
                    for (auto& curve : meshPlaneIntersect) 
                        renderer.createLine(curve, plane);
                }
                else {
                    std::cerr << "Invalid object selection!\n";
                }
            }
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }
};

} // namespace meshIntersect

#endif
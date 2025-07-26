#ifndef TOOL_PATH_2_5D_GUI_HPP
#define TOOL_PATH_2_5D_GUI_HPP

#include "renderer.hpp"
#include "toolPathGUIBase.hpp"

#include "toolPath2_5D.hpp"

namespace toolPath2_5D {
class FacePassGUI : public ToolPathGUIBase {
public:
    FacePass2_5DInfo info;
    
    FacePassGUI() {
        selectedIndex1 = -1;
        selectedIndex2 = -1;
    }

    void draw(VulkanRenderEngine& renderer) override {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Face Pass##2_5dToolPath")) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);
            
            // Validate indices
            selectedIndex1 = (selectedIndex1 >= 0 && selectedIndex1 < (int)objects.size()) ? selectedIndex1 : -1;
            selectedIndex2 = (selectedIndex2 >= 0 && selectedIndex2 < (int)objects.size()) ? selectedIndex2 : -1;

            // First dropdown - Plane selection
            ImGui::PushID("face_pass_plane_select_group");
            ImGui::Text("Select Plane:");
            const char* preview1 = (selectedIndex1 != -1) ? objectNames[selectedIndex1].c_str() : "-";
            if (ImGui::BeginCombo("##plane_select_combo", preview1)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedIndex1 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex1 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();
            
            // Second dropdown - Stock selection
            ImGui::PushID("face_pass_shape_select_group");
            ImGui::Text("Select Stock:");
            const char* preview2 = (selectedIndex2 != -1) ? objectNames[selectedIndex2].c_str() : "-";
            if (ImGui::BeginCombo("##stock_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex2 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex2 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();
            
            // Aditional toolpath info
            ImGui::InputDouble("Traverse height [mm]##face_pass", &info.safeTraverseHeight);
            ImGui::InputDouble("Stepover [mm]##face_pass", &info.stepOver);

            if (ImGui::Button("Generate!##face_pass")) {
                if(selectedIndex1 != -1 && selectedIndex2 != -1) {
                    auto planeObj = objects.begin();
                    std::advance(planeObj, selectedIndex1);
                    auto stockObj = objects.begin();
                    std::advance(stockObj, selectedIndex2);

                    info.slicingPlane = (*planeObj)->getPlane();
                    info.stock = (*stockObj)->getObjectShape();

                    core::Polyline2_5D result = generateFacePass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createLine(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }

    std::string name() const override {
        return "2.5D Face pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<FacePassGUI>();
    }

private:
    int selectedIndex1;
    int selectedIndex2;
};

class SurfacePassGUI : public ToolPathGUIBase {
public:
    SurfacePass2_5DInfo info;

    SurfacePassGUI() {
        selectedIndex1 = -1;
        selectedIndex2 = -1;
        selectedIndex3 = -1;
    }

    void draw(VulkanRenderEngine& renderer) override {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Surface Pass##2_5dToolPath")) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            // Validate indices
            selectedIndex1 = (selectedIndex1 >= 0 && selectedIndex1 < (int)objects.size()) ? selectedIndex1 : -1;
            selectedIndex2 = (selectedIndex2 >= 0 && selectedIndex2 < (int)objects.size()) ? selectedIndex2 : -1;
            selectedIndex3 = (selectedIndex3 >= 0 && selectedIndex3 < (int)objects.size()) ? selectedIndex3 : -1;

            // First dropdown - Start Plane selection
            ImGui::PushID("face_pass_start_plane_select_group");
            ImGui::Text("Select Start Plane:");
            const char* preview1 = (selectedIndex1 != -1) ? objectNames[selectedIndex1].c_str() : "-";
            if (ImGui::BeginCombo("##start_plane_select_combo", preview1)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedIndex1 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex1 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Second dropdown - End Plane selection
            ImGui::PushID("face_pass_end_plane_select_group");
            ImGui::Text("Select End Plane:");
            const char* preview2 = (selectedIndex2 != -1) ? objectNames[selectedIndex2].c_str() : "-";
            if (ImGui::BeginCombo("##end_plane_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex2 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex2 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Third dropdown - Shape selection
            ImGui::PushID("face_pass_shape_select_group");
            ImGui::Text("Select Shape:");
            const char* preview3 = (selectedIndex3 != -1) ? objectNames[selectedIndex3].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview3)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex3 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex3 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // additional toolpath settings
            ImGui::InputDouble("Traverse height [mm]##surface_pass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##surface_pass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##surface_pass", &info.depthOfCut);

            if (ImGui::Button("Generate!##surface_pass")) {
                if(selectedIndex1 != -1 && selectedIndex2 != -1 && selectedIndex3 != -1) {
                    auto startPlaneObj = objects.begin();
                    std::advance(startPlaneObj, selectedIndex1);
                    auto endPlaneObj = objects.begin();
                    std::advance(endPlaneObj, selectedIndex2);
                    auto shapeObj = objects.begin();
                    std::advance(shapeObj, selectedIndex3);

                    info.startPlane = (*startPlaneObj)->getPlane();
                    info.endPlane = (*endPlaneObj)->getPlane();
                    info.shape = (*shapeObj)->getObjectShape();

                    core::Polyline2_5D result = generateSurfacePass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createLine(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }

    std::string name() const override {
        return "2.5D Surface pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<SurfacePassGUI>();
    }

private:
    int selectedIndex1;
    int selectedIndex2;
    int selectedIndex3;
};

class ClearingPassGUI : public ToolPathGUIBase {
public:
    ClearingPass2_5DInfo info;

    ClearingPassGUI() {
        selectedIndex1 = -1;
        selectedIndex2 = -1;
        selectedIndex3 = -1;
        selectedIndex4 = -1;
    }

    void draw(VulkanRenderEngine& renderer) override {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Clearing Pass##2_5dToolPath")) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            // Validate indices
            selectedIndex1 = (selectedIndex1 >= 0 && selectedIndex1 < (int)objects.size()) ? selectedIndex1 : -1;
            selectedIndex2 = (selectedIndex2 >= 0 && selectedIndex2 < (int)objects.size()) ? selectedIndex2 : -1;
            selectedIndex3 = (selectedIndex3 >= 0 && selectedIndex3 < (int)objects.size()) ? selectedIndex3 : -1;
            selectedIndex4 = (selectedIndex4 >= 0 && selectedIndex4 < (int)objects.size()) ? selectedIndex4 : -1;

            // First dropdown - Start Plane selection
            ImGui::PushID("clearing_pass_start_plane_select_group");
            ImGui::Text("Select Start Plane:");
            const char* preview1 = (selectedIndex1 != -1) ? objectNames[selectedIndex1].c_str() : "-";
            if (ImGui::BeginCombo("##start_plane_select_combo", preview1)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedIndex1 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex1 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Second dropdown - End Plane selection
            ImGui::PushID("clearing_pass_end_plane_select_group");
            ImGui::Text("Select End Plane:");
            const char* preview2 = (selectedIndex2 != -1) ? objectNames[selectedIndex2].c_str() : "-";
            if (ImGui::BeginCombo("##end_plane_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex2 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex2 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Third dropdown - Shape selection
            ImGui::PushID("clearing_pass_shape_select_group");
            ImGui::Text("Select Shape:");
            const char* preview3 = (selectedIndex3 != -1) ? objectNames[selectedIndex3].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview3)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex3 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex3 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // Fourth dropdown - Stock selection
            ImGui::PushID("clearing_pass_stock_select_group");
            ImGui::Text("Select Stock:");
            const char* preview4 = (selectedIndex4 != -1) ? objectNames[selectedIndex4].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview4)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIndex4 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIndex4 = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            // additional toolpath settings
            ImGui::InputDouble("Traverse height [mm]##clearing_pass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##clearing_pass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##clearing_pass", &info.depthOfCut);
            ImGui::InputDouble("Stepover [mm]##clearing_pass", &info.stepOver);

            if (ImGui::Button("Generate!##clearing_pass")) {
                if(selectedIndex1 != -1 && selectedIndex2 != -1 && selectedIndex3 != -1 && selectedIndex4 != -1) {
                    auto startPlaneObj = objects.begin();
                    std::advance(startPlaneObj, selectedIndex1);
                    auto endPlaneObj = objects.begin();
                    std::advance(endPlaneObj, selectedIndex2);
                    auto shapeObj = objects.begin();
                    std::advance(shapeObj, selectedIndex3);
                    auto stockObj = objects.begin();
                    std::advance(stockObj, selectedIndex4);

                    info.startPlane = (*startPlaneObj)->getPlane();
                    info.endPlane = (*endPlaneObj)->getPlane();
                    info.shape = (*shapeObj)->getObjectShape();
                    info.stock = (*stockObj)->getObjectShape();

                    core::Polyline2_5D result = generateClearingPass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createLine(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }

    std::string name() const override {
        return "2.5D Clearing pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<ClearingPassGUI>();
    }

private:
    int selectedIndex1;
    int selectedIndex2;
    int selectedIndex3;
    int selectedIndex4;
};

} // namespace toolPath2_5D

#endif
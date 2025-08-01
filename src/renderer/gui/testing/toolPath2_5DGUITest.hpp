#ifndef TOOL_PATH_2_5D_GUI_TEST_HPP
#define TOOL_PATH_2_5D_GUI_TEST_HPP

#include "renderer/core/RenderEngine.hpp"
#include "renderer/geometries/RendererObject.hpp"
#include "renderer/geometries/RendererCurve.hpp"

#include "toolPath2_5D.hpp"

namespace toolPath2_5D {
class ToolPath2_5DGUITest {
public:
    ToolPath2_5DGUITest(RenderEngine& renderer) {
        registerWithRenderer(renderer);
    }

    void drawGUI(RenderEngine& renderer) {
        if (ImGui::CollapsingHeader("2.5D Toolpaths##2_5dToolPath")) {
            drawFacePass(renderer);
            drawSurfacePass(renderer);
            drawClearingPass(renderer);
        } 
    }

    void registerWithRenderer(RenderEngine& renderer) {
        renderer.registerGuiModule([this](RenderEngine& r) {
            this->drawGUI(r); 
        });
    }

private:
    FacePass2_5DInfo facePassInfo;
    SurfacePass2_5DInfo surfacePassInfo;
    ClearingPass2_5DInfo clearingPassInfo;

    void drawFacePass(RenderEngine& renderer) {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Face Pass##2_5dToolPath")) {
            auto& objects = renderer.getObjects();

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());
            

            // Safe static storage with proper scoping
            static int selectedIdx1 = -1;
            static int selectedIdx2 = -1;

            // Validate indices
            selectedIdx1 = (selectedIdx1 >= 0 && selectedIdx1 < (int)objects.size()) ? selectedIdx1 : -1;
            selectedIdx2 = (selectedIdx2 >= 0 && selectedIdx2 < (int)objects.size()) ? selectedIdx2 : -1;

            // First dropdown - Plane selection
            ImGui::PushID("face_pass_plane_select_group");
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
            
            // Second dropdown - Stock selection
            ImGui::PushID("face_pass_shape_select_group");
            ImGui::Text("Select Stock:");
            const char* preview2 = (selectedIdx2 != -1) ? objectNames[selectedIdx2].c_str() : "-";
            if (ImGui::BeginCombo("##stock_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
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
            
            // Aditional toolpath info
            ImGui::InputDouble("Traverse height [mm]##face_pass", &facePassInfo.safeTraverseHeight);
            ImGui::InputDouble("Stepover [mm]##face_pass", &facePassInfo.stepOver);

            if (ImGui::Button("Generate!##face_pass")) {
                if(selectedIdx1 != -1 && selectedIdx2 != -1) {
                    auto planeObj = objects.begin();
                    std::advance(planeObj, selectedIdx1);
                    auto stockObj = objects.begin();
                    std::advance(stockObj, selectedIdx2);

                    facePassInfo.slicingPlane = (*planeObj)->getPlane();
                    facePassInfo.stock = (*stockObj)->getObjectShape();

                    core::Polyline2_5D result = generateFacePass2_5D(facePassInfo);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }

    void drawSurfacePass(RenderEngine& renderer) {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Surface Pass##2_5dToolPath")) {
            auto& objects = renderer.getObjects();

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());

            // Safe static storage with proper scoping
            static int selectedIdx1 = -1;
            static int selectedIdx2 = -1;
            static int selectedIdx3 = -1;

            // Validate indices
            selectedIdx1 = (selectedIdx1 >= 0 && selectedIdx1 < (int)objects.size()) ? selectedIdx1 : -1;
            selectedIdx2 = (selectedIdx2 >= 0 && selectedIdx2 < (int)objects.size()) ? selectedIdx2 : -1;
            selectedIdx3 = (selectedIdx3 >= 0 && selectedIdx3 < (int)objects.size()) ? selectedIdx3 : -1;

            // First dropdown - Start Plane selection
            ImGui::PushID("face_pass_start_plane_select_group");
            ImGui::Text("Select Start Plane:");
            const char* preview1 = (selectedIdx1 != -1) ? objectNames[selectedIdx1].c_str() : "-";
            if (ImGui::BeginCombo("##start_plane_select_combo", preview1)) {
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

            // Second dropdown - End Plane selection
            ImGui::PushID("face_pass_end_plane_select_group");
            ImGui::Text("Select End Plane:");
            const char* preview2 = (selectedIdx2 != -1) ? objectNames[selectedIdx2].c_str() : "-";
            if (ImGui::BeginCombo("##end_plane_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
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

            // Third dropdown - Shape selection
            ImGui::PushID("face_pass_shape_select_group");
            ImGui::Text("Select Shape:");
            const char* preview3 = (selectedIdx3 != -1) ? objectNames[selectedIdx3].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview3)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIdx3 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIdx3 = i;
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
            ImGui::InputDouble("Traverse height [mm]##surface_pass", &surfacePassInfo.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##surface_pass", &surfacePassInfo.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##surface_pass", &surfacePassInfo.depthOfCut);

            if (ImGui::Button("Generate!##surface_pass")) {
                if(selectedIdx1 != -1 && selectedIdx2 != -1 && selectedIdx3 != -1) {
                    auto startPlaneObj = objects.begin();
                    std::advance(startPlaneObj, selectedIdx1);
                    auto endPlaneObj = objects.begin();
                    std::advance(endPlaneObj, selectedIdx2);
                    auto shapeObj = objects.begin();
                    std::advance(shapeObj, selectedIdx3);

                    surfacePassInfo.startPlane = (*startPlaneObj)->getPlane();
                    surfacePassInfo.endPlane = (*endPlaneObj)->getPlane();
                    surfacePassInfo.shape = (*shapeObj)->getObjectShape();

                    core::Polyline2_5D result = generateSurfacePass2_5D(surfacePassInfo);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }

    void drawClearingPass(RenderEngine& renderer) {
        ImGui::Indent(15.0f);
        if (ImGui::CollapsingHeader("Clearing Pass##2_5dToolPath")) {
            auto& objects = renderer.getObjects();

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());

            // Safe static storage with proper scoping
            static int selectedIdx1 = -1;
            static int selectedIdx2 = -1;
            static int selectedIdx3 = -1;
            static int selectedIdx4 = -1;

            // Validate indices
            selectedIdx1 = (selectedIdx1 >= 0 && selectedIdx1 < (int)objects.size()) ? selectedIdx1 : -1;
            selectedIdx2 = (selectedIdx2 >= 0 && selectedIdx2 < (int)objects.size()) ? selectedIdx2 : -1;
            selectedIdx3 = (selectedIdx3 >= 0 && selectedIdx3 < (int)objects.size()) ? selectedIdx3 : -1;
            selectedIdx4 = (selectedIdx4 >= 0 && selectedIdx4 < (int)objects.size()) ? selectedIdx4 : -1;

            // First dropdown - Start Plane selection
            ImGui::PushID("clearing_pass_start_plane_select_group");
            ImGui::Text("Select Start Plane:");
            const char* preview1 = (selectedIdx1 != -1) ? objectNames[selectedIdx1].c_str() : "-";
            if (ImGui::BeginCombo("##start_plane_select_combo", preview1)) {
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

            // Second dropdown - End Plane selection
            ImGui::PushID("clearing_pass_end_plane_select_group");
            ImGui::Text("Select End Plane:");
            const char* preview2 = (selectedIdx2 != -1) ? objectNames[selectedIdx2].c_str() : "-";
            if (ImGui::BeginCombo("##end_plane_select_combo", preview2)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
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

            // Third dropdown - Shape selection
            ImGui::PushID("clearing_pass_shape_select_group");
            ImGui::Text("Select Shape:");
            const char* preview3 = (selectedIdx3 != -1) ? objectNames[selectedIdx3].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview3)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIdx3 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIdx3 = i;
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
            const char* preview4 = (selectedIdx4 != -1) ? objectNames[selectedIdx4].c_str() : "-";
            if (ImGui::BeginCombo("##shape_select_combo", preview4)) {
                for (int i = 0; i < (int)objects.size(); ++i) {
                    ImGui::PushID(i + 2*objects.size()); // Different ID range
                    bool isSelected = (selectedIdx4 == i);
                    if (ImGui::Selectable(objectNames[i].c_str(), isSelected)) {
                        selectedIdx4 = i;
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
            ImGui::InputDouble("Traverse height [mm]##clearing_pass", &clearingPassInfo.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##clearing_pass", &clearingPassInfo.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##clearing_pass", &clearingPassInfo.depthOfCut);
            ImGui::InputDouble("Stepover [mm]##clearing_pass", &clearingPassInfo.stepOver);

            if (ImGui::Button("Generate!##clearing_pass")) {
                if(selectedIdx1 != -1 && selectedIdx2 != -1 && selectedIdx3 != -1 && selectedIdx4 != -1) {
                    auto startPlaneObj = objects.begin();
                    std::advance(startPlaneObj, selectedIdx1);
                    auto endPlaneObj = objects.begin();
                    std::advance(endPlaneObj, selectedIdx2);
                    auto shapeObj = objects.begin();
                    std::advance(shapeObj, selectedIdx3);
                    auto stockObj = objects.begin();
                    std::advance(stockObj, selectedIdx4);

                    clearingPassInfo.startPlane = (*startPlaneObj)->getPlane();
                    clearingPassInfo.endPlane = (*endPlaneObj)->getPlane();
                    clearingPassInfo.shape = (*shapeObj)->getObjectShape();
                    clearingPassInfo.stock = (*stockObj)->getObjectShape();

                    core::Polyline2_5D result = generateClearingPass2_5D(clearingPassInfo);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        }
        ImGui::Unindent(15.0f);
    }
};

} // namespace toolPath2_5D

#endif
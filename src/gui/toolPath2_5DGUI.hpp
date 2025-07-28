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

    void draw(VulkanRenderEngine& renderer, std::string ID) override {
        ImGui::PushID(ID.c_str());
        ImGui::AlignTextToFramePadding();
        bool isOpen = ImGui::TreeNodeEx((ID + ": Face Pass").c_str(), ImGuiTreeNodeFlags_SpanAvailWidth);
        if (isOpen) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);
            
            // First dropdown: Start Plane selection
            handleDropdown("facePassPlaneSelectGroup", "Select Plane:", objectNames, selectedIndex1);
            
            // Second dropdown: Stock selection
            handleDropdown("facePassStockSelectGroup", "Select Stock:", objectNames, selectedIndex2);
            
            // Aditional toolpath info
            ImGui::InputDouble("Traverse height [mm]##facePass", &info.safeTraverseHeight);
            ImGui::InputDouble("Stepover [mm]##facePass", &info.stepOver);

            if (ImGui::Button("Generate##facePass")) {
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
            ImGui::TreePop();
        }
        ImGui::PopID();
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

    void draw(VulkanRenderEngine& renderer, std::string ID) override {
        ImGui::PushID(ID.c_str());
        ImGui::AlignTextToFramePadding();
        bool isOpen = ImGui::TreeNodeEx((ID + ": Surface Pass").c_str(), ImGuiTreeNodeFlags_SpanAvailWidth);
        if (isOpen) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            // First dropdown: Start Plane selection
            handleDropdown("surfacePassStartPlaneSelectGroup", "Select Start Plane:", objectNames, selectedIndex1);

            // Second dropdown: End Plane selection
            handleDropdown("surfacePassEndPlaneSelectGroup", "Select End Plane:", objectNames, selectedIndex2);

            // Third dropdown: Shape selection
            handleDropdown("surfacePassShapeSelectGroup", "Select Shape:", objectNames, selectedIndex3);

            // additional toolpath settings
            ImGui::InputDouble("Traverse height [mm]##surfacePass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##surfacePass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##surfacePass", &info.depthOfCut);

            if (ImGui::Button("Generate##surfacePass")) {
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
            ImGui::TreePop();
        }
        ImGui::PopID();
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

    void draw(VulkanRenderEngine& renderer, std::string ID) override {
        ImGui::PushID(ID.c_str());
        ImGui::AlignTextToFramePadding();
        bool isOpen = ImGui::TreeNodeEx((ID + ": Clearing Pass").c_str(), ImGuiTreeNodeFlags_SpanAvailWidth);
        if (isOpen) {
            auto& objects = renderer.loadedObjects;

            // Convert list to display vectors
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->name);

            // First dropdown: Start Plane selection
            handleDropdown("clearingPassStartPlaneSelectGroup", "Select Start Plane:", objectNames, selectedIndex1);

            // Second dropdown: End Plane selection
            handleDropdown("clearingPassEndPlaneSelectGroup", "Select End Plane:", objectNames, selectedIndex2);

            // Third dropdown: Shape selection
            handleDropdown("clearingPassShapeSelectGroup", "Select Shape:", objectNames, selectedIndex3);

            // Fourth dropdown: Stock selection
            handleDropdown("clearingPassStockSelectGroup", "Select Stock:", objectNames, selectedIndex4);

            // additional toolpath settings
            ImGui::InputDouble("Traverse height [mm]##clearingPass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##clearingPass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##clearingPass", &info.depthOfCut);
            ImGui::InputDouble("Stepover [mm]##clearingPass", &info.stepOver);

            if (ImGui::Button("Generate!##clearingPass")) {
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
            ImGui::TreePop();
        }
        ImGui::PopID();
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
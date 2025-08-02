#ifndef TOOL_PATH_2_5D_GUI_HPP
#define TOOL_PATH_2_5D_GUI_HPP

#include "toolPathGUIBase.hpp"

#include "toolPath2_5D.hpp"

// TODO properly rework this with the new renderer
namespace toolPath2_5D {
class FacePassGUI : public ToolPathGUIBase {
public:
    FacePass2_5DInfo info;
    
    FacePassGUI(RenderEngine& renderer) : ToolPathGUIBase(renderer) {
        selectedIndex1 = -1;

        planeObject = renderer.createDefaultPlane("facePassPlane", Vector3f{0.1, 0.7, 0.2});
        planeObject->isObjectShownInGui = false;
        planeObject->isObjectRendered = false;
    }

    ~FacePassGUI() override {
        planeObject->remove();
    }

    void draw(bool isOpen) override {
        // TODO rewrite 'handleDropdown' so that it can handle not being given all objects, like not showing the 'hideInGui' objects
        if (isOpen) {
            planeObject->isObjectRendered = true;

            auto& objects = renderer.getObjects();
            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());
            
            ImGui::SeparatorText("Face Plane");
            planeObject->drawGUI();
            handleDropdown("facePassStockSelectGroup", "Select Stock:", "...", objectNames, selectedIndex1);
            
            ImGui::InputDouble("Traverse height [mm]##facePass", &info.safeTraverseHeight);
            ImGui::InputDouble("Stepover [mm]##facePass", &info.stepOver);

            if (ImGui::Button("Generate##facePass")) {
                if(selectedIndex1 != -1) {
                    auto stockObj = objects.begin();
                    std::advance(stockObj, selectedIndex1);

                    info.slicingPlane = planeObject->getPlane();
                    info.stock = (*stockObj)->getObjectShape();

                    core::Polyline2_5D result = generateFacePass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        } else
            planeObject->isObjectRendered = false;
    }

    std::string name() const override {
        return "2.5D Face pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<FacePassGUI>(renderer);
    }

private:
    int selectedIndex1;

    std::shared_ptr<renderer::Object> planeObject;
};

class SurfacePassGUI : public ToolPathGUIBase {
public:
    SurfacePass2_5DInfo info;

    SurfacePassGUI(RenderEngine& renderer) : ToolPathGUIBase(renderer) {
        selectedIndex1 = -1;
        selectedIndex2 = -1;
        selectedIndex3 = -1;
    }

    void draw(bool isOpen) override {
        if (isOpen) {
            auto& objects = renderer.getObjects();

            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());

            handleDropdown("surfacePassStartPlaneSelectGroup", "Select Start Plane:", "...", objectNames, selectedIndex1);
            handleDropdown("surfacePassEndPlaneSelectGroup", "Select End Plane:", "...", objectNames, selectedIndex2);
            handleDropdown("surfacePassShapeSelectGroup", "Select Shape:", "...", objectNames, selectedIndex3);

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
                        renderer.createCurve(result);
                }
            }
        }
    }

    std::string name() const override {
        return "2.5D Surface pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<SurfacePassGUI>(renderer);
    }

private:
    int selectedIndex1;
    int selectedIndex2;
    int selectedIndex3;
};

class ClearingPassGUI : public ToolPathGUIBase {
public:
    ClearingPass2_5DInfo info;

    ClearingPassGUI(RenderEngine& renderer) : ToolPathGUIBase(renderer) {
        selectedIndex1 = -1;
        selectedIndex2 = -1;
        selectedIndex3 = -1;
        selectedIndex4 = -1;
    }

    void draw(bool isOpen) override {
        if (isOpen) {
            auto& objects = renderer.getObjects();

            std::vector<std::string> objectNames;
            for (auto& obj : objects)
                objectNames.push_back(obj->getName());

            handleDropdown("clearingPassStartPlaneSelectGroup", "Select Start Plane:", "...", objectNames, selectedIndex1);
            handleDropdown("clearingPassEndPlaneSelectGroup", "Select End Plane:", "...", objectNames, selectedIndex2);
            handleDropdown("clearingPassShapeSelectGroup", "Select Shape:", "...", objectNames, selectedIndex3);
            handleDropdown("clearingPassStockSelectGroup", "Select Stock:", "...", objectNames, selectedIndex4);

            ImGui::InputDouble("Traverse height [mm]##clearingPass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##clearingPass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##clearingPass", &info.depthOfCut);
            ImGui::InputDouble("Stepover [mm]##clearingPass", &info.stepOver);

            if (ImGui::Button("Generate##clearingPass")) {
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
                        renderer.createCurve(result);
                }
            }
        }
    }

    std::string name() const override {
        return "2.5D Clearing pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<ClearingPassGUI>(renderer);
    }

private:
    int selectedIndex1;
    int selectedIndex2;
    int selectedIndex3;
    int selectedIndex4;
};

} // namespace toolPath2_5D

#endif
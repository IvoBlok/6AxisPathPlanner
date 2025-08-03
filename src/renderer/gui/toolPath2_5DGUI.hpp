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
        planeObject = renderer.createDefaultPlane("facePassPlane", Vector3f{0.1, 0.7, 0.2}, Vector3d::Zero(), Vector3d::Ones(), Vector3d::Zero(), 0.6f, false, false);
    }

    ~FacePassGUI() override {
        planeObject->remove();
    }

    void draw(bool isOpen) override {
        if (isOpen) {
            if (stockObject)
                planeObject->isObjectRendered = true;

            auto& objects = renderer.getObjects();
            
            handleDropdown("facePassStockSelectGroup", "Select Stock:", objects, stockObject, [](const auto& obj) { return obj->isObjectShownInGui; });
            
            ImGui::AlignTextToFramePadding();
            if (ImGui::TreeNodeEx("Face Plane", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                if (stockObject)
                    planeObject->drawGUI("planeObject", false, false);
                ImGui::TreePop();
            }

            ImGui::InputDouble("Traverse height [mm]##facePass", &info.safeTraverseHeight);
            ImGui::InputDouble("Stepover [mm]##facePass", &info.stepOver);

            if (ImGui::Button("Generate##facePass")) {
                if(stockObject) {

                    info.slicingPlane = planeObject->getPlane();
                    info.stock = stockObject->getObjectShape();

                    core::Polyline2_5D result = generateFacePass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        } else
            planeObject->isObjectRendered = false;
    }

    std::string getName() const override {
        return "2.5D Face pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<FacePassGUI>(renderer);
    }

private:
    std::shared_ptr<renderer::Object> planeObject;
    std::shared_ptr<renderer::Object> stockObject;
};

class SurfacePassGUI : public ToolPathGUIBase {
public:
    SurfacePass2_5DInfo info;

    SurfacePassGUI(RenderEngine& renderer) : ToolPathGUIBase(renderer) {
        startPlane = renderer.createDefaultPlane("surfacePassStartPlane", Vector3f{0.1, 0.7, 0.2}, Vector3d::Zero(), Vector3d::Ones(), Vector3d::Zero(), 0.5f, false, false);
        endPlane = renderer.createDefaultPlane("surfacePassEndPlane", Vector3f{0.7, 0.7, 0.1}, Vector3d::Zero(), Vector3d::Ones(), Vector3d::Zero(), 0.5f, false, false);
    }

    ~SurfacePassGUI() override {
        startPlane->remove();
        endPlane->remove();
    }

    void draw(bool isOpen) override {
        if (isOpen) {
            if (shapeObject) {
                startPlane->isObjectRendered = true;
                endPlane->isObjectRendered = true;
            }

            auto& objects = renderer.getObjects();

            handleDropdown("surfacePassShapeSelectGroup", "Select Shape:", objects, shapeObject, [](const auto& obj) {return obj->isObjectShownInGui; });

            ImGui::AlignTextToFramePadding();
            if (ImGui::TreeNodeEx("Start Plane", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                if (shapeObject)
                    startPlane->drawGUI("startPlane", false, false);
                ImGui::TreePop();
            }
            ImGui::AlignTextToFramePadding();
            if (ImGui::TreeNodeEx("End Plane", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                if (shapeObject)
                    endPlane->drawGUI("endPlane", false, false);
                ImGui::TreePop();
            }
    
            ImGui::InputDouble("Traverse height [mm]##surfacePass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##surfacePass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##surfacePass", &info.depthOfCut);

            if (ImGui::Button("Generate##surfacePass")) {
                if(startPlane && endPlane && shapeObject) {

                    info.startPlane = startPlane->getPlane();
                    info.endPlane = endPlane->getPlane();
                    info.shape = shapeObject->getObjectShape();

                    core::Polyline2_5D result = generateSurfacePass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        } else {
            startPlane->isObjectRendered = false;
            endPlane->isObjectRendered = false;
        }
    }

    std::string getName() const override {
        return "2.5D Surface pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<SurfacePassGUI>(renderer);
    }

private:
    std::shared_ptr<renderer::Object> startPlane;
    std::shared_ptr<renderer::Object> endPlane;
    std::shared_ptr<renderer::Object> shapeObject;
};

class ClearingPassGUI : public ToolPathGUIBase {
public:
    ClearingPass2_5DInfo info;

    ClearingPassGUI(RenderEngine& renderer) : ToolPathGUIBase(renderer) {
        startPlane = renderer.createDefaultPlane("clearingPassStartPlane", Vector3f{0.1, 0.7, 0.2}, Vector3d::Zero(), Vector3d::Ones(), Vector3d::Zero(), 0.5f, false, false);
        endPlane = renderer.createDefaultPlane("clearingPassEndPlane", Vector3f{0.7, 0.7, 0.1}, Vector3d::Zero(), Vector3d::Ones(), Vector3d::Zero(), 0.5f, false, false);
    }

    void draw(bool isOpen) override {
        if (isOpen) {
            if (shapeObject && stockObject) {
                startPlane->isObjectRendered = true;
                endPlane->isObjectRendered = true;
            }

            auto& objects = renderer.getObjects();

            handleDropdown("clearingPassShapeSelectGroup", "Select Shape:", objects, shapeObject, [](const auto& obj) {return obj->isObjectShownInGui; });
            handleDropdown("clearingPassStockSelectGroup", "Select Stock:", objects, stockObject, [](const auto& obj) {return obj->isObjectShownInGui; });
            
            ImGui::AlignTextToFramePadding();
            if (ImGui::TreeNodeEx("Start Plane", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                if (shapeObject && stockObject)
                    startPlane->drawGUI("startPlane", false, false);
                ImGui::TreePop();
            }
            ImGui::AlignTextToFramePadding();
            if (ImGui::TreeNodeEx("End Plane", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                if (shapeObject && stockObject)
                    endPlane->drawGUI("endPlane", false, false);
                ImGui::TreePop();
            }

            ImGui::InputDouble("Traverse height [mm]##clearingPass", &info.safeTraverseHeight);
            ImGui::InputDouble("Tool radius [mm]##clearingPass", &info.toolRadius);
            ImGui::InputDouble("Depth of Cut [mm]##clearingPass", &info.depthOfCut);
            ImGui::InputDouble("Stepover [mm]##clearingPass", &info.stepOver);

            if (ImGui::Button("Generate##clearingPass")) {
                if(startPlane && endPlane && shapeObject && stockObject) {
                    info.startPlane = startPlane->getPlane();
                    info.endPlane = endPlane->getPlane();
                    info.shape = shapeObject->getObjectShape();
                    info.stock = stockObject->getObjectShape();

                    core::Polyline2_5D result = generateClearingPass2_5D(info);
                    
                    if (!result.isEmpty())
                        renderer.createCurve(result);
                }
            }
        } else {
            startPlane->isObjectRendered = false;
            endPlane->isObjectRendered = false;
        }
    }

    std::string getName() const override {
        return "2.5D Clearing pass";
    }

    std::unique_ptr<ToolPathGUIBase> clone() const override {
        return std::make_unique<ClearingPassGUI>(renderer);
    }

private:
    std::shared_ptr<renderer::Object> startPlane;
    std::shared_ptr<renderer::Object> endPlane;
    std::shared_ptr<renderer::Object> shapeObject;
    std::shared_ptr<renderer::Object> stockObject;
};

} // namespace toolPath2_5D

#endif
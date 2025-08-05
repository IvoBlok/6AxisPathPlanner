#include "RendererCurve.hpp"

namespace renderer {
    Curve::Curve(RenderEngine& renderer) : Curve(renderer, "curve") { }    

    Curve::Curve(RenderEngine& renderer, std::string name, bool isCurveRendered, bool isCurveShownInGui) 
        : alive(true), renderer(renderer), curveBuffer(renderer), name(name), isCurveRendered(isCurveRendered), isCurveShownInGui(isCurveShownInGui) 
    {
        lineWidth = 3.f;

        defaultColor = Vector3f::UnitX();
        useDefaultColor = true;
    }

    Curve::~Curve() {
        cleanup();
    }

    void Curve::remove() {
        renderer.removeCurve(this);
    }

    bool Curve::isAlive() const {
        return alive;
    }

    void Curve::setName(std::string nameIn) {
        name = nameIn;
    }

    std::string Curve::getName() const {
        return name;
    }

    float Curve::getTransparency() const {
        return curveBuffer.transparency;
    }
    
    int Curve::getNumberOfVertices() const {
        if(!alive) return 0;
        return curveBuffer.vertices.size();
    }

    const core::Polyline2_5D& Curve::getPolyline() const {
        return polyline;
    }

    void Curve::drawGUI(std::string ID) {
        ImGui::PushID(ID.c_str());

        float colorArray[3] = { defaultColor.x(), defaultColor.y(), defaultColor.z() };
        ImGui::Text("Color ");
        ImGui::SameLine();
        ImGui::Checkbox("##OneColorToggle", &useDefaultColor);
        if (useDefaultColor) {
            ImGui::SameLine();
            ImVec4 colVec4 = ImVec4(colorArray[0], colorArray[1], colorArray[2], 1.0f);
            if (ImGui::ColorButton("MyColor##3", colVec4, ImGuiColorEditFlags_NoTooltip)) {
                ImGui::OpenPopup("colorPicker");
            }
            if (ImGui::BeginPopup("colorPicker")) {
                if (ImGui::ColorPicker3("##picker", colorArray, 
                    ImGuiColorEditFlags_DisplayRGB | 
                    ImGuiColorEditFlags_NoSidePreview |
                    ImGuiColorEditFlags_NoSmallPreview)) 
                {
                    defaultColor.x() = colorArray[0];
                    defaultColor.y() = colorArray[1];
                    defaultColor.z() = colorArray[2];
                }
                ImGui::EndPopup();
            }
        }
        
        ImGui::SliderFloat("Transparency", &curveBuffer.transparency, 0.0f, 1.0f);
        ImGui::SliderFloat("Line Width", &lineWidth, 0.0f, 10.0f);
        
        ImGui::PopID();
    }

    void Curve::load(core::Polyline2_5D& polylineIn, Vector3f defaultColorIn, float transparency) {
        polyline = core::Polyline2_5D(polylineIn);
        
        curveBuffer.load(polylineIn);
        curveBuffer.transparency = transparency;   
        defaultColor = defaultColorIn;
    }

    void Curve::render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
        vkCmdSetLineWidth(commandBuffer, lineWidth);

        CurveShadersPushConstant pushConstant{};
        pushConstant.color = glm::vec3{ defaultColor.x(), defaultColor.y(), defaultColor.z() };;
        pushConstant.isOneColor = useDefaultColor;
        pushConstant.transparency = curveBuffer.transparency;

        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(CurveShadersPushConstant), &pushConstant);

        curveBuffer.render(commandBuffer);
    }

    void Curve::cleanup() {
        if (renderer.getContext().device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(renderer.getContext().device);
            curveBuffer.destroy();
        }
    }
}
#include "RendererObject.hpp"

#include "imconfig.h"
#include "imgui_internal.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan_but_better.hpp"	
#include "implot.h"

namespace renderer {
    void Object::remove() {
        renderer.deleteObject(this);
    }

    void Object::setPose(Matrix4d matrix) {
        // given a 'standard' transformation matrix, extract the position, scale and rotation elements and set the respective object variables
        position = matrix.block<3, 1>(0, 3);
        
        Matrix3d rotationScaleMatrix = matrix.topLeftCorner<3, 3>();
        for (int i = 0; i < 3; i++) {
            scale(i) = rotationScaleMatrix.col(i).norm();
            rotationScaleMatrix.col(i).normalize();
        }
        rotation = rotationScaleMatrix;
    }

    Object::Object(RenderEngine& renderer) : Object(renderer, "obj") { }

    Object::Object(RenderEngine& renderer, std::string name, bool isObjectRendered, bool isObjectShownInGui) 
        : model(renderer), renderer(renderer), name(name), isObjectRendered(isObjectRendered), isObjectShownInGui(isObjectShownInGui) 
    {
        texture.first.emplace(renderer);
        texture.second = false;

        position = Vector3d::Zero();
        scale = Vector3d::Zero();
        rotation = static_cast<Vector3d>(Vector3d::Zero());
        
        color = Vector3f::Zero();
        useTexture = false;
    }

    void Object::load(const char* modelPath, const char* texturePath, Vector3d basePosition, Vector3d baseScale, Vector3d baseRotation, float modelTransparency) {
        model.load(modelPath, modelTransparency);
        texture.first.value().load(texturePath);
        texture.second = true;

        useTexture = true;
        position = basePosition;
        scale = baseScale;
        rotation = baseRotation;
    }   

    void Object::load(const char* modelPath, Vector3f objectColor, Vector3d basePosition, Vector3d baseScale, Vector3d baseRotation, float modelTransparency) {
        model.load(modelPath, modelTransparency);
        texture.first.value().loadDummy();
        texture.second = false;

        color = objectColor;
        useTexture = false;
        position = basePosition;
        scale = baseScale;
        rotation = baseRotation;
    }
    
    glm::mat4 Object::getPoseGLM() {
        glm::mat4 transformationMatrix = glm::mat4{1.f};
        transformationMatrix = glm::translate(transformationMatrix, glm::vec3{position(0), position(1), position(2)});  // Translate first
        transformationMatrix = transformationMatrix * rotation.glmMatrix();                                             // Then rotate
        transformationMatrix = glm::scale(transformationMatrix, glm::vec3{scale(0), scale(1), scale(2)});               // Scale last
        return transformationMatrix;
    }

    void Object::drawGUI() {
        float positionArray[3] = { (float)position.x(), (float)position.y(), (float)position.z() };
        if (ImGui::InputFloat3("Position", positionArray)) {
            position.x() = positionArray[0];
            position.y() = positionArray[1];
            position.z() = positionArray[2];
        }

        float scaleArray[3] = { (float)scale.x(), (float)scale.y(), (float)scale.z() };
        if (ImGui::InputFloat3("Scale", scaleArray)) {
            scale.x() = scaleArray[0];
            scale.y() = scaleArray[1];
            scale.z() = scaleArray[2];
        }

        Vector3d eulerAngles = rotation.angles();
        float rotationArray[3] = { (float)eulerAngles.x(), (float)eulerAngles.y(), (float)eulerAngles.z() };
        if (ImGui::InputFloat3("Rotation", rotationArray)) {
            eulerAngles(0) = rotationArray[0];
            eulerAngles(1) = rotationArray[1];
            eulerAngles(2) = rotationArray[2];
            rotation = eulerAngles;
        }
        
        float colorArray[3] = { color.x(), color.y(), color.z() };
        ImGui::Text("Color ");
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
                color.x() = colorArray[0];
                color.y() = colorArray[1];
                color.z() = colorArray[2];
            }
            ImGui::EndPopup();
        }
        
        ImGui::SliderFloat("Transparency", &model.transparency, 0.0f, 1.0f);
    }

    void Object::render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
        glm::mat4 transformationMatrix = getPoseGLM();

        ObjectShaderPushConstant pushConstant{};
        pushConstant.modelMatrix = transformationMatrix;
        pushConstant.transparency = model.transparency;
        pushConstant.color = glm::vec3{color(0), color(1), color(2)};
        bool canAndShouldUseTexture = (texture.second && useTexture);
        pushConstant.isOneColor = !canAndShouldUseTexture;

        // bind the right texture
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &texture.first.value().descriptorSet, 0, nullptr);

        // bind the model matrix, and render the object
        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(ObjectShaderPushConstant), &pushConstant);
        model.render(commandBuffer);
     }

    void Object::cleanup() {
        model.destroy();
        texture.first.value().destroy();
    }

}
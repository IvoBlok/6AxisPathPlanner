#include "RendererObject.hpp"

#include <glm/gtx/string_cast.hpp>

namespace renderer {

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

    Object::~Object() {
        cleanup();
    }

    void Object::remove() {
        renderer.removeObject(this);
    }

    bool Object::isAlive() const {
        return alive;
    }

    void Object::setPose(Matrix4d matrix) {
        if(!alive) return;

        // given a 'standard' transformation matrix, extract the position, scale and rotation elements and set the respective object variables
        position = matrix.block<3, 1>(0, 3);
        
        Matrix3d rotationScaleMatrix = matrix.topLeftCorner<3, 3>();
        for (int i = 0; i < 3; i++) {
            scale(i) = rotationScaleMatrix.col(i).norm();
            rotationScaleMatrix.col(i).normalize();
        }
        rotation = rotationScaleMatrix;
    }

    void Object::setName(std::string nameIn) {
        name = nameIn;
    }

    std::string Object::getName() const {
        return name;
    }

    float Object::getTransparency() const {
        return model.transparency;
    }

    int Object::getNumberOfVertices() const {
        if(!alive) return 0;
        return model.vertices.size();
    }

    int Object::getNumberOfIndices() const {
        if(!alive) return 0;
        return model.indices.size();
    }

    core::ObjectShape Object::getObjectShape() {
        if(!alive) return core::ObjectShape{};

        glm::mat4 transformationMatrix = getPoseGLM();
        //std::cout << glm::to_string(transformationMatrix);
        core::ObjectShape objectShape{};
        
        objectShape.vertices.reserve(model.vertices.size());
        objectShape.indices.reserve(model.indices.size());
        objectShape.normals.reserve(model.vertices.size());

        for (size_t i = 0; i < model.vertices.size(); i++)
        {
            glm::vec3 vertexWithAppliedMatrix = transformationMatrix * glm::vec4{ model.vertices[i].pos, 1.f };
            glm::vec3 normalWithAppliedMatrix = transformationMatrix * glm::vec4{ model.vertices[i].normal, 0.f };
            objectShape.vertices.push_back(Vector3d{ vertexWithAppliedMatrix.x, vertexWithAppliedMatrix.y, vertexWithAppliedMatrix.z });
            objectShape.normals.push_back(Vector3d{ normalWithAppliedMatrix.x, normalWithAppliedMatrix.y, normalWithAppliedMatrix.z });
        }

        for (size_t i = 0; i < model.indices.size(); i++)
        {
            objectShape.indices.push_back(model.indices[i]);
        }
        return objectShape;
    }

    core::Plane Object::getPlane() {
        if(!alive) return core::Plane{};

        Vector3d normal = Vector3d::UnitZ();
        normal = rotation.matrix3d() * normal;

        core::Plane objectPlane{position, normal};
        return objectPlane;
    }

    void Object::drawGUI(std::string ID, bool showColor, bool showTransparency) {
        ImGui::PushID(ID.c_str());
        
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
        
        if (showColor) {
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
        }
        
        if (showTransparency)
            ImGui::SliderFloat("Transparency", &model.transparency, 0.0f, 1.0f);

        ImGui::PopID();
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

    void Object::render(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
        glm::mat4 transformationMatrix = getPoseGLM();

        ObjectShaderPushConstant pushConstant{};
        pushConstant.modelMatrix = transformationMatrix;
        pushConstant.transparency = model.transparency;
        pushConstant.color = glm::vec3{color(0), color(1), color(2)};
        bool canAndShouldUseTexture = (texture.second && useTexture);
        pushConstant.isOneColor = !canAndShouldUseTexture;

        // bind the right texture
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &texture.first.value().getDescriptorSet(), 0, nullptr);

        // bind the model matrix, and render the object
        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(ObjectShaderPushConstant), &pushConstant);
        model.render(commandBuffer);
    }

    void Object::cleanup() {
        if (renderer.getContext().device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(renderer.getContext().device);
            model.destroy();
            texture.first.value().destroy();
        }
    }
}
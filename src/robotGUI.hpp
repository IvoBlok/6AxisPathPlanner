#ifndef ROBOT_GUI_HPP
#define ROBOT_GUI_HPP

#include "renderer.hpp"
#include "robotKinematics.hpp"
#include "core/mathUtils.hpp"
#include <iostream>

namespace kinematics {
class RobotGUI {
public:
    RobotKinematics robotKinematics;

    LoadedObject* joint = nullptr;
    LoadedObject* base = nullptr;
    LoadedObject* effector = nullptr;
    
    std::vector<double> jointStates;
    std::vector<float> jointStatesFloat;

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {
            if (ImGui::Button("Set robot Joints")) {
                robotKinematics = RobotKinematics{};

                // Linear joint with its axes aligned with the robot main axes, and its joint zero point at the robot main zero point
                
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);

                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 1.0, 0.0},
                                                                   {1.0, 0.0, 0.0, 0.5},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Linear, 
                                                    -1.0, 
                                                    1.0});

                // Robot axes are aligned with the world axes, and robot zero is located at the world zero
                robotKinematics.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                {0.0, 1.0, 0.0, 0.0},
                                                                {0.0, 0.0, 1.0, 0.0},
                                                                {0.0, 0.0, 0.0, 1.0}};

                robotKinematics.endEffector.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 1.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 1.0, 0.2},
                                                                            {0.0, 0.0, 0.0, 1.0}};


                // create an object for each joint, the robot base, and the effector
                base = &renderer.createObject(
                        "../../resources/assets/cube.obj",
                        Vector3d{ 0.f, 0.f, 1.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ .1f, .1f, .1f }  // scale
                    );
                base->name = "base";

                effector = &renderer.createObject(
                        "../../resources/assets/cube.obj",
                        Vector3d{ 0.f, 1.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ .1f, .1f, .1f }  // scale
                    );
                effector->name = "effector";

                joint = &renderer.createObject(
                        "../../resources/assets/cube.obj",
                        Vector3d{ 1.f, 0.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ .1f, .1f, .1f }  // scale
                    );
                joint->name = "joint";
            }

            if (joint != nullptr && base != nullptr && effector != nullptr) {
                
                int index = 0;
                for(auto& const joint : robotKinematics.joints) {
                    char label[32];
                    snprintf(label, sizeof(label), "Joint %zu", index);
                    ImGui::SliderFloat(label, &jointStatesFloat[index], joint.negativeLimit, joint.positiveLimit);

                    jointStates[index] = (double)jointStatesFloat[index];
                    ++index;
                }


                // perform the forward kinematics
                std::vector<Matrix4d> robotMatrices = robotKinematics.forwardKinematics(jointStates);


                // update the renderer objects
                base->position = glm::vec3{ robotMatrices[0](0, 3), robotMatrices[0](1, 3), robotMatrices[0](2, 3) };
                joint->position = glm::vec3{ robotMatrices[1](0, 3), robotMatrices[1](1, 3), robotMatrices[1](2, 3) };
                effector->position = glm::vec3{ robotMatrices[2](0, 3), robotMatrices[2](1, 3), robotMatrices[2](2, 3) };
                
                // very temp way of 
                Vector3d baseYPR = (180.0 / core::PI) * matrixToEulerAngles(robotMatrices[0].topLeftCorner<3, 3>());
                Vector3d jointYPR = (180.0 / core::PI) * matrixToEulerAngles(robotMatrices[0].topLeftCorner<3, 3>());
                Vector3d effectorYPR = (180.0 / core::PI) * matrixToEulerAngles(robotMatrices[0].topLeftCorner<3, 3>());

                base->yawPitchRoll = glm::vec3{ baseYPR.x(), baseYPR.y(), baseYPR.z() };
                joint->yawPitchRoll = glm::vec3{ jointYPR.x(), jointYPR.y(), jointYPR.z() };
                effector->yawPitchRoll = glm::vec3{ effectorYPR.x(), effectorYPR.y(), effectorYPR.z() };
            }
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }
};

}

#endif // ROBOT_GUI_HPP
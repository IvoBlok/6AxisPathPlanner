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

    std::vector<LoadedObject*> joints;
    LoadedObject* base = nullptr;
    LoadedObject* effector = nullptr;
    
    std::vector<double> jointStates;
    std::vector<float> jointStatesFloat;

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {
            if (ImGui::Button("Set robot Joints")) {
                robotKinematics = RobotKinematics{};

                // Linear joint with its axes aligned with the robot main axes, and its joint zero point at the robot main zero point

                // Robot axes are aligned with the world axes, and robot zero is located at the world zero
                robotKinematics.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                {0.0, 1.0, 0.0, 0.0},
                                                                {0.0, 0.0, 1.0, 0.0},
                                                                {0.0, 0.0, 0.0, 1.0}};
                
                // Axis 1 in terms of the World
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                                   {1.0, 0.0, 0.0, 0.0},
                                                                   {0.0, 1.0, 0.0, 0.1},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Linear, 
                                                    -1.3, 
                                                    1.3});

                // Axis 2 in terms of Axis 1
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 1.0, 0.1},
                                                                   {1.0, 0.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -3.14, 
                                                    3.14});

                // Axis 3 in terms of Axis 2
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 1.0, 0.0},
                                                                   {1.0, 0.0, 0.0, 0.1},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -0.5, 
                                                    2.0});


                // Axis 4 in terms of Axis 3
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{1.0, 0.0, 0.0, 0.4},
                                                                   {0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 1.0, 0.0},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -1.5, 
                                                    2.0});

                // Axis 5 in terms of Axis 4
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.2},
                                                                   {1.0, 0.0, 0.0, 0.0},
                                                                   {0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -3.14, 
                                                    3.14});

                // Axis 6 in terms of Axis 5
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 1.0, 0.0},
                                                                   {1.0, 0.0, 0.0, 0.2},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -1.7, 
                                                    1.7});

                // Axis 7 in terms of Axis 6
                jointStates.emplace_back(0.0);
                jointStatesFloat.emplace_back(0.f);
                
                robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.1},
                                                                   {1.0, 0.0, 0.0, 0.0},
                                                                   {0.0, 1.0, 0.0, 0.0},
                                                                   {0.0, 0.0, 0.0, 1.0}}, 
                                                    JointType::Rotation, 
                                                    -3.14, 
                                                    3.14});

                robotKinematics.endEffector.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 1.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 1.0, 0.2},
                                                                            {0.0, 0.0, 0.0, 1.0}};


                // create an object for each joint, the robot base, and the effector
                base = &renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 0.f, 0.f, 1.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    );
                base->name = "base";

                effector = &renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 0.f, 1.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    );
                effector->name = "effector";

                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 1.f, 0.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 0";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 1.f, 1.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 1";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 0.f, 1.f, 1.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 2";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 1.f, 0.f, 1.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 3";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 0.f, 1.f, 0.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 4";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ 0.f, 0.f, 1.f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 5";
                
                joints.emplace_back(&renderer.createObject(
                        "../../resources/assets/Gizmo.obj",
                        Vector3d{ .5f, .5f, .5f }, // color
                        Vector3d{ 0.f, 0.f, 0.f }, // position
                        Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
                    ));
                joints.back()->name = "joint 6";                

            }

            if (base != nullptr && effector != nullptr) {
                
                int index = 0;
                for(auto& joint : robotKinematics.joints) {
                    char label[32];
                    snprintf(label, sizeof(label), "Joint %d", index + 1);
                    ImGui::SliderFloat(label, &jointStatesFloat[index], joint.negativeLimit, joint.positiveLimit);

                    jointStates[index] = (double)jointStatesFloat[index];
                    ++index;
                }

                // perform the forward kinematics
                std::vector<Matrix4d> robotMatrices = robotKinematics.forwardKinematics(jointStates);

                // update the renderer objects
                base->locateWithMatrix(robotMatrices.front());
                effector->locateWithMatrix(robotMatrices.back());
                for (int i = 1; i < robotMatrices.size() - 1; i++) {
                    joints[i - 1]->locateWithMatrix(robotMatrices[i]);
                }

                for (int i = 0; i < robotMatrices.size(); i++) {
                    ShowMatrix4dGrid(robotMatrices[i]);
                }
            }
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }

private:
    void ShowMatrix4dGrid(const Eigen::Matrix4d& matrix) {
        if (ImGui::BeginTable("Matrix4d", 4, ImGuiTableFlags_Borders)) {
            for (int row = 0; row < 4; ++row) {
                ImGui::TableNextRow();
                for (int col = 0; col < 4; ++col) {
                    ImGui::TableSetColumnIndex(col);
                    ImGui::Text("%.3f", matrix(row, col));
                }
            }
            ImGui::EndTable();
        }
    }
};

}

#endif // ROBOT_GUI_HPP
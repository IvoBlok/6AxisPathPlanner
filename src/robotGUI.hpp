#ifndef ROBOT_GUI_HPP
#define ROBOT_GUI_HPP

#include "renderer.hpp"
#include "robotKinematics.hpp"
#include "core/mathUtils.hpp"
#include <iostream>

using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::Vector3f;

namespace kinematics {
class RobotGUI {
public:
    RobotKinematics robotKinematics;

    std::vector<LoadedObject*> joints;
    LoadedObject* base;
    LoadedObject* effector;
    
    VectorXd jointStates;
    VectorXf jointStatesFloat;

    Vector3f endPoint;

    RobotGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);

        robotDefined = false;
        base = nullptr;
        effector = nullptr;

        endPoint = Vector3f::Zero();
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {

            if (!robotDefined) {
                defineRobot(renderer);
                robotDefined = true;
            }

            int index = 0;
            for(auto& joint : robotKinematics.joints) {
                char label[32];
                snprintf(label, sizeof(label), "Joint %d", index + 1);
            
                // because ImGui doesn't have slider for doubles, we need to do this weird double copy to ensure that jointStates and jointStatesFloat remain the same, independent of which is changed
                jointStatesFloat(index) = jointStates(index);
                ImGui::SliderFloat(label, &jointStatesFloat(index), joint.lowerLimit, joint.upperLimit);
                jointStates(index) = (double)jointStatesFloat(index);

                ++index;
            }

            ImGui::SeparatorText("Desired endpoint");
            ImGui::SliderFloat("x", &endPoint(0), -1.5, 1.5);
            ImGui::SliderFloat("y", &endPoint(1), -1.5, 1.5);
            ImGui::SliderFloat("z", &endPoint(2), -1.5, 1.5);

            // calculate the joint state based on the given desired effector orientation
            if (ImGui::Button("do SQP IK")) {
                Matrix4d desiredOrientation = Matrix4d::Identity();
                desiredOrientation.topRightCorner<3,1>() = endPoint.cast<double>();

                jointStates = robotKinematics.inverseKinematics(desiredOrientation);
            }

            // perform the forward kinematics
            std::vector<Matrix4d> robotMatrices = robotKinematics.forwardKinematics(jointStates);

            // show the matrices and joint controls
            for (int i = 0; i < robotMatrices.size(); i++) {
                if (i == 0)
                    ImGui::SeparatorText("Base");
                else if (i == robotMatrices.size() - 1)
                    ImGui::SeparatorText("End Effector");
                else {
                    char label[32];
                    snprintf(label, sizeof(label), "Joint %d", i);
                    ImGui::SeparatorText(label);
                }
                ShowMatrix4dGrid(robotMatrices[i]);
            }

            // update the renderer objects
            base->locateWithMatrix(robotMatrices.front());
            effector->locateWithMatrix(robotMatrices.back());
            for (int i = 1; i < robotMatrices.size() - 1; i++) {
                joints[i - 1]->locateWithMatrix(robotMatrices[i]);
            }
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }

private:
    bool robotDefined;

    void defineRobot(VulkanRenderEngine& renderer) {
        robotKinematics = RobotKinematics{};

        // Linear joint with its axes aligned with the robot main axes, and its joint zero point at the robot main zero point
        // ------------- Simple 3-axis robot arm ------------------------------------
        // This defines a basic 3-axis (3 linear joints) cartesian robot, to test te SQP IK
        robotKinematics.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                        {0.0, 1.0, 0.0, 0.0},
                                                        {0.0, 0.0, 1.0, 0.0},
                                                        {0.0, 0.0, 0.0, 1.0}};      

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                           {1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Linear, 
                                            -1.0, 
                                            1.0});

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                           {1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Linear, 
                                            -1.0, 
                                            1.0});

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                           {1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Linear, 
                                            -1.0, 
                                            1.0});

        robotKinematics.endEffector.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                    {0.0, 1.0, 0.0, 0.0},
                                                                    {0.0, 0.0, 1.0, 0.0},
                                                                    {0.0, 0.0, 0.0, 1.0}};

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
        joints.back()->name = "joint 1";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 2";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 3";

        /*
        // ------------- Big fancy 7-axis robot arm ------------------------------------
        // Robot axes are aligned with the world axes, and robot zero is located at the world zero
        robotKinematics.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                        {0.0, 1.0, 0.0, 0.0},
                                                        {0.0, 0.0, 1.0, 0.0},
                                                        {0.0, 0.0, 0.0, 1.0}};

        // Axis 1 in terms of the robot base
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                            {1.0, 0.0, 0.0, 0.0},
                                                            {0.0, 1.0, 0.0, 0.1},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Linear, 
                                            -1.3, 
                                            1.3});

        // Axis 2 in terms of Axis 1
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                            {0.0, 0.0, 1.0, 0.1},
                                                            {1.0, 0.0, 0.0, 0.0},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -3.14, 
                                            3.14});

        // Axis 3 in terms of Axis 2
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                            {0.0, 0.0, 1.0, 0.0},
                                                            {1.0, 0.0, 0.0, 0.1},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -0.5, 
                                            2.0});


        // Axis 4 in terms of Axis 3
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{1.0, 0.0, 0.0, 0.4},
                                                            {0.0, 1.0, 0.0, 0.0},
                                                            {0.0, 0.0, 1.0, 0.0},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -1.5, 
                                            2.0});

        // Axis 5 in terms of Axis 4
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.2},
                                                            {1.0, 0.0, 0.0, 0.0},
                                                            {0.0, 1.0, 0.0, 0.0},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -3.14, 
                                            3.14});

        // Axis 6 in terms of Axis 5
        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 1.0, 0.0, 0.0},
                                                            {0.0, 0.0, 1.0, 0.0},
                                                            {1.0, 0.0, 0.0, 0.2},
                                                            {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -1.7, 
                                            1.7});

        // Axis 7 in terms of Axis 6
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
        joints.back()->name = "joint 1";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 2";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 3";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 4";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 5";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 6";
        
        joints.emplace_back(&renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ .5f, .5f, .5f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 7";    
        */

        jointStates = VectorXd::Zero(robotKinematics.joints.size());
        jointStatesFloat = VectorXf::Zero(robotKinematics.joints.size());
    }

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
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

    std::vector<shared_ptr<LoadedObject>> joints;
    shared_ptr<LoadedObject> base;
    shared_ptr<LoadedObject> effector;
    
    VectorXd jointStates;

    Vector3f endPoint;
    Vector3f endRotation;
    shared_ptr<LoadedObject> goalObject;

    RobotGUI(VulkanRenderEngine& renderer) {
        registerWithRenderer(renderer);

        continuousIK = false;
        base = nullptr;
        effector = nullptr;
        goalObject = nullptr;

        endPoint = Vector3f::Zero();
        endRotation = Vector3f::Zero();
    }

    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {

            // robot selection dropdown
            const char* items[] = { "3D Rotation", "3D Translation", "7D Kuka Inspired" };
            static int selectedItem = -1;
            ImGui::PushID("robot_GUI_select_robot_type");
            ImGui::Text("Select Robot Type:");
            const char* preview = (selectedItem > -1) ? items[selectedItem] : "-";
            if (ImGui::BeginCombo("##robot_select_combo", preview)) {
                for (int i = 0; i < 3; ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedItem == i);
                    if (ImGui::Selectable(items[i], isSelected)) {
                        selectedItem = i;

                        if (selectedItem >= 0)
                            defineFullRobot(renderer, selectedItem);
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            if (joints.size() <= 0)
                return;

            // output the joint angles within their limits
            int index = 0;
            for(auto& joint : robotKinematics.joints) {
                char label[32];
                snprintf(label, sizeof(label), "Joint %d", index + 1);
                
                float temp = static_cast<float>(jointStates(index)); 
                ImGui::SliderFloat(label, &temp, joint.lowerLimit, joint.upperLimit, "%.3f", ImGuiSliderFlags_NoInput);
                ++index;
            }

            // input for the goal
            ImGui::SeparatorText("Desired endpoint");
            ImGui::SliderFloat("x", &endPoint(0), -1.5, 1.5);
            ImGui::SliderFloat("y", &endPoint(1), -1.5, 1.5);
            ImGui::SliderFloat("z", &endPoint(2), -1.5, 1.5);
            ImGui::SliderFloat("yaw", &endRotation(0), -3.14, 3.14);
            ImGui::SliderFloat("pitch", &endRotation(1), -3.14, 3.14);
            ImGui::SliderFloat("roll", &endRotation(2), -3.14, 3.14);
            ImGui::Checkbox("continuous IK##RenderToggle", &continuousIK);
            
            // input IK settings
            static bool useRotation = false;
            static float rotationIgnoreDirection[3] = {0.f, 0.f, 0.f};
            static int maxIterations = 30;
            static int maxAttempts = 3;
            static double tolerance = 1e-4;
            static bool warmStart = true;

            ImGui::SeparatorText("Inverse Kinematics Settings");
            ImGui::Checkbox("rotation", &useRotation);
            ImGui::InputFloat3("ignore direction", rotationIgnoreDirection);
            ImGui::InputInt("max iterations", &maxIterations);
            ImGui::InputInt("max attempts", &maxAttempts);
            ImGui::InputDouble("tolerance", &tolerance);
            ImGui::Checkbox("warm start", &warmStart);
        
            // debug IK calling
            Eigen::Affine3d transform = 
            Eigen::Translation3d(endPoint.cast<double>()) *
            Eigen::AngleAxisd(endRotation.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(endRotation.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(endRotation.x(), Eigen::Vector3d::UnitX());

            Eigen::Matrix4d desiredOrientation = transform.matrix();

            if (continuousIK || ImGui::Button("do SQP IK")) {
                IKResult result = robotKinematics.inverseKinematics(desiredOrientation, useRotation, Vector3d((double)rotationIgnoreDirection[0], (double)rotationIgnoreDirection[1], (double)rotationIgnoreDirection[2]), maxIterations, maxAttempts, tolerance, warmStart);

                if (result.type == IKResultType::Success) {
                    jointStates = result.state;
                    std::vector<Matrix4d> robotMatrices = robotKinematics.forwardKinematics(jointStates);

                    // update the renderer objects
                    base->locateWithMatrix(robotMatrices.front());
                    effector->locateWithMatrix(robotMatrices.back());
                    for (int i = 1; i < robotMatrices.size() - 1; i++) {
                        joints[i - 1]->locateWithMatrix(robotMatrices[i]);
                    }
                }
            }

            goalObject->locateWithMatrix(desiredOrientation);
        }
    }

    void registerWithRenderer(VulkanRenderEngine& renderer) {
        renderer.registerGuiModule([this](VulkanRenderEngine& r) {
            this->drawGUI(r); 
        });
    }

private:
    bool robotDefined;
    bool continuousIK;

    void defineFullRobot(VulkanRenderEngine& renderer, int robotOption) {
        renderer.deleteObject(goalObject);
        renderer.deleteObject(base);
        renderer.deleteObject(effector);
        for (auto& joint : joints)
            renderer.deleteObject(joint);
        joints.clear();

        robotKinematics = RobotKinematics{};
        goalObject = renderer.createObject(
                "../../resources/assets/cube.obj",
                Vector3d{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 0.015f, 0.015f, 0.015f }  // scale
            );
        goalObject->name = "goal";
        
        switch (robotOption) {
            case 0: 
                define3DRotationRobot(renderer);
                break;
            case 1:
                define3DTranslationRobot(renderer);
                break;
            case 2:
                define7DRobot(renderer);
                break;
        }
        
        jointStates = VectorXd::Zero(robotKinematics.joints.size());
    }

    void define3DRotationRobot(VulkanRenderEngine& renderer) {
        // This defines a basic 3-axis (3 rotation joints) robot, to test te SQP IK
        robotKinematics.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                        {0.0, 1.0, 0.0, 0.0},
                                                        {0.0, 0.0, 1.0, 0.0},
                                                        {0.0, 0.0, 0.0, 1.0}};      

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.0},
                                                           {0.0, 0.0, 1.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -3.14, 
                                            3.14});

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{0.0, 0.0, 1.0, 0.0},
                                                           {1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -3.14, 
                                            3.14});

        robotKinematics.joints.emplace_back(Joint{Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                           {0.0, 1.0, 0.0, 0.5},
                                                           {0.0, 0.0, 1.0, 0.0},
                                                           {0.0, 0.0, 0.0, 1.0}}, 
                                            JointType::Rotation, 
                                            -3.14, 
                                            3.14});

        robotKinematics.endEffector.transformationMatrix = Matrix4d{{1.0, 0.0, 0.0, 0.0},
                                                                    {0.0, 1.0, 0.0, 0.5},
                                                                    {0.0, 0.0, 1.0, 0.0},
                                                                    {0.0, 0.0, 0.0, 1.0}};

        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        base->name = "base";

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        effector->name = "effector";

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 1";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 2";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 3";
    }

    void define3DTranslationRobot(VulkanRenderEngine& renderer) {
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

        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        base->name = "base";

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        effector->name = "effector";

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 1";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 2";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 3";
    }

    void define7DRobot(VulkanRenderEngine& renderer) {
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
        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        base->name = "base";

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );
        effector->name = "effector";

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 1";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 2";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 3";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 4";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 5";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 6";
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                Vector3d{ .5f, .5f, .5f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        joints.back()->name = "joint 7";    
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
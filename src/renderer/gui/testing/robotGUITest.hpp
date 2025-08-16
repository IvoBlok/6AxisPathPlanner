#ifndef ROBOT_GUI_TEST_HPP
#define ROBOT_GUI_TEST_HPP

#include "renderer/core/RenderEngine.hpp"
#include "renderer/geometries/RendererObject.hpp"
#include "renderer/geometries/RendererCurve.hpp"

#include "robotKinematics.hpp"
#include "core/mathUtils.hpp"

#include <iostream>

using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::Vector3f;

namespace kinematics {
class RobotGUITest {
public:
    RobotKinematics robotKinematics;

    std::vector<std::shared_ptr<renderer::Object>> joints;
    std::shared_ptr<renderer::Object> base;
    std::shared_ptr<renderer::Object> effector;
    
    VectorXd jointStates;

    Vector3f endPoint;
    Vector3f endRotation;
    std::shared_ptr<renderer::Object> goalObject;

    RobotGUITest(RenderEngine& renderer) {
        registerWithRenderer(renderer);

        continuousIK = false;
        endPoint = Vector3f::Zero();
        endRotation = Vector3f::Zero();
    }

    void drawGUI(RenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {

            // robot selection dropdown
            const char* items[] = { "3D Rotation", "3D Translation", "7D Kuka Inspired", "" };
            static int selectedItem = -1;
            ImGui::PushID("robot_GUI_select_robot_type");
            const char* preview = (selectedItem > -1) ? items[selectedItem] : "-";
            if (ImGui::BeginCombo("##robot_select_combo", preview)) {
                for (int i = 0; i < 4; ++i) {
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
            // -----------------------------------------------------------------
            // input for the goal
            ImGui::SeparatorText("Desired endpoint");
            ImGui::SliderFloat("x", &endPoint(0), -1.5, 1.5);
            ImGui::SliderFloat("y", &endPoint(1), -1.5, 1.5);
            ImGui::SliderFloat("z", &endPoint(2), -1.5, 1.5);
            ImGui::SliderFloat("yaw", &endRotation(0), -3.14, 3.14);
            ImGui::SliderFloat("pitch", &endRotation(1), -3.14, 3.14);
            ImGui::SliderFloat("roll", &endRotation(2), -3.14, 3.14);
            ImGui::Checkbox("continuous IK##RenderToggle", &continuousIK);

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
                    base->setPose(robotMatrices.front());
                    effector->setPose(robotMatrices.back());
                    for (int i = 1; i < (int)robotMatrices.size() - 1; i++) {
                        joints[i - 1]->setPose(robotMatrices[i]);
                    }
                }
            }
            goalObject->setPose(desiredOrientation);

            // validating a polyline
            // -----------------------------------------------------------------
            ImGui::SeparatorText("Check polyline");

            auto& curves = renderer.getCurves();
            
            // Safe static storage with proper scoping
            static int selectedIdx = -1;

            // Convert list to display vectors
            std::vector<std::string> curveNames;
            for (auto& curve : curves)
                curveNames.push_back(curve->getName());

            // Validate indices
            selectedIdx = (selectedIdx >= 0 && selectedIdx < (int)curves.size()) ? selectedIdx : -1;

            // Polyline selection
            ImGui::PushID("polyline_selection_group");
            ImGui::Text("Select Curve:");
            preview = (selectedIdx != -1) ? curveNames[selectedIdx].c_str() : "-";
            if (ImGui::BeginCombo("##polyline_select_combo", preview)) {
                for (int i = 0; i < (int)curves.size(); ++i) {
                    ImGui::PushID(i);
                    bool isSelected = (selectedIdx == i);
                    if (ImGui::Selectable(curveNames[i].c_str(), isSelected)) {
                        selectedIdx = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();

            if (ImGui::Button("Check Path")) {
                if (selectedIdx != -1) {

                    auto line = curves.begin();
                    std::advance(line, selectedIdx);
                    
                    const core::Polyline2_5D& polyline = (*line)->getPolyline();

                    validationResult = robotKinematics.validatePath(polyline, maxIterations, maxAttempts, tolerance, warmStart);
                }
            }

            if (validationResult.type == IKResultType::Success) {
                static int pathStep = 1;
                ImGui::SliderInt("animation", &pathStep, 1, validationResult.states.size());

                std::vector<Matrix4d> robotMatrices = robotKinematics.forwardKinematics(validationResult.states[pathStep - 1]);

                // update the renderer objects
                base->setPose(robotMatrices.front());
                effector->setPose(robotMatrices.back());
                for (int i = 1; i < (int)robotMatrices.size() - 1; i++) {
                    joints[i - 1]->setPose(robotMatrices[i]);
                }
            }

        }
    }

    void registerWithRenderer(RenderEngine& renderer) {
        renderer.registerGuiModule([this](RenderEngine& r) {
            this->drawGUI(r); 
        });
    }

private:
    bool robotDefined;
    bool continuousIK;

    PathValidationResult validationResult;

    void defineFullRobot(RenderEngine& renderer, int robotOption) {
        validationResult.type = IKResultType::OutOfReach;
        validationResult.states.clear();

        renderer.removeObject(goalObject);
        renderer.removeObject(base);
        renderer.removeObject(effector);
        for (auto& joint : joints)
            renderer.removeObject(joint);
        joints.clear();

        robotKinematics = RobotKinematics{};
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

    void define3DRotationRobot(RenderEngine& renderer) {
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

        goalObject = renderer.createObject(
                "../../resources/assets/cube.obj",
                "goal",
                Vector3f{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 0.015f, 0.015f, 0.015f }  // scale
            );

        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "base",
                Vector3f{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "effector",
                Vector3f{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 1",
                Vector3f{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 2",
                Vector3f{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 3",
                Vector3f{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
    }

    void define3DTranslationRobot(RenderEngine& renderer) {
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

        goalObject = renderer.createObject(
                "../../resources/assets/cube.obj",
                "goal",
                Vector3f{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 0.015f, 0.015f, 0.015f }  // scale
            );
            
        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "base",
                Vector3f{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "effector",
                Vector3f{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 1",
                Vector3f{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 2",
                Vector3f{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 3",
                Vector3f{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
    }

    void define7DRobot(RenderEngine& renderer) {
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

        goalObject = renderer.createObject(
                "../../resources/assets/cube.obj",
                "goal",
                Vector3f{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 0.015f, 0.015f, 0.015f }  // scale
            );

        // create an object for each joint, the robot base, and the effector
        base = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "base",
                Vector3f{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        effector = renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "effector",
                Vector3f{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            );

        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 1",
                Vector3f{ 1.f, 0.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 2",
                Vector3f{ 1.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 3",
                Vector3f{ 0.f, 1.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 4",
                Vector3f{ 1.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 5",
                Vector3f{ 0.f, 1.f, 0.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 6",
                Vector3f{ 0.f, 0.f, 1.f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
        
        joints.emplace_back(renderer.createObject(
                "../../resources/assets/Gizmo.obj",
                "joint 7",
                Vector3f{ .5f, .5f, .5f }, // color
                Vector3d{ 0.f, 0.f, 0.f }, // position
                Vector3d{ 1.5f, 1.5f, 1.5f }  // scale
            ));
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

#endif
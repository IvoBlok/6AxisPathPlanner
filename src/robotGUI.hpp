#ifndef ROBOT_GUI_HPP
#define ROBOT_GUI_HPP

#include "renderer.hpp"
#include "robotKinematics.hpp"

namespace kinematics {
class RobotGUI {
public:
    RobotKinematics robotKinematics;
    void drawGUI(VulkanRenderEngine& renderer) {
        if (ImGui::CollapsingHeader("Define Robot##robot_GUI")) {
            if (ImGui::Button("Set robot Joints")) {
                robotKinematics = RobotKinematics{};
                robotKinematics.joints.emplace_back(LinearJoint{});
                robotKinematics.joints.emplace_back(RotationJoint{});
                robotKinematics.joints.emplace_back(RotationJoint{});
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
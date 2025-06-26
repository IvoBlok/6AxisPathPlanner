#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <list>
#include <variant>

#include "core/vector3.hpp"

namespace kinematics {

// the kinematics of the robot is defined by a connected series of joints. Joints are limited to the linear or single-axis rotation types. 
// each joint defines the vector from its zero point to a point on the line of motion. 
// It also defines this line/vector of motion, be it in the direction of motion for linear type, or the direction of rotation for the single-axis rotation type.
// Lastly it lists joint limits in both directions. Future additions might add stuff like allowed velocities, accelerations, etc...
struct LinearJoint {
    core::Vector3<double> position;
    core::Vector3<double> direction;
    float negativeLimit;
    float positiveLimit;

    LinearJoint();
    LinearJoint(core::Vector3<double> inputPosition, core::Vector3<double> inputDirection, float inputNegLimit, float inputPosLimit);
};

struct RotationJoint {
    core::Vector3<double> position;
    core::Vector3<double> direction;
    float negativeLimit;
    float positiveLimit;

    RotationJoint();
    RotationJoint(core::Vector3<double> inputPosition, core::Vector3<double> inputDirection, float inputNegLimit, float inputPosLimit);
};



class RobotKinematics {
public:
    core::Vector3<double> position;
    std::list<std::variant<LinearJoint, RotationJoint>> joints;

    RobotKinematics();
};



}
#endif // ROBOT_KINEMATICS_HPP
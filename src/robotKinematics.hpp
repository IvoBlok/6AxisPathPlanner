#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <list>
#include <variant>

#include "../external/Eigen/CustomEigen.hpp"

namespace kinematics {

// the kinematics of the robot is defined by a connected series of joints. Joints are limited to the linear or single-axis rotation types. 
// each joint defines the vector from its zero point to a point on the line of motion. 
// It also defines this line/vector of motion, be it in the direction of motion for linear type, or the direction of rotation for the single-axis rotation type.
// Lastly it lists joint limits in both directions. Future additions might add stuff like allowed velocities, accelerations, etc...
struct LinearJoint {
    Vector3d position;
    Vector3d direction;
    float negativeLimit;
    float positiveLimit;

    LinearJoint();
    LinearJoint(Vector3d inputPosition, Vector3d inputDirection, float inputNegLimit, float inputPosLimit);
};

struct RotationJoint {
    Vector3d position;
    Vector3d direction;
    float negativeLimit;
    float positiveLimit;

    RotationJoint();
    RotationJoint(Vector3d inputPosition, Vector3d inputDirection, float inputNegLimit, float inputPosLimit);
};



class RobotKinematics {
public:
    Vector3d position;
    std::list<std::variant<LinearJoint, RotationJoint>> joints;

    RobotKinematics();
};



}
#endif // ROBOT_KINEMATICS_HPP
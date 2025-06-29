#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <list>
#include <variant>
#include <vector>

#include "../external/Eigen/CustomEigen.hpp"

namespace kinematics {

enum class JointType {
    Linear,
    Rotation
};

// the kinematics of the robot is defined by a connected series of joints. Joints are limited to the linear or single-axis rotation types. 
// each joint defines the vector from its zero point to a point on the line of motion. 
// The defining direction of the motion, i.e. the direction of translation or the direction around which the translation is done, is defined by the Z axis of the transformation matrix
// Lastly it lists joint limits in both directions. Future additions might add stuff like allowed velocities, accelerations, etc...
struct Joint {
    Matrix4d transformationMatrix; // this matrix defines the coordinate system of this joint relative to the previous joint. It also, with its translation components, defines the zero point (again relative to the previous joint)
    double negativeLimit;
    double positiveLimit;

    JointType type;

    Joint();
    Joint(Matrix4d inputMatrix, JointType inputType, double inputNegLimit, double inputPosLimit);
};

struct EndEffector {
    Matrix4d transformationMatrix; // this matrix defines the coordinate system of the end effector relative to the last joint. It also, with its translation components, defines the zero point (again relative to the last joint)
};


class RobotKinematics {
public:

    // 'transformationMatrix' defines the translation and orientation of the whole robot relative to the world zero
    Matrix4d transformationMatrix;

    // 'joints' defines the shape/type of robot by its (types of) joints and their respective relations
    std::list<Joint> joints; 

    // 'endEffector' defines the transformation from the last joint to the actual robot manipulation point, whose location/orientation we are interested in
    EndEffector endEffector;


    RobotKinematics();

    // 'forwardKinematics' calculates the transformation matrices for both the joints and the end effector. These are effectively the axes and zero point of each joint, expressed in world coordinates.
    // So right multiplying some joint matrix from this function converts a vector from joint space to world space. 
    // input 'jointStates' defines the state of the robot; each vector element corresponds to the angle/displacement for the respective joint
    std::vector<Matrix4d> forwardKinematics(std::vector<double>& jointStates);
};



}
#endif // ROBOT_KINEMATICS_HPP
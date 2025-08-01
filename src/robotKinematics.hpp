#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <list>
#include <variant>
#include <vector>

#include "CustomEigen.hpp"
#include "core/polyline.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace kinematics {

enum class IKResultType {
    Success,
    OutOfReach
};

struct IKResult {
    IKResultType type;
    VectorXd state;
};

struct PathValidationResult {
    IKResultType type;
    std::vector<VectorXd> states;
};

// the kinematics of the robot is defined by a connected series of joints. Joints are limited to the linear or single-axis rotation types. 
// each joint defines the vector from its zero point to a point on the line of motion. 
// The defining direction of the motion, i.e. the direction of translation or the direction around which the translation is done, is defined by the Z axis of the transformation matrix
// Lastly it lists joint limits in both directions. Future additions might add stuff like allowed velocities, accelerations, etc...
enum class JointType {
    Linear,
    Rotation
};

struct Joint {
    Matrix4d transformationMatrix; // this matrix defines the coordinate system of this joint relative to the previous joint. It also, with its translation components, defines the zero point (again relative to the previous joint)
    double lowerLimit;
    double upperLimit;

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
    // input 'jointStates' defines the state of the robot; each vector element corresponds to the angle/displacement for the respective joint.
    std::vector<Matrix4d> forwardKinematics(VectorXd& jointStates);

    // 'fastForwardKinematics' does the same thing as 'forwardKinematics', but only calculates the matrix for the end effector, cutting down on computational costs.
    // returns the endEffector matrix in terms of the world axes; its translation vector is equal to the effector location in world space.
    Matrix4d fastForwardKinematics(VectorXd& jointStates);

    // 'inverseKinematics' calculates the required joint states so that the end effector matrix lines up with the given goal matrix. 
    IKResult inverseKinematics(Matrix4d& goal, const bool useRotation = true, Vector3d rotationAxisIgnore = Vector3d::Zero(), int maxIterations = 33, int maxAttempts = 3, double tolerance = 1e-3, bool startAtLast = true);
    
    // 'validatePath' checks if the robot defined by this class can travel the given polyline, under the given constraints.
    PathValidationResult validatePath(const core::Polyline2_5D& polyline, int maxIterations = 33, int maxAttempts = 3, double tolerance = 1e-3, bool startAtLast = true);

private:
    // 'lastIKResult' stores, as the name implies, the last result from 'inverseKinematics'. If defined, and toggled on, 'inverseKinematics' uses this as a starting point for its next call.
    VectorXd lastIKResult;

    // 'costFunction' defines when a given endEffector pose is valid or not. The closer to zero, the better the input.
    double costFunction(const Matrix4d& input, const Matrix4d& goal, const bool useRotation = true, Vector3d rotationAxisIgnore = Vector3d::Zero());

    // 'costGradient' estimates the gradient of 'costFunction' using a central difference approximation.
    VectorXd costGradientEstimate(const VectorXd& jointStates, const Matrix4d& goal, const bool useRotation = true, Vector3d rotationAxisIgnore = Vector3d::Zero(), const double stepSize = 1e-6);

    // 'jointConstraints' transforms the individual joint boundaries into a set of inequalities of the form $h_i(\textbf{x}_k) \ge 0$. It then calculates these function values for the given joint states.
    VectorXd jointConstraints(const VectorXd& jointStates);

    // 'jointConstraintsGradient' calculates $\grad h_i(\textbf{x}_k)$, for all $i$, where $h_i(\textbf{x}_k)$ is defined by the 'jointConstraints' function. Each column refers to the gradient of a single $i$.
    MatrixXd jointConstraintsGradients(const VectorXd& jointStates);

    VectorXd jointLowerBounds();
    VectorXd jointUpperBounds();

    // 'isValidState' checks if all given state elements fall within the defined joint boundaries.
    bool isValidState(const VectorXd& jointStates);

    VectorXd randomJointStateInBounds();
};

}
#endif // ROBOT_KINEMATICS_HPP
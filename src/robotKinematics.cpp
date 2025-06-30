#include "robotKinematics.hpp"
#include "../Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace kinematics {

Joint::Joint() {
    transformationMatrix = Matrix4d::Identity();

    negativeLimit = 0.0;
    positiveLimit = 0.0;

    type = JointType::Linear;
}

Joint::Joint(Matrix4d inputMatrix, JointType inputType, double inputNegLimit, double inputPosLimit) {
    transformationMatrix = inputMatrix;

    negativeLimit = inputNegLimit;
    positiveLimit = inputPosLimit;

    type = inputType;
}


// public:
RobotKinematics::RobotKinematics() {
    transformationMatrix = Matrix4d::Identity();
}

std::vector<Matrix4d> RobotKinematics::forwardKinematics(std::vector<double>& jointStates) {

    std::vector<Matrix4d> results;
    results.emplace_back(transformationMatrix);

    size_t index = 0;
    for (const auto& joint : joints)
    {
        // check if joint state is within the defined limits
        if (jointStates[index] < joint.negativeLimit - 1e-6 || jointStates[index] > joint.positiveLimit + 1e-6) 
            throw std::runtime_error("Given joint states contains an out of bounds element!\n");


        Matrix4d matrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            matrix.col(3) += matrix.col(2) * jointStates[index];
        } 
        else if (joint.type == JointType::Rotation) {
            matrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(jointStates[index], Eigen::Vector3d::UnitZ()).toRotationMatrix();;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1); i.e. the frame of joint i-1 expressed in the world frame
        // since we want World -> Joint_(i), we do:  {World -> Joint_(i-1)} * {Joint_(i-1) -> Joint_(i)}
        results.emplace_back(results.back() * matrix);

        ++index;
    }

    results.emplace_back(results.back() * endEffector.transformationMatrix);

    return results;
}

Matrix4d RobotKinematics::fastForwardKinematics(std::vector<double>& jointStates) {
    Matrix4d result = transformationMatrix;
    Matrix4d tempMatrix;
    const double tol = 1e-6;
    
    size_t index = 0;
    for (const auto& joint : joints)
    {
        const double& state = jointStates[index];

        // check if joint state is within the defined limits
        if (state < (joint.negativeLimit - tol) || state > (joint.positiveLimit + tol)) 
            throw std::runtime_error("Joint state out of bounds!\n");

        tempMatrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            tempMatrix.col(3) += tempMatrix.col(2) * jointStates[index];
        } 
        else if (joint.type == JointType::Rotation) {
            tempMatrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(jointStates[index], Eigen::Vector3d::UnitZ()).toRotationMatrix();;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1); i.e. the frame of joint i-1 expressed in the world frame
        // since we want World -> Joint_(i), we do:  {World -> Joint_(i-1)} * {Joint_(i-1) -> Joint_(i)}
        result *= tempMatrix;
        ++index;
    }

    result *= endEffector.transformationMatrix;
    return result;
}

std::vector<double> RobotKinematics::inverseKinematics(Matrix4d goal, int maxIterations, double tolerance) {
    /* This IK algorithm attemps to solve the following problem:
        $\text{min}_x \;f(\textbf{x}) $
        $\text{subject to} \; h_i(\textbf{x}) \ge 0, \; i \in (0,1,2...m)$
        where $\textbf{x}$ is a vector containing the joint states.
        A basic SQP approach is used here. This approach works by linearizing the problem into:
        $\text{min}_p \; f(\textbf{x}_k) + (\grad f(\textbf{x}_k))^T p + \frac{1}{2}p^T\grad_{xx}^2\mathcal{L}_k p$
        $\text{subject to} \; (\grad h_i(\textbf{x}_k))^T p + h_i(\textbf{x}_k) \ge 0, \; i \in (0,1,2...m)$
        where $\textbf{x}_k$ is the $k$'th guess of the optimal joint state, and $\mathcal{L}_k = \mathcal{L}(\textbf{x}_k, \textbf{l}_k)= f(\textbf{x}_k) - \textbf{l}_k^T\textbf{h}(\textbf{x}_k)$
    */
    
    int n = joints.size();              // n: number of joints
    int m = n * 2;                      // m: number of inequality constraints (for now we use 2 for each joint; 1 for the positive joint limit, 1 for the negative limit)

    std::vector<double> x (n, 0.0);     // x: the current joint state guess. initial joints guess is set as a zero vector
    std::vector<double> l (m, 0.0);     // l: the lagrange multiplier for inequalities. Initially set as a zero vector
}
// private:

}
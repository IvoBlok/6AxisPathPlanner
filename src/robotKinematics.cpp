#include "robotKinematics.hpp"
#include "../Eigen/Dense"

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



RobotKinematics::RobotKinematics() {
    transformationMatrix = Matrix4d::Identity();
}

std::vector<Matrix4d> RobotKinematics::forwardKinematics(std::vector<double>& jointStates) {

    std::vector<Matrix4d> results;
    results.emplace_back(transformationMatrix);

    size_t index = 0;
    for (auto const& joint : joints)
    {
        // check if joint state is within the defined limits
        if (jointStates[index] < joint.negativeLimit || jointStates[index] > joint.positiveLimit) 
            throw std::runtime_error("Given joint states contains an out of bounds element!\n");


        Matrix4d matrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            matrix.col(3) += matrix.col(2) * jointStates[index];
        } 
        else if (joint.type == JointType::Rotation) {
            Matrix4d rotation = Matrix4d::Identity();
            rotation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(jointStates[index]).toRotationMatrix();

            matrix = rotation * matrix;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1)
        // since we want World -> Joint_(i), we do: {Joint_(i-1) -> Joint_(i)} * {World -> Joint_(i-1)}
        results.emplace_back(matrix * results.back());

        ++index;
    }

    results.emplace_back(endEffector.transformationMatrix * results.back());

    return results;
}

}
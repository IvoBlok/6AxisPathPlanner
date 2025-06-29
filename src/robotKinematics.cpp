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
        if (jointStates[index] < joint.negativeLimit - 1e-6 || jointStates[index] > joint.positiveLimit + 1e-6) 
            throw std::runtime_error("Given joint states contains an out of bounds element!\n");


        Matrix4d matrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            matrix.col(3) += matrix.col(2) * jointStates[index];
        } 
        else if (joint.type == JointType::Rotation) {
            // extract rotation and translation
            Eigen::Matrix3d R = matrix.block<3,3>(0,0);
            Eigen::Vector3d t = matrix.block<3,1>(0,3);

            // apply local Z-rotation
            Eigen::Matrix3d local_z_rot = Eigen::AngleAxisd(jointStates[index], Eigen::Vector3d::UnitZ()).toRotationMatrix();
            R = R * local_z_rot;  // Rotate around LOCAL Z

            // rebuild matrix
            matrix.block<3,3>(0,0) = R;
            matrix.block<3,1>(0,3) = t;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1); i.e. the frame of joint i-1 expressed in the world frame
        // since we want World -> Joint_(i), we do:  {World -> Joint_(i-1)} * {Joint_(i-1) -> Joint_(i)}
        results.emplace_back(results.back() * matrix);

        ++index;
    }

    results.emplace_back(results.back() * endEffector.transformationMatrix);

    return results;
}

}
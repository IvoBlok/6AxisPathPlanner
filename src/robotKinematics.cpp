#include "robotKinematics.hpp"

namespace kinematics {

LinearJoint::LinearJoint() {
    position = core::Vector3<double>{};
    direction = core::Vector3<double>{};
    negativeLimit = 0.f;
    positiveLimit = 0.f;
}

LinearJoint::LinearJoint(core::Vector3<double> inputPosition, core::Vector3<double> inputDirection, float inputNegLimit, float inputPosLimit) {
    position = inputPosition;
    direction = inputDirection;
    negativeLimit = inputNegLimit;
    positiveLimit = inputPosLimit;
}

RotationJoint::RotationJoint() {
    position = core::Vector3<double>{};
    direction = core::Vector3<double>{};
    negativeLimit = 0.f;
    positiveLimit = 0.f;
}

RotationJoint::RotationJoint(core::Vector3<double> inputPosition, core::Vector3<double> inputDirection, float inputNegLimit, float inputPosLimit) {
    position = inputPosition;
    direction = inputDirection;
    negativeLimit = inputNegLimit;
    positiveLimit = inputPosLimit;
}

RobotKinematics::RobotKinematics() {
    position = core::Vector3<double>{};
}

}
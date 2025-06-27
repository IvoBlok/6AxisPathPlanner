#include "robotKinematics.hpp"

namespace kinematics {

LinearJoint::LinearJoint() {
    position = Vector3d::Zero();
    direction = Vector3d::Zero();
    negativeLimit = 0.f;
    positiveLimit = 0.f;
}

LinearJoint::LinearJoint(Vector3d inputPosition, Vector3d inputDirection, float inputNegLimit, float inputPosLimit) {
    position = inputPosition;
    direction = inputDirection;
    negativeLimit = inputNegLimit;
    positiveLimit = inputPosLimit;
}

RotationJoint::RotationJoint() {
    position = Vector3d::Zero();
    direction = Vector3d::Zero();
    negativeLimit = 0.f;
    positiveLimit = 0.f;
}

RotationJoint::RotationJoint(Vector3d inputPosition, Vector3d inputDirection, float inputNegLimit, float inputPosLimit) {
    position = inputPosition;
    direction = inputDirection;
    negativeLimit = inputNegLimit;
    positiveLimit = inputPosLimit;
}

RobotKinematics::RobotKinematics() {
    position = Vector3d::Zero();
}

}
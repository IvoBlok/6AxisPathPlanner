#include "RendererGeometryBase.hpp"

#include <cmath>

namespace renderer {
    

    // Rotation class implementation
    // ======================================================================================

    Rotation::Rotation() {
        eulerAngles = Vector3d::Zero();
        rotationMatrix = Matrix3d::Unit();
    }

    Rotation& Rotation::operator=(const Vector3d& euler) {
        eulerAngles = euler;
        updateRotationMatrix();

        return *this;
    }

    Rotation& Rotation::operator=(const Matrix3d& matrix) {
        rotationMatrix = matrix;
        updateEulerAngles();

        return *this;
    }

    glm::mat4 Rotation::glmMatrix() const {
        glm::mat4 result{1.f};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result[j][i] = rotationMatrix(i, j); // Both eigen and glm are column-major in memory, their access is reversed; i.e. glm::mat[i] gives the i'th column, where Eigen::Matrix(i) gives the i'th row

        return result;
    }

    Matrix4d Rotation::matrix4d() const {
        Matrix4d result = Matrix4d::Unit();
        result.topLeftCorner<3,3>() = rotationMatrix;
        return result;
    }

    Matrix3d Rotation::matrix3d() const {
        return rotationMatrix;
    }

    Vector3d Rotation::angles() const {
        return eulerAngles;
    }

    void Rotation::updateEulerAngles() {
        auto angles = Eigen::EulerAnglesXYZd{rotationMatrix};
        eulerAngles = angles.angles() * (360.0 / (2 * M_PI));
    }

    void Rotation::updateRotationMatrix() {
        auto angles = Eigen::EulerAnglesXYZd{eulerAngles * ((2 * M_PI) / 360.0)};
        rotationMatrix = angles.matrix();
    }
}
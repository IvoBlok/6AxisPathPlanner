#ifndef RENDERER_GEOMETRY_BASE_HPP
#define RENDERER_GEOMETRY_BASE_HPP

#include "renderer/core/RenderEngine.hpp"
#include "CustomEigen.hpp"

#include <glm/glm.hpp>

namespace renderer {
    class Texture {

    };

    class Model {

    };

    class Rotation {
    public:
        Rotation();

        Rotation& operator=(const Vector3d& euler);
        Rotation& operator=(const Matrix3d& matrix);

        glm::mat4 glmMatrix() const;
        Matrix4d matrix4d() const;
        Matrix3d matrix3d() const;
        
        Vector3d angles() const;

    private:
        Vector3d eulerAngles;
        Matrix3d rotationMatrix;

        void updateEulerAngles();
        void updateRotationMatrix();
    };
}   

#endif
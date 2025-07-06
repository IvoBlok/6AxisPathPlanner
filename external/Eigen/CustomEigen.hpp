#ifndef CUSTOM_EIGEN_HPP
#define CUSTOM_EIGEN_HPP

#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "Eigen/Dense"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Eigen::Matrix4d;

#include <cmath>
#include <type_traits>

const static Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

constexpr double DOUBLE_THRESHOLD = 1e-8;


// ---------------------------------------------------------------------------
// Matrix4d
// ---------------------------------------------------------------------------
inline Eigen::Matrix4d createTransformationMatrix(const Eigen::Vector3d& translation, const Eigen::Vector3d& zVector) {
    // Normalize the z-axis vector to ensure it's a unit vector
    Eigen::Vector3d zAxis = zVector.normalized();

    // Choose an arbitrary vector not parallel to z_axis to compute x_axis
    Eigen::Vector3d tempVector(1, 0, 0);
    if (zAxis.cross(tempVector).norm() < 1e-6) {
        // If z_axis is parallel to (1,0,0), choose a different temp_vector
        tempVector = Eigen::Vector3d(0, 1, 0);
    }

    // Compute x_axis as perpendicular to z_axis
    Eigen::Vector3d xAxis = zAxis.cross(tempVector).normalized();

    // Compute y_axis to complete the orthonormal basis
    Eigen::Vector3d yAxis = zAxis.cross(xAxis);

    // Construct the 3x3 rotation matrix
    Eigen::Matrix3d rotation;
    rotation.col(0) = xAxis;
    rotation.col(1) = yAxis;
    rotation.col(2) = zAxis;

    // Construct the 4x4 transformation matrix
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3,3>(0,0) = rotation;
    transformation.block<3,1>(0,3) = translation;

    return transformation;
}

// ---------------------------------------------------------------------------
// VectorXd
// ---------------------------------------------------------------------------
/**
 * Checks if all components of a vector are approximately zero.
 * Works with any Eigen vector type (Vector2d, Vector3d, VectorXd, etc.)
 */
template <typename Derived>
bool fuzzyZero(const Eigen::MatrixBase<Derived>& v, double epsilon = DOUBLE_THRESHOLD) {
    static_assert(std::is_same<typename Derived::Scalar, double>::value, 
                 "This function only works with double-precision vectors");
    
    for (int i = 0; i < v.size(); ++i) {
        if (std::abs(v[i]) >= epsilon) {
            return false;
        }
    }
    return true;
}

/**
 * Checks if two vectors are approximately equal component-wise.
 * Works with any Eigen vector type of the same dimension and scalar type.
 */
template <typename Derived1, typename Derived2>
bool fuzzyEqual(const Eigen::MatrixBase<Derived1>& v1, 
               const Eigen::MatrixBase<Derived2>& v2,
               double epsilon = DOUBLE_THRESHOLD) {
    static_assert(std::is_same<typename Derived1::Scalar, double>::value &&
                 std::is_same<typename Derived2::Scalar, double>::value,
                 "This function only works with double-precision vectors");
    
    if (v1.size() != v2.size()) {
        return false;
    }

    for (int i = 0; i < v1.size(); ++i) {
        if (std::abs(v1[i] - v2[i]) >= epsilon) {
            return false;
        }
    }
    return true;
}

/// Returns squared distance between two vectors of any size (double precision)
template <typename Derived1, typename Derived2>
double distSquared(const Eigen::MatrixBase<Derived1>& p0, 
                   const Eigen::MatrixBase<Derived2>& p1) {
    static_assert(std::is_same<typename Derived1::Scalar, double>::value &&
                 std::is_same<typename Derived2::Scalar, double>::value,
                 "This function only works with double-precision vectors");
    
    Eigen::VectorXd diff = p1 - p0;
    return diff.squaredNorm();
}



// ---------------------------------------------------------------------------
// Vector2d
// ---------------------------------------------------------------------------
// --- Basic Vector Operations -----------------------------------------------

/// Perpendicular vector to v (rotating counter clockwise)
inline Vector2d perp(const Vector2d& v) {
    return Vector2d(-v.y(), v.x());
}

/// Normalized perpendicular vector to v (rotating counter clockwise)
inline Vector2d unitPerp(const Vector2d& v) {
    Vector2d result(-v.y(), v.x());
    return result.normalized();
}

/// Perpendicular dot product (equivalent to dot(v0, perp(v1)))
inline double perpDot(const Vector2d& v0, const Vector2d& v1) {
    return v0.x() * v1.y() - v0.y() * v1.x();
}

// --- Geometric Calculations ------------------------------------------------

/// Counter clockwise angle of the vector going from p0 to p1
inline double angle(const Vector2d& p0, const Vector2d& p1) {
    return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

/// Returns the midpoint between p0 and p1
inline Vector2d midpoint(const Vector2d& p0, const Vector2d& p1) {
    return Vector2d((p0.x() + p1.x()) / 2.0, (p0.y() + p1.y()) / 2.0);
}

/// Computes the point on the circle with radius, center, and polar angle given
inline Vector2d pointOnCircle(double radius, const Vector2d& center, double angle) {
    return Vector2d(
        center.x() + radius * std::cos(angle),
        center.y() + radius * std::sin(angle)
    );
}

/// Returns the point on the segment going from p0 to p1 at parametric value t
inline Vector2d pointFromParametric(const Vector2d& p0, const Vector2d& p1, double t) {
    return p0 + t * (p1 - p0);
}

// --- Line/Point Relationships ----------------------------------------------

/// Returns the closest point that lies on the line segment from p0 to p1 to the point given
inline Vector2d closestPointOnLineSeg(const Vector2d& p0, const Vector2d& p1, const Vector2d& point) {
    Vector2d v = p1 - p0;
    Vector2d w = point - p0;
    
    double c1 = w.dot(v);
    if (c1 <= DOUBLE_THRESHOLD) {
        return p0;
    }
    
    double c2 = v.dot(v);
    if (c2 <= c1 + DOUBLE_THRESHOLD) {
        return p1;
    }
    
    double b = c1 / c2;
    return p0 + b * v;
}

/// Returns true if point is left of the line (p0 -> p1)
inline bool isLeft(const Vector2d& p0, const Vector2d& p1, const Vector2d& point) {
    return (p1.x() - p0.x()) * (point.y() - p0.y()) - 
           (p1.y() - p0.y()) * (point.x() - p0.x()) > 0.0;
}

/// Same as isLeft but uses <= operator for boundary inclusion
inline bool isLeftOrEqual(const Vector2d& p0, const Vector2d& p1, const Vector2d& point) {
    return (p1.x() - p0.x()) * (point.y() - p0.y()) - 
           (p1.y() - p0.y()) * (point.x() - p0.x()) >= 0.0;
}

/// Fuzzy version of isLeft with epsilon tolerance
inline bool isLeftOrCoincident(const Vector2d& p0, const Vector2d& p1, 
                              const Vector2d& point, double epsilon = DOUBLE_THRESHOLD) {
    return (p1.x() - p0.x()) * (point.y() - p0.y()) - 
           (p1.y() - p0.y()) * (point.x() - p0.x()) > -epsilon;
}

/// Fuzzy version of isRight with epsilon tolerance
inline bool isRightOrCoincident(const Vector2d& p0, const Vector2d& p1, 
                               const Vector2d& point, double epsilon = DOUBLE_THRESHOLD) {
    return (p1.x() - p0.x()) * (point.y() - p0.y()) - 
           (p1.y() - p0.y()) * (point.x() - p0.x()) < epsilon;
}

// --- Arc Testing -----------------------------------------------------------

/// Test if a point is within an arc sweep angle region
inline bool pointWithinArcSweepAngle(const Vector2d& center, const Vector2d& arcStart,
                                    const Vector2d& arcEnd, double bulge, const Vector2d& point) {
    if (std::abs(bulge) <= DOUBLE_THRESHOLD) {
        return false;  // Not actually an arc
    }
    if (std::abs(bulge) > 1.0) {
        return false;  // Invalid bulge value
    }

    if (bulge > 0.0) {
        return isLeftOrCoincident(center, arcStart, point) && 
               isRightOrCoincident(center, arcEnd, point);
    }
    return isRightOrCoincident(center, arcStart, point) && 
           isLeftOrCoincident(center, arcEnd, point);
}

#endif // CUSTOM_EIGEN_HPP
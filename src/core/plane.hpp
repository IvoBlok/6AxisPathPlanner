/*
This file defines a mathematical 2D plane in 3D space. 
*/
#ifndef CORE_PLANE_HPP
#define CORE_PLANE_HPP

#include "vector2.hpp"
#include "vector3.hpp"

namespace core {
template <typename Real> class Plane {
public:
	core::Vector3<Real> origin{ 0.f, 0.f, 0.f };
	core::Vector3<Real> normal{ 0.f, 0.f, 1.f };

	core::Vector3<Real> front{};
	core::Vector3<Real> right{};

	Plane(core::Vector3<Real> n_origin = core::Vector3<Real>{ 0.f, 0.f, 0.f }, core::Vector3<Real> n_normal = core::Vector3<Real>{ 0.f, 0.f, 1.f }) {
		origin = n_origin;
		normal = normalize(n_normal);
		createFrontAndRightVectors();
	};

	void createFrontAndRightVectors() {
		float Ax = std::abs(normal.x());
		float Ay = std::abs(normal.y());
		float Az = std::abs(normal.z());

		core::Vector3<Real> baseVec;
		if (Ax <= Ay && Ax <= Az) {
			baseVec = core::Vector3<Real>{ 0, -Az, Ay };
		}
		else if (Ay <= Ax && Ay <= Az) {
			baseVec = core::Vector3<Real>{ Az, 0, -Ax };
		}
		else {
			baseVec = core::Vector3<Real>{ -Ay, Ax, 0 };
		}

		front = core::cross(normal, baseVec);
		right = core::cross(normal, front);

		front = core::normalize(front);
		right = core::normalize(right);
	}

	core::Vector2<Real> getLocalCoords(core::Vector3<Real> pointOnPlane) const {
		// check that the point lies in the plane
		if (dot(pointOnPlane - origin, normal) > 1e-4) {
			std::cout << "point does not lie on plane! " << dot(pointOnPlane - origin, normal) << "\n";
			pointOnPlane -= normal * dot(pointOnPlane - origin, normal);
		}

		core::Vector2<Real> result;

		result[0] = dot(pointOnPlane - origin, right);
		result[1] = dot(pointOnPlane - origin, front);

		return result;
	}

	core::Vector3<Real> getGlobalCoords(core::Vector2<Real> pointOnPlane) const {
		core::Vector3<Real> result{ 0.f, 0.f, 0.f };
		result += pointOnPlane.x() * right;
		result += pointOnPlane.y() * front;

		result += origin;
		return result;
	}

	core::Vector3<Real> getClosestPointOnPlane(core::Vector3<Real>& inputPoint) {
		core::Vector3<Real> deltaVec = origin - inputPoint;
		return dot(deltaVec, normal) * normal;
	}
};
} // namespace core

#endif // CORE_PLANE_HPP
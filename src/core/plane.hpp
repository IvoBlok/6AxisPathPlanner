/*
This file defines a mathematical 2D plane in 3D space. 
*/
#ifndef CORE_PLANE_HPP
#define CORE_PLANE_HPP

#include "CustomEigen.hpp"

using Eigen::Vector3d;
using Eigen::Vector2d;

namespace core {
class Plane {
public:
	Vector3d origin{ 0.f, 0.f, 0.f };
	Vector3d normal{ 0.f, 0.f, 1.f };

	Vector3d front{};
	Vector3d right{};

	Plane(Vector3d n_origin = Vector3d{ 0.f, 0.f, 0.f }, Vector3d n_normal = Vector3d{ 0.f, 0.f, 1.f }) {
		origin = n_origin;
		normal = n_normal.normalized();
		createFrontAndRightVectors();
	};

	void createFrontAndRightVectors() {
		float Ax = std::abs(normal.x());
		float Ay = std::abs(normal.y());
		float Az = std::abs(normal.z());

		Vector3d baseVec;
		if (Ax <= Ay && Ax <= Az) {
			baseVec = Vector3d{ 0, -Az, Ay };
		}
		else if (Ay <= Ax && Ay <= Az) {
			baseVec = Vector3d{ Az, 0, -Ax };
		}
		else {
			baseVec = Vector3d{ -Ay, Ax, 0 };
		}

		front = normal.cross(baseVec);
		right = normal.cross(front);

		front.normalize();
		right.normalize();
	}

	Vector2d getLocalCoords(Vector3d pointOnPlane) const {
		// check that the point lies in the plane
		if (normal.dot(pointOnPlane - origin) > 1e-4) {
			std::cout << "point does not lie on plane! " << normal.dot(pointOnPlane - origin) << "\n";
			pointOnPlane -= normal * normal.dot(pointOnPlane - origin);
		}

		Vector2d result;

		result[0] = right.dot(pointOnPlane - origin);
		result[1] = front.dot(pointOnPlane - origin);

		return result;
	}

	Vector3d getGlobalCoords(Vector2d pointOnPlane) const {
		Vector3d result{ 0.f, 0.f, 0.f };
		result += pointOnPlane.x() * right;
		result += pointOnPlane.y() * front;

		result += origin;
		return result;
	}

	Vector3d getClosestPointOnPlane(Vector3d& inputPoint) {
		Vector3d deltaVec = origin - inputPoint;
		return normal.dot(deltaVec) * normal;
	}
};
} // namespace core

#endif // CORE_PLANE_HPP
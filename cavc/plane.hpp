#ifndef CAVC_PLANE_HPP
#define CAVC_PLANE_HPP

#include "vector2.hpp"
#include "vector3.hpp"

namespace cavc {
template <typename Real> class Plane {
public:
	cavc::Vector3<Real> origin{ 0.f, 0.f, 0.f };
	cavc::Vector3<Real> normal{ 0.f, 0.f, 1.f };

	cavc::Vector3<Real> front{};
	cavc::Vector3<Real> right{};

	Plane(cavc::Vector3<Real> n_origin = cavc::Vector3<Real>{ 0.f, 0.f, 0.f }, cavc::Vector3<Real> n_normal = cavc::Vector3<Real>{ 0.f, 0.f, 1.f }) {
		origin = n_origin;
		normal = normalize(n_normal);
		createFrontAndRightVectors();
	};

	void createFrontAndRightVectors() {
		float Ax = std::abs(normal.x());
		float Ay = std::abs(normal.y());
		float Az = std::abs(normal.z());

		cavc::Vector3<Real> baseVec;
		if (Ax <= Ay && Ax <= Az) {
			baseVec = cavc::Vector3<Real>{ 0, -Az, Ay };
		}
		else if (Ay <= Ax && Ay <= Az) {
			baseVec = cavc::Vector3<Real>{ Az, 0, -Ax };
		}
		else {
			baseVec = cavc::Vector3<Real>{ -Ay, Ax, 0 };
		}

		front = cavc::cross(normal, baseVec);
		right = cavc::cross(normal, front);

		front = cavc::normalize(front);
		right = cavc::normalize(right);
	}

	cavc::Vector2<Real> getLocalCoords(cavc::Vector3<Real> pointOnPlane) const {
		// check that the point lies in the plane
		if (dot(pointOnPlane - origin, normal) > 1e-4) {
			std::cout << "point does not lie on plane! " << dot(pointOnPlane - origin, normal) << "\n";
			pointOnPlane -= normal * dot(pointOnPlane - origin, normal);
		}

		cavc::Vector2<Real> result;

		result[0] = dot(pointOnPlane - origin, right);
		result[1] = dot(pointOnPlane - origin, front);

		return result;
	}

	cavc::Vector3<Real> getGlobalCoords(cavc::Vector2<Real> pointOnPlane) const {
		cavc::Vector3<Real> result{ 0.f, 0.f, 0.f };
		result += pointOnPlane.x() * right;
		result += pointOnPlane.y() * front;

		result += origin;
		return result;
	}

	cavc::Vector3<Real> getClosestPointOnPlane(cavc::Vector3<Real>& inputPoint) {
		cavc::Vector3<Real> deltaVec = origin - inputPoint;
		return dot(deltaVec, normal) * normal;
	}
};
}

#endif
#ifndef CORE_VECTOR3_HPP
#define CORE_VECTOR3_HPP

#include "mathutils.hpp"
#include "vector.hpp"
#include <cmath>

namespace core {
	template <typename Real> using Vector3 = Vector<Real, 3>;

	template <typename Real> Vector3<Real> cross(Vector3<Real> const& v1, Vector3<Real> const& v2) {
		Vector3<Real> result;
		result[0] = v1[1] * v2[2] - v1[2] * v2[1];
		result[1] = v1[2] * v2[0] - v1[0] * v2[2];
		result[2] = v1[0] * v2[1] - v1[1] * v2[0];
		normalize(result);
		return result;
	}

	/// Returns the distance squared between p0 and p1. Equivalent to dot(p1 - p0, p1 - p0).
	template <typename Real> Real distSquared(Vector3<Real> const& p0, Vector3<Real> const& p1) {
		Vector3<Real> d = p1 - p0;
		return dot(d, d);
	}
}

#endif
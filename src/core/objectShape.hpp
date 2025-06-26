/*
This file defines the 'mathematical' version of 3D shapes. Similar to how line data is stored twice in 2 different formats, one for the renderer and 1 for math, ObjectShape is the math variant for 3D objects.
This currently consists of merely the vertex, normal and index data in the core datatype formats.
the positions in 'vertices' are world positions; there is no separate translation, rotation or scaling properties.
*/
#ifndef OBJECT_SHAPE_HPP
#define OBJECT_SHAPE_HPP

#include <vector>
#include <utility>
#include <functional>
#include <string>

#include "../../external/Eigen/CustomEigen.hpp"

namespace core {
class ObjectShape {
public:
	std::vector<Vector3d> vertices;
	std::vector<Vector3d> normals;
	std::vector<uint32_t> indices;
};
} // namespace core

#endif // OBJECT_SHAPE_HPP
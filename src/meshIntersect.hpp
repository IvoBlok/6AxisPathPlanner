/*
This file defines the MeshIntersect namespace. This provides the functionality to find the intersection of a given mesh and plane.
*/
#ifndef MESH_INTERSECT_HPP
#define MESH_INTERSECT_HPP

#include <vector>
#include <array>
#include <map>
#include <algorithm>
#include <iterator>
#include <iostream>

#include "core/objectShape.hpp"
#include "core/polyline.hpp"

namespace meshIntersect {
typedef std::array< uint32_t, 3 > Face;
typedef std::pair<uint32_t, uint32_t> Edge;
typedef std::vector<Edge> EdgePath;

typedef std::map<Edge, uint32_t> CrossingFaceMap;

double signedDistanceOfVertexToPlane(const core::Vector3<double>& vertex, const core::Plane<double>& plane);

const std::vector<double> signedDistancesToPlane(const std::vector<core::Vector3<double>>& vertices, const core::Plane<double>& plane);

CrossingFaceMap getCrossingFaces(const std::vector<Face>& faces, const std::vector<double>& vertexDistances);

bool getNextPoint(CrossingFaceMap::const_iterator& currentFace, CrossingFaceMap& crossingFaces);

void alignEdge(Edge& edge);

EdgePath getEdgePath(CrossingFaceMap& crossingFaces);

std::vector<EdgePath> getEdgePaths(const std::vector<Face>& faces, const std::vector<double>& vertexDistances);

bool getStartingItem(const std::vector<core::Polyline2D<double>>& items, std::vector<bool>& usedItems, core::Polyline2D<double>& startItem);

bool insertConnectingEdgePath(const std::vector<core::Polyline2D<double>>& paths, std::vector<bool>& usedPaths, core::Polyline2D<double>& currentChain);

void chainEdgePaths(std::vector<core::Polyline2D<double>>& paths);

std::vector<core::Polyline2D<double>> constructGeometricPaths(const core::Plane<double>& plane, const std::vector<core::Vector3<double>>& vertices, const std::vector<core::Vector3<double>>& normals, const std::vector<EdgePath>& edgePaths, const std::vector<double>& vertexDistances);

std::vector<Face> constructFaces(const std::vector<uint32_t> indices);

std::vector<core::Polyline2D<double>> getMeshPlaneIntersection(core::Plane<double>& plane, core::ObjectShape& desiredShape);

} // namespace meshIntersect

#endif // MESH_INTERSECT_HPP
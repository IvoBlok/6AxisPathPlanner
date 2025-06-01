#include "meshIntersect.hpp"

namespace meshIntersect {
double signedDistanceOfVertexToPlane(const core::Vector3<double>& vertex, const core::Plane<double>& plane) {
    // the dot product of the plane normal and the vector from the vertex to the plane gives you the square of the shortest distance between the point and the plane
    // this result is signed, where the sign signifies on which side of the plane it lies
    return dot(plane.normal, vertex - plane.origin);
}

const std::vector<double> signedDistancesToPlane(const std::vector<core::Vector3<double>>& vertices, const core::Plane<double>& plane) {
    // to optimize for speed, the vector is signaled the final size it needs to be
    std::vector<double> distances;
    distances.reserve(vertices.size());

    // for every vertex, calculate the signed squared distance and insert it at the same index in the distances vector
    std::transform(vertices.begin(), vertices.end(), std::back_inserter(distances),
        [&plane](const auto& vertex) {
            return signedDistanceOfVertexToPlane(vertex, plane);
        }
    );
    return distances;
}

CrossingFaceMap getCrossingFaces(const std::vector<Face>& faces, const std::vector<double>& vertexDistances) {
	std::vector<std::pair<Edge, uint32_t>> crossingFaces;

	for (const auto& face : faces) {
		// if ony two points of a triangular face lay on opposite sides of the plane, clearly the line between the two points crosses the plane
		const bool edge1Crosses = vertexDistances[face[0]] * vertexDistances[face[1]] < 0;
		const bool edge2Crosses = vertexDistances[face[1]] * vertexDistances[face[2]] < 0;

		if (edge1Crosses || edge2Crosses) {
			// oddVertex is the index into the current face of the vertex that is alone/separated from the other two on one side of the plane
			int oddVertex = edge2Crosses - edge1Crosses + 1;

			// no clue what this does, my guess is some smart way of choosing what key to choose for each face so that finding the connected faces that cross the plane is easier
			const bool oddIsHigher = vertexDistances[face[oddVertex]] > 0;
			int v0 = oddVertex + 1 + oddIsHigher;
			if (v0 > 2) {
				v0 -= 3;
			}
			int v2 = oddVertex + 2 - oddIsHigher;
			if (v2 > 2) {
				v2 -= 3;
			}
			crossingFaces.push_back({
				{ static_cast<uint32_t>(face[v0]), static_cast<uint32_t>(face[oddVertex]) },
				static_cast<uint32_t>(face[v2]) });
		}
	}
	return CrossingFaceMap(crossingFaces.begin(), crossingFaces.end());
}

bool getNextPoint(CrossingFaceMap::const_iterator& currentFace, CrossingFaceMap& crossingFaces) {
	// next key in the map is the one with the first in the pair being the separated vertex from the previous one, and the second one being a random other one from that same face
	auto nextKey(std::make_pair(currentFace->first.second, currentFace->second));
	crossingFaces.erase(currentFace);
	// update the current face reference with the new point in the map
	currentFace = crossingFaces.find(nextKey);

	// check if the end has been reached
	if (currentFace == crossingFaces.end()) {
		std::swap(nextKey.first, nextKey.second);
		currentFace = crossingFaces.find(nextKey);
	}
	return currentFace != crossingFaces.end();
}

void alignEdge(Edge& edge) {
	if (edge.first > edge.second) {
		std::swap(edge.first, edge.second);
	}
}

EdgePath getEdgePath(CrossingFaceMap& crossingFaces) {
	// currentFace here is an iterator, where it points to the pair of an Edge and int
	auto currentFace = crossingFaces.cbegin();
	// edgePath is initialized with the edge between the isolated vertex of the face and another point of the face, no clue how that one is chosen
	EdgePath edgePath({ currentFace->first });

	// the closing vertex is the one point that isn't yet included in the edgePath
	int closingVertex = currentFace->second;

	// retrieve the list of index pairs of which the line in between crosses the plane, in order
	while (getNextPoint(currentFace, crossingFaces)) {
		edgePath.push_back(currentFace->first);
		closingVertex = currentFace->second;
	}

	// add the last edge, closing the loop
	edgePath.push_back({ edgePath.back().second, closingVertex });

	// switch the index order for each edge based on size
	for (auto& edge : edgePath) {
		alignEdge(edge);
	}
	return edgePath;
}

std::vector<EdgePath> getEdgePaths(const std::vector<Face>& faces, const std::vector<double>& vertexDistances) {
	// retrieve all faces in some unknown weird format that in some way intersect the plane
	auto crossingFaces = getCrossingFaces(faces, vertexDistances);

	std::vector<EdgePath> edgePaths{};
	// ensure that all closed loops of intersecting faces are found ...
	while (crossingFaces.size() > 0) {
		// retrieve the loop ...
		edgePaths.push_back(getEdgePath(crossingFaces));
	}
	return edgePaths;
}

bool getStartingItem(const std::vector<core::Polyline2D<double>>& items, std::vector<bool>& usedItems, core::Polyline2D<double>& startItem) {
	for (size_t i = 0; i < items.size(); ++i) {
		if (!usedItems[i]) {
			startItem = items[i];
			usedItems[i] = true;
			return true;
		}
	}
	return false;
}

bool insertConnectingEdgePath(const std::vector<core::Polyline2D<double>>& paths, std::vector<bool>& usedPaths, core::Polyline2D<double>& currentChain) {
	int iPath = -1;
	for (auto& path : paths) {
		++iPath;
		if (usedPaths[iPath]) {
			continue;
		}
		
		if (core::fuzzyEqual(path.vertexes().front().pos(), currentChain.vertexes().back().pos(), 1e-6)) {
			currentChain.vertexes().insert(currentChain.vertexes().end(), path.vertexes().begin() + 1, path.vertexes().end());
		}
		else if (core::fuzzyEqual(path.vertexes().back().pos(), currentChain.vertexes().back().pos(), 1e-6)) {
			currentChain.vertexes().insert(currentChain.vertexes().end(), path.vertexes().rbegin() + 1, path.vertexes().rend());
		}
		else if (core::fuzzyEqual(path.vertexes().back().pos(), currentChain.vertexes().front().pos(), 1e-6)) {
			currentChain.vertexes().insert(currentChain.vertexes().begin(), path.vertexes().begin(), path.vertexes().end() - 1);
		}
		else if (core::fuzzyEqual(path.vertexes().front().pos(), currentChain.vertexes().front().pos(), 1e-6)) {
			currentChain.vertexes().insert(currentChain.vertexes().begin(), path.vertexes().rbegin(), path.vertexes().rend() - 1);
		}
		else continue;

		usedPaths[iPath] = true;
		return true;
	}
	return false;
}

void chainEdgePaths(std::vector<core::Polyline2D<double>>& paths) {
	if (paths.size() == 0) {
		return;
	}

	std::vector<bool> usedPaths(paths.size());
	std::vector<core::Polyline2D<double>> chainedPaths;
	core::Polyline2D<double> chain;

	while (getStartingItem(paths, usedPaths, chain)) {
		while(insertConnectingEdgePath(paths, usedPaths, chain)) {}
		chainedPaths.push_back(chain);
	}

	paths = chainedPaths;
}

std::vector<core::Polyline2D<double>> constructGeometricPaths(const core::Plane<double>& plane, const std::vector<core::Vector3<double>>& vertices, const std::vector<core::Vector3<double>>& normals, const std::vector<EdgePath>& edgePaths, const std::vector<double>& vertexDistances) {
	std::vector<core::Polyline2D<double>> paths;
	
	// for every edgePath, construct a new core::Polyline2D<double> that actually lies in the plane
	for (const auto& edgePath : edgePaths) {
		core::Polyline2D<double> path;
		// if the first and last edges are identical, the loop is closed
		bool skipThisPoint = path.isClosed() = edgePath.front() == edgePath.back();

		for (const auto& edge : edgePath) {
			// only skip the first edge if that is applicable
			if (skipThisPoint) {
				skipThisPoint = false;
			}
			// no clue why this check exists, this case should never happen anyway right???
			else if (edge.first == edge.second) {
				// make the point local
				path.vertexes().push_back(core::PlineVertex2D<double>{ plane.getLocalCoords(vertices[edge.first]), 0.f });
			}
			else {
				// calculate the point on the line between the two vertices that actually lies on the plane with some basic linear interpolation
				const auto& distance1(vertexDistances[edge.first]);
				const auto& distance2(vertexDistances[edge.second]);
				const auto factor = distance1 / (distance1 - distance2);

				const auto& edgeStart(vertices[edge.first]);
				const auto& edgeEnd(vertices[edge.second]);

				const auto& normalStart(normals[edge.first]);
				const auto& normalEnd(normals[edge.second]);

				core::Vector3<double> newPoint = edgeStart + (edgeEnd - edgeStart) * factor;
				core::Vector3<double> newNormal = normalStart + (normalEnd - normalStart) * factor;
				if(!core::fuzzyZero(newNormal))
					newNormal = core::normalize(newNormal);
				path.vertexes().push_back(core::PlineVertex2D<double>{ plane.getLocalCoords(newPoint), 0.f });
				path.vertexes().back().normal() = newNormal;
			}
		}
		paths.push_back(path);
	}
	return paths;
}

std::vector<Face> constructFaces(const std::vector<uint32_t> indices) {
	std::vector<Face> faces;

	// each three indices make a singular face, so just go through all the indices and group them in a vector of faces in trios
	for (size_t i = 0; i < indices.size(); i+= 3)
	{
		Face newFace = { indices[i], indices[i + 1], indices[i + 2] };
		faces.push_back(newFace);
	}
	return faces;
}

std::vector<core::Polyline2D<double>> getMeshPlaneIntersection(core::Plane<double>& plane, core::ObjectShape& desiredShape) {
	
	// retrieve the squared signed distances from the vertices to the plane
	const auto vertexDistances = signedDistancesToPlane(desiredShape.vertices, plane);

	// form all faces
	auto faces = constructFaces(desiredShape.indices);

	// retrieve all loops of edges that intersect the plane
	// each edge in the edgePaths vector consists of the indices of the appropriate vertices in the 'vertices' vector
	auto edgePaths = getEdgePaths(faces, vertexDistances);

	// form the actual polyline from the edgepaths
	std::vector<core::Polyline2D<double>> newPaths = constructGeometricPaths(plane, desiredShape.vertices, desiredShape.normals, edgePaths, vertexDistances);

	// due to some reason the getEdgePaths isn't reliable in actually forming the full loops, so a full scan is done one more time based on the real distance between the ends of the lines to form the final loops
	chainEdgePaths(newPaths);

	// check that all edges are actual loops, since the geometry being sliced is always a solid body, making any slice always a loop
	for (size_t i = 0; i < newPaths.size(); i++)
	{
		newPaths[i].isClosed() = true;
		
		bool isClosed = core::fuzzyEqual(newPaths[i].vertexes().front().pos(), newPaths[i].vertexes().back().pos(), 1e-6);
		if (isClosed) {
			newPaths[i].vertexes().pop_back();
		}
	}

	// ensure that the result of the intersecting operation is clockwise with respect to the normal of the plane, ensuring that an offset will always go in the expected direction
	for (auto& path : newPaths) {
		if (!core::isPathClockwise(path))
			path.invertDirection();
	}

	return newPaths;
}

} // namespace meshIntersect
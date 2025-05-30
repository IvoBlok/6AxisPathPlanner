#include "toolPath2_5D.hpp"

std::vector<core::Polyline2D<double>> toolPath2_5D::filterOutPocketIntersects(std::vector<core::Polyline2D<double>>& intersectPaths, core::Plane<double> slicingPlane) {
	std::vector<core::Polyline2D<double>> nonPocketIntersectPaths;

	// filter out all pocket intersects
	for (core::Polyline2D<double>& path : intersectPaths) {

		// get a non-zero normal from the path
		for (const core::PlineVertex2D<double>& vertex : path.vertexes()) {
			if (core::length(vertex.normal()) == 0.f)
				continue;

			// project the normal on the plane
			core::Vector3<double> projectedVector = vertex.normal() - slicingPlane.normal * (core::dot(slicingPlane.normal, vertex.normal()));
			projectedVector = core::normalize(projectedVector);

			// retrieve a point in the direction of the projected normal
			core::Vector2<double> normalInLocalCoords = slicingPlane.getLocalCoords(projectedVector + slicingPlane.origin);
			core::Vector2<double> projectedPoint = vertex.pos() + (double)0.0001f * normalInLocalCoords;

			// if the projected point lays outside the area enclosed by the loop, the loop is not a pocket
			if (core::getWindingNumber(path, projectedPoint) == 0)
				nonPocketIntersectPaths.push_back(path);

			// if one suitable normal has been found, we already know if it is a pocket or not, so just skip to the next loop to check
			break;
		}
	}

	return nonPocketIntersectPaths;
}


core::Polyline2_5D<double> toolPath2_5D::generateClearingPass2_5D(ClearingPass2_5DInfo& info) {

	if (!core::fuzzyEqual(info.startPlane.normal, info.endPlane.normal)) {
		std::cout << "Given start and end plane don't have the same orientation!\n";
		return core::Polyline2_5D<double>{};
	}

	// initialize the relevant planes
	core::Vector3<double> startingPoint = info.startPlane.origin;
	core::Vector3<double> planeNormal = info.startPlane.normal;
	// ensure that the plane normal (of the start plane) is facing 'away' from the end plane
	if(!core::dot(planeNormal, info.startPlane.origin - info.endPlane.origin))
		planeNormal = -planeNormal;

	core::Plane<double> safeTraversalPlane = core::Plane<double>(startingPoint + info.safeTraverseHeight * 0.001f * planeNormal, planeNormal);
	core::Plane<double> slicingPlane = info.startPlane;
	
	std::vector<std::vector<core::Polyline2D<double>>> offsetPaths;
	std::vector<std::vector<core::Polyline2_5D<double>>> offsetPaths3D;
	core::Polyline2_5D<double> result;

	core::ParallelOffsetIslands<double> pathGeneration;

	// create the basic 2.5D toolpaths
	// ========================================================================================

	// calculate perpendicular distance between start- and endplanes.
	double remainingHeight = core::dot(info.startPlane.origin - info.endPlane.origin, planeNormal);
	while (remainingHeight > (double)0.f) {
		// check if this is the last plane of the path, and update the remaining height variable accordingly
		remainingHeight = remainingHeight - info.depthOfCut * 0.001f;

		if (remainingHeight < 0.f)
			remainingHeight = 0.f;

		// update the slicing plane
		slicingPlane = core::Plane<double>(info.endPlane.origin + remainingHeight * planeNormal, planeNormal);

		// create the boundary path of the stock
		std::vector<core::Polyline2D<double>> boundaryPaths = meshIntersect::getMeshPlaneIntersection(slicingPlane, info.stock);
		if (boundaryPaths.size() == 0)
			// no boundary was found, making this layer invalid. Skip to the next layer
			continue;
		if (boundaryPaths.size() > 1) {
			std::cout << "Only convex boundaries/stock shapes supported!\n";
		}
		core::Polyline2D<double> boundaryPath = boundaryPaths[0];

		// slice the desired object
		std::vector<core::Polyline2D<double>> intersectPaths = meshIntersect::getMeshPlaneIntersection(slicingPlane, info.shape);
		
		// filter out all pocket intersects
		intersectPaths = filterOutPocketIntersects(intersectPaths, slicingPlane);

		// if this is the closest pass, do it with an offset of the radius of the cutting tool, since this is the pass that needs to closely match the actual desiredPath
		std::vector<core::Polyline2D<double>> finalPass = core::parallelOffsetToClosedLoops(intersectPaths, info.toolRadius * 0.001f);

		// add a new entree for this layer of the 2.5D pass, and initialize it with the boundary path, since the outside edge is the first bit of material that should be removed
		offsetPaths.push_back(std::vector<core::Polyline2D<double>>{boundaryPath});

		// as long as bigger offsets still generate paths, the whole surface that needs to be milled isn't covered yet, so keep generating paths with bigger offsets
		bool finished = false;
		int loopCount = 0;
		while (!finished) {
			// TODO do a boolean union between the newPaths generated for this layer, and all previous layers. This ensures that the path for this layer doesn't remove material from higher layers

			loopCount++;
			// create the offsets, with an offet value equal to a full multiple of the stepover of the cutting tool used
			std::vector<core::Polyline2D<double>> newPaths = pathGeneration.computeOneSidedOffsets(finalPass, boundaryPath, (double)loopCount * info.stepOver * 0.001f);
			offsetPaths.back().insert(offsetPaths.back().end(), newPaths.begin(), newPaths.end());

			finished = newPaths.size() == 0;
		}

		// To ensure that the final pass is fully passed over, add it to the end of the path for this layer
		offsetPaths.back().insert(offsetPaths.back().end(), finalPass.begin(), finalPass.end());

		// convert all the 2D paths to the 3D version
		offsetPaths3D.push_back(std::vector<core::Polyline2_5D<double>>{});
		for (size_t i = 0; i < offsetPaths.back().size(); i++)
			offsetPaths3D.back().push_back(core::Polyline2_5D<double>(offsetPaths.back()[i], slicingPlane));
	}
	
	// stitch the paths together
	// ========================================================================================
	for (size_t i = 0; i < offsetPaths3D.size(); i++)
	{
		for (size_t j = 0; j < offsetPaths3D[i].size(); j++)
		{
			if (offsetPaths3D[i][j].isClosed()) {
				// move to the safe point
				core::Vector3<double> safePoint = offsetPaths3D[i][j].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[i][j].vertexes().front().point);
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane });

				// add the main path
				result.addPolyline2_5D(offsetPaths3D[i][j]);

				// add the first vertex one more time, so the loop is closed. This is a traverse move, since the move starting from this point goes to the safe traverse height
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, offsetPaths3D[i][j].vertexes().front().point, offsetPaths3D[i][j].vertexes().front().plane });

				// move up to the safe traverse height
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane });
			}
		}
	}

	return result;
}

core::Polyline2_5D<double> toolPath2_5D::generateSurfacePass2_5D(SurfacePass2_5DInfo& info) {

	if (!core::fuzzyEqual(info.startPlane.normal, info.endPlane.normal)) {
		std::cout << "Given start and end plane don't have the same orientation!\n";
		return core::Polyline2_5D<double>{};
	}

	// initialize the relevant planes
	core::Vector3<double> startingPoint = info.startPlane.origin;
	core::Vector3<double> planeNormal = info.startPlane.normal;
	// ensure that the plane normal (of the start plane) is facing 'away' from the end plane
	if(!core::dot(planeNormal, info.startPlane.origin - info.endPlane.origin))
		planeNormal = -planeNormal;

	core::Plane<double> safeTraversalPlane = core::Plane<double>(startingPoint + info.safeTraverseHeight * 0.001f * planeNormal, planeNormal);
	core::Plane<double> slicingPlane = info.startPlane;

	std::vector<std::vector<core::Polyline2D<double>>> offsetPaths;
	std::vector<std::vector<core::Polyline2_5D<double>>> offsetPaths3D;
	core::Polyline2_5D<double> result;

	core::ParallelOffsetIslands<double> pathGeneration;
	
	// create the basic 2.5D toolpaths
	// ========================================================================================

	// calculate perpendicular distance between start- and endplanes.
	double remainingHeight = core::dot(info.startPlane.origin - info.endPlane.origin, planeNormal);
	while (remainingHeight > (double)0.f) {
		// check if this is the last plane of the path, and update the remaining height variable accordingly
		remainingHeight = remainingHeight - info.depthOfCut * 0.001f;

		// ensure the toolpath onyl goes up to the end plane, not under it
		if (remainingHeight < 0.f)
			remainingHeight = 0.f;

		// update the slicing plane
		slicingPlane = core::Plane<double>(info.endPlane.origin + remainingHeight * planeNormal, planeNormal);

		// slice the desired object
		std::vector<core::Polyline2D<double>> intersectPaths = meshIntersect::getMeshPlaneIntersection(slicingPlane, info.shape);

		// filter out all pocket intersects
		intersectPaths = filterOutPocketIntersects(intersectPaths, slicingPlane);

		// if this is the closest pass, do it with an offset of the radius of the cutting tool, since this is the pass that needs to closely match the actual desiredPath
		std::vector<core::Polyline2D<double>> finalPass = core::parallelOffsetToClosedLoops(intersectPaths, info.toolRadius * 0.001f);

		// add a new entree for this layer of the 2.5D pass, and initialize it with the boundary path, since the outside edge is the first bit of material that should be removed
		offsetPaths.push_back(finalPass);

		// convert all the 2D paths to the 3D version
		offsetPaths3D.push_back(std::vector<core::Polyline2_5D<double>>{});
		for (size_t i = 0; i < offsetPaths.back().size(); i++)
			offsetPaths3D.back().push_back(core::Polyline2_5D<double>(offsetPaths.back()[i], slicingPlane));
	}


	// stitch the paths together
	// ========================================================================================
	for (size_t i = 0; i < offsetPaths3D.size(); i++)
	{
		for (size_t j = 0; j < offsetPaths3D[i].size(); j++)
		{
			if (offsetPaths3D[i][j].isClosed()) {
				// move to the safe point
				core::Vector3<double> safePoint = offsetPaths3D[i][j].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[i][j].vertexes().front().point);
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane });

				// add the main path
				result.addPolyline2_5D(offsetPaths3D[i][j]);

				// add the first vertex one more time, so the loop is closed. This is a traverse move, since the move starting from this point goes to the safe traverse height
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, offsetPaths3D[i][j].vertexes().front().point, offsetPaths3D[i][j].vertexes().front().plane });

				// move up to the safe traverse height
				result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane });
			}
		}
	}

	return result;
}

core::Polyline2_5D<double> toolPath2_5D::generateFacePass2_5D(FacePass2_5DInfo& info) {
	// initialize the plane
	// ========================================================================================
	core::Vector3<double> startingPoint = info.slicingPlane.origin;
	core::Vector3<double> planeNormal = info.slicingPlane.normal;
	core::Plane<double> safeTraversalPlane = core::Plane<double>(startingPoint + info.safeTraverseHeight * 0.001f * planeNormal, planeNormal);

	std::vector<core::Polyline2D<double>> offsetPaths;
	std::vector<core::Polyline2_5D<double>> offsetPaths3D;
	core::Polyline2_5D<double> result;

	// create the basic 2D toolpaths
	// ========================================================================================

	// create the boundary path of the stock
	std::vector<core::Polyline2D<double>> boundaryPaths = meshIntersect::getMeshPlaneIntersection(info.slicingPlane, info.stock);

	if (boundaryPaths.size() == 0) {
		// no boundary was found, making this layer invalid. Skip to the next layer
		return result;
	}

	// ensure that the boundary path is defined counterclockwise, so that the offset is in the correct direction
	for (auto& path : boundaryPaths) {
		if(core::isPathClockwise(path))
			path.invertDirection();
	}

	// as long as bigger offsets still generate paths, the whole surface that needs to be milled isn't covered yet, so keep generating paths with bigger offsets
	bool finished = false;
	int loopCount = 0;
	while (!finished) {
		loopCount++;

		// create the offsets, with an offet value equal to a full multiple of the stepover of the cutting tool used
		std::vector<core::Polyline2D<double>> newPaths = core::parallelOffset(boundaryPaths, (double)loopCount * info.stepOver * 0.001f);

		if(newPaths.size() == 0)
			finished = true;
		else
			offsetPaths.insert(offsetPaths.end(), newPaths.begin(), newPaths.end());

	}

	// convert all the 2D paths to the 3D version
	// ========================================================================================
	for (size_t i = 0; i < offsetPaths.size(); i++)
		offsetPaths3D.push_back(core::Polyline2_5D<double>(offsetPaths[i], info.slicingPlane));

	// stitch the paths together
	// ========================================================================================
	// move to the safe point

	core::Vector3<double> safePoint = offsetPaths3D[0].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[0].vertexes().front().point);
	result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane });

	for (size_t i = 0; i < offsetPaths3D.size(); i++)
	{
		if (offsetPaths3D[i].isClosed()) {

			// add the main path
			result.addPolyline2_5D(offsetPaths3D[i]);
			result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, offsetPaths3D[i].vertexes().front().point, offsetPaths3D[i].vertexes().front().plane});
		}
	}
	result.vertexes().push_back(core::PlineVertex2_5D<double>{ 0.f, safePoint, safeTraversalPlane});

	return result;
}



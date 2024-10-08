#pragma once

#include "DataStructs.h"
#include "MeshIntersect.h"
#include "cavc/polylineoffset.hpp"
#include "cavc/polylineoffsetislands.hpp"
#include "PolylineSimplifier.h"

namespace ToolpathGenerator {

	std::vector<cavc::Polyline2D<double>> filterOutPocketIntersects(std::vector<cavc::Polyline2D<double>>& intersectPaths, cavc::Plane<double> slicingPlane) {
		std::vector<cavc::Polyline2D<double>> nonPocketIntersectPaths;

		// filter out all pocket intersects
		for (cavc::Polyline2D<double>& path : intersectPaths) {

			// get a non-zero normal from the path
			for (const cavc::PlineVertex<double>& vertex : path.vertexes()) {
				if (cavc::length(vertex.normal()) == 0.f)
					continue;

				// project the normal on the plane
				cavc::Vector3<double> projectedVector = vertex.normal() - slicingPlane.normal * (cavc::dot(slicingPlane.normal, vertex.normal()));
				projectedVector = cavc::normalize(projectedVector);

				// retrieve a point in the direction of the projected normal
				cavc::Vector2<double> normalInLocalCoords = slicingPlane.getLocalCoords(projectedVector + slicingPlane.origin);
				cavc::Vector2<double> projectedPoint = vertex.pos() + (double)0.01f * normalInLocalCoords;

				// if the projected point lays outside the area enclosed by the loop, the loop is not a pocket
				if (cavc::getWindingNumber(path, projectedPoint) == 0)
					nonPocketIntersectPaths.push_back(path);

				// if one suitable normal has been found, we already know if it is a pocket or not, so just skip to the next loop to check
				break;
			}
		}

		return nonPocketIntersectPaths;
	}


	// this function generates the full 2.5D toolpath given the milling info, where the planar slices of the 2.5D are defined by the given plane normal, start height and end height
	// this only applies to situations where the milling needs to happen from the outside of the stock up to some given curve. For pocket milling use some other function
	// 'planeStartingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
	// 'planeEndingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
	cavc::Polyline3D<double> generate2_5DClearingPass(MillingPass2_5DInfo& millingInfo, cavc::Vector3<double> planeNormal, double planeStartingHeight, double planeEndingHeight) {
		// initialize the plane
		cavc::Vector3<double> startingPoint = millingInfo.stockInfo.zeroPoint + planeStartingHeight * planeNormal;
		cavc::Plane<double> slicingPlane = cavc::Plane<double>(startingPoint, planeNormal);
		cavc::Plane<double> safeTraversalPlane = cavc::Plane<double>(startingPoint + millingInfo.safeTraverseHeight * 0.001f * planeNormal, planeNormal);

		std::vector<std::vector<cavc::Polyline2D<double>>> offsetPaths;
		std::vector<std::vector<cavc::Polyline3D<double>>> offsetPaths3D;
		cavc::Polyline3D<double> result;

		cavc::ParallelOffsetIslands<double> pathGeneration;

		// create the basic 2.5D toolpaths
		// ========================================================================================
		// keep generating planes untill the planeEndingHeight has been reached
		double remainingHeight = planeStartingHeight - planeEndingHeight;
		while (remainingHeight > (double)0.f) {
			// check if this is the last plane of the path, and update the remaining height variable accordingly
			remainingHeight = remainingHeight - millingInfo.depthOfCut * 0.001f;

			double localRemainingHeight = remainingHeight;
			if (localRemainingHeight < 0.f)
				localRemainingHeight = 0.f;

			// update the slicing plane
			slicingPlane = cavc::Plane<double>(millingInfo.stockInfo.zeroPoint + (planeEndingHeight + localRemainingHeight) * planeNormal, planeNormal);

			// create the boundary path of the stock
			cavc::Polyline2D<double> boundaryPath = MeshIntersect::getBasicBoxIntersection(slicingPlane, millingInfo.stockInfo);
			if (boundaryPath.vertexes().size() == 0)
				// no boundary was found, making this layer invalid. Skip to the next layer
				continue;

			// slice the desired object
			std::vector<cavc::Polyline2D<double>> intersectPaths = MeshIntersect::getMeshPlaneIntersection(slicingPlane, millingInfo.desiredShape);
			
			// filter out all pocket intersects
			intersectPaths = filterOutPocketIntersects(intersectPaths, slicingPlane);

			// if this is the closest pass, do it with an offset of the radius of the cutting tool, since this is the pass that needs to closely match the actual desiredPath
			std::vector<cavc::Polyline2D<double>> finalPass = cavc::parallelOffsetToClosedLoops(intersectPaths, millingInfo.toolInfo.mainToolRadius * 0.001f);

			// add a new entree for this layer of the 2.5D pass, and initialize it with the boundary path, since the outside edge is the first bit of material that should be removed
			offsetPaths.push_back(std::vector<cavc::Polyline2D<double>>{boundaryPath});

			// as long as bigger offsets still generate paths, the whole surface that needs to be milled isn't covered yet, so keep generating paths with bigger offsets
			bool finished = false;
			int loopCount = 0;
			while (!finished) {
				// TODO do a boolean union between the newPaths generated for this layer, and all previous layers. This ensures that the path for this layer doesn't remove material from higher layers

				loopCount++;
				// create the offsets, with an offet value equal to a full multiple of the stepover of the cutting tool used
				std::vector<cavc::Polyline2D<double>> newPaths = pathGeneration.computeOneSidedOffsets(finalPass, boundaryPath, (double)loopCount * millingInfo.stepOver * 0.001f);
				offsetPaths.back().insert(offsetPaths.back().end(), newPaths.begin(), newPaths.end());

				finished = newPaths.size() == 0;
			}

			// To ensure that the final pass is fully passed over, add it to the end of the path for this layer
			offsetPaths.back().insert(offsetPaths.back().end(), finalPass.begin(), finalPass.end());

			// convert all the 2D paths to the 3D version
			offsetPaths3D.push_back(std::vector<cavc::Polyline3D<double>>{});
			for (size_t i = 0; i < offsetPaths.back().size(); i++)
				offsetPaths3D.back().push_back(cavc::Polyline3D<double>(offsetPaths.back()[i], slicingPlane));
		}

		
		// TEMP TEMP TEMP TEMP TEMP
		for (size_t i = 0; i < offsetPaths3D.size(); i++)
		{
			for (size_t j = 0; j < offsetPaths3D[i].size(); j++)
			{
				if (offsetPaths3D[i][j].isClosed()) {
					// move to the safe point
					cavc::Vector3<double> safePoint = offsetPaths3D[i][j].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[i][j].vertexes().front().point);
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, true });

					// add the main path
					result.insertPolyLine3D(offsetPaths3D[i][j]);

					// add the first vertex one more time, so the loop is closed. This is a traverse move, since the move starting from this point goes to the safe traverse height
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, offsetPaths3D[i][j].vertexes().front().point, offsetPaths3D[i][j].vertexes().front().plane, true });

					// move up to the safe traverse height
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, true });
				}
			}
		}

		// TODO simplify the intersectPaths into a set of line and arc segments to speed up the rest of this algorithm and avoid annoying edgecases, maybe do this with a tolerance that can go only outwards, so no unnecessary material is removed?

		// convert all points to be relative to the some zero point on the stock
		// zero point is the most X-, Y-, Z- point on the stock
		result.movePolyline(-millingInfo.stockInfo.zeroPoint);

		// TODO check robot self-collision, robot-stock collision and robot reach

		return result;
	}


	cavc::Polyline3D<double> generate2_5DFinalPass(MillingPass2_5DInfo& millingInfo, cavc::Vector3<double> planeNormal, double planeStartingHeight, double planeEndingHeight) {
		// initialize the relevant planes
		cavc::Vector3<double> startingPoint = millingInfo.stockInfo.zeroPoint + planeStartingHeight * planeNormal;
		cavc::Plane<double> slicingPlane = cavc::Plane<double>(startingPoint, planeNormal);
		cavc::Plane<double> safeTraversalPlane = cavc::Plane<double>(startingPoint + millingInfo.safeTraverseHeight * 0.001f * planeNormal, planeNormal);

		std::vector<std::vector<cavc::Polyline2D<double>>> offsetPaths;
		std::vector<std::vector<cavc::Polyline3D<double>>> offsetPaths3D;
		cavc::Polyline3D<double> result;

		cavc::ParallelOffsetIslands<double> pathGeneration;

		// create the basic 2.5D toolpaths
		// ========================================================================================
		double remainingHeight = planeStartingHeight - planeEndingHeight;
		while (remainingHeight > (double)0.f) {
			// check if this is the last plane of the path, and update the remaining height variable accordingly
			remainingHeight = remainingHeight - millingInfo.depthOfCut * 0.001f;

			double localRemainingHeight = remainingHeight;
			if (localRemainingHeight < 0.f)
				localRemainingHeight = 0.f;

			// update the slicing plane
			slicingPlane = cavc::Plane<double>(millingInfo.stockInfo.zeroPoint + (planeEndingHeight + localRemainingHeight) * planeNormal, planeNormal);

			// create the boundary path of the stock
			cavc::Polyline2D<double> boundaryPath = MeshIntersect::getBasicBoxIntersection(slicingPlane, millingInfo.stockInfo);
			if (boundaryPath.vertexes().size() == 0)
				// no boundary was found, making this layer invalid. Skip to the next layer
				continue;

			// slice the desired object
			std::vector<cavc::Polyline2D<double>> intersectPaths = MeshIntersect::getMeshPlaneIntersection(slicingPlane, millingInfo.desiredShape);

			// filter out all pocket intersects
			intersectPaths = filterOutPocketIntersects(intersectPaths, slicingPlane);

			// if this is the closest pass, do it with an offset of the radius of the cutting tool, since this is the pass that needs to closely match the actual desiredPath
			std::vector<cavc::Polyline2D<double>> finalPass = cavc::parallelOffsetToClosedLoops(intersectPaths, millingInfo.toolInfo.mainToolRadius * 0.001f);

			// add a new entree for this layer of the 2.5D pass, and initialize it with the boundary path, since the outside edge is the first bit of material that should be removed
			offsetPaths.push_back(finalPass);

			// convert all the 2D paths to the 3D version
			offsetPaths3D.push_back(std::vector<cavc::Polyline3D<double>>{});
			for (size_t i = 0; i < offsetPaths.back().size(); i++)
				offsetPaths3D.back().push_back(cavc::Polyline3D<double>(offsetPaths.back()[i], slicingPlane));
		}

		// TODO simplify the intersectPaths into a set of line and arc segments to speed up the rest of this algorithm and avoid annoying edgecases, maybe do this with a tolerance that can go only outwards, so no unnecessary material is removed?

		// TEMP TEMP TEMP TEMP TEMP
		for (size_t i = 0; i < offsetPaths3D.size(); i++)
		{
			for (size_t j = 0; j < offsetPaths3D[i].size(); j++)
			{
				if (offsetPaths3D[i][j].isClosed()) {
					// move to the safe point
					cavc::Vector3<double> safePoint = offsetPaths3D[i][j].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[i][j].vertexes().front().point);
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, true });

					// add the main path
					result.insertPolyLine3D(offsetPaths3D[i][j]);

					// add the first vertex one more time, so the loop is closed. This is a traverse move, since the move starting from this point goes to the safe traverse height
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, offsetPaths3D[i][j].vertexes().front().point, offsetPaths3D[i][j].vertexes().front().plane, true });

					// move up to the safe traverse height
					result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, true });
				}
			}
		}

		// convert all points to be relative to the some zero point on the stock
		// zero point is the most X-, Y-, Z- point on the stock
		result.movePolyline(-millingInfo.stockInfo.zeroPoint);

		// TODO check robot self-collision, robot-stock collision and robot reach

		return result;
	}


	cavc::Polyline3D<double> generate2_5DStockFacePass(MillingPass2_5DInfo& millingInfo, cavc::Vector3<double> planeNormal, double planeHeight) {
		// initialize the plane
		cavc::Vector3<double> startingPoint = millingInfo.stockInfo.zeroPoint + planeHeight * planeNormal;
		cavc::Plane<double> slicingPlane = cavc::Plane<double>(startingPoint, planeNormal);
		cavc::Plane<double> safeTraversalPlane = cavc::Plane<double>(startingPoint + millingInfo.safeTraverseHeight * 0.001f * planeNormal, planeNormal);

		std::vector<cavc::Polyline2D<double>> offsetPaths;
		std::vector<cavc::Polyline3D<double>> offsetPaths3D;
		cavc::Polyline3D<double> result;

		// create the basic 2D toolpaths
		// ========================================================================================

		// create the boundary path of the stock
		cavc::Polyline2D<double> boundaryPath = MeshIntersect::getBasicBoxIntersection(slicingPlane, millingInfo.stockInfo);
		if (boundaryPath.vertexes().size() == 0)
			// no boundary was found, making this layer invalid. Skip to the next layer
			return result;

		offsetPaths.push_back(boundaryPath);

		// ensure that the boundary path is defined counterclockwise, so that the offset is in the correct direction
		if (cavc::isPathClockwise(boundaryPath))
			boundaryPath.invertDirection();

		// as long as bigger offsets still generate paths, the whole surface that needs to be milled isn't covered yet, so keep generating paths with bigger offsets
		bool finished = false;
		int loopCount = 0;
		while (!finished) {
			loopCount++;

			// create the offsets, with an offet value equal to a full multiple of the stepover of the cutting tool used
			std::vector<cavc::Polyline2D<double>> newPaths = cavc::parallelOffset(boundaryPath, (double)loopCount * millingInfo.stepOver * 0.001f);
			offsetPaths.insert(offsetPaths.end(), newPaths.begin(), newPaths.end());

			finished = newPaths.size() == 0;
		}

		// convert all the 2D paths to the 3D version
		for (size_t i = 0; i < offsetPaths.size(); i++)
			offsetPaths3D.push_back(cavc::Polyline3D<double>(offsetPaths[i], slicingPlane));


		// TODO simplify the intersectPaths into a set of line and arc segments to speed up the rest of this algorithm and avoid annoying edgecases, maybe do this with a tolerance that can go only outwards, so no unnecessary material is removed?

		// move to the safe point
		cavc::Vector3<double> safePoint = offsetPaths3D[0].vertexes().front().point + safeTraversalPlane.getClosestPointOnPlane(offsetPaths3D[0].vertexes().front().point);
		result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, false });

		for (size_t i = 0; i < offsetPaths3D.size(); i++)
		{
			if (offsetPaths3D[i].isClosed()) {

				// add the main path
				result.insertPolyLine3D(offsetPaths3D[i]);
				result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, offsetPaths3D[i].vertexes().front().point, offsetPaths3D[i].vertexes().front().plane, false });
			}
		}
		result.vertexes().push_back(cavc::PlineVertex3D<double>{ 0.f, safePoint, false });

		// convert all points to be relative to the some zero point on the stock
		// zero point is the most X-, Y-, Z- point on the stock
		result.movePolyline(-millingInfo.stockInfo.zeroPoint);

		// TODO check robot self-collision, robot-stock collision and robot reach

		return result;
	}
};

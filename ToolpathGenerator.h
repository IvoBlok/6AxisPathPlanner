#pragma once

#include "DataStructs.h"
#include "MeshIntersect.h"
#include "cavc/polylineoffset.hpp"
#include "cavc/polylineoffsetislands.hpp"
#include "PolylineSimplifier.h"

namespace ToolpathGenerator {

	// this function generates the full 2.5D toolpath given the milling info, where the planar slices of the 2.5D are defined by the given plane normal, start height and end height
	// this only applies to situations where the milling needs to happen from the outside of the stock up to some given curve. For pocket milling use some other function
	// 'planeStartingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
	// 'planeEndingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
	cavc::Polyline3D<double> generate2_5DOutsideToolPath(MillingPass2_5DInfo& millingInfo, cavc::Vector3<double> planeNormal, double planeStartingHeight, double planeEndingHeight) {
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
			if (boundaryPath.vertexes().size() == 0) {
				// no boundary was found, making this layer invalid. Skip to the next layer
				continue;
			}

			// slice the desired object
			std::vector<cavc::Polyline2D<double>> intersectPaths = MeshIntersect::getMeshPlaneIntersection(slicingPlane, millingInfo.desiredShape);

			// TODO simplify the intersectPaths into a set of line and arc segments to speed up the rest of this algorithm and avoid annoying edgecases,  maybe do this with a tolerance that can go only outwards, so no unnecessary material is removed?

			// if this is the closest pass, do it with an offset of the radius of the cutting tool, since this is the pass that needs to closely match the actual desiredPath
			std::vector<cavc::Polyline2D<double>> finalPass = cavc::parallelOffsetToClosedLoops(intersectPaths, millingInfo.toolInfo.mainToolRadius * 0.001f);

			// add a new entree for this layer of the 2.5D pass, and initialize it with the boundary path, since the outside edge is the first bit of material that should be removed
			offsetPaths.push_back(std::vector<cavc::Polyline2D<double>>{boundaryPath});

			// total area of the inside curves, used later to check if the final loop has been found
			// TODO only checking the area is probably not good enough, since different loops can have identical area's, but for now I'm fine with it
			double totalInsideArea = 0.f;
			for (cavc::Polyline2D<double>& insidePath : intersectPaths)
				totalInsideArea += cavc::getArea(insidePath);

			// as long as bigger offsets still generate paths, the whole surface that needs to be milled isn't covered yet, so keep generating paths with bigger offsets
			bool finished = false;
			int loopCount = 0;
			while (!finished && loopCount < 14) {
				loopCount++;
				// create the offsets, with an offet value equal to a full multiple of the stepover of the cutting tool used
				std::vector<cavc::Polyline2D<double>> newPaths = pathGeneration.computeOneSidedOffsets(finalPass, boundaryPath, (double)loopCount * millingInfo.stepOver * 0.001f);
				offsetPaths.back().insert(offsetPaths.back().end(), newPaths.begin(), newPaths.end());

				double totalOffsetPathsArea = 0.f;
				for (cavc::Polyline2D<double>& path : newPaths)
					totalOffsetPathsArea += cavc::getArea(path);
				
				finished = (newPaths.size() == 0 || (abs(totalInsideArea) == abs(totalOffsetPathsArea)));
			}

			// To ensure that the final pass is fully passed over, add it to the end of the path for this layer
			offsetPaths.back().insert(offsetPaths.back().end(), finalPass.begin(), finalPass.end());

			// convert all the 2D paths to the 3D version
			offsetPaths3D.push_back(std::vector<cavc::Polyline3D<double>>{});
			for (size_t i = 0; i < offsetPaths.back().size(); i++)
			{
				offsetPaths3D.back().push_back(cavc::Polyline3D<double>(offsetPaths.back()[i], slicingPlane));
			}
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
		// actual zero point, which is the same point of the base origin set in the KRC, is the most X-, Y-, Z+ point on the stock
		cavc::Vector3<double> realZeroPoint = millingInfo.stockInfo.zeroPoint;
		realZeroPoint.z() += 0.001f * millingInfo.stockInfo.height;

		for (auto& move : result.vertexes())
		{
			move.point -= realZeroPoint;
			move.plane.origin -= realZeroPoint;
		}

		// TODO check robot self-collision, robot-stock collision and robot reach

		return result;
	}
};

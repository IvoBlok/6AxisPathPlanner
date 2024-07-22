#pragma once

#include "Settings.h"
#include "cavc/polyline.hpp"
#include "cavc/plane.hpp"

namespace PolylineSimplifier {
	
	// this basic polyline simplify function just prunes any arcs or lines that are too small
	cavc::Polyline3D<double> basicPolylineSimplify(cavc::Polyline3D<double>& input, float lineSegmentMinLength = 0.02f, float arcSegmentMinLength = 0.04f) {
		cavc::Polyline3D<double> result;

		result.vertexes().push_back(input.vertexes().front());

		for (size_t i = 1; i < input.vertexes().size(); i++)
		{
			cavc::PlineVertex3D<double>& thisVertex = result.vertexes().back();
			cavc::PlineVertex3D<double>& nextVertex = input.vertexes()[i];

			// the tolerance for LIN and ARCS are different...
			if (thisVertex.bulgeIsZero()) {
				// if a LIN move is longer then 20 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.point, nextVertex.point) > lineSegmentMinLength * lineSegmentMinLength) {
					result.vertexes().push_back(nextVertex);
				}
			}
			else {
				// if an ARC move is longer then 40 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.point, nextVertex.point) > arcSegmentMinLength * arcSegmentMinLength) {
					result.vertexes().push_back(nextVertex);
				}
			}
		}

		// account for the edgecase of a closed loop
		if (input.isClosed()) {
			result.isClosed() = true;

			cavc::PlineVertex3D<double>& thisVertex = result.vertexes().back();
			cavc::PlineVertex3D<double>& nextVertex = input.vertexes().front();

			// the tolerance for LIN and ARCS are different...
			if (thisVertex.bulgeIsZero()) {
				// if a LIN move is longer then 20 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.point, nextVertex.point) < lineSegmentMinLength * lineSegmentMinLength) {
					result.vertexes().pop_back();
				}
			}
			else {
				// if an ARC move is longer then 40 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.point, nextVertex.point) < arcSegmentMinLength * arcSegmentMinLength) {
					result.vertexes().pop_back();
				}
			}
		}

		return result;
	}


	// this basic polyline simplify function just prunes any arcs or lines that are too small
	cavc::Polyline2D<double> basicPolylineSimplify(cavc::Polyline2D<double>& input, float lineSegmentMinLength = 0.02f, float arcSegmentMinLength = 0.04f) {
		cavc::Polyline2D<double> result;

		result.vertexes().push_back(input.vertexes().front());

		for (size_t i = 1; i < input.vertexes().size(); i++)
		{
			cavc::PlineVertex<double>& thisVertex = result.vertexes().back();
			cavc::PlineVertex<double>& nextVertex = input.vertexes()[i];

			// the tolerance for LIN and ARCS are different...
			if (thisVertex.bulgeIsZero()) {
				// if a LIN move is longer then 20 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.pos(), nextVertex.pos()) > lineSegmentMinLength * lineSegmentMinLength) {
					result.vertexes().push_back(nextVertex);
				}
			}
			else {
				// if an ARC move is longer then 40 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.pos(), nextVertex.pos()) > arcSegmentMinLength * arcSegmentMinLength) {
					result.vertexes().push_back(nextVertex);
				}
			}
		}

		// account for the edgecase of a closed loop
		if (input.isClosed()) {
			result.isClosed() = true;

			cavc::PlineVertex<double>& thisVertex = result.vertexes().back();
			cavc::PlineVertex<double>& nextVertex = input.vertexes().front();

			// the tolerance for LIN and ARCS are different...
			if (thisVertex.bulgeIsZero()) {
				// if a LIN move is longer then 20 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.pos(), nextVertex.pos()) < lineSegmentMinLength * lineSegmentMinLength) {
					result.vertexes().pop_back();
				}
			}
			else {
				// if an ARC move is longer then 40 millimeters, let it through to the output
				if (cavc::distSquared(thisVertex.pos(), nextVertex.pos()) < arcSegmentMinLength * arcSegmentMinLength) {
					result.vertexes().pop_back();
				}
			}
		}

		return result;
	}
}
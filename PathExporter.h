#pragma once

#include <iostream>
#include <fstream>
#include <string>

#include "Settings.h"
#include "cavc/polyline.hpp"
#include "cavc/plane.hpp"
#include "PolylineSimplifier.h"

namespace PathExporter {

	double roundTo(double value, double precision = 1.0) {
		return std::round(value / precision) * precision;
	}

	std::string getInitialCommand(cavc::PlineVertex3D<double>& move) {
		std::string result = std::string("");
		result += "P ";
		result += std::to_string(roundTo(move.point.x() * 1000.0, 0.1)) + " ";
		result += std::to_string(roundTo(move.point.y() * 1000.0, 0.1)) + " ";
		result += std::to_string(roundTo(move.point.z() * 1000.0, 0.1)) + " ";
		//TODO add orientation based on the plane of the move

		return result;
	}

	std::string getMoveCommand(cavc::PlineVertex3D<double>& move, cavc::PlineVertex3D<double>& nextMove) {
		std::string result = std::string("");

		// if the move is not an arc, just add the final point, converted to millimeters and rounded to a tenth of a millimeter
		if (move.bulgeIsZero()) {
			if (move.traverse)
				result += "P ";
			else
				result += "L ";

			result += std::to_string(roundTo(nextMove.point.x() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(nextMove.point.y() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(nextMove.point.z() * 1000.0, 0.1)) + " ";
			//TODO add orientation based on the plane of the move
		}
		// if the move is an arc, add both the final point an dan auxiliary point
		else {
			result += "C ";
			result += std::to_string(roundTo(nextMove.point.x() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(nextMove.point.y() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(nextMove.point.z() * 1000.0, 0.1)) + " ";
			//TODO add orientation based on the plane of the move


			cavc::Vector2<double> localv1Coords = move.getPointInPlaneCoords();
			cavc::Vector2<double> localv2Coords = move.plane.getLocalCoords(nextMove.point);
			cavc::ArcRadiusAndCenter<double> arcInfo = cavc::arcRadiusAndCenter(move.getVertexInPlaneCoords(), cavc::PlineVertex<double>{localv2Coords, 0.f});

			float startAngle = cavc::angle(arcInfo.center, localv1Coords);
			float endAngle = cavc::angle(arcInfo.center, localv2Coords);

			float deltaAngle = cavc::utils::deltaAngle(startAngle, endAngle);

			cavc::Vector2<double> localPosition;
			localPosition.x() = arcInfo.center.x() + arcInfo.radius * std::cos(startAngle + (deltaAngle / 2.f));
			localPosition.y() = arcInfo.center.y() + arcInfo.radius * std::sin(startAngle + (deltaAngle / 2.f));

			cavc::Vector3<double> arcMidPoint = move.plane.getGlobalCoords(localPosition);

			result += std::to_string(roundTo(arcMidPoint.x() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(arcMidPoint.y() * 1000.0, 0.1)) + " ";
			result += std::to_string(roundTo(arcMidPoint.z() * 1000.0, 0.1)) + " ";
		}
		return result;
	}

	// function to reduce every arc in the path to a sequence of line segments
	cavc::Polyline3D<double> reducePathComplexity(cavc::Polyline3D<double>& path) {
		cavc::Polyline3D<double> result;

		for (size_t i = 0; i < path.vertexes().size(); i++)
		{
			// if it is an arc, split it up into line segments
			if (!path.vertexes()[i].bulgeIsZero()) {

				cavc::PlineVertex3D<double> nextVertex;
				if (path.isClosed() && i == path.vertexes().size() - 1) {
					nextVertex = path.vertexes()[0];
				}
				else if (i == path.vertexes().size() - 1) {
					result.vertexes().push_back(path.vertexes()[i]);
					result.vertexes().back().bulge = 0.f;
					continue;
				}
				else {
					nextVertex = path.vertexes()[i + 1];
				}

				cavc::Vector2<double> localv1Coords = path.vertexes()[i].getPointInPlaneCoords();
				cavc::Vector2<double> localv2Coords = nextVertex.getPointInPlaneCoords();
				cavc::ArcRadiusAndCenter<double> arcInfo = cavc::arcRadiusAndCenter(path.vertexes()[i].getVertexInPlaneCoords(), nextVertex.getVertexInPlaneCoords());

				float startAngle = cavc::angle(arcInfo.center, localv1Coords);
				float endAngle = cavc::angle(arcInfo.center, localv2Coords);

				float deltaAngle = cavc::utils::deltaAngle(startAngle, endAngle);

				float arcLength = std::abs(deltaAngle * arcInfo.radius);
				int arcSplitupSegmentCount = std::ceil((arcLength * 1000.f) / arcSimplifierLineSegmentLength);

				for (size_t k = 0; k < arcSplitupSegmentCount; k++)
				{
					cavc::Vector2<double> localPosition;
					localPosition.x() = arcInfo.center.x() + arcInfo.radius * std::cos(startAngle + (deltaAngle / (float)arcSplitupSegmentCount) * k);
					localPosition.y() = arcInfo.center.y() + arcInfo.radius * std::sin(startAngle + (deltaAngle / (float)arcSplitupSegmentCount) * k);

					result.vertexes().push_back(cavc::PlineVertex3D<double>{});
					result.vertexes().back().bulge = 0.f;
					result.vertexes().back().plane = path.vertexes()[i].plane;
					result.vertexes().back().traverse = path.vertexes()[i].traverse;
					result.vertexes().back().point = path.vertexes()[i].plane.getGlobalCoords(localPosition);
				}
			}
			else {
				result.vertexes().push_back(path.vertexes()[i]);
			}
		}

		return result;
	}

	void export3DPath(cavc::Polyline3D<double>& path, std::string fileName) {
		if (path.vertexes().size() < 2 || (!path.vertexes().back().bulgeIsZero() && !path.isClosed()))
			return;

		// first, replace all arcs with line segments, since the robot is much happier with those
		cavc::Polyline3D<double> lineBasedPath = reducePathComplexity(path);

		// secondly, remove all elements that are very short
		cavc::Polyline3D<double> simplifiedPath = PolylineSimplifier::basicPolylineSimplify(lineBasedPath, 0.006f);

		// create and open text file
		std::ofstream file;
		file.open(fileName);

		int line = 0;
		if (file.is_open()) {
			std::string text;

			text = getInitialCommand(simplifiedPath.vertexes().front()) + "\n";
			file << text;

			do {
				// this assumes the path is closed, which it ofcourse should be
				int nextVertexIndex = (line + 1) % simplifiedPath.vertexes().size();
				text = getMoveCommand(simplifiedPath.vertexes()[line], simplifiedPath.vertexes()[nextVertexIndex]) + "\n";
				file << text;

				line++;
			} while (line < simplifiedPath.vertexes().size());
		}
		file.close();
	}
}

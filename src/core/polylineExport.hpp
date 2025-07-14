/*
This file defines the export options for 2D, 2.5D (and 3D) polylines
*/
#ifndef CORE_POLYLINE_EXPORT_HPP
#define CORE_POLYLINE_EXPORT_HPP

#include "polyline.hpp"

#include <fstream>
#include <string>

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace core {

std::string getInitialCommand(const PlineVertex2_5D& move) {
    std::string result = "";
    result += "P ";
    result += std::to_string(utils::roundTo(move.point.x() * 1000.0, 0.1)) + " ";
    result += std::to_string(utils::roundTo(move.point.y() * 1000.0, 0.1)) + " ";
    result += std::to_string(utils::roundTo(move.point.z() * 1000.0, 0.1)) + " ";
    result += "\n";

    return result;
}

std::string getMoveCommand(const PlineVertex2_5D& move,const PlineVertex2_5D& nextMove) {
    std::string result = "";

    if (move.bulgeIsZero()) {
        result += "L ";
        result += std::to_string(utils::roundTo(move.point.x() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(move.point.y() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(move.point.z() * 1000.0, 0.1)) + " ";
    } else {
        result += "C ";
        result += std::to_string(utils::roundTo(move.point.x() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(move.point.y() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(move.point.z() * 1000.0, 0.1)) + " ";

        Plane plane{move.plane};
        Vector2d localV1Coords = move.getPointInPlaneCoords();
        Vector2d localV2Coords = plane.getLocalCoords(nextMove.point);
        ArcRadiusAndCenter arcInfo = arcRadiusAndCenter(move.getVertexInPlaneCoords(), PlineVertex2D{localV2Coords, 0.0});

        double startAngle = angle(arcInfo.center, localV1Coords);
        double endAngle = angle(arcInfo.center, localV2Coords);

        double deltaAngle = utils::deltaAngle(startAngle, endAngle);

        Vector2d localPosition;
        localPosition.x() = arcInfo.center.x() + arcInfo.radius * std::cos(startAngle + (deltaAngle / 2.0));
        localPosition.y() = arcInfo.center.y() + arcInfo.radius * std::sin(startAngle + (deltaAngle / 2.0));

        Vector3d arcMidPoint = plane.getGlobalCoords(localPosition);

        result += std::to_string(utils::roundTo(arcMidPoint.x() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(arcMidPoint.y() * 1000.0, 0.1)) + " ";
        result += std::to_string(utils::roundTo(arcMidPoint.z() * 1000.0, 0.1)) + " ";        
    }

    result += "\n";
    return result;
}

struct exportSettings {
    std::string fileName;
    bool useArcs;
    double arcReplacementMaxLength;
};

inline void exportPath(Polyline2_5D polyline, exportSettings settings) {
    if (polyline.size() < 2 || (!polyline.vertexes().back().bulgeIsZero() && !polyline.isClosed()))
        return;
    
    if (!settings.useArcs) {
        polyline = replacePolylineArcs(polyline, settings.arcReplacementMaxLength);
    }

    std::ofstream file;
    file.open(settings.fileName);

    int line = 0;
    if (file.is_open()) {
        std::string text;

        text = getInitialCommand(polyline[0]);
        file << text;

        do {
            text = getMoveCommand(polyline[line], polyline[line + 1]);
            file << text;
            line++;
        } while (line < polyline.size());
    }
    file.close();
}

inline void exportPath(const Polyline2D& polyline, Plane plane, exportSettings settings) {
    Polyline2_5D newPolyline(polyline, plane);

    exportPath(newPolyline, settings);
}

} // namespace core

#endif // CORE_POLYLINE_EXPORT_HPP
#ifndef TOOL_PATH_2_5D_HPP
#define TOOL_PATH_2_5D_HPP

#include "meshIntersect.hpp"
#include "core/offsets/polylineOffsetIslands.hpp"

namespace toolPath2_5D {
struct ClearingPass2_5DInfo {
    ClearingPass2_5DInfo() {
        safeTraverseHeight = 0.f;
        stepOver = 0.f;
        toolRadius = 0.f;
        depthOfCut = 0.f;
    }

    double safeTraverseHeight;
    double stepOver;
    double toolRadius;
    double depthOfCut;

    core::ObjectShape shape;
    core::ObjectShape stock;
    core::Plane<double> startPlane;
    core::Plane<double> endPlane;
};

struct SurfacePass2_5DInfo {
    SurfacePass2_5DInfo() {
        safeTraverseHeight = 0.f;
        toolRadius = 0.f;
        depthOfCut = 0.f;
    }

    double safeTraverseHeight;
    double toolRadius;
    double depthOfCut;

    core::ObjectShape shape;
    core::Plane<double> startPlane;
    core::Plane<double> endPlane;
};

struct FacePass2_5DInfo {
    FacePass2_5DInfo() {
        safeTraverseHeight = 0.f;
        stepOver = 0.f;
    }

    double safeTraverseHeight;
    double stepOver;

    core::ObjectShape stock;
    core::Plane<double> slicingPlane;
};

std::vector<core::Polyline2D<double>> filterOutPocketIntersects(std::vector<core::Polyline2D<double>>& intersectPaths, core::Plane<double> slicingPlane);

// this function generates the full 2.5D toolpath given the milling info, where the planar slices of the 2.5D are defined by the given plane normal, start height and end height
// this only applies to situations where the milling needs to happen from the outside of the stock up to some given curve. For pocket milling use some other function
// 'planeStartingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
// 'planeEndingHeight' is defined along the length of 'planeNormal', where the start of planeNormal is the zero point of the stock given in 'millingInfo'
core::Polyline2_5D<double> generateClearingPass2_5D(ClearingPass2_5DInfo& info);

core::Polyline2_5D<double> generateSurfacePass2_5D(SurfacePass2_5DInfo& info);


core::Polyline2_5D<double> generateFacePass2_5D(FacePass2_5DInfo& info);

}  // namespace toolPath2_5D

#endif
/*
This file defines the 'PlineVertex'. A (mathematical) polyline is simply a sequency of polyline vertices. 
A strictly 2D version is the focus here. A 2.5D version is also here, which is effectively just identical to the 2D version, but now the polyline lays in a 2d plane in 2d space.
The points in the 2_5D version are the 3D world coordinates of the points. 

This file also defines general functions involving polyline vertices. The primary of which is the intersect; it finds the intersection point(s) between two vertices. 

Each vertex represents a curve between a start and endpoint. the endpoint of one vertex is the startpoint of the next.
For memory efficiency purposes only the startpoint is stored in the vertex. The endpoint is only stored as the startpoint of the next vertex.

A polyline vertex can either go straight/linearly from start to end, or using an arc/circle segment. 
In the case of the latter you need to define 'bulge', which is defined as tan(e) where e is an angle. 
See 'https://www.afralisp.net/archive/lisp/Bulges1.htm' for the geometric relations.
*/
#ifndef CORE_POLYLINE_SEGMENT_HPP
#define CORE_POLYLINE_SEGMENT_HPP

#include "intersect/circleCircleIntersect.hpp"
#include "intersect/circleLineIntersect.hpp"
#include "intersect/LineLineIntersect.hpp"

#include "CustomEigen.hpp"

#include "mathUtils.hpp"
#include "plane.hpp"

namespace core {
class PlineVertex2D {
public:
  PlineVertex2D() = default;
  PlineVertex2D(double x, double y, double bulge) : positionVec(x, y), bulgeVal(bulge) {}
  PlineVertex2D(Vector2d position, double bulge)
      : PlineVertex2D(position.x(), position.y(), bulge) {}

  double x() const { return positionVec.x(); }
  double &x() { return positionVec.x(); }

  double y() const { return positionVec.y(); }
  double &y() { return positionVec.y(); }

  double bulge() const { return bulgeVal; }
  double &bulge() { return bulgeVal; }

  bool bulgeIsZero(double epsilon = utils::realPrecision<double>()) const {
    return std::abs(bulgeVal) < epsilon;
  }

  bool bulgeIsNeg() const { return bulgeVal < 0.f; }
  bool bulgeIsPos() const { return bulgeVal > 0.f; }

  Vector2d const &pos() const { return positionVec; }
  Vector2d &pos() { return positionVec; }

  Vector3d const& normal() const { return normalVec; }
  Vector3d& normal() { return normalVec; }

private:
  Vector2d positionVec;
  Vector3d normalVec = Vector3d{ 0.f, 0.f, 0.f };
  double bulgeVal;
};

class PlineVertex2_5D {
public:
  PlineVertex2_5D() {
    bulge = 0.f;
    point = Vector3d(0.f, 0.f, 0.f);
    plane = Plane();
  }

  PlineVertex2_5D(double newBulge, Vector3d newPoint, Plane newPlane) {
    plane = newPlane;
    bulge = newBulge;
    point = newPoint;
  }

  PlineVertex2D getVertexInPlaneCoords() {
    PlineVertex2D result;
    result.bulge() = bulge;
    result.pos() = plane.getLocalCoords(point);
    return result;
  }

  Vector2d getPointInPlaneCoords() {
    return plane.getLocalCoords(point);
  }

  bool bulgeIsZero(double epsilon = utils::realPrecision<double>()) const {
    return std::abs(bulge) < epsilon;
  }

  double bulge;
  Vector3d point;
  Plane plane;
};

/// Axis aligned bounding box (AABB).
struct AABB {
  double xMin;
  double yMin;
  double xMax;
  double yMax;

  void expand(double val) {
    xMin -= val;
    yMin -= val;
    xMax += val;
    yMax += val;
  }
};

/// Result from computing the arc radius and arc center of a segment.
struct ArcRadiusAndCenter {
  double radius;
  Vector2d center;
};

/// Compute the arc radius and arc center of a arc segment defined by v1 to v2.
inline ArcRadiusAndCenter arcRadiusAndCenter(PlineVertex2D const &v1,
                                            PlineVertex2D const &v2) {
  CORE_ASSERT(!v1.bulgeIsZero(), "v1 to v2 must be an arc");
  CORE_ASSERT(!fuzzyEqual(v1.pos(), v2.pos()), "v1 must not be ontop of v2");

  // compute radius
  double b = std::abs(v1.bulge());
  Vector2d v = v2.pos() - v1.pos();
  double d = v.norm();
  double r = d * (b * b + double(1)) / (double(4) * b);

  // compute center
  double s = b * d / double(2);
  double m = r - s;
  double offsX = -m * v.y() / d;
  double offsY = m * v.x() / d;
  if (v1.bulgeIsNeg()) {
    offsX = -offsX;
    offsY = -offsY;
  }

  Vector2d c(v1.x() + v.x() / double(2) + offsX, v1.y() + v.y() / double(2) + offsY);
  return ArcRadiusAndCenter{r, c};
}

/// Result of splitting a segment v1 to v2.
struct SplitResult {
  /// Updated starting vertex.
  PlineVertex2D updatedStart;
  /// Vertex at the split point.
  PlineVertex2D splitVertex;
};

/// Split the segment defined by v1 to v2 at some point defined along it.
inline SplitResult splitAtPoint(PlineVertex2D const &v1, PlineVertex2D const &v2,
                               Vector2d const &point) {
  SplitResult result;
  if (v1.bulgeIsZero()) {
    result.updatedStart = v1;
    result.splitVertex = PlineVertex2D(point, 0.f);
  } else if (fuzzyEqual(v1.pos(), v2.pos(), utils::realPrecision<double>()) ||
             fuzzyEqual(v1.pos(), point, utils::realPrecision<double>())) {
    result.updatedStart = PlineVertex2D(point, 0.f);
    result.splitVertex = PlineVertex2D(point, v1.bulge());
  } else if (fuzzyEqual(v2.pos(), point, utils::realPrecision<double>())) {
    result.updatedStart = v1;
    result.splitVertex = PlineVertex2D(v2.pos(), 0.f);
  } else {
    auto radiusAndCenter = arcRadiusAndCenter(v1, v2);
    Vector2d arcCenter = radiusAndCenter.center;
    double a = angle(arcCenter, point);
    double arcStartAngle = angle(arcCenter, v1.pos());
    double theta1 = utils::deltaAngle(arcStartAngle, a);
    double bulge1 = std::tan(theta1 / double(4));
    double arcEndAngle = angle(arcCenter, v2.pos());
    double theta2 = utils::deltaAngle(a, arcEndAngle);
    double bulge2 = std::tan(theta2 / double(4));

    result.updatedStart = PlineVertex2D(v1.pos(), bulge1);
    result.splitVertex = PlineVertex2D(point, bulge2);
  }

  return result;
}


inline Vector2d segTangentVector(PlineVertex2D const &v1, PlineVertex2D const &v2,
                               Vector2d const &pointOnSeg) {
  if (v1.bulgeIsZero()) {
    return v2.pos() - v1.pos();
  }

  auto arc = arcRadiusAndCenter(v1, v2);
  if (v1.bulgeIsPos()) {
    // ccw, rotate vector from center to pointOnCurve 90 degrees
    return Vector2d(-(pointOnSeg.y() - arc.center.y()), pointOnSeg.x() - arc.center.x());
  }

  // cw, rotate vector from center to pointOnCurve -90 degrees
  return Vector2d(pointOnSeg.y() - arc.center.y(), -(pointOnSeg.x() - arc.center.x()));
}

/// Compute the closest point on a segment defined by v1 to v2 to the point given.
inline Vector2d closestPointOnSeg(PlineVertex2D const &v1, PlineVertex2D const &v2,
                                Vector2d const &point) {
  if (v1.bulgeIsZero()) {
    return closestPointOnLineSeg(v1.pos(), v2.pos(), point);
  }

  auto arc = arcRadiusAndCenter(v1, v2);

  if (fuzzyEqual(point, arc.center)) {
    // avoid normalizing zero length vector (point is at center, just return start point)
    return v1.pos();
  }

  if (pointWithinArcSweepAngle(arc.center, v1.pos(), v2.pos(), v1.bulge(), point)) {
    // closest point is on the arc
    Vector2d vToPoint = point - arc.center;
    vToPoint.normalize();
    return arc.radius * vToPoint + arc.center;
  }

  // else closest point is one of the ends
  double dist1 = distSquared(v1.pos(), point);
  double dist2 = distSquared(v2.pos(), point);
  if (dist1 < dist2) {
    return v1.pos();
  }

  return v2.pos();
}

/// Computes a fast approximate AABB of a segment described by v1 to v2, bounding box may be larger
/// than the true bounding box for the segment
inline AABB createFastApproxBoundingBox(PlineVertex2D const &v1, PlineVertex2D const &v2) {
  AABB result;
  if (v1.bulgeIsZero()) {
    if (v1.x() < v2.x()) {
      result.xMin = v1.x();
      result.xMax = v2.x();
    } else {
      result.xMin = v2.x();
      result.xMax = v1.x();
    }

    if (v1.y() < v2.y()) {
      result.yMin = v1.y();
      result.yMax = v2.y();
    } else {
      result.yMin = v2.y();
      result.yMax = v1.y();
    }

    return result;
  }

  // For arcs we don't compute the actual extents which is slower, instead we create an approximate
  // bounding box from the rectangle formed by extending the chord by the sagitta, NOTE: this
  // approximate bounding box is always equal to or bigger than the true bounding box
  double b = v1.bulge();
  double offsX = b * (v2.y() - v1.y()) / double(2);
  double offsY = -b * (v2.x() - v1.x()) / double(2);

  double pt1X = v1.x() + offsX;
  double pt2X = v2.x() + offsX;
  double pt1Y = v1.y() + offsY;
  double pt2Y = v2.y() + offsY;

  double endPointXMin, endPointXMax;
  if (v1.x() < v2.x()) {
    endPointXMin = v1.x();
    endPointXMax = v2.x();
  } else {
    endPointXMin = v2.x();
    endPointXMax = v1.x();
  }

  double ptXMin, ptXMax;
  if (pt1X < pt2X) {
    ptXMin = pt1X;
    ptXMax = pt2X;
  } else {
    ptXMin = pt2X;
    ptXMax = pt1X;
  }

  double endPointYMin, endPointYMax;
  if (v1.y() < v2.y()) {
    endPointYMin = v1.y();
    endPointYMax = v2.y();
  } else {
    endPointYMin = v2.y();
    endPointYMax = v1.y();
  }

  double ptYMin, ptYMax;
  if (pt1Y < pt2Y) {
    ptYMin = pt1Y;
    ptYMax = pt2Y;
  } else {
    ptYMin = pt2Y;
    ptYMax = pt1Y;
  }

  result.xMin = std::min(endPointXMin, ptXMin);
  result.yMin = std::min(endPointYMin, ptYMin);
  result.xMax = std::max(endPointXMax, ptXMax);
  result.yMax = std::max(endPointYMax, ptYMax);
  return result;
}

/// Calculate the path length for the segment defined from v1 to v2.
inline double segLength(PlineVertex2D const &v1, PlineVertex2D const &v2) {
  if (fuzzyEqual(v1.pos(), v2.pos())) {
    return 0.f;
  }

  if (v1.bulgeIsZero()) {
    return std::sqrt(distSquared(v1.pos(), v2.pos()));
  }

  auto arc = arcRadiusAndCenter(v1, v2);
  double startAngle = angle(arc.center, v1.pos());
  double endAngle = angle(arc.center, v2.pos());
  return std::abs(arc.radius * utils::deltaAngle(startAngle, endAngle));
}

/// Return the mid point along a segment path.
inline Vector2d segMidpoint(PlineVertex2D const &v1, PlineVertex2D const &v2) {
  if (v1.bulgeIsZero()) {
    return midpoint(v1.pos(), v2.pos());
  }

  auto arc = arcRadiusAndCenter(v1, v2);
  double a1 = angle(arc.center, v1.pos());
  double a2 = angle(arc.center, v2.pos());
  double angleOffset = std::abs(utils::deltaAngle(a1, a2) / double(2));
  // use arc direction to determine offset sign to robustly handle half circles
  double midAngle = v1.bulgeIsPos() ? a1 + angleOffset : a1 - angleOffset;
  return pointOnCircle(arc.radius, arc.center, midAngle);
}

enum class PlineSegIntrType {
  NoIntersect,
  TangentIntersect,
  OneIntersect,
  TwoIntersects,
  SegmentOverlap,
  ArcOverlap
};

struct IntrPlineSegsResult {
  PlineSegIntrType intrType;
  Vector2d point1;
  Vector2d point2;
};

inline IntrPlineSegsResult intrPlineSegs(PlineVertex2D const &v1, PlineVertex2D const &v2,
                                        PlineVertex2D const &u1, PlineVertex2D const &u2) {
  IntrPlineSegsResult result;
  const bool vIsLine = v1.bulgeIsZero();
  const bool uIsLine = u1.bulgeIsZero();

  // helper function to process line arc intersect
  auto processLineArcIntr = [&result](Vector2d const &p0, Vector2d const &p1,
                                      PlineVertex2D const &a1, PlineVertex2D const &a2) {
    auto arc = arcRadiusAndCenter(a1, a2);
    auto intrResult = intrLineSeg2Circle2(p0, p1, arc.radius, arc.center);

    // helper function to test and get point within arc sweep
    auto pointInSweep = [&](double t) {
      if (t + utils::realThreshold<double>() < 0.f ||
          t > double(1) + utils::realThreshold<double>()) {
        return std::make_pair(false, Vector2d());
      }

      Vector2d p = pointFromParametric(p0, p1, t);
      bool withinSweep = pointWithinArcSweepAngle(arc.center, a1.pos(), a2.pos(), a1.bulge(), p);
      return std::make_pair(withinSweep, p);
    };

    if (intrResult.numIntersects == 0) {
      result.intrType = PlineSegIntrType::NoIntersect;
    } else if (intrResult.numIntersects == 1) {
      auto p = pointInSweep(intrResult.t0);
      if (p.first) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = p.second;
      } else {
        result.intrType = PlineSegIntrType::NoIntersect;
      }
    } else {
      CORE_ASSERT(intrResult.numIntersects == 2, "shouldn't get here without 2 intersects");
      auto p1 = pointInSweep(intrResult.t0);
      auto p2 = pointInSweep(intrResult.t1);

      if (p1.first && p2.first) {
        result.intrType = PlineSegIntrType::TwoIntersects;
        result.point1 = p1.second;
        result.point2 = p2.second;
      } else if (p1.first) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = p1.second;
      } else if (p2.first) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = p2.second;
      } else {
        result.intrType = PlineSegIntrType::NoIntersect;
      }
    }
  };

  if (vIsLine && uIsLine) {
    auto intrResult = intrLineSeg2LineSeg2(v1.pos(), v2.pos(), u1.pos(), u2.pos());
    switch (intrResult.intrType) {
    case LineSeg2LineSeg2IntrType::None:
      result.intrType = PlineSegIntrType::NoIntersect;
      break;
    case LineSeg2LineSeg2IntrType::True:
      result.intrType = PlineSegIntrType::OneIntersect;
      result.point1 = intrResult.point;
      break;
    case LineSeg2LineSeg2IntrType::Coincident:
      result.intrType = PlineSegIntrType::SegmentOverlap;
      // build points from parametric parameters (using second segment as defined by the function)
      result.point1 = pointFromParametric(u1.pos(), u2.pos(), intrResult.t0);
      result.point2 = pointFromParametric(u1.pos(), u2.pos(), intrResult.t1);
      break;
    case LineSeg2LineSeg2IntrType::False:
      result.intrType = PlineSegIntrType::NoIntersect;
      break;
    }

  } else if (vIsLine) {
    processLineArcIntr(v1.pos(), v2.pos(), u1, u2);
  } else if (uIsLine) {
    processLineArcIntr(u1.pos(), u2.pos(), v1, v2);
  } else {
    auto arc1 = arcRadiusAndCenter(v1, v2);
    auto arc2 = arcRadiusAndCenter(u1, u2);

    auto startAndSweepAngle = [](Vector2d const &sp, Vector2d const &center, double bulge) {
      double startAngle = utils::normalizeRadians(angle(center, sp));
      double sweepAngle = double(4) * std::atan(bulge);
      return std::make_pair(startAngle, sweepAngle);
    };

    auto bothArcsSweepPoint = [&](Vector2d const &pt) {
      return pointWithinArcSweepAngle(arc1.center, v1.pos(), v2.pos(), v1.bulge(), pt) &&
             pointWithinArcSweepAngle(arc2.center, u1.pos(), u2.pos(), u1.bulge(), pt);
    };

    auto intrResult = intrCircle2Circle2(arc1.radius, arc1.center, arc2.radius, arc2.center);

    switch (intrResult.intrType) {
    case Circle2Circle2IntrType::NoIntersect:
      result.intrType = PlineSegIntrType::NoIntersect;
      break;
    case Circle2Circle2IntrType::OneIntersect:
      if (bothArcsSweepPoint(intrResult.point1)) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = intrResult.point1;
      } else {
        result.intrType = PlineSegIntrType::NoIntersect;
      }
      break;
    case Circle2Circle2IntrType::TwoIntersects: {
      const bool pt1InSweep = bothArcsSweepPoint(intrResult.point1);
      const bool pt2InSweep = bothArcsSweepPoint(intrResult.point2);
      if (pt1InSweep && pt2InSweep) {
        result.intrType = PlineSegIntrType::TwoIntersects;
        result.point1 = intrResult.point1;
        result.point2 = intrResult.point2;
      } else if (pt1InSweep) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = intrResult.point1;
      } else if (pt2InSweep) {
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = intrResult.point2;
      } else {
        result.intrType = PlineSegIntrType::NoIntersect;
      }
    } break;
    case Circle2Circle2IntrType::Coincident:
      // determine if arcs overlap along their sweep
      // start and sweep angles
      auto arc1StartAndSweep = startAndSweepAngle(v1.pos(), arc1.center, v1.bulge());
      // we have the arcs go the same direction to simplify checks
      auto arc2StartAndSweep = [&] {
        if (v1.bulgeIsNeg() == u1.bulgeIsNeg()) {
          return startAndSweepAngle(u1.pos(), arc2.center, u1.bulge());
        }

        return startAndSweepAngle(u2.pos(), arc2.center, -u1.bulge());
      }();
      // end angles (start + sweep)
      auto arc1End = arc1StartAndSweep.first + arc1StartAndSweep.second;
      auto arc2End = arc2StartAndSweep.first + arc2StartAndSweep.second;

      if (std::abs(utils::deltaAngle(arc1StartAndSweep.first, arc2End)) <
          utils::realThreshold<double>()) {
        // only end points touch at start of arc1
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = v1.pos();
      } else if (std::abs(utils::deltaAngle(arc2StartAndSweep.first, arc1End)) <
                 utils::realThreshold<double>()) {
        // only end points touch at start of arc2
        result.intrType = PlineSegIntrType::OneIntersect;
        result.point1 = u1.pos();
      } else {
        const bool arc2StartsInArc1Sweep = utils::angleIsWithinSweep(
            arc1StartAndSweep.first, arc1StartAndSweep.second, arc2StartAndSweep.first);
        const bool arc2EndsInArc1Sweep =
            utils::angleIsWithinSweep(arc1StartAndSweep.first, arc1StartAndSweep.second, arc2End);
        if (arc2StartsInArc1Sweep && arc2EndsInArc1Sweep) {
          // arc2 is fully overlapped by arc1
          result.intrType = PlineSegIntrType::ArcOverlap;
          result.point1 = u1.pos();
          result.point2 = u2.pos();
        } else if (arc2StartsInArc1Sweep) {
          // overlap from arc2 start to arc1 end
          result.intrType = PlineSegIntrType::ArcOverlap;
          result.point1 = u1.pos();
          result.point2 = v2.pos();
        } else if (arc2EndsInArc1Sweep) {
          // overlap from arc1 start to arc2 end
          result.intrType = PlineSegIntrType::ArcOverlap;
          result.point1 = v1.pos();
          result.point2 = u2.pos();
        } else {
          const bool arc1StartsInArc2Sweep = utils::angleIsWithinSweep(
              arc2StartAndSweep.first, arc2StartAndSweep.second, arc1StartAndSweep.first);
          if (arc1StartsInArc2Sweep) {
            result.intrType = PlineSegIntrType::ArcOverlap;
            result.point1 = v1.pos();
            result.point2 = v2.pos();
          } else {
            result.intrType = PlineSegIntrType::NoIntersect;
          }
        }
      }

      break;
    }
  }

  return result;
}

} // namespace core
#endif // CORE_POLYLINE_SEGMENT_HPP
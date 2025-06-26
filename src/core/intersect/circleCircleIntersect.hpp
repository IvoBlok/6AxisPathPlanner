#ifndef CORE_CIRCLE_CIRCLE_INTERSECT_HPP
#define CORE_CIRCLE_CIRCLE_INTERSECT_HPP

#include "../../../external/Eigen/CustomEigen.hpp"

namespace core {
enum class Circle2Circle2IntrType {
  // no intersect between circles
  NoIntersect,
  // one intersect between circles (tangent)
  OneIntersect,
  // two intersects between circles
  TwoIntersects,
  // circles are coincident
  Coincident
};

struct IntrCircle2Circle2Result {
  // type of intersect
  Circle2Circle2IntrType intrType;
  // first intersect point if intrType is OneIntersect or TwoIntersects, undefined otherwise
  Vector2d point1;
  // second intersect point if intrType is TwoIntersects, undefined otherwise
  Vector2d point2;
};

// Find intersect between two circles in 2D.
IntrCircle2Circle2Result intrCircle2Circle2(double radius1, Vector2d const &center1,
                                                  double radius2, Vector2d const &center2) {
  // Reference algorithm: http://paulbourke.net/geometry/circlesphere/

  IntrCircle2Circle2Result result;
  Vector2d cv = center2 - center1;
  double d2 = cv.dot(cv);
  double d = std::sqrt(d2);
  if (d < utils::realThreshold<double>()) {
    // same center position
    if (utils::fuzzyEqual(radius1, radius2)) {
      result.intrType = Circle2Circle2IntrType::Coincident;
    } else {
      result.intrType = Circle2Circle2IntrType::NoIntersect;
    }
  } else {
    // different center position
    if (d > radius1 + radius2 + utils::realThreshold<double>() ||
        d + utils::realThreshold<double>() < std::abs(radius1 - radius2)) {
      result.intrType = Circle2Circle2IntrType::NoIntersect;
    } else {
      double rad1Sq = radius1 * radius1;
      double a = (rad1Sq - radius2 * radius2 + d2) / (double(2) * d);
      Vector2d midPoint = center1 + a * cv / d;
      double diff = rad1Sq - a * a;
      if (diff < double(0)) {
        result.intrType = Circle2Circle2IntrType::OneIntersect;
        result.point1 = midPoint;
      } else {
        double h = std::sqrt(diff);
        double hOverD = h / d;
        double xTerm = hOverD * cv.y();
        double yTerm = hOverD * cv.x();
        double x1 = midPoint.x() + xTerm;
        double y1 = midPoint.y() - yTerm;
        double x2 = midPoint.x() - xTerm;
        double y2 = midPoint.y() + yTerm;
        result.point1 = Vector2d(x1, y1);
        result.point2 = Vector2d(x2, y2);
        if (fuzzyEqual(result.point1, result.point2)) {
          result.intrType = Circle2Circle2IntrType::OneIntersect;
        } else {
          result.intrType = Circle2Circle2IntrType::TwoIntersects;
        }
      }
    }
  }

  return result;
}
} // namespace core

#endif // CORE_CIRCLE_CIRCLE_INTERSECT_HPP
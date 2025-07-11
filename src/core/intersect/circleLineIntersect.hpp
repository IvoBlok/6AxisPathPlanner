#ifndef CORE_CIRCLE_LINE_INTERSECT_HPP
#define CORE_CIRCLE_LINE_INTERSECT_HPP

#include "CustomEigen.hpp"
#include "../mathUtils.hpp"

namespace core {
struct IntrLineSeg2Circle2Result {
  // number of interescts found (0, 1, or 2)
  int numIntersects;
  // parametric value for first intersect (if numIntersects > 0) otherwise undefined
  double t0;
  // parametric value for second intersect (if numintersects > 1) otherwise undefined
  double t1;
};

// Gets the intersect between a segment and a circle, returning the parametric solution t to the
// segment equation P(t) = v1 + t * (v2 - v1) for t = 0 to t = 1, if t < 0 or t > 1 then intersect
// occurs only when extending the segment out past the points given (if t < 0 intersect nearest v1,
// if t > 0 then intersect nearest v2), intersects are "sticky" and "snap" to tangent points, e.g. a
// segment very close to being a tangent will be returned as a single intersect point
inline IntrLineSeg2Circle2Result intrLineSeg2Circle2(Vector2d const &p0,
                                                    Vector2d const &p1, double radius,
                                                    Vector2d const &circleCenter) {
  // This function solves for t by substituting the parametric equations for the segment x = v1.X +
  // t * (v2.X - v1.X) and y = v1.Y + t * (v2.Y - v1.Y) for t = 0 to t = 1 into the circle equation
  // (x-h)^2 + (y-k)^2 = r^2 and then solving the resulting equation in the form a*t^2 + b*t + c = 0
  // using the quadratic formula
  IntrLineSeg2Circle2Result result;
  double dx = p1.x() - p0.x();
  double dy = p1.y() - p0.y();
  double h = circleCenter.x();
  double k = circleCenter.y();

  double a = dx * dx + dy * dy;
  if (std::abs(a) < utils::realThreshold<double>()) {
    // v1 = v2, test if point is on the circle
    double xh = p0.x() - h;
    double yk = p0.y() - k;
    if (utils::fuzzyEqual(xh * xh + yk * yk, radius * radius)) {
      result.numIntersects = 1;
      result.t0 = double(0);
    } else {
      result.numIntersects = 0;
    }
  } else {
    double b = double(2) * (dx * (p0.x() - h) + dy * (p0.y() - k));
    double c = (p0.x() * p0.x() - 2.0 * h * p0.x() + h * h) +
             (p0.y() * p0.y() - 2.0 * k * p0.y() + k * k) - radius * radius;
    double discr = b * b - 4.0 * a * c;

    if (std::abs(discr) < utils::realThreshold<double>()) {
      // 1 solution (tangent line)
      result.numIntersects = 1;
      result.t0 = -b / (double(2) * a);
    } else if (discr < double(0)) {
      result.numIntersects = 0;
    } else {
      result.numIntersects = 2;
      std::pair<double, double> sols = utils::quadraticSolutions(a, b, c, discr);
      result.t0 = sols.first;
      result.t1 = sols.second;
    }
  }

  CORE_ASSERT(result.numIntersects >= 0 && result.numIntersects <= 2, "invalid intersect count");
  return result;
}
} // namespace core

#endif // CORE_CIRCLE_LINE_INTERSECT_HPP
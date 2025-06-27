/*
This file defines the 2D and 2.5D polylines, and various functions that operate on them.
*/
#ifndef CORE_POLYLINE_HPP
#define CORE_POLYLINE_HPP

#include "polylineVertex.hpp"
#include "staticSpatialIndex.hpp"
#include "plane.hpp"

#include <algorithm>
#include <vector>
#include <utility>
#include <algorithm>

namespace core {
class Polyline2D {
public:

  // when the polyline is closed, 'isPocket' signifies if the area enclosed by the polyline is desired material, or material to be removed
  bool isPocket = false;

  /// Construct an empty open polyline.
  Polyline2D() : isClosedVal(false), vertices() {}

  using PVertex = PlineVertex2D;
  inline PVertex const &operator[](std::size_t i) const { return vertices[i]; }
  inline PVertex &operator[](std::size_t i) { return vertices[i]; }

  bool isClosed() const { return isClosedVal; }
  bool &isClosed() { return isClosedVal; }

  void invertDirection() {
    if (vertices.size() < 2) {
      return;
    }

    std::reverse(std::begin(vertices), std::end(vertices));
    
    double firstBulge = vertices[0].bulge();

    for (size_t i = 1; i < vertices.size(); ++i)
      vertices[i - 1].bulge() = -vertices[i].bulge();

    vertices.back().bulge() = -firstBulge;
  }

  void addVertex(double x, double y, double bulge) { vertices.emplace_back(x, y, bulge); }
  void addVertex(PVertex vertex) { addVertex(vertex.x(), vertex.y(), vertex.bulge()); }

  std::size_t size() const { return vertices.size(); }

  PVertex const &lastVertex() const { return vertices.back(); }
  PVertex &lastVertex() { return vertices.back(); }

  std::vector<PVertex> &vertexes() { return vertices; }
  std::vector<PVertex> const &vertexes() const { return vertices; }

  /// Iterate the segment indices of the polyline. visitor function is invoked for each segment
  /// index pair, stops when all indices have been visited or visitor returns false. visitor
  /// signature is bool(std::size_t, std::size_t).
  template <typename VisitorF> void visitSegIndices(VisitorF &&visitor) const {
    if (vertices.size() < 2) {
      return;
    }
    std::size_t i;
    std::size_t j;
    if (isClosedVal) {
      i = 0;
      j = vertices.size() - 1;
    } else {
      i = 1;
      j = 0;
    }

    while (i < vertices.size() && visitor(j, i)) {
      j = i;
      i = i + 1;
    }
  }

private:
  bool isClosedVal;
  std::vector<PVertex> vertices;
};

class Polyline2_5D {
public:
  /// Construct an empty open polyline.
  Polyline2_5D() : isClosedVal(false), vertices() {}

  Polyline2_5D(Polyline2D& polyline, Plane& plane) {
    insertPolyLine2D(polyline, plane);
  }

  Polyline2_5D(Polyline2_5D& polyline) {
    insertPolyLine2_5D(polyline);
  }

  void insertPolyLine2D(Polyline2D& polyline, Plane& plane) {
    vertices.clear();

    isClosedVal = polyline.isClosed();

    std::vector<PlineVertex2D>& inputVertices = polyline.vertexes();
    for (size_t i = 0; i < inputVertices.size(); i++)
    {
      PlineVertex2_5D new3DVertex;
      new3DVertex.bulge = inputVertices[i].bulge();
      new3DVertex.point = plane.getGlobalCoords(inputVertices[i].pos());
      new3DVertex.plane = plane;

      vertices.push_back(new3DVertex);
    }
  }

  void insertPolyLine2_5D(Polyline2_5D& polyline) {
    vertices.clear();

    isClosedVal = polyline.isClosed();
    vertices.insert(vertices.end(), polyline.vertexes().begin(), polyline.vertexes().end());
  }

  void addPolyline2_5D(Polyline2_5D& polyline) {
    if(vertices.size() == 0) {
      insertPolyLine2_5D(polyline);
    } else {
      isClosedVal = false;

      vertices.insert(vertices.end(), polyline.vertexes().begin(), polyline.vertexes().end());
    }
  }

  void movePolyline(Vector3d translation) {
    for (PlineVertex2_5D& vertex : vertices) {
      vertex.point += translation;
      vertex.plane.origin += translation;
    }
  }

  bool& isClosed() {
    return isClosedVal;
  }

  std::vector<PlineVertex2_5D>& vertexes() {
    return vertices;
  }

  bool isEmpty() {
    return vertices.size() == 0;
  }

private:
  bool isClosedVal;
  std::vector<PlineVertex2_5D> vertices;
};

/// Scale X and Y of polyline by scaleFactor.
inline void scalePolyline(Polyline2D &pline, double scaleFactor) {
  for (auto &v : pline.vertexes()) {
    v = PlineVertex2D(scaleFactor * v.pos(), v.bulge());
  }
}

/// Translate the polyline by some offset vector.
inline void translatePolyline(Polyline2D &pline, Vector2d const &offset) {
  for (auto &v : pline.vertexes()) {
    v = PlineVertex2D(offset.x() + v.x(), offset.y() + v.y(), v.bulge());
  }
}

/// Compute the extents of a polyline, if there are no vertexes than -infinity to infinity bounding
/// box is returned.
inline AABB getExtents(Polyline2D const &pline) {
  if (pline.size() == 0) {
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  }

  AABB result{pline[0].x(), pline[0].y(), pline[0].x(), pline[0].y()};

  auto visitor = [&](std::size_t i, std::size_t j) {
    PlineVertex2D const &v1 = pline[i];
    if (v1.bulgeIsZero()) {
      if (v1.x() < result.xMin)
        result.xMin = v1.x();
      if (v1.y() < result.yMin)
        result.yMin = v1.y();
      if (v1.x() > result.xMax)
        result.xMax = v1.x();
      if (v1.y() > result.yMax)
        result.yMax = v1.y();
    } else {
      PlineVertex2D const &v2 = pline[j];
      auto arc = arcRadiusAndCenter(v1, v2);

      double startAngle = angle(arc.center, v1.pos());
      double endAngle = angle(arc.center, v2.pos());
      double sweepAngle = utils::deltaAngle(startAngle, endAngle);

      double arcXMin, arcYMin, arcXMax, arcYMax;

      // crosses PI/2
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, 0.5f * utils::pi<double>())) {
        arcYMax = arc.center.y() + arc.radius;
      } else {
        arcYMax = std::max(v1.y(), v2.y());
      }

      // crosses PI
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, utils::pi<double>())) {
        arcXMin = arc.center.x() - arc.radius;
      } else {
        arcXMin = std::min(v1.x(), v2.x());
      }

      // crosses 3PI/2
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, 1.5f * utils::pi<double>())) {
        arcYMin = arc.center.y() - arc.radius;
      } else {
        arcYMin = std::min(v1.y(), v2.y());
      }

      // crosses 2PI
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, 2.f * utils::pi<double>())) {
        arcXMax = arc.center.x() + arc.radius;
      } else {
        arcXMax = std::max(v1.x(), v2.x());
      }

      if (arcXMin < result.xMin)
        result.xMin = arcXMin;
      if (arcYMin < result.yMin)
        result.yMin = arcYMin;
      if (arcXMax > result.xMax)
        result.xMax = arcXMax;
      if (arcYMax > result.yMax)
        result.yMax = arcYMax;
    }

    // return true to iterate all segments
    return true;
  };

  pline.visitSegIndices(visitor);

  return result;
}

/// Compute the area of a closed polyline, assumes no self intersects, returns positive number if
/// polyline direction is counter clockwise, negative if clockwise, zero if not closed
inline double getArea(Polyline2D const &pline) {
  // Implementation notes:
  // Using the shoelace formula (https://en.wikipedia.org/wiki/Shoelace_formula) modified to support
  // arcs defined by a bulge value. The shoelace formula returns a negative value for clockwise
  // oriented polygons and positive value for counter clockwise oriented polygons. The area of each
  // circular segment defined by arcs is then added if it is a counter clockwise arc or subtracted
  // if it is a clockwise arc. The area of the circular segments are computed by finding the area of
  // the arc sector minus the area of the triangle defined by the chord and center of circle.
  // See https://en.wikipedia.org/wiki/Circular_segment
  if (!pline.isClosed() || pline.size() < 2) {
    return 0.f;
  }

  double doubleAreaTotal = 0.f;

  auto visitor = [&](std::size_t i, std::size_t j) {
    double doubleArea = pline[i].x() * pline[j].y() - pline[i].y() * pline[j].x();
    if (!pline[i].bulgeIsZero()) {
      // add arc segment area
      double b = std::abs(pline[i].bulge());
      double sweepAngle = 4.f * std::atan(b);
      double triangleBase = (pline[j].pos() - pline[i].pos()).norm();
      double radius = triangleBase * (b * b + 1.f) / (4.f * b);
      double sagitta = b * triangleBase / 2.f;
      double triangleHeight = radius - sagitta;
      double doubleSectorArea = sweepAngle * radius * radius;
      double doubleTriangleArea = triangleBase * triangleHeight;
      double doubleArcSegArea = doubleSectorArea - doubleTriangleArea;
      if (pline[i].bulgeIsNeg()) {
        doubleArcSegArea = -doubleArcSegArea;
      }

      doubleArea += doubleArcSegArea;
    }

    doubleAreaTotal += doubleArea;

    // iterate all segments
    return true;
  };

  pline.visitSegIndices(visitor);

  return doubleAreaTotal / 2.f;
}

inline bool isPathClockwise(Polyline2D const& pline) {
  return getArea(pline) < 0.f;
}

/// Class to compute the closest point and starting vertex index from a polyline to a point given.
class ClosestPoint {
public:
  /// Constructs the object to hold the results and performs the computation
  explicit ClosestPoint(Polyline2D const &pline, Vector2d const &point) {
    compute(pline, point);
  }

  void compute(Polyline2D const &pline, Vector2d const &point) {
    CORE_ASSERT(pline.vertexes().size() > 0, "empty polyline has no closest point");
    if (pline.vertexes().size() == 1) {
      m_index = 0;
      m_distance = (point - pline[0].pos()).norm();
      m_point = pline[0].pos();
      return;
    }

    m_distance = std::numeric_limits<double>::infinity();

    auto visitor = [&](std::size_t i, std::size_t j) {
      Vector2d cp = closestPointOnSeg(pline[i], pline[j], point);
      auto diffVec = point - cp;
      double dist2 = diffVec.dot(diffVec);
      if (dist2 < m_distance) {
        m_index = i;
        m_point = cp;
        m_distance = dist2;
      }

      // iterate all segments
      return true;
    };

    pline.visitSegIndices(visitor);
    // check if index is offset (due to point being ontop of vertex)
    std::size_t nextIndex = utils::nextWrappingIndex(m_index, pline);
    if (fuzzyEqual(m_point, pline[nextIndex].pos())) {
      m_index = nextIndex;
    }
    if (!pline.isClosed() && pline.size() > 1 && m_index == pline.size() - 1) {
      m_index -= 1;
    }
    // we used the squared distance while iterating and comparing, take sqrt for actual distance
    m_distance = std::sqrt(m_distance);
  }

  /// Starting vertex index of the segment that has the closest point
  std::size_t index() const { return m_index; }
  /// The closest point
  Vector2d const &point() const { return m_point; }
  /// Distance between the points
  double distance() const { return m_distance; }

private:
  std::size_t m_index = 0;
  Vector2d m_point = Vector2d::Zero();
  double m_distance;
};

/// Returns a new polyline with all arc segments converted to line segments, error is the maximum
/// distance from any line segment to the arc it is approximating. Line segments are circumscribed
/// by the arc (all end points lie on the arc path).
inline Polyline2D convertArcsToLines(Polyline2D const &pline, double error) {
  core::Polyline2D result;
  result.isClosed() = pline.isClosed();
  auto visitor = [&](std::size_t i, std::size_t j) {
    const auto &v1 = pline[i];
    const auto &v2 = pline[j];
    if (v1.bulgeIsZero()) {
      result.addVertex(v1);
    } else {
      auto arc = arcRadiusAndCenter(v1, v2);
      if (arc.radius < error + utils::realThreshold<double>()) {
        result.addVertex(v1);
        return true;
      }

      auto startAngle = angle(arc.center, v1.pos());
      auto endAngle = angle(arc.center, v2.pos());
      double deltaAngle = std::abs(core::utils::deltaAngle(startAngle, endAngle));

      error = std::abs(error);
      double segmentSubAngle = std::abs(2.f * std::acos(1.f - error / arc.radius));
      std::size_t segmentCount = static_cast<std::size_t>(std::ceil(deltaAngle / segmentSubAngle));
      // update segment subangle for equal length segments
      segmentSubAngle = deltaAngle / segmentCount;

      if (v1.bulgeIsNeg()) {
        segmentSubAngle = -segmentSubAngle;
      }
      // add the start point
      result.addVertex(v1.x(), v1.y(), 0.0);
      // add the remaining points
      for (std::size_t i = 1; i < segmentCount; ++i) {
        double angle = i * segmentSubAngle + startAngle;
        result.addVertex(arc.radius * std::cos(angle) + arc.center.x(),
                         arc.radius * std::sin(angle) + arc.center.y(), 0);
      }
    }

    return true;
  };

  pline.visitSegIndices(visitor);
  if (!pline.isClosed()) {
    result.addVertex(pline.lastVertex());
  }

  return result;
}

/// Returns a new polyline with all singularities (repeating vertex positions) from the polyline
/// given removed.
inline Polyline2D pruneSingularities(Polyline2D const &pline, double epsilon) {
  Polyline2D result;
  result.isClosed() = pline.isClosed();

  if (pline.size() == 0) {
    return result;
  }

  // allocate up front (most of the time the number of repeated positions are much less than the
  // total number of vertexes so we're not using very much more memory than required)
  result.vertexes().reserve(pline.size());

  result.addVertex(pline[0]);

  for (std::size_t i = 1; i < pline.size(); ++i) {
    if (fuzzyEqual(result.lastVertex().pos(), pline[i].pos(), epsilon)) {
      result.lastVertex().bulge() = pline[i].bulge();
    } else {
      result.addVertex(pline[i]);
    }
  }

  if (result.isClosed() && result.size() > 1) {
    if (fuzzyEqual(result.lastVertex().pos(), result[0].pos(), epsilon)) {
      result.vertexes().pop_back();
    }
  }

  return result;
}

/// Inverts the direction of the polyline given. If polyline is closed then this just changes the
/// direction from clockwise to counter clockwise, if polyline is open then the starting vertex
/// becomes the end vertex and the end vertex becomes the starting vertex.
inline void invertDirection(Polyline2D &pline) {
  if (pline.size() < 2) {
    return;
  }
  std::reverse(std::begin(pline.vertexes()), std::end(pline.vertexes()));

  // shift and negate bulge (to maintain same geometric path)
  double firstBulge = pline[0].bulge();

  for (std::size_t i = 1; i < pline.size(); ++i) {
    pline[i - 1].bulge() = -pline[i].bulge();
  }

  pline.lastVertex().bulge() = -firstBulge;
}

/// Creates an approximate spatial index for all the segments in the polyline given using
/// createFastApproxBoundingBox.
inline StaticSpatialIndex createApproxSpatialIndex(Polyline2D const &pline) {
  CORE_ASSERT(pline.size() > 1, "need at least 2 vertexes to form segments for spatial index");

  std::size_t segmentCount = pline.isClosed() ? pline.size() : pline.size() - 1;
  StaticSpatialIndex result(segmentCount);

  for (std::size_t i = 0; i < pline.size() - 1; ++i) {
    AABB approxBB = createFastApproxBoundingBox(pline[i], pline[i + 1]);
    result.add(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax);
  }

  if (pline.isClosed()) {
    // add final segment from last to first
    AABB approxBB = createFastApproxBoundingBox(pline.lastVertex(), pline[0]);
    result.add(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax);
  }

  result.finish();

  return result;
}

/// Calculate the total path length of a polyline.
inline double getPathLength(Polyline2D const &pline) {
  if (pline.size() < 2) {
    return 0.f;
  }
  double result = 0.f;
  auto visitor = [&](std::size_t i, std::size_t j) {
    result += segLength(pline[i], pline[j]);
    return true;
  };

  pline.visitSegIndices(visitor);

  return result;
}

/// Compute the winding number for the point in relation to the polyline. If polyline is open and
/// the first vertex does not overlap the last vertex then 0 is always returned. This algorithm is
/// adapted from http://geomalgorithms.com/a03-_inclusion.html to support arc segments. NOTE: The
/// result is not defined if the point lies ontop of the polyline.
inline int getWindingNumber(Polyline2D const &pline, Vector2d const &point) {
  if (!pline.isClosed() || pline.size() < 2) {
    return 0;
  }

  int windingNumber = 0;

  auto lineVisitor = [&](const auto &v1, const auto &v2) {
    if (v1.y() <= point.y()) {
      if (v2.y() > point.y() && isLeft(v1.pos(), v2.pos(), point))
        // left and upward crossing
        windingNumber += 1;
    } else if (v2.y() <= point.y() && !(isLeft(v1.pos(), v2.pos(), point))) {
      // right and downward crossing
      windingNumber -= 1;
    }
  };

  // Helper function to determine if point is inside an arc sector area
  auto distToArcCenterLessThanRadius = [](const auto &v1, const auto &v2, const auto &pt) {
    auto arc = arcRadiusAndCenter(v1, v2);
    double dist2 = distSquared(arc.center, pt);
    return dist2 < arc.radius * arc.radius;
  };

  auto arcVisitor = [&](const auto &v1, const auto &v2) {
    bool isCCW = v1.bulgeIsPos();
    // to robustly handle the case where point is on the chord of an x axis aligned arc we must
    // count it as left going one direction and not left going the other (similar to using <= for
    // end points)
    bool pointIsLeft =
        isCCW ? isLeft(v1.pos(), v2.pos(), point) : isLeftOrEqual(v1.pos(), v2.pos(), point);

    if (v1.y() <= point.y()) {
      if (v2.y() > point.y()) {
        // upward crossing of arc chord
        if (isCCW) {
          if (pointIsLeft) {
            // counter clockwise arc left of chord
            windingNumber += 1;
          } else {
            // counter clockwise arc right of chord
            if (distToArcCenterLessThanRadius(v1, v2, point)) {
              windingNumber += 1;
            }
          }
        } else {
          if (pointIsLeft) {
            // clockwise arc left of chord
            if (!distToArcCenterLessThanRadius(v1, v2, point)) {
              windingNumber += 1;
            }
          }
          // else clockwise arc right of chord, no crossing
        }
      } else {
        // not crossing arc chord and chord is below, check if point is inside arc sector
        if (isCCW && !pointIsLeft) {
          if (v2.x() < point.x() && point.x() < v1.x() &&
              distToArcCenterLessThanRadius(v1, v2, point)) {
            windingNumber += 1;
          }
        } else if (!isCCW && pointIsLeft) {
          if (v1.x() < point.x() && point.x() < v2.x() &&
              distToArcCenterLessThanRadius(v1, v2, point)) {
            windingNumber -= 1;
          }
        }
      }
    } else {
      if (v2.y() <= point.y()) {
        // downward crossing of arc chord
        if (isCCW) {
          if (!pointIsLeft) {
            // counter clockwise arc right of chord
            if (!distToArcCenterLessThanRadius(v1, v2, point)) {
              windingNumber -= 1;
            }
          }
          // else counter clockwise arc left of chord, no crossing
        } else {
          if (pointIsLeft) {
            // clockwise arc left of chord
            if (distToArcCenterLessThanRadius(v1, v2, point)) {
              windingNumber -= 1;
            }
          } else {
            // clockwise arc right of chord
            windingNumber -= 1;
          }
        }
      } else {
        // not crossing arc chord and chord is above, check if point is inside arc sector
        if (isCCW && !pointIsLeft) {
          if (v1.x() < point.x() && point.x() < v2.x() &&
              distToArcCenterLessThanRadius(v1, v2, point)) {
            windingNumber += 1;
          }
        } else if (!isCCW && pointIsLeft) {
          if (v2.x() < point.x() && point.x() < v1.x() &&
              distToArcCenterLessThanRadius(v1, v2, point)) {
            windingNumber -= 1;
          }
        }
      }
    }
  };

  auto visitor = [&](std::size_t i, std::size_t j) {
    const auto &v1 = pline[i];
    const auto &v2 = pline[j];
    if (v1.bulgeIsZero()) {
      lineVisitor(v1, v2);
    } else {
      arcVisitor(v1, v2);
    }

    return true;
  };

  pline.visitSegIndices(visitor);

  return windingNumber;
}



namespace internal {
inline void addOrReplaceIfSamePos(Polyline2D &pline, PlineVertex2D const &vertex,
                           double epsilon = utils::realPrecision<double>()) {
  if (pline.size() == 0) {
    pline.addVertex(vertex);
    return;
  }

  if (fuzzyEqual(pline.lastVertex().pos(), vertex.pos(), epsilon)) {
    pline.lastVertex().bulge() = vertex.bulge();
    return;
  }

  pline.addVertex(vertex);
}
} // namespace internal
} // namespace core

#endif // CORE_POLYLINE_HPP
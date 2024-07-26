#ifndef CAVC_POLYLINE_HPP
#define CAVC_POLYLINE_HPP
#include "intrcircle2circle2.hpp"
#include "intrlineseg2circle2.hpp"
#include "intrlineseg2lineseg2.hpp"
#include "plinesegment.hpp"
#include "staticspatialindex.hpp"
#include "vector2.hpp"
#include "plane.hpp"

#include <algorithm>
#include <vector>
#include <utility>
#include <algorithm>

// This header has the polyline definition and common computation functions that work with polylines
// (e.g. path length, area, extents, etc.)

namespace cavc {
template <typename Real> class Polyline2D {
public:

  // when the polyline is closed, 'isPocket' signifies if the area enclosed by the polyline is desired material, or material to be removed
  bool isPocket = false;

  /// Construct an empty open polyline.
  Polyline2D() : m_isClosed(false), m_vertexes() {}

  using PVertex = PlineVertex<Real>;
  inline PVertex const &operator[](std::size_t i) const { return m_vertexes[i]; }
  inline PVertex &operator[](std::size_t i) { return m_vertexes[i]; }

  bool isClosed() const { return m_isClosed; }
  bool &isClosed() { return m_isClosed; }

  void invertDirection() {
    if (m_vertexes.size() < 2) {
      return;
    }

    std::reverse(std::begin(m_vertexes), std::end(m_vertexes));
    
    double firstBulge = m_vertexes[0].bulge();

    for (size_t i = 1; i < m_vertexes.size(); ++i)
      m_vertexes[i - 1].bulge() = -m_vertexes[i].bulge();

    m_vertexes.back().bulge() = -firstBulge;
  }

  void addVertex(Real x, Real y, Real bulge) { m_vertexes.emplace_back(x, y, bulge); }
  void addVertex(PVertex vertex) { addVertex(vertex.x(), vertex.y(), vertex.bulge()); }

  std::size_t size() const { return m_vertexes.size(); }

  PVertex const &lastVertex() const { return m_vertexes.back(); }
  PVertex &lastVertex() { return m_vertexes.back(); }

  std::vector<PVertex> &vertexes() { return m_vertexes; }
  std::vector<PVertex> const &vertexes() const { return m_vertexes; }

  /// Iterate the segment indices of the polyline. visitor function is invoked for each segment
  /// index pair, stops when all indices have been visited or visitor returns false. visitor
  /// signature is bool(std::size_t, std::size_t).
  template <typename VisitorF> void visitSegIndices(VisitorF &&visitor) const {
    if (m_vertexes.size() < 2) {
      return;
    }
    std::size_t i;
    std::size_t j;
    if (m_isClosed) {
      i = 0;
      j = m_vertexes.size() - 1;
    } else {
      i = 1;
      j = 0;
    }

    while (i < m_vertexes.size() && visitor(j, i)) {
      j = i;
      i = i + 1;
    }
  }

private:
  bool m_isClosed;
  std::vector<PVertex> m_vertexes;
};

/// Scale X and Y of polyline by scaleFactor.
template <typename Real> void scalePolyline(Polyline2D<Real> &pline, Real scaleFactor) {
  for (auto &v : pline.vertexes()) {
    v = PlineVertex<Real>(scaleFactor * v.pos(), v.bulge());
  }
}

/// Translate the polyline by some offset vector.
template <typename Real>
void translatePolyline(Polyline2D<Real> &pline, Vector2<Real> const &offset) {
  for (auto &v : pline.vertexes()) {
    v = PlineVertex<Real>(offset.x() + v.x(), offset.y() + v.y(), v.bulge());
  }
}

/// Compute the extents of a polyline, if there are no vertexes than -infinity to infinity bounding
/// box is returned.
template <typename Real> AABB<Real> getExtents(Polyline2D<Real> const &pline) {
  if (pline.size() == 0) {
    return {std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::infinity(),
            -std::numeric_limits<Real>::infinity(), -std::numeric_limits<Real>::infinity()};
  }

  AABB<Real> result{pline[0].x(), pline[0].y(), pline[0].x(), pline[0].y()};

  auto visitor = [&](std::size_t i, std::size_t j) {
    PlineVertex<Real> const &v1 = pline[i];
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
      PlineVertex<Real> const &v2 = pline[j];
      auto arc = arcRadiusAndCenter(v1, v2);

      Real startAngle = angle(arc.center, v1.pos());
      Real endAngle = angle(arc.center, v2.pos());
      Real sweepAngle = utils::deltaAngle(startAngle, endAngle);

      Real arcXMin, arcYMin, arcXMax, arcYMax;

      // crosses PI/2
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, Real(0.5) * utils::pi<Real>())) {
        arcYMax = arc.center.y() + arc.radius;
      } else {
        arcYMax = std::max(v1.y(), v2.y());
      }

      // crosses PI
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, utils::pi<Real>())) {
        arcXMin = arc.center.x() - arc.radius;
      } else {
        arcXMin = std::min(v1.x(), v2.x());
      }

      // crosses 3PI/2
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, Real(1.5) * utils::pi<Real>())) {
        arcYMin = arc.center.y() - arc.radius;
      } else {
        arcYMin = std::min(v1.y(), v2.y());
      }

      // crosses 2PI
      if (utils::angleIsWithinSweep(startAngle, sweepAngle, Real(2) * utils::pi<Real>())) {
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
template <typename Real> Real getArea(Polyline2D<Real> const &pline) {
  // Implementation notes:
  // Using the shoelace formula (https://en.wikipedia.org/wiki/Shoelace_formula) modified to support
  // arcs defined by a bulge value. The shoelace formula returns a negative value for clockwise
  // oriented polygons and positive value for counter clockwise oriented polygons. The area of each
  // circular segment defined by arcs is then added if it is a counter clockwise arc or subtracted
  // if it is a clockwise arc. The area of the circular segments are computed by finding the area of
  // the arc sector minus the area of the triangle defined by the chord and center of circle.
  // See https://en.wikipedia.org/wiki/Circular_segment
  if (!pline.isClosed() || pline.size() < 2) {
    return Real(0);
  }

  Real doubleAreaTotal = Real(0);

  auto visitor = [&](std::size_t i, std::size_t j) {
    Real doubleArea = pline[i].x() * pline[j].y() - pline[i].y() * pline[j].x();
    if (!pline[i].bulgeIsZero()) {
      // add arc segment area
      Real b = std::abs(pline[i].bulge());
      Real sweepAngle = Real(4) * std::atan(b);
      Real triangleBase = length(pline[j].pos() - pline[i].pos());
      Real radius = triangleBase * (b * b + Real(1)) / (Real(4) * b);
      Real sagitta = b * triangleBase / Real(2);
      Real triangleHeight = radius - sagitta;
      Real doubleSectorArea = sweepAngle * radius * radius;
      Real doubleTriangleArea = triangleBase * triangleHeight;
      Real doubleArcSegArea = doubleSectorArea - doubleTriangleArea;
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

  return doubleAreaTotal / Real(2);
}

template <typename Real> bool isPathClockwise(Polyline2D<Real> const& pline) {
  return getArea(pline) < Real(0);
}

/// Class to compute the closest point and starting vertex index from a polyline to a point given.
template <typename Real> class ClosestPoint {
public:
  /// Constructs the object to hold the results and performs the computation
  explicit ClosestPoint(Polyline2D<Real> const &pline, Vector2<Real> const &point) {
    compute(pline, point);
  }

  void compute(Polyline2D<Real> const &pline, Vector2<Real> const &point) {
    CAVC_ASSERT(pline.vertexes().size() > 0, "empty polyline has no closest point");
    if (pline.vertexes().size() == 1) {
      m_index = 0;
      m_distance = length(point - pline[0].pos());
      m_point = pline[0].pos();
      return;
    }

    m_distance = std::numeric_limits<Real>::infinity();

    auto visitor = [&](std::size_t i, std::size_t j) {
      Vector2<Real> cp = closestPointOnSeg(pline[i], pline[j], point);
      auto diffVec = point - cp;
      Real dist2 = dot(diffVec, diffVec);
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
  Vector2<Real> const &point() const { return m_point; }
  /// Distance between the points
  Real distance() const { return m_distance; }

private:
  std::size_t m_index = 0;
  Vector2<Real> m_point = Vector2<Real>::zero();
  Real m_distance;
};

/// Returns a new polyline with all arc segments converted to line segments, error is the maximum
/// distance from any line segment to the arc it is approximating. Line segments are circumscribed
/// by the arc (all end points lie on the arc path).
template <typename Real>
Polyline2D<Real> convertArcsToLines(Polyline2D<Real> const &pline, Real error) {
  cavc::Polyline2D<Real> result;
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
      Real deltaAngle = std::abs(cavc::utils::deltaAngle(startAngle, endAngle));

      error = std::abs(error);
      Real segmentSubAngle = std::abs(Real(2) * std::acos(Real(1) - error / arc.radius));
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
        Real angle = i * segmentSubAngle + startAngle;
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
template <typename Real>
Polyline2D<Real> pruneSingularities(Polyline2D<Real> const &pline, Real epsilon) {
  Polyline2D<Real> result;
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
template <typename Real> void invertDirection(Polyline2D<Real> &pline) {
  if (pline.size() < 2) {
    return;
  }
  std::reverse(std::begin(pline.vertexes()), std::end(pline.vertexes()));

  // shift and negate bulge (to maintain same geometric path)
  Real firstBulge = pline[0].bulge();

  for (std::size_t i = 1; i < pline.size(); ++i) {
    pline[i - 1].bulge() = -pline[i].bulge();
  }

  pline.lastVertex().bulge() = -firstBulge;
}

/// Creates an approximate spatial index for all the segments in the polyline given using
/// createFastApproxBoundingBox.
template <typename Real>
StaticSpatialIndex<Real> createApproxSpatialIndex(Polyline2D<Real> const &pline) {
  CAVC_ASSERT(pline.size() > 1, "need at least 2 vertexes to form segments for spatial index");

  std::size_t segmentCount = pline.isClosed() ? pline.size() : pline.size() - 1;
  StaticSpatialIndex<Real> result(segmentCount);

  for (std::size_t i = 0; i < pline.size() - 1; ++i) {
    AABB<Real> approxBB = createFastApproxBoundingBox(pline[i], pline[i + 1]);
    result.add(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax);
  }

  if (pline.isClosed()) {
    // add final segment from last to first
    AABB<Real> approxBB = createFastApproxBoundingBox(pline.lastVertex(), pline[0]);
    result.add(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax);
  }

  result.finish();

  return result;
}

/// Calculate the total path length of a polyline.
template <typename Real> Real getPathLength(Polyline2D<Real> const &pline) {
  if (pline.size() < 2) {
    return Real(0);
  }
  Real result = Real(0);
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
template <typename Real>
int getWindingNumber(Polyline2D<Real> const &pline, Vector2<Real> const &point) {
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
    Real dist2 = distSquared(arc.center, pt);
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

template <typename Real> class PlineVertex3D {
public:
  PlineVertex3D() {
    bulge = 0.f;
    point = Vector3<Real>(0.f, 0.f, 0.f);
    plane = Plane<Real>();
    traverse = false;
  }

  PlineVertex3D(Real newBulge, Vector3<Real> newPoint, bool traverse = false) {
    plane = Plane<Real>();

    bulge = newBulge;
    point = newPoint;
    traverse = false;
  }

  PlineVertex3D(Real newBulge, Vector3<Real> newPoint, Plane<Real> newPlane, bool traverse = false) {
    plane = newPlane;
    bulge = newBulge;
    point = newPoint;
    traverse = false;
  }

  PlineVertex3D(Vector3<Real> newPoint, bool traverse = false) {
    plane = Plane<Real>();
    bulge = 0.f;
    point = newPoint;
    traverse = false;
  }

  PlineVertex<double> getVertexInPlaneCoords() {
    PlineVertex<double> result;
    result.bulge() = bulge;
    result.pos() = plane.getLocalCoords(point);
    return result;
  }

  Vector2<Real> getPointInPlaneCoords() {
    return plane.getLocalCoords(point);
  }

  bool bulgeIsZero(Real epsilon = utils::realPrecision<Real>()) const {
    return std::abs(bulge) < epsilon;
  }

  Real bulge;
  Vector3<Real> point;
  Plane<Real> plane;
  bool traverse;
};

template <typename Real> class Polyline3D {
public:
  /// Construct an empty open polyline.
  Polyline3D() : m_isClosed(false), m_vertexes() {}

  Polyline3D(Polyline2D<Real>& polyLine, Plane<Real>& plane) {
    insertPolyLine2D(polyLine, plane);
  }

  void insertPolyLine2D(Polyline2D<Real>& polyLine, Plane<Real>& plane) {
    m_vertexes.clear();

    m_isClosed = polyLine.isClosed();

    std::vector<PlineVertex<Real>>& vertices = polyLine.vertexes();
    for (size_t i = 0; i < vertices.size(); i++)
    {
      PlineVertex3D<Real> new3DVertex;
      new3DVertex.bulge = vertices[i].bulge();
      new3DVertex.point = plane.getGlobalCoords(vertices[i].pos());
      new3DVertex.plane = plane;

      m_vertexes.push_back(new3DVertex);
    }
  }

  void insertPolyLine3D(Polyline3D<Real>& polyLine) {
    m_vertexes.insert(m_vertexes.end(), polyLine.vertexes().begin(), polyLine.vertexes().end());
  }

  bool& isClosed() {
    return m_isClosed;
  }

  std::vector<PlineVertex3D<Real>>& vertexes() {
    return m_vertexes;
  }

private:
  bool m_isClosed;
  std::vector<PlineVertex3D<Real>> m_vertexes;
};

namespace internal {
template <typename Real>
void addOrReplaceIfSamePos(Polyline2D<Real> &pline, PlineVertex<Real> const &vertex,
                           Real epsilon = utils::realPrecision<Real>()) {
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
} // namespace cavc

#endif // CAVC_POLYLINE_HPP

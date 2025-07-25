#ifndef POLYLINE_OFFSET_HPP
#define POLYLINE_OFFSET_HPP

#include "../polyline.hpp"
#include "../intersect/polylineIntersects.hpp"

#include <unordered_map>
#include <map>
#include <vector>

// This header has functions for offsetting polylines

namespace core {
namespace internal {
/// Represents a raw polyline offset segment.
struct PlineOffsetSegment {
  PlineVertex2D v1;
  PlineVertex2D v2;
  Vector2d origV2Pos;
  bool collapsedArc;
};

/// Creates all the raw polyline offset segments.
inline std::vector<PlineOffsetSegment> createUntrimmedOffsetSegments(Polyline2D const &pline,
                                                                    double offset) {
  std::size_t segmentCount = pline.isClosed() ? pline.size() : pline.size() - 1;

  std::vector<PlineOffsetSegment> result;
  result.reserve(segmentCount);

  auto lineVisitor = [&](PlineVertex2D const &v1, PlineVertex2D const &v2) {
    result.emplace_back();
    PlineOffsetSegment &seg = result.back();
    seg.collapsedArc = false;
    seg.origV2Pos = v2.pos();
    Vector2d edge = v2.pos() - v1.pos();
    Vector2d offsetV = offset * unitPerp(edge);
    seg.v1.pos() = v1.pos() + offsetV;
    seg.v1.bulge() = v1.bulge();
    seg.v2.pos() = v2.pos() + offsetV;
    seg.v2.bulge() = v2.bulge();
  };

  auto arcVisitor = [&](PlineVertex2D const &v1, PlineVertex2D const &v2) {
    auto arc = arcRadiusAndCenter(v1, v2);
    double offs = v1.bulgeIsNeg() ? offset : -offset;
    double radiusAfterOffset = arc.radius + offs;
    Vector2d v1ToCenter = v1.pos() - arc.center;
    v1ToCenter.normalize();
    Vector2d v2ToCenter = v2.pos() - arc.center;
    v2ToCenter.normalize();

    result.emplace_back();
    PlineOffsetSegment &seg = result.back();
    seg.origV2Pos = v2.pos();
    seg.v1.pos() = offs * v1ToCenter + v1.pos();
    seg.v2.pos() = offs * v2ToCenter + v2.pos();
    seg.v2.bulge() = v2.bulge();

    if (radiusAfterOffset < utils::realThreshold<double>()) {
      // collapsed arc, offset arc start and end points towards arc center and turn into line
      // handles case where offset vertexes are equal and simplifies path for clipping algorithm
      seg.collapsedArc = true;
      seg.v1.bulge() = 0.f;
    } else {
      seg.collapsedArc = false;
      seg.v1.bulge() = v1.bulge();
    }
  };

  auto offsetVisitor = [&](PlineVertex2D const &v1, PlineVertex2D const &v2) {
    if (v1.bulgeIsZero()) {
      lineVisitor(v1, v2);
    } else {
      arcVisitor(v1, v2);
    }
  };

  for (std::size_t i = 1; i < pline.size(); ++i) {
    offsetVisitor(pline[i - 1], pline[i]);
  }

  if (pline.isClosed()) {
    offsetVisitor(pline.lastVertex(), pline[0]);
  }

  return result;
}

inline bool falseIntersect(double t) { return t < 0.0 || t > 1.0; }

// Gets the bulge to describe the arc going from start point to end point with the given arc center
// and curve orientation, if orientation is negative then bulge is negative otherwise it is positive
inline double bulgeForConnection(Vector2d const &arcCenter, Vector2d const &sp,
                        Vector2d const &ep, bool isCCW) {
  double a1 = angle(arcCenter, sp);
  double a2 = angle(arcCenter, ep);
  double absSweepAngle = std::abs(utils::deltaAngle(a1, a2));
  double absBulge = std::tan(absSweepAngle / 4.f);
  if (isCCW) {
    return absBulge;
  }

  return -absBulge;
}

inline void lineToLineJoin(PlineOffsetSegment const &s1, PlineOffsetSegment const &s2,
                    bool connectionArcsAreCCW, Polyline2D &result) {
  const auto &v1 = s1.v1;
  const auto &v2 = s1.v2;
  const auto &u1 = s2.v1;
  const auto &u2 = s2.v2;
  CORE_ASSERT(v1.bulgeIsZero() && u1.bulgeIsZero(), "both segs should be lines");

  auto connectUsingArc = [&] {
    auto const &arcCenter = s1.origV2Pos;
    auto const &sp = v2.pos();
    auto const &ep = u1.pos();
    double bulge = bulgeForConnection(arcCenter, sp, ep, connectionArcsAreCCW);
    addOrReplaceIfSamePos(result, PlineVertex2D(sp, bulge));
    addOrReplaceIfSamePos(result, PlineVertex2D(ep, 0.f));
  };

  if (s1.collapsedArc || s2.collapsedArc) {
    // connecting to/from collapsed arc, always connect using arc
    connectUsingArc();
  } else {
    auto intrResult = intrLineSeg2LineSeg2(v1.pos(), v2.pos(), u1.pos(), u2.pos());

    switch (intrResult.intrType) {
    case LineSeg2LineSeg2IntrType::None:
      // just join with straight line
      addOrReplaceIfSamePos(result, PlineVertex2D(v2.pos(), 0.f));
      addOrReplaceIfSamePos(result, u1);
      break;
    case LineSeg2LineSeg2IntrType::True:
      addOrReplaceIfSamePos(result, PlineVertex2D(intrResult.point, 0.f));
      break;
    case LineSeg2LineSeg2IntrType::Coincident:
      addOrReplaceIfSamePos(result, PlineVertex2D(v2.pos(), 0.f));
      break;
    case LineSeg2LineSeg2IntrType::False:
      if (intrResult.t0 > 1.f && falseIntersect(intrResult.t1)) {
        // extend and join the lines together using an arc
        connectUsingArc();
      } else {
        addOrReplaceIfSamePos(result, PlineVertex2D(v2.pos(), 0.f));
        addOrReplaceIfSamePos(result, u1);
      }
      break;
    }
  }
}

inline void lineToArcJoin(PlineOffsetSegment const &s1, PlineOffsetSegment const &s2,
                   bool connectionArcsAreCCW, Polyline2D &result) {

  const auto &v1 = s1.v1;
  const auto &v2 = s1.v2;
  const auto &u1 = s2.v1;
  const auto &u2 = s2.v2;
  CORE_ASSERT(v1.bulgeIsZero() && !u1.bulgeIsZero(),
              "first seg should be arc, second seg should be line");

  auto connectUsingArc = [&] {
    auto const &arcCenter = s1.origV2Pos;
    auto const &sp = v2.pos();
    auto const &ep = u1.pos();
    double bulge = bulgeForConnection(arcCenter, sp, ep, connectionArcsAreCCW);
    addOrReplaceIfSamePos(result, PlineVertex2D(sp, bulge));
    addOrReplaceIfSamePos(result, u1);
  };

  const auto arc = arcRadiusAndCenter(u1, u2);

  auto processIntersect = [&](double t, Vector2d const &intersect) {
    const bool trueSegIntersect = !falseIntersect(t);
    const bool trueArcIntersect =
        pointWithinArcSweepAngle(arc.center, u1.pos(), u2.pos(), u1.bulge(), intersect);
    if (trueSegIntersect && trueArcIntersect) {
      // trim at intersect
      double a = angle(arc.center, intersect);
      double arcEndAngle = angle(arc.center, u2.pos());
      double theta = utils::deltaAngle(a, arcEndAngle);
      // ensure the sign matches (may get flipped if intersect is at the very end of the arc, in
      // which case we do not want to update the bulge)
      if ((theta > 0.f) == u1.bulgeIsPos()) {
        addOrReplaceIfSamePos(result, PlineVertex2D(intersect, std::tan(theta / 4.f)));
      } else {
        addOrReplaceIfSamePos(result, PlineVertex2D(intersect, u1.bulge()));
      }
    } else if (t > 1.f && !trueArcIntersect) {
      connectUsingArc();
    } else if (s1.collapsedArc) {
      // collapsed arc connecting to arc, connect using arc
      connectUsingArc();
    } else {
      // connect using line
      addOrReplaceIfSamePos(result, PlineVertex2D(v2.pos(), 0.f));
      addOrReplaceIfSamePos(result, u1);
    }
  };

  auto intrResult = intrLineSeg2Circle2(v1.pos(), v2.pos(), arc.radius, arc.center);
  if (intrResult.numIntersects == 0) {
    connectUsingArc();
  } else if (intrResult.numIntersects == 1) {
    processIntersect(intrResult.t0, pointFromParametric(v1.pos(), v2.pos(), intrResult.t0));
  } else {
    CORE_ASSERT(intrResult.numIntersects == 2, "should have 2 intersects here");
    // always use intersect closest to original point
    Vector2d i1 = pointFromParametric(v1.pos(), v2.pos(), intrResult.t0);
    double dist1 = distSquared(i1, s1.origV2Pos);
    Vector2d i2 = pointFromParametric(v1.pos(), v2.pos(), intrResult.t1);
    double dist2 = distSquared(i2, s1.origV2Pos);

    if (dist1 < dist2) {
      processIntersect(intrResult.t0, i1);
    } else {
      processIntersect(intrResult.t1, i2);
    }
  }
}

inline void arcToLineJoin(PlineOffsetSegment const &s1, PlineOffsetSegment const &s2,
                   bool connectionArcsAreCCW, Polyline2D &result) {

  const auto &v1 = s1.v1;
  const auto &v2 = s1.v2;
  const auto &u1 = s2.v1;
  const auto &u2 = s2.v2;
  CORE_ASSERT(!v1.bulgeIsZero() && u1.bulgeIsZero(),
              "first seg should be line, second seg should be arc");

  auto connectUsingArc = [&] {
    auto const &arcCenter = s1.origV2Pos;
    auto const &sp = v2.pos();
    auto const &ep = u1.pos();
    double bulge = bulgeForConnection(arcCenter, sp, ep, connectionArcsAreCCW);
    addOrReplaceIfSamePos(result, PlineVertex2D(sp, bulge));
    addOrReplaceIfSamePos(result, u1);
  };

  const auto arc = arcRadiusAndCenter(v1, v2);

  auto processIntersect = [&](double t, Vector2d const &intersect) {
    const bool trueSegIntersect = !falseIntersect(t);
    const bool trueArcIntersect =
        pointWithinArcSweepAngle(arc.center, v1.pos(), v2.pos(), v1.bulge(), intersect);
    if (trueSegIntersect && trueArcIntersect) {
      PlineVertex2D &prevVertex = result.lastVertex();

      if (!prevVertex.bulgeIsZero() && !fuzzyEqual(prevVertex.pos(), v2.pos())) {
        // modify previous bulge and trim at intersect
        double a = angle(arc.center, intersect);
        auto prevArc = arcRadiusAndCenter(prevVertex, v2);
        double prevArcStartAngle = angle(prevArc.center, prevVertex.pos());
        double updatedPrevTheta = utils::deltaAngle(prevArcStartAngle, a);

        // ensure the sign matches (may get flipped if intersect is at the very end of the arc, in
        // which case we do not want to update the bulge)
        if ((updatedPrevTheta > 0.f) == prevVertex.bulgeIsPos()) {
          result.lastVertex().bulge() = std::tan(updatedPrevTheta / 4.f);
        }
      }

      addOrReplaceIfSamePos(result, PlineVertex2D(intersect, 0.0));

    } else {
      connectUsingArc();
    }
  };

  auto intrResult = intrLineSeg2Circle2(u1.pos(), u2.pos(), arc.radius, arc.center);
  if (intrResult.numIntersects == 0) {
    connectUsingArc();
  } else if (intrResult.numIntersects == 1) {
    processIntersect(intrResult.t0, pointFromParametric(u1.pos(), u2.pos(), intrResult.t0));
  } else {
    CORE_ASSERT(intrResult.numIntersects == 2, "should have 2 intersects here");
    const auto &origPoint = s2.collapsedArc ? u1.pos() : s1.origV2Pos;
    Vector2d i1 = pointFromParametric(u1.pos(), u2.pos(), intrResult.t0);
    double dist1 = distSquared(i1, origPoint);
    Vector2d i2 = pointFromParametric(u1.pos(), u2.pos(), intrResult.t1);
    double dist2 = distSquared(i2, origPoint);

    if (dist1 < dist2) {
      processIntersect(intrResult.t0, i1);
    } else {
      processIntersect(intrResult.t1, i2);
    }
  }
}

inline void arcToArcJoin(PlineOffsetSegment const &s1, PlineOffsetSegment const &s2,
                  bool connectionArcsAreCCW, Polyline2D &result) {

  const auto &v1 = s1.v1;
  const auto &v2 = s1.v2;
  const auto &u1 = s2.v1;
  const auto &u2 = s2.v2;
  CORE_ASSERT(!v1.bulgeIsZero() && !u1.bulgeIsZero(), "both segs should be arcs");

  const auto arc1 = arcRadiusAndCenter(v1, v2);
  const auto arc2 = arcRadiusAndCenter(u1, u2);

  auto connectUsingArc = [&] {
    auto const &arcCenter = s1.origV2Pos;
    auto const &sp = v2.pos();
    auto const &ep = u1.pos();
    double bulge = bulgeForConnection(arcCenter, sp, ep, connectionArcsAreCCW);
    addOrReplaceIfSamePos(result, PlineVertex2D(sp, bulge));
    addOrReplaceIfSamePos(result, u1);
  };

  auto processIntersect = [&](Vector2d const &intersect) {
    const bool trueArcIntersect1 =
        pointWithinArcSweepAngle(arc1.center, v1.pos(), v2.pos(), v1.bulge(), intersect);
    const bool trueArcIntersect2 =
        pointWithinArcSweepAngle(arc2.center, u1.pos(), u2.pos(), u1.bulge(), intersect);

    if (trueArcIntersect1 && trueArcIntersect2) {
      PlineVertex2D &prevVertex = result.lastVertex();
      if (!prevVertex.bulgeIsZero() && !fuzzyEqual(prevVertex.pos(), v2.pos())) {
        // modify previous bulge and trim at intersect
        double a1 = angle(arc1.center, intersect);
        auto prevArc = arcRadiusAndCenter(prevVertex, v2);
        double prevArcStartAngle = angle(prevArc.center, prevVertex.pos());
        double updatedPrevTheta = utils::deltaAngle(prevArcStartAngle, a1);

        // ensure the sign matches (may get flipped if intersect is at the very end of the arc, in
        // which case we do not want to update the bulge)
        if ((updatedPrevTheta > 0.f) == prevVertex.bulgeIsPos()) {
          result.lastVertex().bulge() = std::tan(updatedPrevTheta / 4.f);
        }
      }

      // add the vertex at our current trim/join point
      double a2 = angle(arc2.center, intersect);
      double endAngle = angle(arc2.center, u2.pos());
      double theta = utils::deltaAngle(a2, endAngle);

      // ensure the sign matches (may get flipped if intersect is at the very end of the arc, in
      // which case we do not want to update the bulge)
      if ((theta > 0.f) == u1.bulgeIsPos()) {
        addOrReplaceIfSamePos(result, PlineVertex2D(intersect, std::tan(theta / 4.f)));
      } else {
        addOrReplaceIfSamePos(result, PlineVertex2D(intersect, u1.bulge()));
      }

    } else {
      connectUsingArc();
    }
  };

  const auto intrResult = intrCircle2Circle2(arc1.radius, arc1.center, arc2.radius, arc2.center);
  switch (intrResult.intrType) {
  case Circle2Circle2IntrType::NoIntersect:
    connectUsingArc();
    break;
  case Circle2Circle2IntrType::OneIntersect:
    processIntersect(intrResult.point1);
    break;
  case Circle2Circle2IntrType::TwoIntersects: {
    double dist1 = distSquared(intrResult.point1, s1.origV2Pos);
    double dist2 = distSquared(intrResult.point2, s1.origV2Pos);
    if (dist1 < dist2) {
      processIntersect(intrResult.point1);
    } else {
      processIntersect(intrResult.point2);
    }
  } break;
  case Circle2Circle2IntrType::Coincident:
    // same constant arc radius and center, just add the vertex (nothing to trim/extend)
    addOrReplaceIfSamePos(result, u1);
    break;
  }
}

inline void offsetCircleIntersectsWithPline(Polyline2D const &pline, double offset,
                                     Vector2d const &circleCenter,
                                     StaticSpatialIndex const &spatialIndex,
                                     std::vector<std::pair<std::size_t, Vector2d>> &output) {

  const double circleRadius = std::abs(offset);

  std::vector<std::size_t> queryResults;

  spatialIndex.query(circleCenter.x() - circleRadius, circleCenter.y() - circleRadius,
                     circleCenter.x() + circleRadius, circleCenter.y() + circleRadius,
                     queryResults);

  auto validLineSegIntersect = [](double t) {
    return !falseIntersect(t) && std::abs(t) > utils::realPrecision<double>();
  };

  auto validArcSegIntersect = [](Vector2d const &arcCenter, Vector2d const &arcStart,
                                 Vector2d const &arcEnd, double bulge,
                                 Vector2d const &intrPoint) {
    return !fuzzyEqual(arcStart, intrPoint, utils::realPrecision<double>()) &&
           pointWithinArcSweepAngle(arcCenter, arcStart, arcEnd, bulge, intrPoint);
  };

  for (std::size_t sIndex : queryResults) {
    PlineVertex2D const &v1 = pline[sIndex];
    PlineVertex2D const &v2 = pline[sIndex + 1];
    if (v1.bulgeIsZero()) {
      IntrLineSeg2Circle2Result intrResult =
          intrLineSeg2Circle2(v1.pos(), v2.pos(), circleRadius, circleCenter);
      if (intrResult.numIntersects == 0) {
        continue;
      } else if (intrResult.numIntersects == 1) {
        if (validLineSegIntersect(intrResult.t0)) {
          output.emplace_back(sIndex, pointFromParametric(v1.pos(), v2.pos(), intrResult.t0));
        }
      } else {
        CORE_ASSERT(intrResult.numIntersects == 2, "should be two intersects here");
        if (validLineSegIntersect(intrResult.t0)) {
          output.emplace_back(sIndex, pointFromParametric(v1.pos(), v2.pos(), intrResult.t0));
        }
        if (validLineSegIntersect(intrResult.t1)) {
          output.emplace_back(sIndex, pointFromParametric(v1.pos(), v2.pos(), intrResult.t1));
        }
      }
    } else {
      auto arc = arcRadiusAndCenter(v1, v2);
      IntrCircle2Circle2Result intrResult =
          intrCircle2Circle2(arc.radius, arc.center, circleRadius, circleCenter);
      switch (intrResult.intrType) {
      case Circle2Circle2IntrType::NoIntersect:
        break;
      case Circle2Circle2IntrType::OneIntersect:
        if (validArcSegIntersect(arc.center, v1.pos(), v2.pos(), v1.bulge(), intrResult.point1)) {
          output.emplace_back(sIndex, intrResult.point1);
        }
        break;
      case Circle2Circle2IntrType::TwoIntersects:
        if (validArcSegIntersect(arc.center, v1.pos(), v2.pos(), v1.bulge(), intrResult.point1)) {
          output.emplace_back(sIndex, intrResult.point1);
        }
        if (validArcSegIntersect(arc.center, v1.pos(), v2.pos(), v1.bulge(), intrResult.point2)) {
          output.emplace_back(sIndex, intrResult.point2);
        }
        break;
      case Circle2Circle2IntrType::Coincident:
        break;
      }
    }
  }
}

/// Function to test if a point is a valid distance from the original polyline.
inline bool pointValidForOffset(Polyline2D const &pline, double offset,
                         StaticSpatialIndex const &spatialIndex,
                         Vector2d const &point, std::vector<std::size_t> &queryStack,
                         double offsetTol = utils::offsetThreshold<double>()) {
  const double absOffset = std::abs(offset) - offsetTol;
  const double minDist = absOffset * absOffset;

  bool pointValid = true;

  auto visitor = [&](std::size_t i) {
    std::size_t j = utils::nextWrappingIndex(i, pline.vertexes());
    auto closestPoint = closestPointOnSeg(pline[i], pline[j], point);
    double dist = distSquared(closestPoint, point);
    pointValid = dist > minDist;
    return pointValid;
  };

  spatialIndex.visitQuery(point.x() - absOffset, point.y() - absOffset, point.x() + absOffset,
                          point.y() + absOffset, visitor, queryStack);
  return pointValid;
}

/// Creates the raw offset polyline.
inline Polyline2D createRawOffsetPline(Polyline2D const &pline, double offset) {

  Polyline2D result;
  if (pline.size() < 2) {
    return result;
  }

  std::vector<PlineOffsetSegment> rawOffsets = createUntrimmedOffsetSegments(pline, offset);
  if (rawOffsets.size() == 0) {
    return result;
  }

  // detect single collapsed arc segment (this may be removed in the future if invalid segments are
  // tracked in join functions to be pruned at slice creation)
  if (rawOffsets.size() == 1 && rawOffsets[0].collapsedArc) {
    return result;
  }

  result.vertexes().reserve(pline.size());
  result.isClosed() = pline.isClosed();

  const bool connectionArcsAreCCW = offset < 0.f;

  auto joinResultVisitor = [connectionArcsAreCCW](PlineOffsetSegment const &s1,
                                                  PlineOffsetSegment const &s2,
                                                  Polyline2D &result) {
    const bool s1IsLine = s1.v1.bulgeIsZero();
    const bool s2IsLine = s2.v1.bulgeIsZero();
    if (s1IsLine && s2IsLine) {
      internal::lineToLineJoin(s1, s2, connectionArcsAreCCW, result);
    } else if (s1IsLine) {
      internal::lineToArcJoin(s1, s2, connectionArcsAreCCW, result);
    } else if (s2IsLine) {
      internal::arcToLineJoin(s1, s2, connectionArcsAreCCW, result);
    } else {
      internal::arcToArcJoin(s1, s2, connectionArcsAreCCW, result);
    }
  };

  result.addVertex(rawOffsets[0].v1);

  // join first two segments and determine if first vertex was replaced (to know how to handle last
  // two segment joins for closed polyline)
  if (rawOffsets.size() > 1) {

    auto const &seg01 = rawOffsets[0];
    auto const &seg12 = rawOffsets[1];
    joinResultVisitor(seg01, seg12, result);
  }
  const bool firstVertexReplaced = result.size() == 1;

  for (std::size_t i = 2; i < rawOffsets.size(); ++i) {
    const auto &seg1 = rawOffsets[i - 1];
    const auto &seg2 = rawOffsets[i];
    joinResultVisitor(seg1, seg2, result);
  }

  if (pline.isClosed() && result.size() > 1) {
    // joining segments at vertex indexes (n, 0) and (0, 1)
    const auto &s1 = rawOffsets.back();
    const auto &s2 = rawOffsets[0];

    // temp polyline to capture results of joining (to avoid mutating result)
    Polyline2D closingPartResult;
    closingPartResult.addVertex(result.lastVertex());
    joinResultVisitor(s1, s2, closingPartResult);

    // update last vertexes
    result.lastVertex() = closingPartResult[0];
    for (std::size_t i = 1; i < closingPartResult.size(); ++i) {
      result.addVertex(closingPartResult[i]);
    }
    result.vertexes().pop_back();

    // update first vertex (only if it has not already been updated/replaced)
    if (!firstVertexReplaced) {
      const Vector2d &updatedFirstPos = closingPartResult.lastVertex().pos();
      if (result[0].bulgeIsZero()) {
        // just update position
        result[0].pos() = updatedFirstPos;
      } else if (result.size() > 1) {
        // update position and bulge
        const auto arc = arcRadiusAndCenter(result[0], result[1]);
        const double a1 = angle(arc.center, updatedFirstPos);
        const double a2 = angle(arc.center, result[1].pos());
        const double updatedTheta = utils::deltaAngle(a1, a2);
        if ((updatedTheta < 0.f && result[0].bulgeIsPos()) ||
            (updatedTheta > 0.f && result[0].bulgeIsNeg())) {
          // first vertex not valid, just update its position to be removed later
          result[0].pos() = updatedFirstPos;
        } else {
          // update position and bulge
          result[0].pos() = updatedFirstPos;
          result[0].bulge() = std::tan(updatedTheta / 4.f);
        }
      }
    }

    // must do final singularity prune between first and second vertex after joining curves (n, 0)
    // and (0, 1)
    if (result.size() > 1) {
      if (fuzzyEqual(result[0].pos(), result[1].pos(), utils::realPrecision<double>())) {
        result.vertexes().erase(result.vertexes().begin());
      }
    }
  } else {
    internal::addOrReplaceIfSamePos(result, rawOffsets.back().v2);
  }

  // if due to joining of segments we are left with only 1 vertex then return no raw offset (empty
  // polyline)
  if (result.size() == 1) {
    result.vertexes().clear();
  }

  return result;
}

/// Represents an open polyline slice of the raw offset polyline.
struct OpenPolylineSlice {
  std::size_t intrStartIndex;
  Polyline2D pline;
  OpenPolylineSlice() = default;
  OpenPolylineSlice(std::size_t sIndex, Polyline2D slice)
      : intrStartIndex(sIndex), pline(std::move(slice)) {}
};

/// Slices a raw offset polyline at all of its self intersects.
inline std::vector<OpenPolylineSlice> slicesFromRawOffset(Polyline2D const &originalPline,
                                                         Polyline2D const &rawOffsetPline,
                                                         double offset) {
  CORE_ASSERT(originalPline.isClosed(), "use dual slice at intersects for open polylines");

  std::vector<OpenPolylineSlice> result;
  if (rawOffsetPline.size() < 2) {
    return result;
  }

  StaticSpatialIndex origPlineSpatialIndex = createApproxSpatialIndex(originalPline);
  StaticSpatialIndex rawOffsetPlineSpatialIndex = createApproxSpatialIndex(rawOffsetPline);

  std::vector<PlineIntersect> selfIntersects;
  allSelfIntersects(rawOffsetPline, selfIntersects, rawOffsetPlineSpatialIndex);

  std::vector<std::size_t> queryStack;
  queryStack.reserve(8);
  if (selfIntersects.size() == 0) {
    if (!pointValidForOffset(originalPline, offset, origPlineSpatialIndex, rawOffsetPline[0].pos(),
                             queryStack)) {
      return result;
    }
    // copy and convert raw offset into open polyline
    result.emplace_back(std::numeric_limits<std::size_t>::max(), rawOffsetPline);
    result.back().pline.isClosed() = false;
    result.back().pline.addVertex(rawOffsetPline[0]);
    result.back().pline.lastVertex().bulge() = 0.f;
    return result;
  }

  // using unordered_map rather than map for performance (as is used in
  // dualSliceAtIntersectsForOffset) since all slices will stitch together to form closed
  // loops/polylines so later when slices are stitched together the order that slices are visited
  // does not matter
  std::unordered_map<std::size_t, std::vector<Vector2d>> intersectsLookup;
  intersectsLookup.reserve(2 * selfIntersects.size());

  for (PlineIntersect const &si : selfIntersects) {
    intersectsLookup[si.sIndex1].push_back(si.pos);
    intersectsLookup[si.sIndex2].push_back(si.pos);
  }

  // sort intersects by distance from start vertex
  for (auto &kvp : intersectsLookup) {
    Vector2d startPos = rawOffsetPline[kvp.first].pos();
    auto cmp = [&](Vector2d const &si1, Vector2d const &si2) {
      return distSquared(si1, startPos) < distSquared(si2, startPos);
    };
    std::sort(kvp.second.begin(), kvp.second.end(), cmp);
  }

  auto intersectsOrigPline = [&](PlineVertex2D const &v1, PlineVertex2D const &v2) {
    AABB approxBB = createFastApproxBoundingBox(v1, v2);
    bool hasIntersect = false;
    auto visitor = [&](std::size_t i) {
      using namespace internal;
      std::size_t j = utils::nextWrappingIndex(i, originalPline);
      IntrPlineSegsResult intrResult =
          intrPlineSegs(v1, v2, originalPline[i], originalPline[j]);
      hasIntersect = intrResult.intrType != PlineSegIntrType::NoIntersect;
      return !hasIntersect;
    };

    origPlineSpatialIndex.visitQuery(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax,
                                     visitor, queryStack);

    return hasIntersect;
  };

  for (auto const &kvp : intersectsLookup) {
    // start index for the slice we're about to build
    std::size_t sIndex = kvp.first;
    // self intersect list for this start index
    std::vector<Vector2d> const &siList = kvp.second;

    const auto &startVertex = rawOffsetPline[sIndex];
    std::size_t nextIndex = utils::nextWrappingIndex(sIndex, rawOffsetPline);
    const auto &endVertex = rawOffsetPline[nextIndex];

    if (siList.size() != 1) {
      // build all the segments between the N intersects in siList (N > 1), skipping the first
      // segment (to be processed at the end)
      SplitResult firstSplit = splitAtPoint(startVertex, endVertex, siList[0]);
      auto prevVertex = firstSplit.splitVertex;
      for (std::size_t i = 1; i < siList.size(); ++i) {
        SplitResult split = splitAtPoint(prevVertex, endVertex, siList[i]);
        // update prevVertex for next loop iteration
        prevVertex = split.splitVertex;
        // skip if they're ontop of each other
        if (fuzzyEqual(split.updatedStart.pos(), split.splitVertex.pos(),
                       utils::realPrecision<double>())) {
          continue;
        }

        // test start point
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           split.updatedStart.pos(), queryStack)) {
          continue;
        }

        // test end point
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           split.splitVertex.pos(), queryStack)) {
          continue;
        }

        // test mid point
        auto midpoint = segMidpoint(split.updatedStart, split.splitVertex);
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, midpoint,
                                           queryStack)) {
          continue;
        }

        // test intersection with original polyline
        if (intersectsOrigPline(split.updatedStart, split.splitVertex)) {
          continue;
        }

        result.emplace_back();
        result.back().intrStartIndex = sIndex;
        result.back().pline.addVertex(split.updatedStart);
        result.back().pline.addVertex(split.splitVertex);
      }
    }

    // build the segment between the last intersect in siList and the next intersect found

    // check that the first point is valid
    if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, siList.back(),
                                       queryStack)) {
      continue;
    }

    SplitResult split = splitAtPoint(startVertex, endVertex, siList.back());
    Polyline2D currSlice;
    currSlice.addVertex(split.splitVertex);

    std::size_t index = nextIndex;
    bool isValidPline = true;
    std::size_t loopCount = 0;
    const std::size_t maxLoopCount = rawOffsetPline.size();
    while (true) {
      if (loopCount++ > maxLoopCount) {
        CORE_ASSERT(false, "Bug detected, should never loop this many times!");
        // break to avoid infinite loop
        break;
      }
      // check that vertex point is valid
      if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                         rawOffsetPline[index].pos(), queryStack)) {
        isValidPline = false;
        break;
      }

      // check that the segment does not intersect original polyline
      if (intersectsOrigPline(currSlice.lastVertex(), rawOffsetPline[index])) {
        isValidPline = false;
        break;
      }

      // add vertex
      internal::addOrReplaceIfSamePos(currSlice, rawOffsetPline[index]);

      // check if segment that starts at vertex we just added has an intersect
      auto nextIntr = intersectsLookup.find(index);
      if (nextIntr != intersectsLookup.end()) {
        // there is an intersect, slice is done, check if final segment is valid

        // check intersect pos is valid (which will also be end vertex position)
        Vector2d const &intersectPos = nextIntr->second[0];
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           intersectPos, queryStack)) {
          isValidPline = false;
          break;
        }

        std::size_t nextIndex = utils::nextWrappingIndex(index, rawOffsetPline);
        SplitResult split =
            splitAtPoint(currSlice.lastVertex(), rawOffsetPline[nextIndex], intersectPos);

        PlineVertex2D sliceEndVertex = PlineVertex2D(intersectPos, 0.f);
        // check mid point is valid
        Vector2d mp = segMidpoint(split.updatedStart, sliceEndVertex);
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, mp,
                                           queryStack)) {
          isValidPline = false;
          break;
        }

        // trim last added vertex and add final intersect position
        currSlice.lastVertex() = split.updatedStart;
        internal::addOrReplaceIfSamePos(currSlice, sliceEndVertex);

        break;
      }
      // else there is not an intersect, increment index and continue
      index = utils::nextWrappingIndex(index, rawOffsetPline);
    }

    isValidPline = isValidPline && currSlice.size() > 1;

    if (isValidPline && fuzzyEqual(currSlice[0].pos(), currSlice.lastVertex().pos())) {
      // discard very short slice loops (invalid loops may arise due to valid offset distance
      // thresholding)
      isValidPline = getPathLength(currSlice) > 0.01f;
    }

    if (isValidPline) {
      result.emplace_back(sIndex, std::move(currSlice));
    }
  }

  return result;
}

/// Slices a raw offset polyline at all of its self intersects and intersects with its dual.
inline std::vector<OpenPolylineSlice>
dualSliceAtIntersectsForOffset(Polyline2D const &originalPline,
                               Polyline2D const &rawOffsetPline,
                               Polyline2D const &dualRawOffsetPline, double offset) {
  std::vector<OpenPolylineSlice> result;
  if (rawOffsetPline.size() < 2) {
    return result;
  }

  StaticSpatialIndex origPlineSpatialIndex = createApproxSpatialIndex(originalPline);
  StaticSpatialIndex rawOffsetPlineSpatialIndex = createApproxSpatialIndex(rawOffsetPline);

  std::vector<PlineIntersect> selfIntersects;
  allSelfIntersects(rawOffsetPline, selfIntersects, rawOffsetPlineSpatialIndex);

  PlineIntersectsResult dualIntersects;
  findIntersects(rawOffsetPline, dualRawOffsetPline, rawOffsetPlineSpatialIndex, dualIntersects);

  // using map rather than unordered map since we want to construct the slices in vertex index order
  // and we do so by looping through all intersects (required later when slices are stitched
  // together, because slices may not all form closed loops/polylines so must go in order of
  // indexes to ensure longest sitched results are formed)
  std::map<std::size_t, std::vector<Vector2d>> intersectsLookup;

  if (!originalPline.isClosed()) {
    // find intersects between circles generated at original open polyline end points and raw offset
    // polyline
    std::vector<std::pair<std::size_t, Vector2d>> intersects;
    internal::offsetCircleIntersectsWithPline(rawOffsetPline, offset, originalPline[0].pos(),
                                              rawOffsetPlineSpatialIndex, intersects);
    internal::offsetCircleIntersectsWithPline(rawOffsetPline, offset,
                                              originalPline.lastVertex().pos(),
                                              rawOffsetPlineSpatialIndex, intersects);
    for (auto const &pair : intersects) {
      intersectsLookup[pair.first].push_back(pair.second);
    }
  }

  for (PlineIntersect const &si : selfIntersects) {
    intersectsLookup[si.sIndex1].push_back(si.pos);
    intersectsLookup[si.sIndex2].push_back(si.pos);
  }

  for (PlineIntersect const &intr : dualIntersects.intersects) {
    intersectsLookup[intr.sIndex1].push_back(intr.pos);
  }

  for (PlineCoincidentIntersect const &intr : dualIntersects.coincidentIntersects) {
    intersectsLookup[intr.sIndex1].push_back(intr.point1);
    intersectsLookup[intr.sIndex1].push_back(intr.point2);
  }

  std::vector<std::size_t> queryStack;
  queryStack.reserve(8);
  if (intersectsLookup.size() == 0) {
    if (!pointValidForOffset(originalPline, offset, origPlineSpatialIndex, rawOffsetPline[0].pos(),
                             queryStack)) {
      return result;
    }
    // copy and convert raw offset into open polyline
    result.emplace_back(std::numeric_limits<std::size_t>::max(), rawOffsetPline);
    result.back().pline.isClosed() = false;
    if (originalPline.isClosed()) {
      result.back().pline.addVertex(rawOffsetPline[0]);
      result.back().pline.lastVertex().bulge() = 0.f;
    }
    return result;
  }

  // sort intersects by distance from start vertex
  for (auto &kvp : intersectsLookup) {
    Vector2d startPos = rawOffsetPline[kvp.first].pos();
    auto cmp = [&](Vector2d const &si1, Vector2d const &si2) {
      return distSquared(si1, startPos) < distSquared(si2, startPos);
    };
    std::sort(kvp.second.begin(), kvp.second.end(), cmp);
  }

  auto intersectsOrigPline = [&](PlineVertex2D const &v1, PlineVertex2D const &v2) {
    AABB approxBB = createFastApproxBoundingBox(v1, v2);
    bool intersects = false;
    auto visitor = [&](std::size_t i) {
      using namespace internal;
      std::size_t j = utils::nextWrappingIndex(i, originalPline);
      IntrPlineSegsResult intrResult =
          intrPlineSegs(v1, v2, originalPline[i], originalPline[j]);
      intersects = intrResult.intrType != PlineSegIntrType::NoIntersect;
      return !intersects;
    };

    origPlineSpatialIndex.visitQuery(approxBB.xMin, approxBB.yMin, approxBB.xMax, approxBB.yMax,
                                     visitor, queryStack);

    return intersects;
  };

  if (!originalPline.isClosed()) {
    // build first open polyline that ends at the first intersect since we will not wrap back to
    // capture it as in the case of a closed polyline
    Polyline2D firstSlice;
    std::size_t index = 0;
    std::size_t loopCount = 0;
    const std::size_t maxLoopCount = rawOffsetPline.size();
    while (true) {
      if (loopCount++ > maxLoopCount) {
        CORE_ASSERT(false, "Bug detected, should never loop this many times!");
        // break to avoid infinite loop
        break;
      }
      auto iter = intersectsLookup.find(index);
      if (iter == intersectsLookup.end()) {
        // no intersect found, test segment will be valid before adding the vertex
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           rawOffsetPline[index].pos(), queryStack)) {
          break;
        }

        // index check (only test segment if we're not adding the first vertex)
        if (index != 0 && intersectsOrigPline(firstSlice.lastVertex(), rawOffsetPline[index])) {
          break;
        }

        internal::addOrReplaceIfSamePos(firstSlice, rawOffsetPline[index]);
      } else {
        // intersect found, test segment will be valid before finishing first open polyline
        Vector2d const &intersectPos = iter->second[0];
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           intersectPos, queryStack)) {
          break;
        }

        SplitResult split =
            splitAtPoint(rawOffsetPline[index], rawOffsetPline[index + 1], intersectPos);

        PlineVertex2D sliceEndVertex = PlineVertex2D(intersectPos, 0.f);
        auto midpoint = segMidpoint(split.updatedStart, sliceEndVertex);
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, midpoint,
                                           queryStack)) {
          break;
        }

        if (intersectsOrigPline(split.updatedStart, sliceEndVertex)) {
          break;
        }

        internal::addOrReplaceIfSamePos(firstSlice, split.updatedStart);
        internal::addOrReplaceIfSamePos(firstSlice, sliceEndVertex);
        result.emplace_back(0, std::move(firstSlice));
        break;
      }

      index += 1;
    }
  }

  for (auto const &kvp : intersectsLookup) {
    // start index for the slice we're about to build
    std::size_t sIndex = kvp.first;
    // self intersect list for this start index
    std::vector<Vector2d> const &siList = kvp.second;

    const auto &startVertex = rawOffsetPline[sIndex];
    std::size_t nextIndex = utils::nextWrappingIndex(sIndex, rawOffsetPline);
    const auto &endVertex = rawOffsetPline[nextIndex];

    if (siList.size() != 1) {
      // build all the segments between the N intersects in siList (N > 1), skipping the first
      // segment (to be processed at the end)
      SplitResult firstSplit = splitAtPoint(startVertex, endVertex, siList[0]);
      auto prevVertex = firstSplit.splitVertex;
      for (std::size_t i = 1; i < siList.size(); ++i) {
        SplitResult split = splitAtPoint(prevVertex, endVertex, siList[i]);
        // update prevVertex for next loop iteration
        prevVertex = split.splitVertex;
        // skip if they're ontop of each other
        if (fuzzyEqual(split.updatedStart.pos(), split.splitVertex.pos(),
                       utils::realPrecision<double>())) {
          continue;
        }

        // test start point
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           split.updatedStart.pos(), queryStack)) {
          continue;
        }

        // test end point
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           split.splitVertex.pos(), queryStack)) {
          continue;
        }

        // test mid point
        auto midpoint = segMidpoint(split.updatedStart, split.splitVertex);
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, midpoint,
                                           queryStack)) {
          continue;
        }

        // test intersection with original polyline
        if (intersectsOrigPline(split.updatedStart, split.splitVertex)) {
          continue;
        }

        result.emplace_back();
        result.back().intrStartIndex = sIndex;
        result.back().pline.addVertex(split.updatedStart);
        result.back().pline.addVertex(split.splitVertex);
      }
    }

    // build the segment between the last intersect in siList and the next intersect found

    // check that the first point is valid
    if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, siList.back(),
                                       queryStack)) {
      continue;
    }

    SplitResult split = splitAtPoint(startVertex, endVertex, siList.back());
    Polyline2D currSlice;
    currSlice.addVertex(split.splitVertex);

    std::size_t index = nextIndex;
    bool isValidPline = true;
    std::size_t loopCount = 0;
    const std::size_t maxLoopCount = rawOffsetPline.size();
    while (true) {
      if (loopCount++ > maxLoopCount) {
        CORE_ASSERT(false, "Bug detected, should never loop this many times!");
        // break to avoid infinite loop
        break;
      }
      // check that vertex point is valid
      if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                         rawOffsetPline[index].pos(), queryStack)) {
        isValidPline = false;
        break;
      }

      // check that the segment does not intersect original polyline
      if (intersectsOrigPline(currSlice.lastVertex(), rawOffsetPline[index])) {
        isValidPline = false;
        break;
      }

      // add vertex
      internal::addOrReplaceIfSamePos(currSlice, rawOffsetPline[index]);

      // check if segment that starts at vertex we just added has an intersect
      auto nextIntr = intersectsLookup.find(index);
      if (nextIntr != intersectsLookup.end()) {
        // there is an intersect, slice is done, check if final segment is valid

        // check intersect pos is valid (which will also be end vertex position)
        Vector2d const &intersectPos = nextIntr->second[0];
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex,
                                           intersectPos, queryStack)) {
          isValidPline = false;
          break;
        }

        std::size_t nextIndex = utils::nextWrappingIndex(index, rawOffsetPline);
        SplitResult split =
            splitAtPoint(currSlice.lastVertex(), rawOffsetPline[nextIndex], intersectPos);

        PlineVertex2D sliceEndVertex = PlineVertex2D(intersectPos, 0.f);
        // check mid point is valid
        Vector2d mp = segMidpoint(split.updatedStart, sliceEndVertex);
        if (!internal::pointValidForOffset(originalPline, offset, origPlineSpatialIndex, mp,
                                           queryStack)) {
          isValidPline = false;
          break;
        }

        // trim last added vertex and add final intersect position
        currSlice.lastVertex() = split.updatedStart;
        internal::addOrReplaceIfSamePos(currSlice, sliceEndVertex);

        break;
      }
      // else there is not an intersect, increment index and continue
      if (index == rawOffsetPline.size() - 1) {
        if (originalPline.isClosed()) {
          // wrap index
          index = 0;
        } else {
          // open polyline, we're done
          break;
        }
      } else {
        index += 1;
      }
    }

    if (isValidPline && currSlice.size() > 1) {
      result.emplace_back(sIndex, std::move(currSlice));
    }
  }

  return result;
}

/// Stitches raw offset polyline slices together, discarding any that are not valid.
inline std::vector<Polyline2D>
stitchOffsetSlicesTogether(std::vector<OpenPolylineSlice> const &slices, bool closedPolyline,
                           std::size_t origMaxIndex,
                           double joinThreshold = utils::sliceJoinThreshold<double>()) {
  std::vector<Polyline2D> result;
  if (slices.size() == 0) {
    return result;
  }

  if (slices.size() == 1) {
    result.emplace_back(slices[0].pline);
    if (closedPolyline &&
        fuzzyEqual(result[0][0].pos(), result[0].lastVertex().pos(), joinThreshold)) {
      result[0].isClosed() = true;
      result[0].vertexes().pop_back();
    }

    return result;
  }

  // load spatial index with all start points
  StaticSpatialIndex spatialIndex(slices.size());

  for (const auto &slice : slices) {
    auto const &point = slice.pline[0].pos();
    spatialIndex.add(point.x() - joinThreshold, point.y() - joinThreshold,
                     point.x() + joinThreshold, point.y() + joinThreshold);
  }

  spatialIndex.finish();

  std::vector<bool> visitedIndexes(slices.size(), false);
  std::vector<std::size_t> queryResults;
  std::vector<std::size_t> queryStack;
  queryStack.reserve(8);
  for (std::size_t i = 0; i < slices.size(); ++i) {
    if (visitedIndexes[i]) {
      continue;
    }

    visitedIndexes[i] = true;

    Polyline2D currPline;
    std::size_t currIndex = i;
    auto const &initialStartPoint = slices[i].pline[0].pos();
    std::size_t loopCount = 0;
    const std::size_t maxLoopCount = slices.size();
    while (true) {
      if (loopCount++ > maxLoopCount) {
        CORE_ASSERT(false, "Bug detected, should never loop this many times!");
        // break to avoid infinite loop
        break;
      }
      const std::size_t currLoopStartIndex = slices[currIndex].intrStartIndex;
      auto const &currSlice = slices[currIndex].pline;
      auto const &currEndPoint = slices[currIndex].pline.lastVertex().pos();
      currPline.vertexes().insert(currPline.vertexes().end(), currSlice.vertexes().begin(),
                                  currSlice.vertexes().end());
      queryResults.clear();
      spatialIndex.query(currEndPoint.x() - joinThreshold, currEndPoint.y() - joinThreshold,
                         currEndPoint.x() + joinThreshold, currEndPoint.y() + joinThreshold,
                         queryResults, queryStack);

      queryResults.erase(std::remove_if(queryResults.begin(), queryResults.end(),
                                        [&](std::size_t index) { return visitedIndexes[index]; }),
                         queryResults.end());

      auto indexDistAndEqualInitial = [&](std::size_t index) {
        auto const &slice = slices[index];
        std::size_t indexDist;
        if (currLoopStartIndex <= slice.intrStartIndex) {
          indexDist = slice.intrStartIndex - currLoopStartIndex;
        } else {
          // forward wrapping distance (distance to end + distance to index)
          indexDist = origMaxIndex - currLoopStartIndex + slice.intrStartIndex;
        }

        bool equalToInitial = fuzzyEqual(slice.pline.lastVertex().pos(), initialStartPoint,
                                         utils::realPrecision<double>());

        return std::make_pair(indexDist, equalToInitial);
      };

      std::sort(queryResults.begin(), queryResults.end(),
                [&](std::size_t index1, std::size_t index2) {
                  auto distAndEqualInitial1 = indexDistAndEqualInitial(index1);
                  auto distAndEqualInitial2 = indexDistAndEqualInitial(index2);
                  if (distAndEqualInitial1.first == distAndEqualInitial2.first) {
                    // index distances are equal, compare on position being equal to initial start
                    // (testing index1 < index2, we want the longest closed loop possible)
                    return distAndEqualInitial1.second < distAndEqualInitial2.second;
                  }

                  return distAndEqualInitial1.first < distAndEqualInitial2.first;
                });

      if (queryResults.size() == 0) {
        // we're done
        if (currPline.size() > 1) {
          if (closedPolyline && fuzzyEqual(currPline[0].pos(), currPline.lastVertex().pos(),
                                           utils::realPrecision<double>())) {
            currPline.vertexes().pop_back();
            currPline.isClosed() = true;
          }
          result.emplace_back(std::move(currPline));
        }
        break;
      }

      // else continue stitching
      visitedIndexes[queryResults[0]] = true;
      currPline.vertexes().pop_back();
      currIndex = queryResults[0];
    }
  }

  return result;
}
} // namespace internal

/// Creates the paralell offset polylines to the polyline given.
inline std::vector<Polyline2D> parallelOffset(Polyline2D const &pline, double offset,
                                           bool hasSelfIntersects = false) {
  using namespace internal;
  if (pline.size() < 2) {
    return std::vector<Polyline2D>();
  }
  auto rawOffset = createRawOffsetPline(pline, offset);
  if (pline.isClosed() && !hasSelfIntersects) {
    auto slices = slicesFromRawOffset(pline, rawOffset, offset);
    return stitchOffsetSlicesTogether(slices, pline.isClosed(), rawOffset.size() - 1);
  }

  // not closed polyline or has self intersects, must apply dual clipping
  auto dualRawOffset = createRawOffsetPline(pline, -offset);
  auto slices = dualSliceAtIntersectsForOffset(pline, rawOffset, dualRawOffset, offset);
  return stitchOffsetSlicesTogether(slices, pline.isClosed(), rawOffset.size() - 1);
}

inline std::vector<Polyline2D> parallelOffset(std::vector<Polyline2D> const& plines, double offset,
  bool hasSelfIntersects = false) {
  std::vector<Polyline2D> result;

  //TODO replace this with the smarter system in polylineoffsetIslands, where if the offset from multiple given plines intersect, they are merged together
  for (size_t i = 0; i < plines.size(); i++)
  {
    std::vector<Polyline2D> newOffsets = parallelOffset(plines[i], offset, hasSelfIntersects);
    result.insert(result.end(), newOffsets.begin(), newOffsets.end());
  }
  return result;
}

inline std::vector<Polyline2D> parallelOffsetToClosedLoop(Polyline2D& pline, double offset,
  bool hasSelfIntersects = false) {
  using namespace internal;
  
  std::vector<Polyline2D> finalResult;
  float scale = 1.f;
  bool allLoopsAreClosed = false;

  while (!allLoopsAreClosed) {
    finalResult.clear();
    scalePolyline(pline, (double)10.f);

    scale *= 10.f;

    finalResult = parallelOffset(pline, scale * offset, hasSelfIntersects);

    // check if this scaled version did not result in open polylines
    allLoopsAreClosed = true;
    for (auto& path : finalResult)
    {
      if (!path.isClosed())
        allLoopsAreClosed = false;
    }
  }
  // scale finalPass back down to standard
  for (auto& path : finalResult)
  {
    scalePolyline(path, (double)(1.f / scale));
    scalePolyline(pline, (double)(1.f / scale));
  }
  return finalResult;
}

inline std::vector<Polyline2D> parallelOffsetToClosedLoops(std::vector<Polyline2D>& plines, double offset,
  bool hasSelfIntersects = false) {
  std::vector<Polyline2D> result;

  //TODO replace this with the smarter system in polylineoffsetIslands, where if the offset from multiple given plines intersect, they are merged together
  for (size_t i = 0; i < plines.size(); i++)
  {
    std::vector<Polyline2D> newOffsets = parallelOffsetToClosedLoop(plines[i], offset, hasSelfIntersects);
    result.insert(result.end(), newOffsets.begin(), newOffsets.end());
  }
  return result;
}

} // namespace core
#endif // POLYLINE_OFFSET_HPP
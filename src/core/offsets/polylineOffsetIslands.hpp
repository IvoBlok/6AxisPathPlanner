#ifndef POLYLINE_OFFSET_ISLANDS_HPP
#define POLYLINE_OFFSET_ISLANDS_HPP

#include "../polyline.hpp"
#include "../intersect/polylineCombine.hpp"
#include "polylineOffset.hpp"

#include <unordered_map>
#include <vector>

namespace core {

struct OffsetLoop {
  std::size_t parentLoopIndex;
  Polyline2D polyline;
  StaticSpatialIndex spatialIndex;
};

struct OffsetLoopSet {
  std::vector<OffsetLoop> ccwLoops;
  std::vector<OffsetLoop> cwLoops;
};

class ParallelOffsetIslands {
public:
  ParallelOffsetIslands() {}
  OffsetLoopSet computeOneSidedOffsets(OffsetLoopSet& input, double offsetDelta);
  OffsetLoopSet computeTwoSidedOffsets(OffsetLoopSet const& input, double offsetDelta);
  std::vector<core::Polyline2D> computeTwoSidedOffsets(std::vector<core::Polyline2D> innerBounds, std::vector<core::Polyline2D> outerBounds, double offset);
  std::vector<core::Polyline2D> computeOneSidedOffsets(std::vector<core::Polyline2D> innerBounds, core::Polyline2D outerBound, double offset);

private:
  // type to represent a slice point (intersect) on an OffsetLoop
  struct LoopSlicePoint {
    // intersect between loops
    PlineIntersect intr;
    bool noSliceAfterForIndex1;
  };

  // type to represent a set of slice points (intersects) between two loops
  struct SlicePointSet {
    // index of first loop involved
    std::size_t loopIndex1;
    // index of second loop involved
    std::size_t loopIndex2;
    // all of the slice points (intersects) between the two loops
    std::vector<LoopSlicePoint> slicePoints;
  };

  // get an offset loop by index i, maps to ccw then cw offset loops
  OffsetLoop &getOffsetLoop(std::size_t i) {
    return i < m_ccwOffsetLoops.size() ? m_ccwOffsetLoops[i]
                                       : m_cwOffsetLoops[i - m_ccwOffsetLoops.size()];
  }

  OffsetLoop const &getParentLoop(std::size_t i) {
    return i < m_inputSet->ccwLoops.size() ? m_inputSet->ccwLoops[i]
                                           : m_inputSet->cwLoops[i - m_inputSet->ccwLoops.size()];
  }

  void createOffsetLoops(const OffsetLoopSet& input, double absDelta);
  void createOneSidedOffsetLoops(OffsetLoopSet &input, double absDelta);
  void createOffsetLoopsIndex();
  void createSlicePoints();

  struct DissectionPoint {
    std::size_t otherLoopIndex;
    Vector2d pos;
  };

  struct DissectedSlice {
    // open polyline representing the slice
    Polyline2D pline;
    // index of the loop the slice is from
    std::size_t sliceParentIndex;
    // index of the loop that intersected the parent loop to form the start of the slice
    std::size_t startLoopIndex;
    // index of the loop that intersected the parent loop to form the end of the slice
    std::size_t endLoopIndex;
  };

  bool pointOnOffsetValid(std::size_t skipIndex, Vector2d const &pt, double absDelta);
  void createSlicesFromLoop(std::size_t loopIndex, double absDelta,
                            std::vector<DissectedSlice> &result);

  OffsetLoopSet const *m_inputSet;

  // counter clockwise offset loops, these surround the clockwise offset loops
  std::vector<OffsetLoop> m_ccwOffsetLoops;
  // clockwise (island) offset loops, these are surrounded by the counter clockwise loops
  std::vector<OffsetLoop> m_cwOffsetLoops;
  std::size_t totalOffsetLoopsCount() { return m_ccwOffsetLoops.size() + m_cwOffsetLoops.size(); }
  // spatial index of all the offset loops
  std::unique_ptr<StaticSpatialIndex> m_offsetLoopsIndex;
  using IndexPair = std::pair<std::size_t, std::size_t>;
  // set to keep track of already visited pairs of loops when finding intersects
  std::unordered_set<IndexPair, utils::IndexPairHash> m_visitedLoopPairs;
  // buffers to use for querying spatial indexes
  std::vector<std::size_t> m_queryStack;
  std::vector<std::size_t> m_queryResults;
  // slice point sets from intersects between loops
  std::vector<SlicePointSet> m_slicePointSets;
  // lookup used to get slice points for a particular loop index (holds indexes to sets in
  // m_slicePointSets)
  std::vector<std::vector<std::size_t>> m_slicePointsLookup;
  // dissection points used to form slices for a particular loop in createSlicesFromLoop
  std::unordered_map<std::size_t, std::vector<DissectionPoint>> m_loopDissectionPoints;
};

inline void ParallelOffsetIslands::createOffsetLoops(const OffsetLoopSet &input,
                                                    double absDelta) {
  // create counter clockwise offset loops
  m_ccwOffsetLoops.clear();
  std::size_t parentIndex = 0;
  for (auto const &loop : input.ccwLoops) {
    auto offsets = parallelOffset(loop.polyline, absDelta);
    for (auto &offset : offsets) {
      // must check if orientation inverted (due to collapse of very narrow or small input)
      if (getArea(offset) < 0.f) {
        continue;
      }
      auto index = createApproxSpatialIndex(offset);
      m_ccwOffsetLoops.push_back({parentIndex, std::move(offset), std::move(index)});
    }
    parentIndex += 1;
  }

  // create clockwise offset loops (note counter clockwise loops may result from outward offset)
  m_cwOffsetLoops.clear();
  for (auto const &loop : input.cwLoops) {
    auto offsets = parallelOffset(loop.polyline, absDelta);
    for (auto &offset : offsets) {
      auto index = createApproxSpatialIndex(offset);
      if (getArea(offset) < 0.f) {
        m_cwOffsetLoops.push_back({parentIndex, std::move(offset), std::move(index)});
      } else {
        m_ccwOffsetLoops.push_back({parentIndex, std::move(offset), std::move(index)});
      }
    }
    parentIndex += 1;
  }
}

inline void ParallelOffsetIslands::createOneSidedOffsetLoops(OffsetLoopSet& input,
  double absDelta) {
  // create counter clockwise offset loops
  m_ccwOffsetLoops.clear();
  std::size_t parentIndex = 0;

  auto offsets = parallelOffset(input.ccwLoops.front().polyline, absDelta);
  for (auto& offset : offsets) {
    // must check if orientation inverted (due to collapse of very narrow or small input)
    if (getArea(offset) < 0.f) {
      continue;
    }
    auto index = createApproxSpatialIndex(offset);
    m_ccwOffsetLoops.push_back({ parentIndex, std::move(offset), std::move(index) });
  }
  parentIndex += 1;
  
  // create clockwise offset loops (note counter clockwise loops may result from outward offset)
  m_cwOffsetLoops.clear();
  for (auto const& loop : input.cwLoops) {
    auto offset = loop.polyline;
    auto index = createApproxSpatialIndex(offset);
    m_cwOffsetLoops.push_back({ parentIndex, std::move(offset), std::move(index) });
    m_cwOffsetLoops.back().parentLoopIndex = parentIndex;
    parentIndex += 1;
  }
}

inline void ParallelOffsetIslands::createOffsetLoopsIndex() {
  // create spatial index for all offset loop bounding boxes
  m_offsetLoopsIndex = std::make_unique<StaticSpatialIndex>(totalOffsetLoopsCount());
  for (auto const &posC : m_ccwOffsetLoops) {
    auto const &i = posC.spatialIndex;
    m_offsetLoopsIndex->add(i.minX(), i.minY(), i.maxX(), i.maxY());
  }

  for (auto const &negC : m_cwOffsetLoops) {
    auto const &i = negC.spatialIndex;
    m_offsetLoopsIndex->add(i.minX(), i.minY(), i.maxX(), i.maxY());
  }
  m_offsetLoopsIndex->finish();
}

inline void ParallelOffsetIslands::createSlicePoints() {
  m_visitedLoopPairs.clear();
  m_slicePointSets.clear();
  m_slicePointsLookup.clear();

  // find all intersects between all offsets
  std::size_t totalOffsetCount = totalOffsetLoopsCount();
  m_slicePointsLookup.resize(totalOffsetCount);
  PlineIntersectsResult intrsResults;
  for (std::size_t i = 0; i < totalOffsetCount; ++i) {
    auto const &loop1 = getOffsetLoop(i);
    auto const &index1 = loop1.spatialIndex;
    m_queryResults.clear();
    m_offsetLoopsIndex->query(index1.minX(), index1.minY(), index1.maxX(), index1.maxY(),
                              m_queryResults, m_queryStack);

    for (std::size_t j : m_queryResults) {
      // skip same index (no self intersects among the offset loops)
      if (i == j) {
        continue;
      }
      // skip reversed index order (would end up comparing the same loops)
      if (m_visitedLoopPairs.find({j, i}) != m_visitedLoopPairs.end()) {
        continue;
      }
      m_visitedLoopPairs.emplace(i, j);

      auto const &loop2 = getOffsetLoop(j);
      intrsResults.intersects.clear();
      intrsResults.coincidentIntersects.clear();
      // finding intersects
      findIntersects(loop1.polyline, loop2.polyline, index1, intrsResults);
      if (intrsResults.hasIntersects()) {
        m_slicePointSets.emplace_back();
        auto &slicePointSet = m_slicePointSets.back();
        slicePointSet.loopIndex1 = i;
        slicePointSet.loopIndex2 = j;
        for (auto &intr : intrsResults.intersects) {
          slicePointSet.slicePoints.push_back({std::move(intr), false});
        }

        // add coincident start and end points
        if (intrsResults.coincidentIntersects.size() != 0) {
          auto coinSliceResult = sortAndjoinCoincidentSlices(intrsResults.coincidentIntersects,
                                                             loop1.polyline, loop2.polyline);
          for (auto &sp : coinSliceResult.sliceStartPoints) {
            slicePointSet.slicePoints.push_back({std::move(sp), false});
          }
          for (auto &ep : coinSliceResult.sliceEndPoints) {
            slicePointSet.slicePoints.push_back({std::move(ep), true});
          }
        }

        m_slicePointsLookup[i].push_back(m_slicePointSets.size() - 1);
        m_slicePointsLookup[j].push_back(m_slicePointSets.size() - 1);
      }
    }
  }
}

inline bool ParallelOffsetIslands::pointOnOffsetValid(std::size_t skipIndex, const Vector2d &pt,
                                                     double absDelta) {
  // test distance against input polylines
  std::size_t const inputTotalCount = m_inputSet->ccwLoops.size() + m_inputSet->cwLoops.size();
  for (std::size_t i = 0; i < inputTotalCount; ++i) {
    if (i == skipIndex) {
      continue;
    }
    auto const &parentLoop = getParentLoop(i);
    if (!internal::pointValidForOffset(parentLoop.polyline, absDelta, parentLoop.spatialIndex, pt,
                                       m_queryStack)) {
      return false;
    }
  }

  return true;
}

inline void ParallelOffsetIslands::createSlicesFromLoop(std::size_t loopIndex, double absDelta,
                                                       std::vector<DissectedSlice> &result) {
  OffsetLoop const &offsetLoop = getOffsetLoop(loopIndex);
  std::size_t const parentIndex = offsetLoop.parentLoopIndex;
  Polyline2D const &pline = offsetLoop.polyline;
  m_loopDissectionPoints.clear();
  for (auto const &setIndex : m_slicePointsLookup[loopIndex]) {
    auto const &set = m_slicePointSets[setIndex];
    bool isFirstIndex = loopIndex == set.loopIndex1;
    if (isFirstIndex) {
      for (auto const &p : set.slicePoints) {
        m_loopDissectionPoints[p.intr.sIndex1].push_back({set.loopIndex2, p.intr.pos});
      }
    } else {
      for (auto const &p : set.slicePoints) {
        m_loopDissectionPoints[p.intr.sIndex2].push_back({set.loopIndex1, p.intr.pos});
      }
    }
  }

  // sort points by distance from start vertex
  for (auto &kvp : m_loopDissectionPoints) {
    Vector2d startPos = pline[kvp.first].pos();
    auto cmp = [&](DissectionPoint const &p1, DissectionPoint const &p2) {
      return distSquared(p1.pos, startPos) < distSquared(p2.pos, startPos);
    };
    std::sort(kvp.second.begin(), kvp.second.end(), cmp);
  }


  for (auto const &kvp : m_loopDissectionPoints) {
    // start index for the slice we're about to build
    std::size_t sIndex = kvp.first;
    // self intersect list for this start index
    std::vector<DissectionPoint> const &intrsList = kvp.second;

    const auto &firstSegStartVertex = pline[sIndex];
    std::size_t nextIndex = utils::nextWrappingIndex(sIndex, pline);
    const auto &firstSegEndVertex = pline[nextIndex];

    if (intrsList.size() != 1) {
      // build all the segments between the N intersects in siList (N > 1), skipping the first
      // segment (to be processed at the end)
      SplitResult firstSplit =
          splitAtPoint(firstSegStartVertex, firstSegEndVertex, intrsList[0].pos);
      auto prevVertex = firstSplit.splitVertex;
      for (std::size_t i = 1; i < intrsList.size(); ++i) {
        std::size_t const sliceStartIndex = intrsList[i - 1].otherLoopIndex;
        std::size_t const sliceEndIndex = intrsList[i].otherLoopIndex;
        SplitResult split = splitAtPoint(prevVertex, firstSegEndVertex, intrsList[i].pos);
        // update prevVertex for next loop iteration
        prevVertex = split.splitVertex;

        if (fuzzyEqual(split.updatedStart.pos(), split.splitVertex.pos(),
                       utils::realPrecision<double>())) {
          continue;
        }

        auto sMidpoint = segMidpoint(split.updatedStart, split.splitVertex);
        if (!pointOnOffsetValid(parentIndex, sMidpoint, absDelta)) {
          // skip slice
          continue;
        }

        result.emplace_back();
        result.back().pline.addVertex(split.updatedStart);
        result.back().pline.addVertex(split.splitVertex);
        result.back().sliceParentIndex = loopIndex;
        result.back().startLoopIndex = sliceStartIndex;
        result.back().endLoopIndex = sliceEndIndex;
      }
    }

    // build the segment between the last intersect in instrsList and the next intersect found
    SplitResult split =
        splitAtPoint(firstSegStartVertex, firstSegEndVertex, intrsList.back().pos);

    DissectedSlice currSlice;
    currSlice.pline.addVertex(split.splitVertex);
    currSlice.sliceParentIndex = loopIndex;
    currSlice.startLoopIndex = intrsList.back().otherLoopIndex;

    std::size_t index = nextIndex;
    std::size_t loopCount = 0;
    const std::size_t maxLoopCount = pline.size();
    while (true) {
      if (loopCount++ > maxLoopCount) {
        CORE_ASSERT(false, "Bug detected, should never loop this many times!");
        // break to avoid infinite loop
        break;
      }
      // add vertex
      internal::addOrReplaceIfSamePos(currSlice.pline, pline[index]);

      // check if segment that starts at vertex we just added has an intersect
      auto nextIntr = m_loopDissectionPoints.find(index);
      if (nextIntr != m_loopDissectionPoints.end()) {
        // there is an intersect, slice is done
        Vector2d const &intersectPos = nextIntr->second[0].pos;

        // trim last added vertex and add final intersect position
        PlineVertex2D endVertex = PlineVertex2D(intersectPos, 0.f);
        std::size_t nextIndex = utils::nextWrappingIndex(index, pline);
        SplitResult split =
            splitAtPoint(currSlice.pline.lastVertex(), pline[nextIndex], intersectPos);
        currSlice.pline.lastVertex() = split.updatedStart;
        internal::addOrReplaceIfSamePos(currSlice.pline, endVertex);
        currSlice.endLoopIndex = nextIntr->second[0].otherLoopIndex;
        break;
      }
      // else there is not an intersect, increment index and continue
      index = utils::nextWrappingIndex(index, pline);
    }

    if (currSlice.pline.size() > 1) {
      auto sMidpoint =
          segMidpoint(currSlice.pline[currSlice.pline.size() - 2], currSlice.pline.lastVertex());
      if (!pointOnOffsetValid(parentIndex, sMidpoint, absDelta)) {
        // skip slice
        continue;
      }
      result.push_back(std::move(currSlice));
    }
  }
}

inline OffsetLoopSet ParallelOffsetIslands::computeTwoSidedOffsets(const OffsetLoopSet &input,
                                                         double offsetDelta) {
  m_inputSet = &input;
  OffsetLoopSet result;
  double absDelta = std::abs(offsetDelta);
  createOffsetLoops(input, absDelta);
  if (totalOffsetLoopsCount() == 0) {
    return result;
  }

  createOffsetLoopsIndex();
  createSlicePoints();

  std::vector<DissectedSlice> slices;
  std::size_t totalOffsetsCount = totalOffsetLoopsCount();

  std::vector<Polyline2D> resultSlices;

  for (std::size_t i = 0; i < totalOffsetsCount; ++i) {
    if (m_slicePointsLookup[i].size() == 0) {
      // no intersects but still must test distance of one vertex position since it may be inside
      // another offset (completely eclipsed by island offset)
      auto &loop = getOffsetLoop(i);
      if (!pointOnOffsetValid(loop.parentLoopIndex, loop.polyline[0].pos(), absDelta)) {
        continue;
      }
      if (i < m_ccwOffsetLoops.size()) {
        result.ccwLoops.push_back(std::move(loop));
      } else {
        result.cwLoops.push_back(std::move(loop));
      }
      continue;
    }
    createSlicesFromLoop(i, absDelta, slices);
  }

  for (auto &slice : slices) {
    resultSlices.push_back(std::move(slice.pline));
  }

  std::vector<Polyline2D> stitched =
      internal::stitchOrderedSlicesIntoClosedPolylines(resultSlices);

  for (auto &r : stitched) {
    double area = getArea(r);
    if (std::abs(area) < 1e-4) {
      continue;
    }
    auto spatialIndex = createApproxSpatialIndex(r);
    if (area < 0.f) {
      result.cwLoops.push_back({0, std::move(r), std::move(spatialIndex)});
    } else {
      result.ccwLoops.push_back({0, std::move(r), std::move(spatialIndex)});
    }
  }

  return result;
}

inline OffsetLoopSet ParallelOffsetIslands::computeOneSidedOffsets(OffsetLoopSet& input,
  double offsetDelta) {
  m_inputSet = &input;
  OffsetLoopSet result;
  double absDelta = std::abs(offsetDelta);
  createOneSidedOffsetLoops(input, absDelta);

  // if the offset outside boundary path is non-existent, there is no path to be made, and thus return an empty path
  if (m_ccwOffsetLoops.size() == 0) {
    return result;
  }
  // if there is an offset boundary path, but not any inside paths, the offset boundary path should simply be returned
  else if (m_cwOffsetLoops.size() == 0) {
    for (OffsetLoop& loop : m_ccwOffsetLoops)
    {
      auto& r = loop.polyline;
      auto spatial = createApproxSpatialIndex(r);
      result.ccwLoops.push_back({ 0, std::move(r), std::move(spatial) });
    }
    return result;
  }

  // if there are both inside and outside path(s), return the edge path of the union of all the surfaces enclosed by these input paths
  // the exception is inside paths that lay outside the boundary path
  Polyline2D unionPath = m_ccwOffsetLoops[0].polyline;
  for (const OffsetLoop& insideLoop : m_cwOffsetLoops)
  {
    // to check if a loop lays entirely outside another, we check if there are any intersects between the two
    // if there are no intersects, no union is required, the output can simply be the offset of the boundary path
    internal::ProcessForCombineResult combineInfo = internal::processForCombine(m_ccwOffsetLoops[0].polyline, insideLoop.polyline, m_ccwOffsetLoops[0].spatialIndex);
    if (combineInfo.anyIntersects()) {
      CombineResult combineResult = combinePolylines(unionPath, insideLoop.polyline, PlineCombineMode::Union);
      unionPath = combineResult.remaining[0];
    }
    else {
      // if the offsetpath is fully contained in any of the inside paths, return an empty path
      Vector2d pointOnBoundary{ m_ccwOffsetLoops[0].polyline.lastVertex().x(), m_ccwOffsetLoops[0].polyline.lastVertex().y() };
      if (getWindingNumber(insideLoop.polyline, pointOnBoundary) != 0)
        return result;
    }
  }

  auto spatial = createApproxSpatialIndex(unionPath);

  result.ccwLoops.push_back({ 0, std::move(unionPath), std::move(spatial) });

  return result;
}

inline std::vector<core::Polyline2D> ParallelOffsetIslands::computeTwoSidedOffsets(std::vector<core::Polyline2D> innerBounds, std::vector<core::Polyline2D> outerBounds, double offset) {
  
  std::vector<core::Polyline2D> result;
  OffsetLoopSet loopSet;

  for (core::Polyline2D& innerBound : innerBounds) {
    // ensure that for inner bounds the path direction is clockwise, so that a positive offset will make an outwards offset
    if (!isPathClockwise(innerBound))
      innerBound.invertDirection();

    loopSet.cwLoops.push_back({ 0, innerBound, createApproxSpatialIndex(innerBound) });
  }

  for (core::Polyline2D& outerBound : outerBounds) {
    // ensure that for outer bounds the path direction is counter-clockwise, so that a positive offset will make an inwards offset
    if (isPathClockwise(outerBound))
      outerBound.invertDirection();

    loopSet.ccwLoops.push_back({ 0, outerBound, createApproxSpatialIndex(outerBound) });
  }

  // calculate the offset paths
  OffsetLoopSet resultingOffsets;
  float scale = 1.f;
  bool allLoopsAreClosed = false;

  while (!allLoopsAreClosed) {
    scale *= 10.f;
    
    for (auto& path : loopSet.cwLoops) {
      scalePolyline(path.polyline, (double)10.f);
      path.spatialIndex = createApproxSpatialIndex(path.polyline);
    }

    for (auto& path : loopSet.ccwLoops) {
      scalePolyline(path.polyline, (double)10.f);
      path.spatialIndex = createApproxSpatialIndex(path.polyline);
    }

    resultingOffsets = computeTwoSidedOffsets(loopSet, scale * offset);

    // check if this scaled version did not result in open polylines
    allLoopsAreClosed = true;
    if (resultingOffsets.ccwLoops.size() == 0 && resultingOffsets.cwLoops.size() == 0)
      break;
    for (auto& path : resultingOffsets.cwLoops)
    {
      if (!path.polyline.isClosed())
        allLoopsAreClosed = false;
    }
    for (auto& path : resultingOffsets.ccwLoops)
    {
      if (!path.polyline.isClosed())
        allLoopsAreClosed = false;
    }
  }
  // if a full succesfull island offset set was found, reset the scale back
  // while doing so, also:
  // convert the offsetLoopSet format into a more usable format
  for (auto& offsetLoop : resultingOffsets.cwLoops) {
    scalePolyline(offsetLoop.polyline, (double)1.f / (double)scale);
    result.push_back(offsetLoop.polyline);
  }
  for (auto& offsetLoop : resultingOffsets.ccwLoops) {
    scalePolyline(offsetLoop.polyline, (double)1.f / (double)scale);
    result.push_back(offsetLoop.polyline);
  }

  return result;
}

inline std::vector<core::Polyline2D> ParallelOffsetIslands::computeOneSidedOffsets(std::vector<core::Polyline2D> innerBounds, core::Polyline2D outerBound, double offset) {

  std::vector<core::Polyline2D> result;
  OffsetLoopSet loopSet;

  for (core::Polyline2D& innerBound : innerBounds) {
    // ensure that for inner bounds the path direction is clockwise, so that a positive offset will make an outwards offset
    if (!isPathClockwise(innerBound))
      innerBound.invertDirection();

    loopSet.cwLoops.push_back({ 0, innerBound, createApproxSpatialIndex(innerBound) });
  }

  // ensure that for the outer bound the path direction is counter-clockwise, so that a positive offset will make an inwards offset
  if (isPathClockwise(outerBound))
    outerBound.invertDirection();

  loopSet.ccwLoops.push_back({ 0, outerBound, createApproxSpatialIndex(outerBound) });

  // calculate the offset paths
  OffsetLoopSet resultingOffsets;
  float scale = 1.f;
  bool allLoopsAreClosed = false;

  while (!allLoopsAreClosed) {
    scale *= 10.f;

    for (auto& path : loopSet.cwLoops) {
      scalePolyline(path.polyline, (double)10.f);
      path.spatialIndex = createApproxSpatialIndex(path.polyline);
    }

    for (auto& path : loopSet.ccwLoops) {
      scalePolyline(path.polyline, (double)10.f);
      path.spatialIndex = createApproxSpatialIndex(path.polyline);
    }

    resultingOffsets = computeOneSidedOffsets(loopSet, scale * offset);

    // check if this scaled version did not result in open polylines
    allLoopsAreClosed = true;
    if (resultingOffsets.ccwLoops.size() == 0 && resultingOffsets.cwLoops.size() == 0)
      break;
    for (auto& path : resultingOffsets.cwLoops)
    {
      if (!path.polyline.isClosed())
        allLoopsAreClosed = false;
    }
    for (auto& path : resultingOffsets.ccwLoops)
    {
      if (!path.polyline.isClosed())
        allLoopsAreClosed = false;
    }
  }
  // if a full succesfull island offset set was found, reset the scale back
  for (auto& offsetLoop : resultingOffsets.ccwLoops) {
    scalePolyline(offsetLoop.polyline, (double)1.f / (double)scale);
    result.push_back(offsetLoop.polyline);
  }

  return result;
}

} // namespace core
#endif // POLYLINE_OFFSET_ISLANDS_HPP
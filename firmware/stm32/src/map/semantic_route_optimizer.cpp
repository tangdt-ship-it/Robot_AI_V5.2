#include <map/semantic_route_optimizer.h>

#include <math.h>

namespace {
// Semantic thresholds are deliberately separate from RouteCleaner thresholds.
// They describe geometry only; no motion/PID/replay setting is changed here.
constexpr float kSemanticStraightDirectionDeg = 12.0f;
constexpr float kSemanticMinStraightLengthMm = 200.0f;
constexpr float kSemanticTurnAngleDeg = 25.0f;
constexpr float kSemanticLineFitMaxResidualMm = 60.0f;
constexpr float kSemanticMaxCornerShiftMm = 100.0f;
constexpr float kSemanticMaxPathDeviationMm = 60.0f;
constexpr uint8_t kSemanticMaxLengthReductionPercent = 15U;
constexpr float kSemanticMinSegmentMm = 20.0f;
constexpr float kSemanticMaxSegmentMm = 5000.0f;
constexpr float kSemanticClosedClosureSkipMm = 20.0f;
constexpr float kSemanticMaxCornerAngleDeg = 150.0f;
constexpr float kRadToDeg = 57.29577951308232f;

enum class SemanticType : uint8_t {
  STRAIGHT_RUN = 0U,
  TURN_REGION = 1U,
  PROTECTED_ANCHOR = 2U,
};

struct SemanticLine {
  bool valid = false;
  float pointX = 0.0f;
  float pointY = 0.0f;
  float directionX = 0.0f;
  float directionY = 0.0f;
  float maxResidualMm = 0.0f;
  float lengthMm = 0.0f;
};

struct CornerCandidate {
  bool valid = false;
  uint16_t incomingEnd = 0U;
  uint16_t outgoingStart = 0U;
  uint16_t outgoingEnd = 0U;
  uint16_t representative = 0U;
  float cornerX = 0.0f;
  float cornerY = 0.0f;
  float angleDeg = 0.0f;
  float incomingDirectionDeg = 0.0f;
  float outgoingDirectionDeg = 0.0f;
  float shiftMm = 0.0f;
  float deviationMm = 0.0f;
  bool syntheticSafe = false;
};

enum class CornerSearchResult : uint8_t {
  NO_TURN = 0U,
  FOUND = 1U,
  UNSAFE = 2U,
};

bool HasFlag(const MapWaypoint& point, uint8_t flag) {
  return (point.flags & flag) != 0U;
}

bool IsProtected(const MapWaypoint& point) {
  return HasFlag(point, MAP_WP_START) || HasFlag(point, MAP_WP_ENDPOINT) ||
         HasFlag(point, MAP_WP_MANUAL_MARK);
}

float Distance(float firstX, float firstY, float secondX, float secondY) {
  return hypotf(firstX - secondX, firstY - secondY);
}

float Distance(const MapWaypoint& first, const MapWaypoint& second) {
  return Distance(static_cast<float>(first.xMm), static_cast<float>(first.yMm),
                  static_cast<float>(second.xMm),
                  static_cast<float>(second.yMm));
}

float SegmentDirectionDeg(const MapWaypoint& first, const MapWaypoint& second) {
  return atan2f(static_cast<float>(second.yMm - first.yMm),
                static_cast<float>(second.xMm - first.xMm)) *
         kRadToDeg;
}

float DirectionDeltaDeg(float firstDeg, float secondDeg) {
  float delta = firstDeg - secondDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta <= -180.0f) delta += 360.0f;
  return fabsf(delta);
}

float CrossProduct(float firstX, float firstY, float secondX,
                   float secondY) {
  return firstX * secondY - firstY * secondX;
}

float PointToLineDistance(float pointX, float pointY,
                          const SemanticLine& line) {
  return fabsf(CrossProduct(line.directionX, line.directionY,
                            pointX - line.pointX, pointY - line.pointY));
}

uint16_t DiagnosticU16(float value) {
  if (!isfinite(value) || value <= 0.0f) return 0U;
  if (value >= 65535.0f) return 65535U;
  return static_cast<uint16_t>(lroundf(value));
}

float PointToSegmentDistance(float pointX, float pointY, float firstX,
                             float firstY, float secondX, float secondY) {
  const float dx = secondX - firstX;
  const float dy = secondY - firstY;
  const float lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= 0.0f) return Distance(pointX, pointY, firstX, firstY);
  float t = ((pointX - firstX) * dx + (pointY - firstY) * dy) /
            lengthSquared;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return Distance(pointX, pointY, firstX + t * dx, firstY + t * dy);
}

float PathLength(const MapRouteData& route, uint16_t first, uint16_t last) {
  if (first >= last || last >= route.header.waypointCount) return 0.0f;
  float total = 0.0f;
  for (uint16_t index = first + 1U; index <= last; ++index) {
    total += Distance(route.waypoints[index - 1U], route.waypoints[index]);
  }
  return total;
}

uint32_t RouteLength(const MapRouteData& route) {
  const uint16_t count = route.header.waypointCount;
  if (count < 2U) return 0U;
  float total = PathLength(route, 0U, count - 1U);
  if (static_cast<MapRouteType>(route.header.routeType) == MapRouteType::CLOSED) {
    const float closure = Distance(route.waypoints[count - 1U],
                                   route.waypoints[0]);
    if (closure > kSemanticClosedClosureSkipMm) total += closure;
  }
  return total <= 0.0f ? 0U : static_cast<uint32_t>(lroundf(total));
}

bool SegmentValid(float firstX, float firstY, float secondX, float secondY) {
  const float length = Distance(firstX, firstY, secondX, secondY);
  return length >= kSemanticMinSegmentMm && length <= kSemanticMaxSegmentMm;
}

bool SegmentValid(const MapWaypoint& first, const MapWaypoint& second) {
  return SegmentValid(static_cast<float>(first.xMm),
                      static_cast<float>(first.yMm),
                      static_cast<float>(second.xMm),
                      static_cast<float>(second.yMm));
}

// PCA-style 2D line fit. This has no vertical-line special case and therefore
// handles horizontal and vertical runs identically without slope division.
bool FitLine(const MapRouteData& route, uint16_t first, uint16_t last,
             SemanticLine& line) {
  line = {};
  if (first >= last || last >= route.header.waypointCount) return false;
  const uint16_t count = static_cast<uint16_t>(last - first + 1U);
  if (count < 2U) return false;

  float meanX = 0.0f;
  float meanY = 0.0f;
  for (uint16_t index = first; index <= last; ++index) {
    meanX += static_cast<float>(route.waypoints[index].xMm);
    meanY += static_cast<float>(route.waypoints[index].yMm);
  }
  meanX /= static_cast<float>(count);
  meanY /= static_cast<float>(count);

  float covarianceXX = 0.0f;
  float covarianceYY = 0.0f;
  float covarianceXY = 0.0f;
  for (uint16_t index = first; index <= last; ++index) {
    const float dx = static_cast<float>(route.waypoints[index].xMm) - meanX;
    const float dy = static_cast<float>(route.waypoints[index].yMm) - meanY;
    covarianceXX += dx * dx;
    covarianceYY += dy * dy;
    covarianceXY += dx * dy;
  }

  const float endpointX = static_cast<float>(route.waypoints[last].xMm -
                                              route.waypoints[first].xMm);
  const float endpointY = static_cast<float>(route.waypoints[last].yMm -
                                              route.waypoints[first].yMm);
  const float endpointLength = hypotf(endpointX, endpointY);
  if (endpointLength <= 0.0f && covarianceXX <= 0.0f && covarianceYY <= 0.0f) {
    return false;
  }

  float directionDeg = 0.5f * atan2f(2.0f * covarianceXY,
                                     covarianceXX - covarianceYY) *
                       kRadToDeg;
  float directionX = cosf(directionDeg / kRadToDeg);
  float directionY = sinf(directionDeg / kRadToDeg);
  if (endpointLength > 0.0f &&
      directionX * endpointX + directionY * endpointY < 0.0f) {
    directionX = -directionX;
    directionY = -directionY;
  }

  float length = PathLength(route, first, last);
  float maxResidual = 0.0f;
  for (uint16_t index = first; index <= last; ++index) {
    const float pointX = static_cast<float>(route.waypoints[index].xMm);
    const float pointY = static_cast<float>(route.waypoints[index].yMm);
    SemanticLine fittedLine;
    fittedLine.pointX = meanX;
    fittedLine.pointY = meanY;
    fittedLine.directionX = directionX;
    fittedLine.directionY = directionY;
    const float residual = PointToLineDistance(pointX, pointY, fittedLine);
    if (residual > maxResidual) maxResidual = residual;
  }

  line.valid = length >= kSemanticMinStraightLengthMm &&
               maxResidual <= kSemanticLineFitMaxResidualMm;
  line.pointX = meanX;
  line.pointY = meanY;
  line.directionX = directionX;
  line.directionY = directionY;
  line.maxResidualMm = maxResidual;
  line.lengthMm = length;
  return line.valid;
}

bool ProtectedBetween(const MapRouteData& route, uint16_t first,
                      uint16_t last) {
  for (uint16_t index = first + 1U; index < last; ++index) {
    if (IsProtected(route.waypoints[index])) return true;
  }
  return false;
}

float PathDistanceToCorner(const MapRouteData& route, uint16_t first,
                           uint16_t last, float cornerX, float cornerY) {
  float nearest = 1000000000.0f;
  for (uint16_t index = first; index <= last; ++index) {
    const float distance = Distance(static_cast<float>(route.waypoints[index].xMm),
                                    static_cast<float>(route.waypoints[index].yMm),
                                    cornerX, cornerY);
    if (distance < nearest) nearest = distance;
  }
  return nearest;
}

float CornerPathDeviation(const MapRouteData& route, uint16_t first,
                          uint16_t last, float cornerX, float cornerY) {
  if (first >= last || last >= route.header.waypointCount) return 1000000000.0f;
  const float firstX = static_cast<float>(route.waypoints[first].xMm);
  const float firstY = static_cast<float>(route.waypoints[first].yMm);
  const float lastX = static_cast<float>(route.waypoints[last].xMm);
  const float lastY = static_cast<float>(route.waypoints[last].yMm);
  float maxDeviation = 0.0f;
  for (uint16_t index = first; index <= last; ++index) {
    const float pointX = static_cast<float>(route.waypoints[index].xMm);
    const float pointY = static_cast<float>(route.waypoints[index].yMm);
    const float deviation =
        fminf(PointToSegmentDistance(pointX, pointY, firstX, firstY, cornerX,
                                     cornerY),
              PointToSegmentDistance(pointX, pointY, cornerX, cornerY, lastX,
                                     lastY));
    if (deviation > maxDeviation) maxDeviation = deviation;
  }
  return maxDeviation;
}

bool LineIntersection(const SemanticLine& first, const SemanticLine& second,
                      float& intersectionX, float& intersectionY) {
  const float determinant =
      CrossProduct(first.directionX, first.directionY, second.directionX,
                   second.directionY);
  if (fabsf(determinant) < 0.01f) return false;
  const float offsetX = second.pointX - first.pointX;
  const float offsetY = second.pointY - first.pointY;
  const float lineT = CrossProduct(offsetX, offsetY, second.directionX,
                                   second.directionY) /
                      determinant;
  intersectionX = first.pointX + lineT * first.directionX;
  intersectionY = first.pointY + lineT * first.directionY;
  return isfinite(intersectionX) && isfinite(intersectionY);
}

uint16_t NearestRepresentative(const MapRouteData& route, uint16_t first,
                               uint16_t last, float targetX, float targetY) {
  uint16_t best = first;
  float bestDistance = 1000000000.0f;
  float bestTurn = -1.0f;
  for (uint16_t index = first; index <= last; ++index) {
    const MapWaypoint& point = route.waypoints[index];
    if (IsProtected(point)) continue;
    float turn = 0.0f;
    if (index > 0U && index + 1U < route.header.waypointCount) {
      turn = DirectionDeltaDeg(
          SegmentDirectionDeg(route.waypoints[index - 1U], point),
          SegmentDirectionDeg(point, route.waypoints[index + 1U]));
    }
    const float distance =
        Distance(static_cast<float>(point.xMm), static_cast<float>(point.yMm),
                 targetX, targetY);
    const bool preferred = HasFlag(point, MAP_WP_AUTO_CORNER);
    if ((preferred && !HasFlag(route.waypoints[best], MAP_WP_AUTO_CORNER)) ||
        (preferred == HasFlag(route.waypoints[best], MAP_WP_AUTO_CORNER) &&
         (turn > bestTurn + 0.5f ||
          (fabsf(turn - bestTurn) <= 0.5f && distance < bestDistance)))) {
      best = index;
      bestDistance = distance;
      bestTurn = turn;
    }
  }
  return best;
}

void RecordCorner(SemanticRouteMetrics& metrics, uint16_t index,
                  const CornerCandidate& candidate,
                  SemanticCornerSource source, bool accepted) {
  if (metrics.cornerDiagnosticCount >= STM32_MAP_MAX_WAYPOINTS) {
    metrics.diagnosticOverflow = true;
    return;
  }
  SemanticCornerDiagnostic& diagnostic =
      metrics.corners[metrics.cornerDiagnosticCount++];
  diagnostic.index = index;
  diagnostic.angleCdeg = static_cast<int16_t>(lroundf(candidate.angleDeg * 100.0f));
  diagnostic.incomingDirectionCdeg = static_cast<int16_t>(lroundf(
      candidate.incomingDirectionDeg * 100.0f));
  diagnostic.outgoingDirectionCdeg = static_cast<int16_t>(lroundf(
      candidate.outgoingDirectionDeg * 100.0f));
  diagnostic.shiftMm = DiagnosticU16(candidate.shiftMm);
  diagnostic.deviationMm = DiagnosticU16(candidate.deviationMm);
  diagnostic.source = source;
  diagnostic.accepted = accepted;
}

void RecordManualCorners(const MapRouteData& route,
                         SemanticRouteMetrics& metrics) {
  for (uint16_t index = 1U; index + 1U < route.header.waypointCount; ++index) {
    if (!HasFlag(route.waypoints[index], MAP_WP_MANUAL_MARK)) continue;
    CornerCandidate candidate;
    candidate.angleDeg = DirectionDeltaDeg(
        SegmentDirectionDeg(route.waypoints[index - 1U],
                            route.waypoints[index]),
        SegmentDirectionDeg(route.waypoints[index],
                            route.waypoints[index + 1U]));
    candidate.incomingDirectionDeg =
        SegmentDirectionDeg(route.waypoints[index - 1U],
                           route.waypoints[index]);
    candidate.outgoingDirectionDeg =
        SegmentDirectionDeg(route.waypoints[index],
                           route.waypoints[index + 1U]);
    RecordCorner(metrics, index, candidate, SemanticCornerSource::MANUAL,
                 true);
  }
}

CornerSearchResult FindCorner(const MapRouteData& route, uint16_t begin,
                              uint16_t end, CornerCandidate& result) {
  result = {};
  if (end <= begin + 1U) return CornerSearchResult::NO_TURN;

  const float incomingBase = SegmentDirectionDeg(route.waypoints[begin],
                                                 route.waypoints[begin + 1U]);
  uint16_t firstTransition = end;
  for (uint16_t segment = begin + 1U; segment < end; ++segment) {
    const float direction =
        SegmentDirectionDeg(route.waypoints[segment],
                            route.waypoints[segment + 1U]);
    if (DirectionDeltaDeg(direction, incomingBase) >= kSemanticTurnAngleDeg) {
      firstTransition = segment;
      break;
    }
  }
  if (firstTransition == end) return CornerSearchResult::NO_TURN;

  // Find the earliest outgoing window that is long and straight enough. This
  // intentionally includes transition samples only when the PCA residual says
  // they still belong to the outgoing corridor.
  for (uint16_t outgoingStart = firstTransition;
       outgoingStart < end; ++outgoingStart) {
    const float outgoingBase =
        SegmentDirectionDeg(route.waypoints[outgoingStart],
                            route.waypoints[outgoingStart + 1U]);
    uint16_t outgoingLastSegment = end - 1U;
    for (uint16_t segment = outgoingStart + 1U; segment < end; ++segment) {
      const float direction =
          SegmentDirectionDeg(route.waypoints[segment],
                              route.waypoints[segment + 1U]);
      if (DirectionDeltaDeg(direction, outgoingBase) >=
          kSemanticTurnAngleDeg) {
        outgoingLastSegment = segment - 1U;
        break;
      }
    }
    if (outgoingLastSegment < outgoingStart) continue;
    const bool outgoingHasStablePair =
        outgoingStart == outgoingLastSegment ||
        DirectionDeltaDeg(
            SegmentDirectionDeg(route.waypoints[outgoingStart],
                                route.waypoints[outgoingStart + 1U]),
            SegmentDirectionDeg(route.waypoints[outgoingStart + 1U],
                                route.waypoints[outgoingStart + 2U])) <=
            kSemanticStraightDirectionDeg;
    if (!outgoingHasStablePair) continue;

    SemanticLine incomingLine;
    SemanticLine outgoingLine;
    if (!FitLine(route, begin, firstTransition, incomingLine) ||
        !FitLine(route, outgoingStart, outgoingLastSegment + 1U,
                 outgoingLine)) {
      continue;
    }

    const float incomingDirection =
        atan2f(incomingLine.directionY, incomingLine.directionX) * kRadToDeg;
    const float outgoingDirection =
        atan2f(outgoingLine.directionY, outgoingLine.directionX) * kRadToDeg;
    const float angle =
        DirectionDeltaDeg(outgoingDirection, incomingDirection);
    if (angle < kSemanticTurnAngleDeg || angle > kSemanticMaxCornerAngleDeg) {
      continue;
    }

    float intersectionX = 0.0f;
    float intersectionY = 0.0f;
    if (!LineIntersection(incomingLine, outgoingLine, intersectionX,
                          intersectionY)) {
      continue;
    }

    result.incomingEnd = firstTransition;
    result.outgoingStart = outgoingStart;
    result.outgoingEnd = outgoingLastSegment + 1U;
    result.angleDeg = angle;
    result.incomingDirectionDeg = incomingDirection;
    result.outgoingDirectionDeg = outgoingDirection;
    result.cornerX = intersectionX;
    result.cornerY = intersectionY;
    result.shiftMm = PathDistanceToCorner(route, firstTransition,
                                          outgoingStart, intersectionX,
                                          intersectionY);
    result.representative = NearestRepresentative(
        route, firstTransition, outgoingStart, intersectionX, intersectionY);
    result.deviationMm = CornerPathDeviation(
        route, begin, result.outgoingEnd, intersectionX, intersectionY);
    const float incomingShift = Distance(
        intersectionX, intersectionY,
        static_cast<float>(route.waypoints[firstTransition].xMm),
        static_cast<float>(route.waypoints[firstTransition].yMm));
    const float outgoingShift = Distance(
        intersectionX, intersectionY,
        static_cast<float>(route.waypoints[outgoingStart].xMm),
        static_cast<float>(route.waypoints[outgoingStart].yMm));
    result.syntheticSafe = !ProtectedBetween(route, firstTransition,
                                              result.outgoingEnd) &&
                           result.shiftMm <= kSemanticMaxCornerShiftMm &&
                           incomingShift <= kSemanticMaxCornerShiftMm &&
                           outgoingShift <= kSemanticMaxCornerShiftMm &&
                           result.deviationMm <= kSemanticMaxPathDeviationMm &&
                           SegmentValid(
                               static_cast<float>(route.waypoints[begin].xMm),
                               static_cast<float>(route.waypoints[begin].yMm),
                               intersectionX, intersectionY) &&
                           SegmentValid(
                               intersectionX, intersectionY,
                               static_cast<float>(
                                   route.waypoints[result.outgoingEnd].xMm),
                               static_cast<float>(
                                   route.waypoints[result.outgoingEnd].yMm));
    result.valid = true;
    return CornerSearchResult::FOUND;
  }

  // A direction change was real, but no pair of reliable straight fits was
  // found. The caller must retain CLEAN rather than cutting this corner.
  return CornerSearchResult::UNSAFE;
}

bool AppendPoint(const MapWaypoint& point, MapRouteData& output,
                 uint16_t& outputCount) {
  if (outputCount >= STM32_MAP_MAX_WAYPOINTS) return false;
  if (outputCount > 0U && output.waypoints[outputCount - 1U].xMm == point.xMm &&
      output.waypoints[outputCount - 1U].yMm == point.yMm &&
      output.waypoints[outputCount - 1U].flags == point.flags) {
    return true;
  }
  output.waypoints[outputCount++] = point;
  return true;
}

bool AppendSyntheticCorner(const CornerCandidate& candidate,
                           MapRouteData& output, uint16_t& outputCount) {
  MapWaypoint point;
  point.xMm = static_cast<int32_t>(lroundf(candidate.cornerX));
  point.yMm = static_cast<int32_t>(lroundf(candidate.cornerY));
  float heading = candidate.outgoingDirectionDeg;
  while (heading > 180.0f) heading -= 360.0f;
  while (heading <= -180.0f) heading += 360.0f;
  point.headingCdeg = static_cast<int16_t>(lroundf(heading * 100.0f));
  point.flags = MAP_WP_AUTO_CORNER;
  return AppendPoint(point, output, outputCount);
}

// Keep existing points only when a direct semantic segment would exceed the
// hard maximum. Normal short/medium straight runs therefore collapse to one
// anchor, while a long run remains bounded without synthetic over-sampling.
bool AppendBoundedStraight(const MapRouteData& route, uint16_t begin,
                           uint16_t end, float targetX, float targetY,
                           MapRouteData& output, uint16_t& outputCount) {
  if (begin >= end) return true;
  float lastX = static_cast<float>(output.waypoints[outputCount - 1U].xMm);
  float lastY = static_cast<float>(output.waypoints[outputCount - 1U].yMm);
  if (Distance(lastX, lastY, targetX, targetY) <= kSemanticMaxSegmentMm) {
    return true;
  }
  for (uint16_t index = begin + 1U; index <= end; ++index) {
    const float pointX = static_cast<float>(route.waypoints[index].xMm);
    const float pointY = static_cast<float>(route.waypoints[index].yMm);
    if (Distance(lastX, lastY, pointX, pointY) >=
        kSemanticMaxSegmentMm * 0.75f) {
      if (!AppendPoint(route.waypoints[index], output, outputCount)) return false;
      lastX = pointX;
      lastY = pointY;
    }
  }
  return Distance(lastX, lastY, targetX, targetY) <= kSemanticMaxSegmentMm;
}

bool AppendSection(const MapRouteData& cleanRoute, uint16_t begin, uint16_t end,
                   MapRouteData& output, uint16_t& outputCount,
                   SemanticRouteMetrics& metrics) {
  if (begin > end || end >= cleanRoute.header.waypointCount) return false;
  if (!AppendPoint(cleanRoute.waypoints[begin], output, outputCount)) return false;
  uint16_t current = begin;
  while (current < end) {
    CornerCandidate candidate;
    const CornerSearchResult search =
        FindCorner(cleanRoute, current, end, candidate);
    if (search == CornerSearchResult::NO_TURN) {
      SemanticLine line;
      if (FitLine(cleanRoute, current, end, line)) ++metrics.straightRuns;
      if (!AppendBoundedStraight(
              cleanRoute, current, end,
              static_cast<float>(cleanRoute.waypoints[end].xMm),
              static_cast<float>(cleanRoute.waypoints[end].yMm), output,
              outputCount) ||
          !AppendPoint(cleanRoute.waypoints[end], output, outputCount)) {
        return false;
      }
      return true;
    }
    if (search == CornerSearchResult::UNSAFE) return false;

    ++metrics.turnRegions;
    const uint16_t diagnosticIndex = outputCount;
    if (candidate.syntheticSafe) {
      if (!AppendBoundedStraight(
              cleanRoute, current, candidate.incomingEnd, candidate.cornerX,
              candidate.cornerY, output, outputCount) ||
          !AppendSyntheticCorner(candidate, output, outputCount)) {
        return false;
      }
      ++metrics.syntheticCorners;
      metrics.straightRuns = static_cast<uint16_t>(metrics.straightRuns + 2U);
      RecordCorner(metrics, diagnosticIndex, candidate,
                   SemanticCornerSource::SYNTHETIC, true);
    } else {
      const MapWaypoint& representative =
          cleanRoute.waypoints[candidate.representative];
      const float representativeDeviation = CornerPathDeviation(
          cleanRoute, current, candidate.outgoingEnd,
          static_cast<float>(representative.xMm),
          static_cast<float>(representative.yMm));
      const bool representativeSafe =
          !IsProtected(representative) &&
          representativeDeviation <= kSemanticMaxPathDeviationMm &&
          SegmentValid(output.waypoints[outputCount - 1U], representative) &&
          SegmentValid(representative,
                       cleanRoute.waypoints[candidate.outgoingEnd]);
      candidate.deviationMm = representativeDeviation;
      if (!representativeSafe ||
          !AppendPoint(representative, output, outputCount)) {
        RecordCorner(metrics, diagnosticIndex, candidate,
                     SemanticCornerSource::RAW, false);
        return false;
      }
      ++metrics.rawCornersUsed;
      metrics.straightRuns = static_cast<uint16_t>(metrics.straightRuns + 2U);
      RecordCorner(metrics, diagnosticIndex, candidate,
                   SemanticCornerSource::RAW, true);
    }

    if (!AppendPoint(cleanRoute.waypoints[candidate.outgoingEnd], output,
                    outputCount)) {
      return false;
    }
    current = candidate.outgoingEnd;
  }
  return true;
}

bool ProtectedAnchorsUnchanged(const MapRouteData& clean,
                               const MapRouteData& semantic) {
  uint16_t semanticSearch = 0U;
  for (uint16_t index = 0U; index < clean.header.waypointCount; ++index) {
    if (!IsProtected(clean.waypoints[index])) continue;
    bool found = false;
    for (; semanticSearch < semantic.header.waypointCount; ++semanticSearch) {
      const MapWaypoint& candidate = semantic.waypoints[semanticSearch];
      if (candidate.xMm == clean.waypoints[index].xMm &&
          candidate.yMm == clean.waypoints[index].yMm &&
          candidate.headingCdeg == clean.waypoints[index].headingCdeg &&
          candidate.flags == clean.waypoints[index].flags) {
        found = true;
        ++semanticSearch;
        break;
      }
    }
    if (!found) return false;
  }
  return semantic.header.waypointCount >= 2U &&
         semantic.waypoints[0].xMm == clean.waypoints[0].xMm &&
         semantic.waypoints[0].yMm == clean.waypoints[0].yMm &&
         semantic.waypoints[0].headingCdeg == clean.waypoints[0].headingCdeg &&
         semantic.waypoints[0].flags == clean.waypoints[0].flags &&
         semantic.waypoints[semantic.header.waypointCount - 1U].xMm ==
             clean.waypoints[clean.header.waypointCount - 1U].xMm &&
         semantic.waypoints[semantic.header.waypointCount - 1U].yMm ==
             clean.waypoints[clean.header.waypointCount - 1U].yMm &&
         semantic.waypoints[semantic.header.waypointCount - 1U].headingCdeg ==
             clean.waypoints[clean.header.waypointCount - 1U].headingCdeg &&
         semantic.waypoints[semantic.header.waypointCount - 1U].flags ==
             clean.waypoints[clean.header.waypointCount - 1U].flags;
}

float PointToSemanticRoute(const MapWaypoint& point,
                           const MapRouteData& semantic) {
  float nearest = 1000000000.0f;
  for (uint16_t index = 1U; index < semantic.header.waypointCount; ++index) {
    const MapWaypoint& first = semantic.waypoints[index - 1U];
    const MapWaypoint& second = semantic.waypoints[index];
    const float distance = PointToSegmentDistance(
        static_cast<float>(point.xMm), static_cast<float>(point.yMm),
        static_cast<float>(first.xMm), static_cast<float>(first.yMm),
        static_cast<float>(second.xMm), static_cast<float>(second.yMm));
    if (distance < nearest) nearest = distance;
  }
  return nearest;
}

bool ValidateSemantic(const MapRouteData& clean, const MapRouteData& semantic,
                      SemanticRouteMetrics& metrics) {
  const uint16_t count = semantic.header.waypointCount;
  if (count < 2U || count > STM32_MAP_MAX_WAYPOINTS ||
      !ProtectedAnchorsUnchanged(clean, semantic)) {
    metrics.fallbackReason = "ANCHOR_CHANGED";
    return false;
  }
  for (uint16_t index = 1U; index < count; ++index) {
    if (!SegmentValid(semantic.waypoints[index - 1U], semantic.waypoints[index])) {
      metrics.fallbackReason = "SEGMENT";
      return false;
    }
  }
  if (static_cast<MapRouteType>(semantic.header.routeType) ==
          MapRouteType::CLOSED &&
      Distance(semantic.waypoints[count - 1U], semantic.waypoints[0]) >
          kSemanticClosedClosureSkipMm) {
    if (!SegmentValid(semantic.waypoints[count - 1U], semantic.waypoints[0])) {
      metrics.fallbackReason = "CLOSURE";
      return false;
    }
  }

  uint32_t maxDeviation = 0U;
  for (uint16_t index = 0U; index < clean.header.waypointCount; ++index) {
    const uint32_t deviation = static_cast<uint32_t>(lroundf(
        PointToSemanticRoute(clean.waypoints[index], semantic)));
    if (deviation > maxDeviation) maxDeviation = deviation;
  }
  metrics.maxDeviationMm = maxDeviation;
  if (maxDeviation > static_cast<uint32_t>(kSemanticMaxPathDeviationMm)) {
    metrics.fallbackReason = "CORRIDOR";
    return false;
  }

  const float cleanLength = static_cast<float>(metrics.cleanLengthMm);
  const float semanticLength = static_cast<float>(metrics.semanticLengthMm);
  if (cleanLength > 0.0f &&
      semanticLength < cleanLength *
                           (100.0f - kSemanticMaxLengthReductionPercent) /
                           100.0f) {
    metrics.fallbackReason = "LENGTH_RATIO";
    return false;
  }
  if (semantic.header.routeLengthMm != 0U &&
      RouteLength(semantic) == 0U) {
    metrics.fallbackReason = "LENGTH";
    return false;
  }
  metrics.fallbackReason = "NONE";
  return true;
}

void SetFallback(const MapRouteData& cleanRoute, MapRouteData& semanticRoute,
                 SemanticRouteMetrics& metrics, const char* reason) {
  semanticRoute = cleanRoute;
  metrics.semanticPoints = cleanRoute.header.waypointCount;
  metrics.semanticLengthMm = metrics.cleanLengthMm;
  metrics.maxDeviationMm = 0U;
  metrics.syntheticCorners = 0U;
  metrics.rawCornersUsed = 0U;
  metrics.accepted = false;
  metrics.fallbackReason = reason;
}
}  // namespace

bool optimizeSemanticRoute(const MapRouteData& cleanRoute,
                           MapRouteData& semanticRoute,
                           SemanticRouteMetrics& metrics) {
  SemanticRouteMetrics resetMetrics;
  metrics = resetMetrics;
  semanticRoute = cleanRoute;
  metrics.rawPoints = cleanRoute.header.waypointCount;
  metrics.cleanPoints = cleanRoute.header.waypointCount;
  metrics.cleanLengthMm = RouteLength(cleanRoute);
  const uint16_t count = cleanRoute.header.waypointCount;
  if (count < 2U || count > STM32_MAP_MAX_WAYPOINTS) {
    SetFallback(cleanRoute, semanticRoute, metrics, "CLEAN_POINTS");
    return false;
  }
  for (uint16_t index = 0U; index < count; ++index) {
    if (HasFlag(cleanRoute.waypoints[index], MAP_WP_MANUAL_MARK)) {
      ++metrics.manualKept;
    }
  }
  RecordManualCorners(cleanRoute, metrics);

  semanticRoute = {};
  semanticRoute.header = cleanRoute.header;
  uint16_t outputCount = 0U;
  uint16_t sectionBegin = 0U;
  // Protected manual anchors split independent sections. For CLOSED routes
  // the Pn -> P0 seam is intentionally not fitted in this first version.
  for (uint16_t index = 1U; index < count; ++index) {
    if (!IsProtected(cleanRoute.waypoints[index])) continue;
    if (!AppendSection(cleanRoute, sectionBegin, index, semanticRoute,
                       outputCount, metrics)) {
      SetFallback(cleanRoute, semanticRoute, metrics, "CORNER_UNSAFE");
      return false;
    }
    sectionBegin = index;
  }
  if (sectionBegin < count - 1U &&
      !AppendSection(cleanRoute, sectionBegin, count - 1U, semanticRoute,
                     outputCount, metrics)) {
    SetFallback(cleanRoute, semanticRoute, metrics, "CORNER_UNSAFE");
    return false;
  }
  if (sectionBegin == count - 1U &&
      !AppendPoint(cleanRoute.waypoints[sectionBegin], semanticRoute,
                   outputCount)) {
    SetFallback(cleanRoute, semanticRoute, metrics, "POINT_OVERFLOW");
    return false;
  }
  semanticRoute.header.waypointCount = outputCount;
  semanticRoute.header.payloadBytes =
      static_cast<uint16_t>(outputCount * sizeof(MapWaypoint));
  semanticRoute.header.routeLengthMm = RouteLength(semanticRoute);
  metrics.semanticPoints = outputCount;
  metrics.semanticLengthMm = semanticRoute.header.routeLengthMm;
  if (!ValidateSemantic(cleanRoute, semanticRoute, metrics)) {
    SetFallback(cleanRoute, semanticRoute, metrics, metrics.fallbackReason);
    return false;
  }
  metrics.accepted = true;
  metrics.fallbackReason = "NONE";
  return true;
}

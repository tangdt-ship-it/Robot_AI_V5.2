#include <map/route_cleaner.h>

#include <math.h>
#include <string.h>

namespace {
// Conservative geometry limits. Keep these in one place so the cleaner is
// auditable and the thresholds cannot drift between optimization passes.
constexpr float kOptimizeDuplicateDistanceMm = 35.0f;
constexpr float kOptimizeDuplicateHeadingDeg = 4.0f;
constexpr float kOptimizeShortSegmentMm = 70.0f;
constexpr float kOptimizeLineDeviationMm = 50.0f;
constexpr float kOptimizeStraightAngleDeg = 10.0f;
constexpr float kOptimizeCornerAngleDeg = 30.0f;
constexpr float kOptimizeCornerClusterRadiusMm = 140.0f;
constexpr float kOptimizeMaxDeviationMm = 60.0f;
constexpr uint8_t kOptimizeMaxLengthReductionPercent = 15U;
constexpr float kOptimizeMinimumSegmentMm = 20.0f;
constexpr float kOptimizeMaximumSegmentMm = 5000.0f;
constexpr float kOptimizeClosedClosureSkipMm = 20.0f;
// A short reverse spike is normally caused by a manual correction while
// teaching. It is removable only when the shortcut remains inside the
// corridor guard below, so a real corner or U-turn is preserved.
constexpr float kOptimizeBacktrackAngleDeg = 135.0f;
constexpr float kOptimizeBacktrackShortLegMm = 180.0f;
constexpr float kOptimizeBacktrackLegRatio = 0.45f;
// Auto-distance sampling can straddle a real turn: one sample is before the
// corner and the next is already on the outgoing leg. Fit their two long-leg
// lines to one measured corner so replay does not turn early at a stale sample.
constexpr float kOptimizeCornerFitMinAngleDeg = 35.0f;
constexpr float kOptimizeCornerFitMaxAngleDeg = 150.0f;
constexpr float kOptimizeCornerFitMaxExtensionMm = 1000.0f;
constexpr float kRadToDeg = 57.29577951308232f;

bool HasFlag(const MapWaypoint& point, uint8_t flag) {
  return (point.flags & flag) != 0U;
}

bool IsProtected(const MapWaypoint& point) {
  return HasFlag(point, MAP_WP_START) || HasFlag(point, MAP_WP_ENDPOINT) ||
         HasFlag(point, MAP_WP_MANUAL_MARK);
}

float Distance(const MapWaypoint& first, const MapWaypoint& second) {
  const float dx = static_cast<float>(first.xMm - second.xMm);
  const float dy = static_cast<float>(first.yMm - second.yMm);
  return sqrtf(dx * dx + dy * dy);
}

float HeadingDelta(const MapWaypoint& first, const MapWaypoint& second) {
  float delta = static_cast<float>(first.headingCdeg - second.headingCdeg) /
                100.0f;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta <= -180.0f) delta += 360.0f;
  return fabsf(delta);
}

float PointToSegmentDistance(const MapWaypoint& point,
                             const MapWaypoint& first,
                             const MapWaypoint& second) {
  const float px = static_cast<float>(point.xMm);
  const float py = static_cast<float>(point.yMm);
  const float ax = static_cast<float>(first.xMm);
  const float ay = static_cast<float>(first.yMm);
  const float dx = static_cast<float>(second.xMm - first.xMm);
  const float dy = static_cast<float>(second.yMm - first.yMm);
  const float lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= 0.0f) return Distance(point, first);
  float t = ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const float closestX = ax + t * dx;
  const float closestY = ay + t * dy;
  const float errorX = px - closestX;
  const float errorY = py - closestY;
  return sqrtf(errorX * errorX + errorY * errorY);
}

float DirectionChange(const MapWaypoint& first, const MapWaypoint& middle,
                      const MapWaypoint& last) {
  const float incoming = atan2f(static_cast<float>(middle.yMm - first.yMm),
                                static_cast<float>(middle.xMm - first.xMm)) *
                         kRadToDeg;
  const float outgoing = atan2f(static_cast<float>(last.yMm - middle.yMm),
                                static_cast<float>(last.xMm - middle.xMm)) *
                         kRadToDeg;
  float delta = outgoing - incoming;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta <= -180.0f) delta += 360.0f;
  return fabsf(delta);
}

float LineHeading(const MapWaypoint& first, const MapWaypoint& second) {
  return atan2f(static_cast<float>(second.yMm - first.yMm),
                static_cast<float>(second.xMm - first.xMm)) *
         kRadToDeg;
}

float LineDirectionChange(const MapWaypoint& first,
                          const MapWaypoint& before,
                          const MapWaypoint& after,
                          const MapWaypoint& last) {
  float delta = LineHeading(after, last) - LineHeading(first, before);
  while (delta > 180.0f) delta -= 360.0f;
  while (delta <= -180.0f) delta += 360.0f;
  return fabsf(delta);
}

float CrossProduct(float firstX, float firstY, float secondX,
                   float secondY) {
  return firstX * secondY - firstY * secondX;
}

void FitAutoCornerTransitions(const MapRouteData& raw,
                              MapRouteData& fitted,
                              uint16_t& fittedCount) {
  fitted = raw;
  fittedCount = 0U;
  const uint16_t count = raw.header.waypointCount;
  if (count < 4U) return;

  for (uint16_t index = 1U; index + 2U < count; ++index) {
    const MapWaypoint& first = raw.waypoints[index - 1U];
    const MapWaypoint& before = raw.waypoints[index];
    const MapWaypoint& after = raw.waypoints[index + 1U];
    const MapWaypoint& last = raw.waypoints[index + 2U];
    if (IsProtected(before) || IsProtected(after) ||
        !HasFlag(before, MAP_WP_AUTO_DISTANCE) ||
        !HasFlag(after, MAP_WP_AUTO_DISTANCE)) {
      continue;
    }

    const float incomingX = static_cast<float>(before.xMm - first.xMm);
    const float incomingY = static_cast<float>(before.yMm - first.yMm);
    const float outgoingX = static_cast<float>(last.xMm - after.xMm);
    const float outgoingY = static_cast<float>(last.yMm - after.yMm);
    const float incomingLength = hypotf(incomingX, incomingY);
    const float outgoingLength = hypotf(outgoingX, outgoingY);
    if (incomingLength < kOptimizeMinimumSegmentMm ||
        outgoingLength < kOptimizeMinimumSegmentMm) {
      continue;
    }

    const float turn = LineDirectionChange(first, before, after, last);
    if (turn < kOptimizeCornerFitMinAngleDeg ||
        turn > kOptimizeCornerFitMaxAngleDeg) {
      continue;
    }

    const float offsetX = static_cast<float>(after.xMm - first.xMm);
    const float offsetY = static_cast<float>(after.yMm - first.yMm);
    const float determinant =
        CrossProduct(incomingX, incomingY, outgoingX, outgoingY);
    if (fabsf(determinant) < 1.0f) continue;

    const float lineT =
        CrossProduct(offsetX, offsetY, outgoingX, outgoingY) / determinant;
    const float lineU =
        CrossProduct(offsetX, offsetY, incomingX, incomingY) / determinant;
    // The fitted intersection must lie after the incoming sample and before
    // the outgoing sample. Otherwise there is no evidence that the samples
    // straddled one physical corner.
    if (lineT < 1.0f || lineU > 0.0f) continue;

    const float cornerX = static_cast<float>(first.xMm) + lineT * incomingX;
    const float cornerY = static_cast<float>(first.yMm) + lineT * incomingY;
    const float beforeOffset = hypotf(cornerX - before.xMm,
                                      cornerY - before.yMm);
    const float afterOffset = hypotf(cornerX - after.xMm,
                                     cornerY - after.yMm);
    if (beforeOffset > kOptimizeCornerFitMaxExtensionMm ||
        afterOffset > kOptimizeCornerFitMaxExtensionMm) {
      continue;
    }

    MapWaypoint& corner = fitted.waypoints[index + 1U];
    corner.xMm = static_cast<int32_t>(lroundf(cornerX));
    corner.yMm = static_cast<int32_t>(lroundf(cornerY));
    corner.flags = static_cast<uint8_t>(
        (corner.flags & ~MAP_WP_AUTO_DISTANCE) | MAP_WP_AUTO_CORNER);
    ++fittedCount;
    // Do not reuse the fitted corner as one half of a second fit. A second
    // corner must have two fresh AUTO_DISTANCE samples around it.
    ++index;
  }
}

bool IsBacktrackNoise(const MapWaypoint& first, const MapWaypoint& middle,
                      const MapWaypoint& last) {
  const float incomingX = static_cast<float>(middle.xMm - first.xMm);
  const float incomingY = static_cast<float>(middle.yMm - first.yMm);
  const float outgoingX = static_cast<float>(last.xMm - middle.xMm);
  const float outgoingY = static_cast<float>(last.yMm - middle.yMm);
  const float incomingLength = hypotf(incomingX, incomingY);
  const float outgoingLength = hypotf(outgoingX, outgoingY);
  if (incomingLength <= 0.0f || outgoingLength <= 0.0f) return false;
  const float shortLeg = incomingLength < outgoingLength ? incomingLength
                                                           : outgoingLength;
  const float longLeg = incomingLength > outgoingLength ? incomingLength
                                                          : outgoingLength;
  if (shortLeg > kOptimizeBacktrackShortLegMm || longLeg <= 0.0f ||
      shortLeg > longLeg * kOptimizeBacktrackLegRatio) {
    return false;
  }
  return DirectionChange(first, middle, last) >=
         kOptimizeBacktrackAngleDeg;
}

float CornerStrength(const MapRouteData& raw, uint16_t index) {
  if (index == 0U || index + 1U >= raw.header.waypointCount) return 0.0f;
  return DirectionChange(raw.waypoints[index - 1U], raw.waypoints[index],
                         raw.waypoints[index + 1U]);
}

uint32_t RouteLength(const MapRouteData& route) {
  const uint16_t count = route.header.waypointCount;
  if (count < 2U) return 0U;
  float total = 0.0f;
  for (uint16_t index = 1U; index < count; ++index) {
    total += Distance(route.waypoints[index - 1U], route.waypoints[index]);
  }
  if (static_cast<MapRouteType>(route.header.routeType) == MapRouteType::CLOSED) {
    const float closure = Distance(route.waypoints[count - 1U],
                                   route.waypoints[0]);
    if (closure > kOptimizeClosedClosureSkipMm) total += closure;
  }
  return total <= 0.0f ? 0U : static_cast<uint32_t>(lroundf(total));
}

bool SegmentValid(const MapWaypoint& first, const MapWaypoint& second) {
  const float length = Distance(first, second);
  return length >= kOptimizeMinimumSegmentMm &&
         length <= kOptimizeMaximumSegmentMm;
}

bool ProtectedBetween(const MapRouteData& raw, uint16_t first,
                      uint16_t last) {
  for (uint16_t index = first + 1U; index < last; ++index) {
    if (IsProtected(raw.waypoints[index])) return true;
  }
  return false;
}

bool CornerRepresentativeNear(const MapRouteData& raw,
                              const uint16_t* selected,
                              uint16_t selectedCount,
                              uint16_t currentPosition) {
  const MapWaypoint& current = raw.waypoints[selected[currentPosition]];
  if (!HasFlag(current, MAP_WP_AUTO_CORNER)) return false;
  for (uint16_t position = 0U; position < selectedCount; ++position) {
    if (position == currentPosition) continue;
    const MapWaypoint& candidate = raw.waypoints[selected[position]];
    if (HasFlag(candidate, MAP_WP_AUTO_CORNER) &&
        Distance(current, candidate) <= kOptimizeCornerClusterRadiusMm) {
      return true;
    }
  }
  return false;
}

bool IsPreferredCorner(const MapRouteData& raw, const uint16_t* selected,
                       uint16_t selectedCount, uint16_t currentPosition) {
  const uint16_t currentIndex = selected[currentPosition];
  const MapWaypoint& current = raw.waypoints[currentIndex];
  if (!HasFlag(current, MAP_WP_AUTO_CORNER)) return false;

  float centerX = static_cast<float>(current.xMm);
  float centerY = static_cast<float>(current.yMm);
  uint16_t clusterCount = 1U;
  for (uint16_t position = 0U; position < selectedCount; ++position) {
    if (position == currentPosition) continue;
    const MapWaypoint& candidate = raw.waypoints[selected[position]];
    if (HasFlag(candidate, MAP_WP_AUTO_CORNER) &&
        Distance(current, candidate) <= kOptimizeCornerClusterRadiusMm) {
      centerX += static_cast<float>(candidate.xMm);
      centerY += static_cast<float>(candidate.yMm);
      ++clusterCount;
    }
  }
  if (clusterCount < 2U) return false;
  centerX /= static_cast<float>(clusterCount);
  centerY /= static_cast<float>(clusterCount);

  const float currentStrength = CornerStrength(raw, currentIndex);
  float bestStrength = currentStrength;
  float bestCenterDistance =
      hypotf(static_cast<float>(current.xMm) - centerX,
             static_cast<float>(current.yMm) - centerY);
  uint16_t bestIndex = currentIndex;
  for (uint16_t position = 0U; position < selectedCount; ++position) {
    if (position == currentPosition) continue;
    const uint16_t candidateIndex = selected[position];
    const MapWaypoint& candidate = raw.waypoints[candidateIndex];
    if (!HasFlag(candidate, MAP_WP_AUTO_CORNER) ||
        Distance(current, candidate) > kOptimizeCornerClusterRadiusMm) {
      continue;
    }
    const float candidateStrength = CornerStrength(raw, candidateIndex);
    const float candidateCenterDistance =
        hypotf(static_cast<float>(candidate.xMm) - centerX,
               static_cast<float>(candidate.yMm) - centerY);
    if (candidateStrength > bestStrength + 0.5f ||
        (fabsf(candidateStrength - bestStrength) <= 0.5f &&
         candidateCenterDistance < bestCenterDistance)) {
      bestStrength = candidateStrength;
      bestCenterDistance = candidateCenterDistance;
      bestIndex = candidateIndex;
    }
  }
  return bestIndex == currentIndex;
}

bool CanSkip(const MapRouteData& raw, uint16_t firstIndex,
             uint16_t middleIndex, uint16_t lastIndex, bool cornerCluster,
             bool backtrackNoise) {
  if (firstIndex >= middleIndex || middleIndex >= lastIndex ||
      ProtectedBetween(raw, firstIndex, lastIndex)) {
    return false;
  }
  const MapWaypoint& first = raw.waypoints[firstIndex];
  const MapWaypoint& middle = raw.waypoints[middleIndex];
  const MapWaypoint& last = raw.waypoints[lastIndex];
  if (IsProtected(middle) || !SegmentValid(first, last)) return false;

  const float angle = DirectionChange(first, middle, last);
  const float allowedAngle = cornerCluster ? kOptimizeCornerAngleDeg
                                           : kOptimizeStraightAngleDeg;
  if (!backtrackNoise && angle > allowedAngle) return false;

  const float middleDeviation = PointToSegmentDistance(middle, first, last);
  if (middleDeviation > kOptimizeLineDeviationMm ||
      middleDeviation > kOptimizeMaxDeviationMm) {
    return false;
  }
  // Check every RAW point crossed by the proposed shortcut. This is the
  // path-corridor guard; checking only the candidate point could cut a corner.
  for (uint16_t index = firstIndex + 1U; index < lastIndex; ++index) {
    if (IsProtected(raw.waypoints[index]) ||
        PointToSegmentDistance(raw.waypoints[index], first, last) >
            kOptimizeMaxDeviationMm) {
      return false;
    }
  }
  return true;
}

void CopySelected(const MapRouteData& raw, const uint16_t* selected,
                  uint16_t selectedCount, MapRouteData& clean) {
  clean = {};
  clean.header = raw.header;
  clean.header.waypointCount = selectedCount;
  for (uint16_t position = 0U; position < selectedCount; ++position) {
    clean.waypoints[position] = raw.waypoints[selected[position]];
  }
  clean.header.payloadBytes = static_cast<uint16_t>(
      selectedCount * sizeof(MapWaypoint));
  clean.header.routeLengthMm = RouteLength(clean);
}

bool CleanGeometryValid(const MapRouteData& route) {
  const uint16_t count = route.header.waypointCount;
  if (count < 2U || count > STM32_MAP_MAX_WAYPOINTS ||
      (route.waypoints[0].flags & MAP_WP_START) == 0U ||
      (route.waypoints[count - 1U].flags & MAP_WP_ENDPOINT) == 0U) {
    return false;
  }
  for (uint16_t index = 1U; index < count; ++index) {
    if (!SegmentValid(route.waypoints[index - 1U], route.waypoints[index])) {
      return false;
    }
  }
  if (static_cast<MapRouteType>(route.header.routeType) == MapRouteType::CLOSED) {
    const float closure = Distance(route.waypoints[count - 1U],
                                   route.waypoints[0]);
    if (closure > kOptimizeClosedClosureSkipMm &&
        (closure < kOptimizeMinimumSegmentMm ||
         closure > kOptimizeMaximumSegmentMm)) {
      return false;
    }
  }
  return RouteLength(route) != 0U;
}

void SetFallback(const MapRouteData& raw, MapRouteData& clean,
                 RouteCleanerMetrics& metrics, const char* reason) {
  clean = raw;
  metrics.removedDuplicate = 0U;
  metrics.removedShort = 0U;
  metrics.removedCollinear = 0U;
  metrics.removedCornerCluster = 0U;
  metrics.fittedCorner = 0U;
  metrics.manualKept = 0U;
  metrics.cornerKept = 0U;
  for (uint16_t index = 0U; index < raw.header.waypointCount; ++index) {
    if (HasFlag(raw.waypoints[index], MAP_WP_MANUAL_MARK)) {
      ++metrics.manualKept;
    }
    if (HasFlag(raw.waypoints[index], MAP_WP_AUTO_CORNER)) {
      ++metrics.cornerKept;
    }
  }
  metrics.maxDeviationMm = 0U;
  metrics.cleanPoints = raw.header.waypointCount;
  metrics.cleanLengthMm = metrics.rawLengthMm;
  metrics.accepted = false;
  metrics.fallbackReason = reason;
}
}  // namespace

bool cleanMapRoute(const MapRouteData& rawRoute, MapRouteData& cleanRoute,
                   RouteCleanerMetrics& metrics) {
  metrics = {};
  metrics.rawPoints = rawRoute.header.waypointCount;
  metrics.rawLengthMm = RouteLength(rawRoute);
  cleanRoute = rawRoute;

  const uint16_t rawCount = rawRoute.header.waypointCount;
  if (rawCount < 2U || rawCount > STM32_MAP_MAX_WAYPOINTS) {
    SetFallback(rawRoute, cleanRoute, metrics, "RAW_POINTS");
    return false;
  }
  if (!CleanGeometryValid(rawRoute)) {
    SetFallback(rawRoute, cleanRoute, metrics, "RAW_GEOMETRY");
    return false;
  }

  MapRouteData fittedRoute{};
  FitAutoCornerTransitions(rawRoute, fittedRoute, metrics.fittedCorner);
  const MapRouteData& source = fittedRoute;
  uint16_t selected[STM32_MAP_MAX_WAYPOINTS] = {};
  uint16_t selectedCount = 0U;
  for (uint16_t index = 0U; index < rawCount; ++index) {
    if (selectedCount == 0U) {
      selected[selectedCount++] = index;
      continue;
    }
    const uint16_t previousIndex = selected[selectedCount - 1U];
    const MapWaypoint& current = source.waypoints[index];
    const MapWaypoint& previous = source.waypoints[previousIndex];
    const float separation = Distance(current, previous);
    const bool nearProtected =
        separation <= kOptimizeDuplicateDistanceMm &&
        (IsProtected(current) || IsProtected(previous));
    const bool duplicate =
        nearProtected ||
        (separation <= kOptimizeDuplicateDistanceMm &&
         HeadingDelta(current, previous) <= kOptimizeDuplicateHeadingDeg);
    if (!duplicate) {
      selected[selectedCount++] = index;
      continue;
    }

    const bool currentProtected = IsProtected(current);
    const bool previousProtected = IsProtected(previous);
    if (currentProtected && !previousProtected) {
      selected[selectedCount - 1U] = index;
      ++metrics.removedDuplicate;
    } else if (!currentProtected && previousProtected) {
      ++metrics.removedDuplicate;
    } else if (HasFlag(current, MAP_WP_AUTO_CORNER) &&
               !HasFlag(previous, MAP_WP_AUTO_CORNER) && !previousProtected) {
      selected[selectedCount - 1U] = index;
      ++metrics.removedDuplicate;
    } else if (!currentProtected && !HasFlag(current, MAP_WP_AUTO_CORNER)) {
      ++metrics.removedDuplicate;
    } else {
      // Two protected/corner anchors are both authoritative. Let normal
      // validation decide if their separation is physically impossible.
      selected[selectedCount++] = index;
    }
  }

  // Repeatedly remove one safe middle point at a time. The fixed selected
  // index array is deterministic and prevents any heap use in control code.
  bool changed = true;
  uint16_t passes = 0U;
  while (changed && selectedCount > 2U && passes < STM32_MAP_MAX_WAYPOINTS) {
    changed = false;
    ++passes;
    for (uint16_t position = 1U; position + 1U < selectedCount; ++position) {
      const uint16_t currentIndex = selected[position];
      const MapWaypoint& current = source.waypoints[currentIndex];
      if (IsProtected(current)) continue;
      const bool isCorner = HasFlag(current, MAP_WP_AUTO_CORNER);
      const bool backtrackNoise =
          IsBacktrackNoise(source.waypoints[selected[position - 1U]],
                           current,
                           source.waypoints[selected[position + 1U]]);
      const bool cornerCluster =
          isCorner && CornerRepresentativeNear(source, selected,
                                               selectedCount, position);
      if (isCorner && !cornerCluster && !backtrackNoise) continue;
      // Keep one existing RAW corner anchor. Prefer the strongest heading
      // change, then the point closest to the cluster centre; never invent a
      // mathematical corner.
      if (cornerCluster && !backtrackNoise &&
          IsPreferredCorner(source, selected, selectedCount, position)) {
        continue;
      }
      if (!CanSkip(source, selected[position - 1U], currentIndex,
                   selected[position + 1U], cornerCluster, backtrackNoise)) {
        continue;
      }
      for (uint16_t move = position; move + 1U < selectedCount; ++move) {
        selected[move] = selected[move + 1U];
      }
      --selectedCount;
      if (cornerCluster) {
        ++metrics.removedCornerCluster;
      } else if (backtrackNoise) {
        ++metrics.removedShort;
      } else if (Distance(source.waypoints[selected[position - 1U]],
                          current) < kOptimizeShortSegmentMm ||
                 Distance(current, source.waypoints[selected[position]]) <
                     kOptimizeShortSegmentMm) {
        ++metrics.removedShort;
      } else {
        ++metrics.removedCollinear;
      }
      changed = true;
      break;
    }
  }

  CopySelected(source, selected, selectedCount, cleanRoute);
  metrics.cleanPoints = selectedCount;
  metrics.cleanLengthMm = cleanRoute.header.routeLengthMm;
  uint32_t maxDeviation = 0U;
  for (uint16_t position = 1U; position < selectedCount; ++position) {
    const uint16_t firstIndex = selected[position - 1U];
    const uint16_t lastIndex = selected[position];
    const MapWaypoint& first = source.waypoints[firstIndex];
    const MapWaypoint& last = source.waypoints[lastIndex];
    for (uint16_t index = firstIndex + 1U; index < lastIndex; ++index) {
      const uint32_t deviation = static_cast<uint32_t>(lroundf(
          PointToSegmentDistance(rawRoute.waypoints[index], first, last)));
      if (deviation > maxDeviation) maxDeviation = deviation;
    }
  }
  metrics.maxDeviationMm = maxDeviation;
  for (uint16_t position = 0U; position < selectedCount; ++position) {
    const MapWaypoint& point = cleanRoute.waypoints[position];
    if (HasFlag(point, MAP_WP_MANUAL_MARK)) ++metrics.manualKept;
    if (HasFlag(point, MAP_WP_AUTO_CORNER)) ++metrics.cornerKept;
  }

  const bool pointsPreserved = cleanRoute.waypoints[0].xMm ==
                                   rawRoute.waypoints[0].xMm &&
                               cleanRoute.waypoints[0].yMm ==
                                   rawRoute.waypoints[0].yMm &&
                               cleanRoute.waypoints[selectedCount - 1U].xMm ==
                                   rawRoute.waypoints[rawCount - 1U].xMm &&
                               cleanRoute.waypoints[selectedCount - 1U].yMm ==
                                   rawRoute.waypoints[rawCount - 1U].yMm;
  const uint16_t rawManual = [&]() {
    uint16_t count = 0U;
    for (uint16_t index = 0U; index < rawCount; ++index) {
      if (HasFlag(rawRoute.waypoints[index], MAP_WP_MANUAL_MARK)) ++count;
    }
    return count;
  }();
  const bool lengthSafe = metrics.rawLengthMm == 0U ||
                          static_cast<float>(metrics.cleanLengthMm) >=
                              static_cast<float>(metrics.rawLengthMm) *
                                  (100.0f -
                                   kOptimizeMaxLengthReductionPercent) /
                                  100.0f;
  const bool accepted = pointsPreserved && metrics.manualKept == rawManual &&
                        metrics.maxDeviationMm <=
                            static_cast<uint32_t>(kOptimizeMaxDeviationMm) &&
                        lengthSafe && CleanGeometryValid(cleanRoute);
  if (!accepted) {
    const char* reason = !pointsPreserved       ? "ANCHOR_CHANGED"
                         : metrics.manualKept != rawManual ? "MANUAL_LOST"
                         : metrics.maxDeviationMm >
                                   static_cast<uint32_t>(kOptimizeMaxDeviationMm)
                             ? "CORRIDOR"
                         : !lengthSafe          ? "LENGTH_RATIO"
                                                : "CLEAN_GEOMETRY";
    SetFallback(rawRoute, cleanRoute, metrics, reason);
    return false;
  }
  metrics.accepted = true;
  metrics.fallbackReason = "NONE";
  return true;
}

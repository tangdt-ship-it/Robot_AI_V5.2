#ifndef STM32_MAP_SEMANTIC_ROUTE_OPTIMIZER_H
#define STM32_MAP_SEMANTIC_ROUTE_OPTIMIZER_H

#include <stdint.h>

#include <map/map_types.h>

enum class SemanticCornerSource : uint8_t {
  SYNTHETIC = 0U,
  RAW = 1U,
  MANUAL = 2U,
};

// A compact fixed-size diagnostic record. It is not stored in Flash and does
// not alter MapRouteData. One record is emitted for each detected turn region.
struct SemanticCornerDiagnostic {
  uint16_t index = 0U;
  int16_t angleCdeg = 0;
  int16_t incomingDirectionCdeg = 0;
  int16_t outgoingDirectionCdeg = 0;
  uint16_t shiftMm = 0U;
  uint16_t deviationMm = 0U;
  SemanticCornerSource source = SemanticCornerSource::RAW;
  bool accepted = false;
};

struct SemanticRouteMetrics {
  uint16_t rawPoints = 0U;
  uint16_t cleanPoints = 0U;
  uint16_t semanticPoints = 0U;
  uint16_t straightRuns = 0U;
  uint16_t turnRegions = 0U;
  uint16_t syntheticCorners = 0U;
  uint16_t rawCornersUsed = 0U;
  uint16_t manualKept = 0U;
  uint16_t cornerDiagnosticCount = 0U;
  uint32_t cleanLengthMm = 0U;
  uint32_t semanticLengthMm = 0U;
  uint32_t maxDeviationMm = 0U;
  bool diagnosticOverflow = false;
  bool accepted = false;
  const char* fallbackReason = "NONE";
  SemanticCornerDiagnostic corners[STM32_MAP_MAX_WAYPOINTS] = {};
};

// Converts a validated CLEAN route into a semantic route. The function is
// fixed-size and fail-safe: on false, semanticRoute is an unchanged copy of
// cleanRoute and the caller can save CLEAN without losing a valid route.
bool optimizeSemanticRoute(const MapRouteData& cleanRoute,
                           MapRouteData& semanticRoute,
                           SemanticRouteMetrics& metrics);

#endif  // STM32_MAP_SEMANTIC_ROUTE_OPTIMIZER_H

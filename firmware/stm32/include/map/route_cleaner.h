#ifndef STM32_MAP_ROUTE_CLEANER_H
#define STM32_MAP_ROUTE_CLEANER_H

#include <stdint.h>

#include <map/map_types.h>

// Metrics are intentionally fixed-size and diagnostic-only. The cleaner never
// allocates and never changes the on-flash route format.
struct RouteCleanerMetrics {
  uint16_t rawPoints = 0U;
  uint16_t cleanPoints = 0U;
  uint16_t removedDuplicate = 0U;
  uint16_t removedShort = 0U;
  uint16_t removedCollinear = 0U;
  uint16_t removedCornerCluster = 0U;
  uint16_t fittedCorner = 0U;
  uint16_t manualKept = 0U;
  uint16_t cornerKept = 0U;
  uint32_t rawLengthMm = 0U;
  uint32_t cleanLengthMm = 0U;
  uint32_t maxDeviationMm = 0U;
  bool accepted = false;
  const char* fallbackReason = "NONE";
};

// Safe simplification only. On false, cleanRoute contains an unchanged RAW
// route so the caller can continue through the normal validation/save path.
bool cleanMapRoute(const MapRouteData& rawRoute,
                   MapRouteData& cleanRoute,
                   RouteCleanerMetrics& metrics);

#endif  // STM32_MAP_ROUTE_CLEANER_H

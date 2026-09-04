#ifndef STM32_MAP_TYPES_H
#define STM32_MAP_TYPES_H

#include <stdint.h>

constexpr uint16_t STM32_MAP_MAX_WAYPOINTS = 128U;
constexpr uint32_t STM32_MAP_MAGIC = 0x4D415032UL;  // MAP2
// Version 2 is intentionally not binary-compatible with the ESP32 MAP V1
// NVS records. The local STM32 store owns its own format and migration is out
// of scope for this rollout.
constexpr uint16_t STM32_MAP_FORMAT_VERSION = 2U;

enum class MapSlot : uint8_t { MAP_1 = 1U, MAP_2 = 2U };
enum class MapStoreState : uint8_t {
  EMPTY = 0U,
  SAVED = 1U,
  INVALID = 2U,
  STORAGE_ERROR = 3U,
};
enum class MapRouteType : uint8_t { OPEN = 0U, CLOSED = 1U };
enum class MapReplayMode : uint8_t {
  ONCE = 0U,
  LOOP = 1U,
  RETURN = 2U,
  PING_PONG = 3U,
};
// Teach acquisition is intentionally explicit by default. AUTO_SEMANTIC is
// retained as a future extension point, but must never be selected by the
// current PS2 workflow.
enum class MapTeachMode : uint8_t {
  MANUAL_KEYFRAME = 0U,
  AUTO_SEMANTIC = 1U,
};
enum class MapControllerMode : uint8_t {
  READY = 0U,
  TEACHING = 1U,
  SAVED = 2U,
  DELETE_CONFIRM = 3U,
  REPLAY_READY = 4U,
  REPLAY_CHECKED = 5U,
  REPLAY_RUNNING = 6U,
  REPLAY_HOLD = 7U,
  REPLAY_COMPLETE = 8U,
  CLOSED_CONFIRM = 9U,
};
enum class MapReplayOperation : uint8_t {
  NONE = 0U,
  MOVE = 1U,
  TURN = 2U,
  HOLD = 3U
};

enum class MapHoldReason : uint8_t {
  NONE = 0U,
  USER = 1U,
  OBSTACLE = 2U,
  EXTERNAL_STOP = 3U,
  PS2_TAKEOVER = 4U
};

enum MapWaypointFlags : uint8_t {
  MAP_WP_START = 1U << 0,
  MAP_WP_MANUAL_MARK = 1U << 1,
  MAP_WP_AUTO_DISTANCE = 1U << 2,
  MAP_WP_AUTO_CORNER = 1U << 3,
  MAP_WP_ENDPOINT = 1U << 4,
};

struct MapWaypoint {
  int32_t xMm = 0;
  int32_t yMm = 0;
  int16_t headingCdeg = 0;
  uint8_t flags = 0U;
  uint8_t reserved = 0U;
};

// Fixed-size, versioned record header. The payload is the first
// waypointCount entries in MapRouteData::waypoints.
struct MapRouteHeader {
  uint32_t magic = 0U;
  uint16_t formatVersion = 0U;
  uint8_t slot = 0U;
  uint8_t valid = 0U;
  uint8_t routeType = 0U;
  uint8_t replayMode = 0U;
  uint16_t waypointCount = 0U;
  uint16_t payloadBytes = 0U;
  uint32_t generation = 0U;
  uint32_t routeLengthMm = 0U;
  uint32_t crc32 = 0U;
  uint32_t reserved = 0U;
};

struct MapRouteData {
  MapRouteHeader header;
  MapWaypoint waypoints[STM32_MAP_MAX_WAYPOINTS] = {};
};

struct MapSlotMetadata {
  MapStoreState state = MapStoreState::EMPTY;
  MapRouteType routeType = MapRouteType::OPEN;
  MapReplayMode replayMode = MapReplayMode::ONCE;
  uint16_t waypointCount = 0U;
  uint32_t routeLengthMm = 0U;
  uint32_t generation = 0U;
};

static_assert(sizeof(MapWaypoint) == 12U, "MAP waypoint wire/storage size changed");
static_assert(sizeof(MapRouteHeader) == 32U, "MAP header must remain 32 bytes");
static_assert(sizeof(MapRouteData) <= 2048U,
              "MAP record must fit in one audited STM32F1 Flash page");

#endif  // STM32_MAP_TYPES_H

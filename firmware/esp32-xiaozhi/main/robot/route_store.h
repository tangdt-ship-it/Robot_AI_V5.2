#ifndef XIAOZHI_ROUTE_STORE_H
#define XIAOZHI_ROUTE_STORE_H

#include <array>
#include <cstddef>
#include <cstdint>

// Persistent storage for the two Teach Route V1 slots. This is intentionally
// separate from MissionManager's transient occupancy map and breadcrumbs.
constexpr size_t kRouteSlotCount = 2;
constexpr size_t kMaxWaypointsPerMap = 128;

enum class RouteSlot : uint8_t {
    MAP_1 = 1,
    MAP_2 = 2,
};

enum RouteWaypointFlags : uint8_t {
    kRouteWaypointManualMark = 1U << 0,
};

struct RouteWaypoint {
    int32_t x_mm = 0;
    int32_t y_mm = 0;
    int16_t heading_cdeg = 0;
    uint8_t flags = 0;
    uint8_t reserved = 0;
};

struct RouteHeader {
    uint32_t magic = 0;
    uint16_t format_version = 0;
    uint8_t slot = 0;
    uint8_t valid = 0;
    uint16_t waypoint_count = 0;
    uint16_t reserved = 0;
    uint32_t generation = 0;
    uint32_t route_length_mm = 0;
    uint32_t crc32 = 0;
};

struct RouteData {
    RouteHeader header;
    std::array<RouteWaypoint, kMaxWaypointsPerMap> waypoints{};
};

enum class RouteSlotState : uint8_t {
    EMPTY,
    SAVED,
    INVALID,
    STORAGE_ERROR,
};

struct RouteSlotMetadata {
    RouteSlotState state = RouteSlotState::EMPTY;
    uint16_t waypoint_count = 0;
    uint32_t route_length_mm = 0;
    uint32_t generation = 0;
};

class RouteStore {
public:
    bool Begin();
    RouteSlotMetadata GetMetadata(RouteSlot slot) const;
    bool Load(RouteSlot slot, RouteData& route) const;
    bool Save(RouteSlot slot, RouteData& route);
    bool Delete(RouteSlot slot);

    static const char* SlotName(RouteSlot slot);
    static const char* StateName(RouteSlotState state);

private:
    static constexpr uint32_t kMagic = 0x52545631U;  // "RTV1"
    static constexpr uint16_t kFormatVersion = 1;
    static constexpr const char* kNamespace = "teach_route_v1";

    static const char* KeyForSlot(RouteSlot slot);
    static uint32_t ComputeCrc(const RouteData& route);
    static bool Validate(RouteSlot slot, const RouteData& route);
    static size_t StoredBytes(uint16_t waypoint_count);

    bool opened_ = false;
};

#endif  // XIAOZHI_ROUTE_STORE_H

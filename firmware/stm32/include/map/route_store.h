#ifndef STM32_MAP_ROUTE_STORE_H
#define STM32_MAP_ROUTE_STORE_H

#include <Arduino.h>
#include <map/map_types.h>

class MapRouteStore {
 public:
  bool begin();
  MapSlotMetadata metadata(MapSlot slot) const;
  bool load(MapSlot slot, MapRouteData& route) const;
  bool save(MapSlot slot, MapRouteData& route);
  bool erase(MapSlot slot);

  static const char* slotName(MapSlot slot);
  static const char* stateName(MapStoreState state);
  static const char* routeTypeName(MapRouteType type);
  static const char* replayModeName(MapReplayMode mode);

 private:
  static constexpr uint32_t kPageSize = 0x800UL;
  static constexpr uint8_t kPageCount = 2U;

  static uint32_t pageAddress(MapSlot slot, uint8_t pageIndex);
  static uint32_t payloadBytes(uint16_t waypointCount);
  static uint32_t computeCrc(const MapRouteData& route);
  static uint32_t computeFlashCrc(uint32_t address, uint16_t payloadBytes);
  static bool headerFieldsValid(const MapRouteHeader& header, MapSlot slot);
  static bool flashRecordValid(uint32_t address, MapSlot slot,
                               MapRouteHeader* headerOut = nullptr);
  static bool generationNewer(uint32_t candidate, uint32_t current);
  int activePage(MapSlot slot, MapRouteHeader* headerOut = nullptr) const;
  static bool programRecord(uint32_t address, const MapRouteData& route);

  bool begun_ = false;
};

#endif  // STM32_MAP_ROUTE_STORE_H

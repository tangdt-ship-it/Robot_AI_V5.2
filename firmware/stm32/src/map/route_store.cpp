#include <map/route_store.h>

#include <map/flash_layout.h>
#include <stm32f1xx_hal.h>

#include <stddef.h>
#include <string.h>

namespace {
constexpr uint32_t kCrcPolynomial = 0xEDB88320UL;
constexpr uint32_t kCrcOffset = offsetof(MapRouteHeader, crc32);

uint32_t CrcUpdate(uint32_t crc, const uint8_t* bytes, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc >> 1U) ^ (kCrcPolynomial & (0U - (crc & 1U)));
    }
  }
  return crc;
}

bool IsErased(const MapRouteHeader& header) {
  const auto* bytes = reinterpret_cast<const uint8_t*>(&header);
  for (size_t i = 0; i < sizeof(header); ++i) {
    if (bytes[i] != 0xFFU) return false;
  }
  return true;
}
}  // namespace

bool MapRouteStore::begin() {
  begun_ = true;
  return true;
}

const char* MapRouteStore::slotName(MapSlot slot) {
  return slot == MapSlot::MAP_2 ? "MAP2" : "MAP1";
}

const char* MapRouteStore::stateName(MapStoreState state) {
  switch (state) {
    case MapStoreState::EMPTY: return "EMPTY";
    case MapStoreState::SAVED: return "SAVED";
    case MapStoreState::INVALID: return "INVALID";
    case MapStoreState::STORAGE_ERROR: return "STORAGE_ERROR";
  }
  return "UNKNOWN";
}

const char* MapRouteStore::routeTypeName(MapRouteType type) {
  return type == MapRouteType::CLOSED ? "CLOSED" : "OPEN";
}

const char* MapRouteStore::replayModeName(MapReplayMode mode) {
  switch (mode) {
    case MapReplayMode::ONCE: return "ONCE";
    case MapReplayMode::LOOP: return "LOOP";
    case MapReplayMode::RETURN: return "RETURN";
    case MapReplayMode::PING_PONG: return "PING_PONG";
  }
  return "ONCE";
}

uint32_t MapRouteStore::pageAddress(MapSlot slot, uint8_t pageIndex) {
  if (pageIndex >= kPageCount) return 0U;
  if (slot == MapSlot::MAP_1) {
    return pageIndex == 0U ? Stm32FlashLayout::kMap1A
                           : Stm32FlashLayout::kMap1B;
  }
  return pageIndex == 0U ? Stm32FlashLayout::kMap2A
                         : Stm32FlashLayout::kMap2B;
}

uint32_t MapRouteStore::payloadBytes(uint16_t waypointCount) {
  return static_cast<uint32_t>(waypointCount) * sizeof(MapWaypoint);
}

uint32_t MapRouteStore::computeCrc(const MapRouteData& route) {
  MapRouteHeader header = route.header;
  header.crc32 = 0U;
  uint32_t crc = CrcUpdate(0xFFFFFFFFUL,
                           reinterpret_cast<const uint8_t*>(&header),
                           sizeof(header));
  return CrcUpdate(crc, reinterpret_cast<const uint8_t*>(route.waypoints),
                   route.header.payloadBytes);
}

uint32_t MapRouteStore::computeFlashCrc(uint32_t address,
                                        uint16_t storedPayloadBytes) {
  MapRouteHeader header = *reinterpret_cast<const MapRouteHeader*>(address);
  header.crc32 = 0U;
  uint32_t crc = CrcUpdate(0xFFFFFFFFUL,
                           reinterpret_cast<const uint8_t*>(&header),
                           sizeof(header));
  return CrcUpdate(crc,
                   reinterpret_cast<const uint8_t*>(address + sizeof(header)),
                   storedPayloadBytes);
}

bool MapRouteStore::headerFieldsValid(const MapRouteHeader& header,
                                      MapSlot slot) {
  if (header.magic != STM32_MAP_MAGIC ||
      header.formatVersion != STM32_MAP_FORMAT_VERSION ||
      header.slot != static_cast<uint8_t>(slot) || header.valid != 1U ||
      header.waypointCount < 2U ||
      header.waypointCount > STM32_MAP_MAX_WAYPOINTS ||
      header.payloadBytes != payloadBytes(header.waypointCount) ||
      header.payloadBytes > sizeof(MapRouteData) - sizeof(MapRouteHeader)) {
    return false;
  }
  if (header.routeType > static_cast<uint8_t>(MapRouteType::CLOSED) ||
      header.replayMode > static_cast<uint8_t>(MapReplayMode::PING_PONG)) {
    return false;
  }
  return true;
}

bool MapRouteStore::flashRecordValid(uint32_t address, MapSlot slot,
                                     MapRouteHeader* headerOut) {
  if (!Stm32FlashLayout::InFlash(address, kPageSize)) return false;
  const MapRouteHeader header = *reinterpret_cast<const MapRouteHeader*>(address);
  if (!headerFieldsValid(header, slot) ||
      header.crc32 != computeFlashCrc(address, header.payloadBytes)) {
    return false;
  }
  if (headerOut != nullptr) *headerOut = header;
  return true;
}

bool MapRouteStore::generationNewer(uint32_t candidate, uint32_t current) {
  return candidate != current &&
         static_cast<int32_t>(candidate - current) > 0;
}

int MapRouteStore::activePage(MapSlot slot, MapRouteHeader* headerOut) const {
  bool found = false;
  int selected = -1;
  MapRouteHeader selectedHeader{};
  for (uint8_t page = 0U; page < kPageCount; ++page) {
    MapRouteHeader header{};
    if (!flashRecordValid(pageAddress(slot, page), slot, &header)) continue;
    if (!found || generationNewer(header.generation, selectedHeader.generation)) {
      found = true;
      selected = page;
      selectedHeader = header;
    }
  }
  if (found && headerOut != nullptr) *headerOut = selectedHeader;
  return selected;
}

MapSlotMetadata MapRouteStore::metadata(MapSlot slot) const {
  MapSlotMetadata result;
  if (!begun_) {
    result.state = MapStoreState::STORAGE_ERROR;
    return result;
  }
  MapRouteHeader header{};
  const int active = activePage(slot, &header);
  if (active >= 0) {
    result.state = MapStoreState::SAVED;
    result.routeType = static_cast<MapRouteType>(header.routeType);
    result.replayMode = static_cast<MapReplayMode>(header.replayMode);
    result.waypointCount = header.waypointCount;
    result.routeLengthMm = header.routeLengthMm;
    result.generation = header.generation;
    return result;
  }

  MapRouteHeader pageHeader{};
  bool hasData = false;
  for (uint8_t page = 0U; page < kPageCount; ++page) {
    pageHeader = *reinterpret_cast<const MapRouteHeader*>(pageAddress(slot, page));
    if (!IsErased(pageHeader)) {
      hasData = true;
      break;
    }
  }
  result.state = hasData ? MapStoreState::INVALID : MapStoreState::EMPTY;
  return result;
}

bool MapRouteStore::load(MapSlot slot, MapRouteData& route) const {
  if (!begun_) return false;
  MapRouteHeader header{};
  const int active = activePage(slot, &header);
  if (active < 0) return false;
  const uint32_t address = pageAddress(slot, static_cast<uint8_t>(active));
  memset(&route, 0, sizeof(route));
  memcpy(&route, reinterpret_cast<const void*>(address),
         sizeof(MapRouteHeader) + header.payloadBytes);
  return headerFieldsValid(route.header, slot) &&
         route.header.crc32 == computeCrc(route);
}

bool MapRouteStore::programRecord(uint32_t address, const MapRouteData& route) {
  const uint32_t bytes = sizeof(MapRouteHeader) + route.header.payloadBytes;
  if (!Stm32FlashLayout::InFlash(address, kPageSize) || bytes > kPageSize ||
      (bytes & 1U) != 0U) {
    return false;
  }
  const auto* halfwords = reinterpret_cast<const uint16_t*>(&route);
  for (uint32_t offset = 0U; offset < bytes; offset += sizeof(uint16_t)) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          address + offset, halfwords[offset / 2U]) != HAL_OK) {
      return false;
    }
  }
  return true;
}

bool MapRouteStore::save(MapSlot slot, MapRouteData& route) {
  // The caller supplies a working route, not a persisted record. Its magic,
  // valid bit, generation and CRC are filled below, so do not call
  // headerFieldsValid() before those fields exist.
  if (!begun_ || (slot != MapSlot::MAP_1 && slot != MapSlot::MAP_2) ||
      route.header.waypointCount < 2U ||
      route.header.waypointCount > STM32_MAP_MAX_WAYPOINTS ||
      route.header.routeType > static_cast<uint8_t>(MapRouteType::CLOSED) ||
      route.header.replayMode >
          static_cast<uint8_t>(MapReplayMode::PING_PONG)) {
    return false;
  }
  MapRouteHeader previous{};
  const int active = activePage(slot, &previous);
  const uint8_t inactive = active == 0 ? 1U : 0U;
  route.header.magic = STM32_MAP_MAGIC;
  route.header.formatVersion = STM32_MAP_FORMAT_VERSION;
  route.header.slot = static_cast<uint8_t>(slot);
  route.header.valid = 1U;
  route.header.payloadBytes = static_cast<uint16_t>(payloadBytes(
      route.header.waypointCount));
  route.header.generation = active >= 0 ? previous.generation + 1U : 1U;
  route.header.crc32 = computeCrc(route);

  const uint32_t address = pageAddress(slot, inactive);
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = 1U;
  uint32_t pageError = 0U;
  bool ok = HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
  if (ok) ok = programRecord(address, route);
  HAL_FLASH_Lock();
  if (!ok) return false;

  MapRouteData verified{};
  return load(slot, verified) && verified.header.generation == route.header.generation;
}

bool MapRouteStore::erase(MapSlot slot) {
  if (!begun_) return false;
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = pageAddress(slot, 0U);
  erase.NbPages = kPageCount;
  uint32_t pageError = 0U;
  const bool ok = HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
  HAL_FLASH_Lock();
  return ok && metadata(slot).state == MapStoreState::EMPTY;
}

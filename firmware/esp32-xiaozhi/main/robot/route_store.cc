#include "route_store.h"

#include <cstring>

#include <esp_log.h>
#include <esp_rom_crc.h>
#include <nvs.h>

namespace {
constexpr const char* kTag = "RouteStore";
}

const char* RouteStore::SlotName(RouteSlot slot) {
    return slot == RouteSlot::MAP_1 ? "MAP 1" : "MAP 2";
}

const char* RouteStore::StateName(RouteSlotState state) {
    switch (state) {
        case RouteSlotState::EMPTY: return "EMPTY";
        case RouteSlotState::SAVED: return "SAVED";
        case RouteSlotState::INVALID: return "INVALID";
        case RouteSlotState::STORAGE_ERROR: return "STORAGE_ERROR";
    }
    return "UNKNOWN";
}

const char* RouteStore::KeyForSlot(RouteSlot slot) {
    return slot == RouteSlot::MAP_1 ? "m1" : "m2";
}

bool RouteStore::Begin() {
    nvs_handle_t handle;
    const esp_err_t error = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (error == ESP_OK) {
        nvs_close(handle);
        opened_ = true;
        return true;
    }
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        opened_ = true;
        return true;
    }
    ESP_LOGE(kTag, "NVS namespace unavailable: %s", esp_err_to_name(error));
    return false;
}

size_t RouteStore::StoredBytes(uint16_t waypoint_count) {
    return sizeof(RouteHeader) + static_cast<size_t>(waypoint_count) *
                                 sizeof(RouteWaypoint);
}

uint32_t RouteStore::ComputeCrc(const RouteData& route) {
    RouteHeader header = route.header;
    header.crc32 = 0;
    uint32_t crc = esp_rom_crc32_le(0, reinterpret_cast<const uint8_t*>(&header),
                                    sizeof(header));
    return esp_rom_crc32_le(crc,
                            reinterpret_cast<const uint8_t*>(route.waypoints.data()),
                            static_cast<uint32_t>(route.header.waypoint_count *
                                                  sizeof(RouteWaypoint)));
}

bool RouteStore::Validate(RouteSlot slot, const RouteData& route) {
    return route.header.magic == kMagic &&
           route.header.format_version == kFormatVersion &&
           route.header.slot == static_cast<uint8_t>(slot) &&
           route.header.valid == 1 &&
           route.header.waypoint_count >= 1 &&
           route.header.waypoint_count <= kMaxWaypointsPerMap &&
           route.header.crc32 == ComputeCrc(route);
}

RouteSlotMetadata RouteStore::GetMetadata(RouteSlot slot) const {
    RouteSlotMetadata metadata;
    if (!opened_) {
        metadata.state = RouteSlotState::STORAGE_ERROR;
        return metadata;
    }
    nvs_handle_t handle;
    esp_err_t error = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) return metadata;
    if (error != ESP_OK) {
        metadata.state = RouteSlotState::STORAGE_ERROR;
        return metadata;
    }
    size_t size = 0;
    error = nvs_get_blob(handle, KeyForSlot(slot), nullptr, &size);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return metadata;
    }
    if (error != ESP_OK || size < sizeof(RouteHeader) || size > sizeof(RouteData)) {
        metadata.state = error == ESP_OK ? RouteSlotState::INVALID :
                                          RouteSlotState::STORAGE_ERROR;
        nvs_close(handle);
        return metadata;
    }
    RouteData route{};
    error = nvs_get_blob(handle, KeyForSlot(slot), &route, &size);
    nvs_close(handle);
    if (error != ESP_OK || !Validate(slot, route) ||
        size != StoredBytes(route.header.waypoint_count)) {
        metadata.state = error == ESP_OK ? RouteSlotState::INVALID :
                                          RouteSlotState::STORAGE_ERROR;
        return metadata;
    }
    metadata.state = RouteSlotState::SAVED;
    metadata.waypoint_count = route.header.waypoint_count;
    metadata.route_length_mm = route.header.route_length_mm;
    metadata.generation = route.header.generation;
    return metadata;
}

bool RouteStore::Load(RouteSlot slot, RouteData& route) const {
    if (!opened_) return false;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return false;
    size_t size = sizeof(route);
    const esp_err_t error = nvs_get_blob(handle, KeyForSlot(slot), &route, &size);
    nvs_close(handle);
    return error == ESP_OK && Validate(slot, route) &&
           size == StoredBytes(route.header.waypoint_count);
}

bool RouteStore::Save(RouteSlot slot, RouteData& route) {
    if (!opened_ || route.header.waypoint_count < 1 ||
        route.header.waypoint_count > kMaxWaypointsPerMap) {
        return false;
    }
    RouteSlotMetadata previous = GetMetadata(slot);
    route.header.magic = kMagic;
    route.header.format_version = kFormatVersion;
    route.header.slot = static_cast<uint8_t>(slot);
    route.header.valid = 1;
    route.header.generation = previous.state == RouteSlotState::SAVED ?
                              previous.generation + 1 : 1;
    route.header.crc32 = ComputeCrc(route);

    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t error = nvs_set_blob(handle, KeyForSlot(slot), &route,
                                   StoredBytes(route.header.waypoint_count));
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "Save %s failed: %s", SlotName(slot), esp_err_to_name(error));
        return false;
    }
    RouteData verified{};
    if (!Load(slot, verified)) {
        ESP_LOGE(kTag, "Save %s read-back verification failed", SlotName(slot));
        return false;
    }
    return true;
}

bool RouteStore::Delete(RouteSlot slot) {
    if (!opened_) return false;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t error = nvs_erase_key(handle, KeyForSlot(slot));
    if (error == ESP_ERR_NVS_NOT_FOUND) error = ESP_OK;
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) return false;
    return GetMetadata(slot).state == RouteSlotState::EMPTY;
}

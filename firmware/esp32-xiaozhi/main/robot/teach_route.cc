#include "teach_route.h"

#include <esp_log.h>

namespace {
constexpr const char* kTag = "TeachRoute";
}

TeachRoute::TeachRoute(RobotUart* robot_uart, MissionManager* mission_manager)
    : robot_uart_(robot_uart), mission_manager_(mission_manager) {}

void TeachRoute::SetDisplay(Display* display) {
    // Compatibility shim only. MAP V1 must never touch the ESP32 TFT/LVGL
    // path. The STM32 20x4 LCD owns page/slot presentation.
    (void)display;
}

bool TeachRoute::Begin() {
    if (!store_.Begin()) {
        ESP_LOGE(kTag, "ROUTE,STORAGE=ERROR");
        return false;
    }
    for (RouteSlot slot : {RouteSlot::MAP_1, RouteSlot::MAP_2}) {
        const RouteSlotMetadata metadata = store_.GetMetadata(slot);
        ESP_LOGI(kTag, "ROUTE,BOOT,SLOT=%u,STATE=%s,POINTS=%u",
                 static_cast<unsigned>(slot),
                 RouteStore::StateName(metadata.state),
                 metadata.waypoint_count);
    }
    return true;
}

bool TeachRoute::StartInputTask() {
    // Stability isolation gate:
    // PS2 is physically attached to STM32, therefore ESP32 must not create a
    // second polling/button task for Map controls. L3/SELECT/TRIANGLE/SQUARE/
    // CIRCLE are owned by STM32. This deliberately keeps RouteStore/NVS
    // initialized while disabling every raw-PS2 Map execution path on ESP32.
    // High-level STM32 -> ESP32 Map events will be reintroduced only after the
    // static L3/SELECT reset regression is proven fixed on hardware.
    ESP_LOGI(kTag,
             "ROUTE,INPUT_OWNER=STM32,ESP32_PS2_POLL=OFF,INPUT_TASK=DISABLED");
    return true;
}

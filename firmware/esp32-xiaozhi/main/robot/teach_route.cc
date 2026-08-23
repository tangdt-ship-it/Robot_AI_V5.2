#include "teach_route.h"

#include "application.h"
#include "mission_manager.h"
#include "robot_uart.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr const char* kTag = "TeachRoute";
constexpr uint64_t kAutoWaypointPeriodUs = 100000ULL;  // 10 Hz while teaching.
constexpr float kAutoDistanceMm = 125.0f;
constexpr float kAutoHeadingDeg = 12.0f;
constexpr float kDuplicateDistanceMm = 10.0f;
constexpr float kDuplicateHeadingDeg = 1.0f;
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

    if (auto_timer_ == nullptr) {
        esp_timer_create_args_t args{};
        args.callback = &TeachRoute::AutoTimerEntry;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "teach_route_auto";
        const esp_err_t err = esp_timer_create(&args, &auto_timer_);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "ROUTE,AUTO_TIMER_CREATE=FAIL,ERR=%s",
                     esp_err_to_name(err));
            auto_timer_ = nullptr;
            return false;
        }
    }

    for (RouteSlot slot : {RouteSlot::MAP_1, RouteSlot::MAP_2}) {
        const RouteSlotMetadata metadata = store_.GetMetadata(slot);
        ESP_LOGI(kTag, "ROUTE,BOOT,SLOT=%u,STATE=%s,POINTS=%u,LENGTH_MM=%lu",
                 static_cast<unsigned>(slot),
                 RouteStore::StateName(metadata.state),
                 metadata.waypoint_count,
                 static_cast<unsigned long>(metadata.route_length_mm));
    }
    return true;
}

bool TeachRoute::StartInputTask() {
    // Deliberately no xTaskCreate here. PS2 is physically owned by STM32 and
    // Map controls arrive as high-level EVENT,MAP frames. This keeps the
    // reset-causing raw PS2 polling path permanently disabled on ESP32.
    ESP_LOGI(kTag,
             "ROUTE,INPUT_OWNER=STM32,ESP32_PS2_POLL=OFF,INPUT_TASK=DISABLED");
    return true;
}

void TeachRoute::OnMapEvent(void* context, const char* action, uint8_t slot) {
    if (context == nullptr) return;
    static_cast<TeachRoute*>(context)->HandleMapEvent(action, slot);
}

void TeachRoute::SetSelectedSlot(uint8_t slot) {
    selected_slot_ = slot == 2U ? RouteSlot::MAP_2 : RouteSlot::MAP_1;
}

void TeachRoute::HandleMapEvent(const char* action, uint8_t slot) {
    if (action == nullptr || (slot != 1U && slot != 2U)) {
        ESP_LOGW(kTag, "ROUTE,EVENT=REJECT,REASON=INVALID,ACT=%s,SLOT=%u",
                 action != nullptr ? action : "NULL",
                 static_cast<unsigned>(slot));
        return;
    }

    const RouteSlot event_slot = slot == 2U ? RouteSlot::MAP_2 : RouteSlot::MAP_1;
    ESP_LOGI(kTag, "ROUTE,EVENT=RX,ACT=%s,SLOT=%u,MODE=%u",
             action, static_cast<unsigned>(slot),
             static_cast<unsigned>(mode_));

    if (strcmp(action, "SLOT") == 0) {
        if (mode_ == Mode::TEACHING || mode_ == Mode::DELETE_CONFIRM) {
            ESP_LOGW(kTag,
                     "ROUTE,EVENT=SLOT,RESULT=IGNORED,REASON=STATE_LOCKED,ACTIVE_SLOT=%u",
                     static_cast<unsigned>(selected_slot_));
            return;
        }
        selected_slot_ = event_slot;
        UpdateMapStatus();
        return;
    }

    // During a stateful operation, the slot that started the operation is
    // authoritative. Reject a mismatched event instead of silently saving or
    // deleting the wrong map if the local LCD slot ever becomes desynchronized.
    if ((mode_ == Mode::TEACHING || mode_ == Mode::DELETE_CONFIRM) &&
        event_slot != selected_slot_) {
        ESP_LOGW(kTag,
                 "ROUTE,EVENT=REJECT,REASON=SLOT_MISMATCH,ACT=%s,EVENT_SLOT=%u,ACTIVE_SLOT=%u",
                 action, static_cast<unsigned>(event_slot),
                 static_cast<unsigned>(selected_slot_));
        return;
    }
    if (mode_ != Mode::TEACHING && mode_ != Mode::DELETE_CONFIRM) {
        selected_slot_ = event_slot;
    }

    if (strcmp(action, "TRIANGLE") == 0) {
        if (mode_ == Mode::TEACHING) {
            AddWaypoint(true);
        } else if (mode_ == Mode::READY || mode_ == Mode::LOADED) {
            StartTeach();
        }
        return;
    }

    if (strcmp(action, "SQUARE") == 0) {
        if (mode_ == Mode::TEACHING) {
            UndoWaypoint();
        } else {
            ESP_LOGI(kTag, "ROUTE,SQUARE_SHORT=IGNORED,MODE=%u",
                     static_cast<unsigned>(mode_));
        }
        return;
    }

    if (strcmp(action, "CIRCLE") == 0) {
        if (mode_ == Mode::TEACHING) {
            SaveTeach();
        } else if (mode_ == Mode::DELETE_CONFIRM) {
            ConfirmDelete();
        } else if (mode_ == Mode::READY || mode_ == Mode::LOADED) {
            LoadSelected();
        }
        return;
    }

    // Reserved for the next STM32 input phase; accepting these here keeps the
    // ESP32 state machine ready without reintroducing raw-button timing.
    if (strcmp(action, "SELECT_LONG") == 0) {
        if (mode_ == Mode::TEACHING) CancelTeach();
        return;
    }
    if (strcmp(action, "SQUARE_LONG") == 0) {
        if (mode_ == Mode::READY || mode_ == Mode::LOADED) RequestDelete();
        return;
    }

    ESP_LOGW(kTag, "ROUTE,EVENT=IGNORED,ACT=%s,SLOT=%u", action,
             static_cast<unsigned>(slot));
}

void TeachRoute::AutoTimerEntry(void* context) {
    if (context == nullptr) return;
    static_cast<TeachRoute*>(context)->AutoTimerTick();
}

void TeachRoute::AutoTimerTick() {
    if (!auto_timer_enabled_.load()) return;
    if (auto_tick_pending_.exchange(true)) return;

    // esp_timer callback runs on the shared ESP timer task. Never block that
    // task with a RobotLink transaction; hand the odometry sample to Xiaozhi's
    // existing main-task scheduler instead. The atomic gate guarantees at most
    // one pending sample if the main task is temporarily busy with camera/audio.
    Application::GetInstance().Schedule([this]() {
        if (auto_timer_enabled_.load()) UpdateAutoWaypoint();
        auto_tick_pending_.store(false);
    });
}

bool TeachRoute::StartAutoTimer() {
    if (auto_timer_ == nullptr) return false;
    StopAutoTimer();
    auto_tick_pending_.store(false);
    auto_timer_enabled_.store(true);
    const esp_err_t err = esp_timer_start_periodic(auto_timer_, kAutoWaypointPeriodUs);
    if (err != ESP_OK) {
        auto_timer_enabled_.store(false);
        ESP_LOGE(kTag, "ROUTE,AUTO_TIMER_START=FAIL,ERR=%s",
                 esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(kTag, "ROUTE,AUTO_TIMER=START,PERIOD_MS=%u",
             static_cast<unsigned>(kAutoWaypointPeriodUs / 1000ULL));
    return true;
}

void TeachRoute::StopAutoTimer() {
    auto_timer_enabled_.store(false);
    if (auto_timer_ != nullptr) {
        const esp_err_t err = esp_timer_stop(auto_timer_);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(kTag, "ROUTE,AUTO_TIMER_STOP=FAIL,ERR=%s",
                     esp_err_to_name(err));
        }
    }
    auto_tick_pending_.store(false);
}

void TeachRoute::Notify(const char* message, int duration_ms) const {
    // Map notifications remain log-only on ESP32. TFT is reserved for
    // Xiaozhi/chat/camera; the physical Map UI belongs to STM32 LCD.
    (void)duration_ms;
    ESP_LOGI(kTag, "ROUTE,NOTICE=%s", message != nullptr ? message : "");
}

void TeachRoute::UpdateMapStatus() const {
    const RouteSlotMetadata metadata = store_.GetMetadata(selected_slot_);
    const char* mode = mode_ == Mode::TEACHING ? "TEACH" :
                       mode_ == Mode::LOADED ? "LOADED" :
                       mode_ == Mode::DELETE_CONFIRM ? "DELETE" : "READY";
    const uint16_t points = mode_ == Mode::TEACHING
                                ? working_route_.header.waypoint_count
                                : metadata.waypoint_count;
    const uint32_t length_mm = mode_ == Mode::TEACHING
                                   ? working_route_.header.route_length_mm
                                   : metadata.route_length_mm;
    ESP_LOGI(kTag,
             "ROUTE,UI,OWNER=STM32,SLOT=%u,STORE=%s,MODE=%s,POINTS=%u,MAX=%u,LENGTH_MM=%lu",
             static_cast<unsigned>(selected_slot_),
             RouteStore::StateName(metadata.state), mode, points,
             static_cast<unsigned>(kMaxWaypointsPerMap),
             static_cast<unsigned long>(length_mm));
}

bool TeachRoute::ReadOdometry() {
    if (robot_uart_ == nullptr) return false;
    RobotOdometry odometry;
    if (!robot_uart_->GetOdometry(odometry, 250) || !odometry.valid) return false;
    start_x_mm_ = odometry.x_mm;
    start_y_mm_ = odometry.y_mm;
    start_heading_rad_ = odometry.heading_rad;
    odometry_valid_ = true;
    return true;
}

int16_t TeachRoute::HeadingCdeg(float heading_rad) {
    float degrees = heading_rad * 57.2957795f;
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees <= -180.0f) degrees += 360.0f;
    return static_cast<int16_t>(lroundf(degrees * 100.0f));
}

float TeachRoute::HeadingDeltaDeg(int16_t first, int16_t second) {
    float delta = static_cast<float>(first - second) / 100.0f;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return fabsf(delta);
}

bool TeachRoute::StartTeach() {
    if (mission_manager_ == nullptr || robot_uart_ == nullptr ||
        mission_manager_->IsActive() || mission_manager_->IsAiObstacleHoldActive() ||
        !robot_uart_->IsConnected()) {
        Notify("TEACH REJECT: ROBOT BUSY", 3500);
        return false;
    }

    // The event itself came from a fresh STM32 PS2 frame. Do not query raw PS2
    // back from ESP32; that polling path was the source of the reset regression.
    if (!ReadOdometry()) {
        Notify("TEACH REJECT: ODOM", 3500);
        return false;
    }

    working_route_ = {};
    working_route_.header.waypoint_count = 1;
    working_route_.waypoints[0] = {0, 0, 0, 0, 0};
    mode_ = Mode::TEACHING;
    if (!StartAutoTimer()) {
        working_route_ = {};
        odometry_valid_ = false;
        mode_ = Mode::READY;
        Notify("TEACH REJECT: TIMER", 3500);
        return false;
    }

    char notification[48];
    snprintf(notification, sizeof(notification), "TEACH %s START",
             RouteStore::SlotName(selected_slot_));
    Notify(notification, 3500);
    UpdateMapStatus();
    return true;
}

void TeachRoute::CancelTeach() {
    if (mode_ != Mode::TEACHING) return;
    StopAutoTimer();
    working_route_ = {};
    odometry_valid_ = false;
    mode_ = Mode::READY;
    Notify("TEACH CANCELLED");
    UpdateMapStatus();
}

void TeachRoute::AddWaypoint(bool manual_mark) {
    if (mode_ != Mode::TEACHING || !odometry_valid_) return;
    RobotOdometry odometry;
    if (robot_uart_ == nullptr || !robot_uart_->GetOdometry(odometry, 250) ||
        !odometry.valid) {
        Notify("ODOMETRY ERROR");
        return;
    }

    const int32_t x = static_cast<int32_t>(lroundf(odometry.x_mm - start_x_mm_));
    const int32_t y = static_cast<int32_t>(lroundf(odometry.y_mm - start_y_mm_));
    const int16_t heading = HeadingCdeg(odometry.heading_rad - start_heading_rad_);
    const uint16_t count = working_route_.header.waypoint_count;
    if (count == 0U) return;

    RouteWaypoint& last = working_route_.waypoints[count - 1];
    const float dx = static_cast<float>(x - last.x_mm);
    const float dy = static_cast<float>(y - last.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    const float heading_delta = HeadingDeltaDeg(heading, last.heading_cdeg);

    if (manual_mark && distance < kDuplicateDistanceMm &&
        heading_delta < kDuplicateHeadingDeg) {
        last.flags |= kRouteWaypointManualMark;
        Notify("POINT MARKED");
        UpdateMapStatus();
        return;
    }
    if (!manual_mark && distance < kAutoDistanceMm &&
        heading_delta < kAutoHeadingDeg) {
        return;
    }
    if (count >= kMaxWaypointsPerMap) {
        Notify("MAP FULL: SAVE OR CANCEL", 4000);
        return;
    }

    RouteWaypoint& point = working_route_.waypoints[count];
    point = {x, y, heading,
             static_cast<uint8_t>(manual_mark ? kRouteWaypointManualMark : 0), 0};
    working_route_.header.waypoint_count = count + 1;
    working_route_.header.route_length_mm +=
        static_cast<uint32_t>(lroundf(distance));

    ESP_LOGI(kTag,
             "ROUTE,POINT=%u,SOURCE=%s,X=%ld,Y=%ld,H_CDEG=%d,LENGTH_MM=%lu",
             static_cast<unsigned>(count + 1),
             manual_mark ? "MANUAL" : "AUTO", static_cast<long>(x),
             static_cast<long>(y), static_cast<int>(heading),
             static_cast<unsigned long>(working_route_.header.route_length_mm));
    UpdateMapStatus();
}

void TeachRoute::UpdateAutoWaypoint() {
    if (mode_ == Mode::TEACHING) AddWaypoint(false);
}

void TeachRoute::UndoWaypoint() {
    if (mode_ != Mode::TEACHING) return;
    const uint16_t count = working_route_.header.waypoint_count;
    if (count <= 1) {
        Notify("CANNOT DELETE START");
        return;
    }

    const RouteWaypoint& last = working_route_.waypoints[count - 1];
    const RouteWaypoint& prior = working_route_.waypoints[count - 2];
    const float dx = static_cast<float>(last.x_mm - prior.x_mm);
    const float dy = static_cast<float>(last.y_mm - prior.y_mm);
    const uint32_t segment =
        static_cast<uint32_t>(lroundf(sqrtf(dx * dx + dy * dy)));
    working_route_.header.route_length_mm =
        working_route_.header.route_length_mm > segment
            ? working_route_.header.route_length_mm - segment
            : 0;
    working_route_.header.waypoint_count = count - 1;

    ESP_LOGI(kTag, "ROUTE,UNDO,REMOVED=%u,POINTS=%u,LENGTH_MM=%lu",
             static_cast<unsigned>(count),
             static_cast<unsigned>(working_route_.header.waypoint_count),
             static_cast<unsigned long>(working_route_.header.route_length_mm));
    UpdateMapStatus();
}

void TeachRoute::SaveTeach() {
    if (mode_ != Mode::TEACHING) return;
    StopAutoTimer();
    if (!store_.Save(selected_slot_, working_route_)) {
        Notify("STORAGE ERROR", 4000);
        (void)StartAutoTimer();
        return;
    }

    const uint16_t points = working_route_.header.waypoint_count;
    mode_ = Mode::READY;
    odometry_valid_ = false;
    char notification[56];
    snprintf(notification, sizeof(notification), "%s SAVED: %u POINTS",
             RouteStore::SlotName(selected_slot_), points);
    Notify(notification, 3500);
    UpdateMapStatus();
}

void TeachRoute::LoadSelected() {
    // RouteData is ~1.5 KiB. Reuse the persistent member buffer; never place a
    // full route object on a small task/main stack.
    loaded_route_ = {};
    if (!store_.Load(selected_slot_, loaded_route_)) {
        Notify("MAP NOT SAVED", 3000);
        return;
    }
    mode_ = Mode::LOADED;
    char notification[56];
    snprintf(notification, sizeof(notification), "%s LOADED: %u POINTS",
             RouteStore::SlotName(selected_slot_),
             loaded_route_.header.waypoint_count);
    Notify(notification, 3500);
    UpdateMapStatus();
}

void TeachRoute::RequestDelete() {
    if (mode_ != Mode::READY && mode_ != Mode::LOADED) return;
    mode_ = Mode::DELETE_CONFIRM;
    char notification[64];
    snprintf(notification, sizeof(notification), "DELETE %s? CIRCLE=YES",
             RouteStore::SlotName(selected_slot_));
    Notify(notification, 4000);
    UpdateMapStatus();
}

void TeachRoute::ConfirmDelete() {
    if (mode_ != Mode::DELETE_CONFIRM) return;
    if (!store_.Delete(selected_slot_)) {
        Notify("STORAGE ERROR", 4000);
        return;
    }
    loaded_route_ = {};
    mode_ = Mode::READY;
    char notification[40];
    snprintf(notification, sizeof(notification), "%s DELETED",
             RouteStore::SlotName(selected_slot_));
    Notify(notification);
    UpdateMapStatus();
}

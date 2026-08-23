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
constexpr uint64_t kAutoWaypointPeriodUs = 100000ULL;  // 10 Hz sampling.

// Smart Waypoint V1: manual points remain authoritative. Automatic points are
// sparse safety checkpoints plus one stabilized point at meaningful corners.
// This keeps the proven 128-point storage format while supporting much longer
// practical routes than the old 125 mm / 12 degree recorder.
constexpr float kAutoSafetyDistanceMm = 750.0f;
constexpr float kCornerTriggerDeg = 25.0f;
constexpr float kCornerReleaseDeg = 15.0f;
constexpr float kCornerStableStepDeg = 2.0f;
constexpr uint8_t kCornerStableSamples = 3U;
constexpr float kDuplicateDistanceMm = 20.0f;
constexpr float kDuplicateHeadingDeg = 2.0f;
constexpr float kEndpointDistanceMm = 50.0f;
constexpr float kEndpointHeadingDeg = 5.0f;
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

    ESP_LOGI(kTag,
             "ROUTE,SMART_WP=ON,AUTO_MM=%u,CORNER_DEG=%u,STABLE_SAMPLES=%u,MAX=%u",
             static_cast<unsigned>(kAutoSafetyDistanceMm),
             static_cast<unsigned>(kCornerTriggerDeg),
             static_cast<unsigned>(kCornerStableSamples),
             static_cast<unsigned>(kMaxWaypointsPerMap));

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
    // Map controls arrive as high-level EVENT,MAP frames.
    if (robot_uart_ == nullptr) {
        ESP_LOGE(kTag, "ROUTE,MAP_EVENT_CALLBACK=FAIL,REASON=NO_UART");
        return false;
    }
    robot_uart_->SetMapEventCallback(&TeachRoute::OnMapEvent, this);
    ESP_LOGI(kTag,
             "ROUTE,INPUT_OWNER=STM32,ESP32_PS2_POLL=OFF,INPUT_TASK=DISABLED,MAP_EVENT_CALLBACK=ON");
    UpdateMapStatus();
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
            UpdateMapStatus();
            return;
        }
        selected_slot_ = event_slot;
        UpdateMapStatus();
        return;
    }

    if ((mode_ == Mode::TEACHING || mode_ == Mode::DELETE_CONFIRM) &&
        event_slot != selected_slot_) {
        ESP_LOGW(kTag,
                 "ROUTE,EVENT=REJECT,REASON=SLOT_MISMATCH,ACT=%s,EVENT_SLOT=%u,ACTIVE_SLOT=%u",
                 action, static_cast<unsigned>(event_slot),
                 static_cast<unsigned>(selected_slot_));
        UpdateMapStatus();
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
            UpdateMapStatus();
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

    if (strcmp(action, "SELECT_LONG") == 0) {
        if (mode_ == Mode::TEACHING) {
            CancelTeach();
        } else if (mode_ == Mode::DELETE_CONFIRM) {
            mode_ = Mode::READY;
            Notify("DELETE CANCELLED");
            UpdateMapStatus();
        }
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
    ESP_LOGI(kTag, "ROUTE,AUTO_TIMER=START,PERIOD_MS=%u,SMART=1",
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

    if (robot_uart_ != nullptr && robot_uart_->IsConnected()) {
        char command[96];
        const int written = snprintf(
            command, sizeof(command), "MAP,UI,%u,%u,%u,%u,%u,%lu",
            static_cast<unsigned>(selected_slot_),
            static_cast<unsigned>(metadata.state),
            static_cast<unsigned>(mode_), static_cast<unsigned>(points),
            static_cast<unsigned>(kMaxWaypointsPerMap),
            static_cast<unsigned long>(length_mm));
        if (written <= 0 || static_cast<size_t>(written) >= sizeof(command) ||
            !robot_uart_->SendFrame(command)) {
            ESP_LOGW(kTag, "ROUTE,UI_TX=FAIL,SLOT=%u",
                     static_cast<unsigned>(selected_slot_));
        }
    }
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

bool TeachRoute::ReadCurrentWaypoint(RouteWaypoint& point) const {
    if (robot_uart_ == nullptr || !odometry_valid_) return false;
    RobotOdometry odometry;
    if (!robot_uart_->GetOdometry(odometry, 250) || !odometry.valid) return false;
    point = {};
    point.x_mm = static_cast<int32_t>(lroundf(odometry.x_mm - start_x_mm_));
    point.y_mm = static_cast<int32_t>(lroundf(odometry.y_mm - start_y_mm_));
    point.heading_cdeg = HeadingCdeg(odometry.heading_rad - start_heading_rad_);
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

const char* TeachRoute::WaypointSourceName(WaypointSource source) {
    switch (source) {
        case WaypointSource::MANUAL: return "MANUAL";
        case WaypointSource::AUTO_DISTANCE: return "AUTO_SAFE";
        case WaypointSource::AUTO_CORNER: return "AUTO_CORNER";
        case WaypointSource::ENDPOINT: return "ENDPOINT";
    }
    return "UNKNOWN";
}

uint8_t TeachRoute::WaypointSourceFlags(WaypointSource source) {
    switch (source) {
        case WaypointSource::MANUAL: return kRouteWaypointManualMark;
        case WaypointSource::AUTO_DISTANCE: return kRouteWaypointAutoDistance;
        case WaypointSource::AUTO_CORNER: return kRouteWaypointAutoCorner;
        case WaypointSource::ENDPOINT: return kRouteWaypointEndpoint;
    }
    return 0;
}

void TeachRoute::ResetCornerTracking() {
    corner_pending_ = false;
    corner_stable_samples_ = 0U;
}

void TeachRoute::ResetSmartTracking() {
    last_sample_ = {};
    last_sample_valid_ = false;
    ResetCornerTracking();
}

void TeachRoute::TrackSample(const RouteWaypoint& point) {
    last_sample_ = point;
    last_sample_valid_ = true;
}

bool TeachRoute::StartTeach() {
    if (mission_manager_ == nullptr || robot_uart_ == nullptr ||
        mission_manager_->IsActive() || mission_manager_->IsAiObstacleHoldActive() ||
        !robot_uart_->IsConnected()) {
        Notify("TEACH REJECT: ROBOT BUSY", 3500);
        UpdateMapStatus();
        return false;
    }

    if (!ReadOdometry()) {
        Notify("TEACH REJECT: ODOM", 3500);
        UpdateMapStatus();
        return false;
    }

    working_route_ = {};
    working_route_.header.waypoint_count = 1;
    working_route_.waypoints[0] = {0, 0, 0, 0, 0};
    ResetSmartTracking();
    TrackSample(working_route_.waypoints[0]);
    mode_ = Mode::TEACHING;
    if (!StartAutoTimer()) {
        working_route_ = {};
        odometry_valid_ = false;
        ResetSmartTracking();
        mode_ = Mode::READY;
        Notify("TEACH REJECT: TIMER", 3500);
        UpdateMapStatus();
        return false;
    }

    char notification[64];
    snprintf(notification, sizeof(notification), "TEACH %s SMART START",
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
    ResetSmartTracking();
    mode_ = Mode::READY;
    Notify("TEACH CANCELLED");
    UpdateMapStatus();
}

bool TeachRoute::AppendWaypoint(const RouteWaypoint& candidate,
                                WaypointSource source) {
    if (mode_ != Mode::TEACHING) return false;
    const uint16_t count = working_route_.header.waypoint_count;
    if (count == 0U) return false;

    RouteWaypoint& last = working_route_.waypoints[count - 1U];
    const float dx = static_cast<float>(candidate.x_mm - last.x_mm);
    const float dy = static_cast<float>(candidate.y_mm - last.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    const float heading_delta =
        HeadingDeltaDeg(candidate.heading_cdeg, last.heading_cdeg);

    if (distance < kDuplicateDistanceMm &&
        heading_delta < kDuplicateHeadingDeg) {
        if (source == WaypointSource::MANUAL) {
            last.flags |= kRouteWaypointManualMark;
            Notify("POINT MARKED");
            ESP_LOGI(kTag, "ROUTE,POINT=%u,SOURCE=MANUAL_MARK_EXISTING",
                     static_cast<unsigned>(count));
            UpdateMapStatus();
        }
        return false;
    }

    if (count >= kMaxWaypointsPerMap) {
        Notify("MAP FULL: SAVE OR CANCEL", 4000);
        return false;
    }

    RouteWaypoint& point = working_route_.waypoints[count];
    point = candidate;
    point.flags |= WaypointSourceFlags(source);
    working_route_.header.waypoint_count = count + 1U;
    working_route_.header.route_length_mm +=
        static_cast<uint32_t>(lroundf(distance));

    ESP_LOGI(kTag,
             "ROUTE,POINT=%u,SOURCE=%s,X=%ld,Y=%ld,H_CDEG=%d,SEG_MM=%lu,LENGTH_MM=%lu",
             static_cast<unsigned>(count + 1U), WaypointSourceName(source),
             static_cast<long>(point.x_mm), static_cast<long>(point.y_mm),
             static_cast<int>(point.heading_cdeg),
             static_cast<unsigned long>(lroundf(distance)),
             static_cast<unsigned long>(working_route_.header.route_length_mm));
    ResetCornerTracking();
    UpdateMapStatus();
    return true;
}

void TeachRoute::AddWaypoint(bool manual_mark) {
    if (mode_ != Mode::TEACHING || !odometry_valid_) return;
    RouteWaypoint point;
    if (!ReadCurrentWaypoint(point)) {
        Notify("ODOMETRY ERROR");
        return;
    }
    TrackSample(point);
    if (manual_mark) {
        (void)AppendWaypoint(point, WaypointSource::MANUAL);
    }
}

void TeachRoute::UpdateAutoWaypoint() {
    if (mode_ != Mode::TEACHING || !odometry_valid_) return;

    RouteWaypoint point;
    if (!ReadCurrentWaypoint(point)) {
        ESP_LOGW(kTag, "ROUTE,SMART_SAMPLE=SKIP,REASON=ODOM");
        return;
    }

    const float sample_heading_delta = last_sample_valid_
        ? HeadingDeltaDeg(point.heading_cdeg, last_sample_.heading_cdeg)
        : 0.0f;
    TrackSample(point);

    const uint16_t count = working_route_.header.waypoint_count;
    if (count == 0U) return;
    const RouteWaypoint& last = working_route_.waypoints[count - 1U];
    const float dx = static_cast<float>(point.x_mm - last.x_mm);
    const float dy = static_cast<float>(point.y_mm - last.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    const float heading_delta =
        HeadingDeltaDeg(point.heading_cdeg, last.heading_cdeg);

    // Hard safety spacing: never let a route segment grow beyond ~0.75 m even
    // if the operator forgets to mark points on a long straight corridor.
    if (distance >= kAutoSafetyDistanceMm) {
        (void)AppendWaypoint(point, WaypointSource::AUTO_DISTANCE);
        return;
    }

    // Corner detection is intentionally two-stage. A 25-degree deviation only
    // arms a candidate. It is stored after heading settles for three samples,
    // preventing a 90-degree turn from consuming several 25-degree points.
    if (heading_delta >= kCornerTriggerDeg) {
        corner_pending_ = true;
        if (sample_heading_delta <= kCornerStableStepDeg) {
            if (corner_stable_samples_ < kCornerStableSamples) {
                ++corner_stable_samples_;
            }
        } else {
            corner_stable_samples_ = 0U;
        }
        if (corner_stable_samples_ >= kCornerStableSamples) {
            (void)AppendWaypoint(point, WaypointSource::AUTO_CORNER);
        }
    } else if (corner_pending_ && heading_delta < kCornerReleaseDeg) {
        ResetCornerTracking();
    }
}

void TeachRoute::UndoWaypoint() {
    if (mode_ != Mode::TEACHING) return;
    const uint16_t count = working_route_.header.waypoint_count;
    if (count <= 1U) {
        Notify("CANNOT DELETE START");
        UpdateMapStatus();
        return;
    }

    const RouteWaypoint& last = working_route_.waypoints[count - 1U];
    const RouteWaypoint& prior = working_route_.waypoints[count - 2U];
    const float dx = static_cast<float>(last.x_mm - prior.x_mm);
    const float dy = static_cast<float>(last.y_mm - prior.y_mm);
    const uint32_t segment =
        static_cast<uint32_t>(lroundf(sqrtf(dx * dx + dy * dy)));
    working_route_.header.route_length_mm =
        working_route_.header.route_length_mm > segment
            ? working_route_.header.route_length_mm - segment
            : 0U;
    working_route_.header.waypoint_count = count - 1U;
    ResetCornerTracking();

    ESP_LOGI(kTag, "ROUTE,UNDO,REMOVED=%u,POINTS=%u,LENGTH_MM=%lu",
             static_cast<unsigned>(count),
             static_cast<unsigned>(working_route_.header.waypoint_count),
             static_cast<unsigned long>(working_route_.header.route_length_mm));
    UpdateMapStatus();
}

void TeachRoute::SaveTeach() {
    if (mode_ != Mode::TEACHING) return;
    StopAutoTimer();

    // Preserve the actual final pose even when the last manual/automatic point
    // is still less than the 0.75 m safety spacing from the robot.
    RouteWaypoint endpoint;
    if (ReadCurrentWaypoint(endpoint)) {
        TrackSample(endpoint);
        const uint16_t count = working_route_.header.waypoint_count;
        if (count > 0U) {
            const RouteWaypoint& last = working_route_.waypoints[count - 1U];
            const float dx = static_cast<float>(endpoint.x_mm - last.x_mm);
            const float dy = static_cast<float>(endpoint.y_mm - last.y_mm);
            const float distance = sqrtf(dx * dx + dy * dy);
            const float heading_delta =
                HeadingDeltaDeg(endpoint.heading_cdeg, last.heading_cdeg);
            if (distance >= kEndpointDistanceMm ||
                heading_delta >= kEndpointHeadingDeg) {
                (void)AppendWaypoint(endpoint, WaypointSource::ENDPOINT);
            }
        }
    }

    if (!store_.Save(selected_slot_, working_route_)) {
        Notify("STORAGE ERROR", 4000);
        (void)StartAutoTimer();
        UpdateMapStatus();
        return;
    }

    const uint16_t points = working_route_.header.waypoint_count;
    mode_ = Mode::READY;
    odometry_valid_ = false;
    ResetSmartTracking();
    char notification[64];
    snprintf(notification, sizeof(notification), "%s SAVED: %u SMART POINTS",
             RouteStore::SlotName(selected_slot_), points);
    Notify(notification, 3500);
    UpdateMapStatus();
}

void TeachRoute::LoadSelected() {
    loaded_route_ = {};
    if (!store_.Load(selected_slot_, loaded_route_)) {
        Notify("MAP NOT SAVED", 3000);
        mode_ = Mode::READY;
        UpdateMapStatus();
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
    const RouteSlotMetadata metadata = store_.GetMetadata(selected_slot_);
    if (metadata.state != RouteSlotState::SAVED) {
        Notify("MAP EMPTY");
        mode_ = Mode::READY;
        UpdateMapStatus();
        return;
    }
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
        UpdateMapStatus();
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

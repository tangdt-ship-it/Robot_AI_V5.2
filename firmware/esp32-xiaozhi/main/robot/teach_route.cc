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
constexpr float kAutoSafetyDistanceMm = 750.0f;
constexpr float kCornerTriggerDeg = 25.0f;
constexpr float kCornerReleaseDeg = 15.0f;
constexpr float kCornerStableStepDeg = 2.0f;
constexpr uint8_t kCornerStableSamples = 3U;
constexpr float kDuplicateDistanceMm = 20.0f;
constexpr float kDuplicateHeadingDeg = 2.0f;
constexpr float kEndpointDistanceMm = 50.0f;
constexpr float kEndpointHeadingDeg = 5.0f;

// Replay V1 commissioning limits. Phase A validates the whole route with
// DRIVE=OFF. Phase B1 may drive only WP1 -> WP2 and deliberately performs no
// automatic turn; the operator must place the robot at the taught start pose
// and align it with the taught start heading before pressing START.
constexpr float kReplayDuplicateDistanceMm = 5.0f;
constexpr float kReplayDuplicateHeadingDeg = 1.0f;
constexpr float kReplayMaxSegmentMm = 2000.0f;
constexpr float kReplayB1MinSegmentMm = 100.0f;
constexpr float kReplayB1MaxSegmentMm = 1000.0f;
constexpr float kReplayB1MaxHeadingDeg = 20.0f;
constexpr int kReplayB1Speed = 10;
constexpr uint32_t kReplayB1TimeoutMs = 20000U;
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
    ESP_LOGI(kTag,
             "ROUTE,REPLAY_V1=PHASE_B1,FIRST_SEGMENT_ONLY=1,TURN=OFF,SPEED=%d,MAX_MM=%u",
             kReplayB1Speed, static_cast<unsigned>(kReplayB1MaxSegmentMm));
    ESP_LOGI(kTag,
             "ROUTE,REPLAY_B1_OBS_POLICY=VALID_FRESH_CLEAR,ECHO0_CLEAR_ALLOWED=1");
    ESP_LOGI(kTag,
             "ROUTE,MAP_UI_PROTO=2,REPLAY_MODES=0-8,LCD_REPLAY_STATUS=ON");

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
    ESP_LOGI(kTag, "ROUTE,EVENT=RX,ACT=%s,SLOT=%u,MODE=%u,REPLAY_RUNNING=%u",
             action, static_cast<unsigned>(slot),
             static_cast<unsigned>(mode_),
             replay_motion_running_.load() ? 1U : 0U);

    // During the B1 worker, only the explicit X/SELECT_LONG cancel path is
    // accepted. This prevents LOAD/SLOT/TEACH events racing an active motor
    // lease while still preserving the operator's immediate stop authority.
    if (replay_motion_running_.load() && strcmp(action, "SELECT_LONG") != 0) {
        ESP_LOGW(kTag, "ROUTE,EVENT=IGNORED,REASON=REPLAY_RUNNING,ACT=%s",
                 action);
        return;
    }

    const bool state_locked = mode_ == Mode::TEACHING ||
                              mode_ == Mode::DELETE_CONFIRM ||
                              mode_ >= Mode::REPLAY_READY ||
                              replay_motion_running_.load();
    if (strcmp(action, "SLOT") == 0) {
        if (state_locked) {
            ESP_LOGW(kTag,
                     "ROUTE,EVENT=SLOT,RESULT=IGNORED,REASON=STATE_LOCKED,ACTIVE_SLOT=%u",
                     static_cast<unsigned>(selected_slot_));
            UpdateMapStatus();
            return;
        }
        selected_slot_ = event_slot;
        replay_plan_valid_ = false;
        UpdateMapStatus();
        return;
    }

    if (state_locked && event_slot != selected_slot_) {
        ESP_LOGW(kTag,
                 "ROUTE,EVENT=REJECT,REASON=SLOT_MISMATCH,ACT=%s,EVENT_SLOT=%u,ACTIVE_SLOT=%u",
                 action, static_cast<unsigned>(event_slot),
                 static_cast<unsigned>(selected_slot_));
        UpdateMapStatus();
        return;
    }
    if (!state_locked) {
        selected_slot_ = event_slot;
    }

    // STM32 START is an alias of TRIANGLE on the MAP page.
    // READY -> Teach
    // LOADED + unchecked -> Arm Replay
    // REPLAY_READY -> whole-route dry-run
    // LOADED + dry-run PASS -> Phase B1 first-segment motion.
    if (strcmp(action, "TRIANGLE") == 0) {
        if (mode_ == Mode::TEACHING) {
            AddWaypoint(true);
        } else if (mode_ == Mode::READY) {
            StartTeach();
        } else if (mode_ == Mode::LOADED) {
            if (replay_plan_valid_) {
                StartReplayFirstSegment();
            } else {
                ArmReplay();
            }
        } else if (mode_ == Mode::REPLAY_READY) {
            RunReplayDryRun();
        } else if (mode_ == Mode::REPLAY_CHECKED) {
            StartReplayFirstSegment();
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

    // STM32 X is an alias of SELECT_LONG. It remains the fastest replay stop.
    if (strcmp(action, "SELECT_LONG") == 0) {
        if (replay_motion_running_.load()) {
            replay_cancel_requested_.store(true);
            ESP_LOGW(kTag, "ROUTE,REPLAY=B1_CANCEL_REQUEST,MOTOR_STOP=REQUESTED");
            if (robot_uart_ != nullptr && !robot_uart_->Ps2OverrideActive()) {
                (void)robot_uart_->Stop(700);
            }
        } else if (mode_ == Mode::TEACHING) {
            CancelTeach();
        } else if (mode_ == Mode::DELETE_CONFIRM) {
            mode_ = Mode::READY;
            Notify("DELETE CANCELLED");
            UpdateMapStatus();
        } else if (mode_ >= Mode::REPLAY_READY || replay_plan_valid_) {
            CancelReplay();
        }
        return;
    }
    if (strcmp(action, "SQUARE_LONG") == 0) {
        if (mode_ == Mode::READY ||
            (mode_ == Mode::LOADED && !replay_plan_valid_)) {
            RequestDelete();
        }
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
                       mode_ == Mode::DELETE_CONFIRM ? "DELETE" :
                       mode_ == Mode::REPLAY_READY ? "REPLAY_READY" :
                       mode_ == Mode::REPLAY_CHECKED ? "REPLAY_CHECKED" :
                       mode_ == Mode::REPLAY_RUNNING ? "REPLAY_RUNNING" :
                       mode_ == Mode::REPLAY_HOLD ? "REPLAY_HOLD" :
                       mode_ == Mode::REPLAY_COMPLETE ? "REPLAY_COMPLETE" :
                       "READY";
    const uint16_t points = mode_ == Mode::TEACHING
                                ? working_route_.header.waypoint_count
                                : metadata.waypoint_count;
    const uint32_t length_mm = mode_ == Mode::TEACHING
                                   ? working_route_.header.route_length_mm
                                   : metadata.route_length_mm;
    ESP_LOGI(kTag,
             "ROUTE,UI,OWNER=STM32,SLOT=%u,STORE=%s,MODE=%s,POINTS=%u,MAX=%u,LENGTH_MM=%lu,REPLAY_CHECKED=%u,REPLAY_RUNNING=%u",
             static_cast<unsigned>(selected_slot_),
             RouteStore::StateName(metadata.state), mode, points,
             static_cast<unsigned>(kMaxWaypointsPerMap),
             static_cast<unsigned long>(length_mm),
             replay_plan_valid_ ? 1U : 0U,
             replay_motion_running_.load() ? 1U : 0U);

    if (robot_uart_ != nullptr && robot_uart_->IsConnected()) {
        char command[160];
        const int written = snprintf(
            command, sizeof(command),
            "MAP,UI,%u,%u,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu",
            static_cast<unsigned>(selected_slot_),
            static_cast<unsigned>(metadata.state),
            static_cast<unsigned>(mode_),
            static_cast<unsigned>(points),
            static_cast<unsigned>(kMaxWaypointsPerMap),
            static_cast<unsigned long>(length_mm),
            static_cast<unsigned>(replay_wp_index_),
            static_cast<unsigned>(replay_wp_total_),
            static_cast<unsigned long>(replay_target_mm_),
            static_cast<unsigned long>(replay_travel_mm_),
            static_cast<unsigned long>(replay_error_mm_));
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
    if (replay_motion_running_.load() || mission_manager_ == nullptr ||
        robot_uart_ == nullptr || mission_manager_->IsActive() ||
        mission_manager_->IsAiObstacleHoldActive() ||
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

    replay_plan_valid_ = false;
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

    if (distance >= kAutoSafetyDistanceMm) {
        (void)AppendWaypoint(point, WaypointSource::AUTO_DISTANCE);
        return;
    }

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
    replay_plan_valid_ = false;
    ResetSmartTracking();
    char notification[64];
    snprintf(notification, sizeof(notification), "%s SAVED: %u SMART POINTS",
             RouteStore::SlotName(selected_slot_), points);
    Notify(notification, 3500);
    UpdateMapStatus();
}

void TeachRoute::LoadSelected() {
    loaded_route_ = {};
    replay_plan_valid_ = false;
    replay_wp_index_ = replay_wp_total_ = 0U;
    replay_target_mm_ = replay_travel_mm_ = replay_error_mm_ = 0U;
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

bool TeachRoute::ArmReplay() {
    if (mode_ != Mode::LOADED || replay_motion_running_.load()) return false;
    if (loaded_route_.header.waypoint_count < 2U) {
        Notify("REPLAY REJECT: NEED 2 POINTS", 3500);
        return false;
    }
    if (mission_manager_ == nullptr || robot_uart_ == nullptr ||
        mission_manager_->IsActive() || mission_manager_->IsAiObstacleHoldActive() ||
        !robot_uart_->IsConnected()) {
        Notify("REPLAY REJECT: ROBOT BUSY", 3500);
        return false;
    }

    RobotState state;
    if (!robot_uart_->GetState(state, 500) || !state.valid) {
        Notify("REPLAY REJECT: STATE", 3500);
        return false;
    }
    if (state.moving || state.left != 0 || state.right != 0) {
        Notify("REPLAY REJECT: MOVING", 3500);
        return false;
    }
    if (state.brake_enabled) {
        Notify("REPLAY REJECT: BRAKE", 3500);
        return false;
    }
    if (!ReadOdometry()) {
        Notify("REPLAY REJECT: ODOM", 3500);
        return false;
    }

    replay_plan_valid_ = false;
    replay_wp_index_ = 1U;
    replay_wp_total_ = loaded_route_.header.waypoint_count;
    replay_target_mm_ = 0U;
    replay_travel_mm_ = 0U;
    replay_error_mm_ = 0U;
    mode_ = Mode::REPLAY_READY;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=ARMED,SLOT=%u,POINTS=%u,ORIGIN_X=%.1f,ORIGIN_Y=%.1f,ORIGIN_H=%.3f,MOTOR=0",
             static_cast<unsigned>(selected_slot_),
             static_cast<unsigned>(loaded_route_.header.waypoint_count),
             start_x_mm_, start_y_mm_, start_heading_rad_);
    Notify("REPLAY READY: START CHECK", 3500);
    UpdateMapStatus();
    return true;
}

void TeachRoute::CancelReplay() {
    if (replay_motion_running_.load()) {
        replay_cancel_requested_.store(true);
        if (robot_uart_ != nullptr && !robot_uart_->Ps2OverrideActive()) {
            (void)robot_uart_->Stop(700);
        }
        return;
    }
    if (mode_ != Mode::REPLAY_READY && !replay_plan_valid_) return;
    replay_plan_valid_ = false;
    odometry_valid_ = false;
    replay_wp_index_ = replay_wp_total_ = 0U;
    replay_target_mm_ = replay_travel_mm_ = replay_error_mm_ = 0U;
    mode_ = Mode::LOADED;
    ESP_LOGI(kTag, "ROUTE,REPLAY=CANCELLED,MOTOR=0");
    Notify("REPLAY CANCELLED");
    UpdateMapStatus();
}

bool TeachRoute::RunReplayDryRun() {
    if (mode_ != Mode::REPLAY_READY || !odometry_valid_) return false;
    const uint16_t count = loaded_route_.header.waypoint_count;
    if (count < 2U) {
        Notify("REPLAY CHECK FAIL: POINTS", 3500);
        CancelReplay();
        return false;
    }

    RobotState state;
    if (robot_uart_ == nullptr || !robot_uart_->GetState(state, 500) ||
        !state.valid || state.moving || state.left != 0 || state.right != 0 ||
        state.brake_enabled) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=DRY_RUN,PASS=0,REASON=STATE,MOTOR=0");
        Notify("REPLAY CHECK FAIL: STATE", 3500);
        mode_ = Mode::LOADED;
        replay_plan_valid_ = false;
        odometry_valid_ = false;
        UpdateMapStatus();
        return false;
    }

    bool valid = true;
    uint32_t summed_mm = 0U;
    for (uint16_t index = 1U; index < count; ++index) {
        const RouteWaypoint& prior = loaded_route_.waypoints[index - 1U];
        const RouteWaypoint& point = loaded_route_.waypoints[index];
        const float dx = static_cast<float>(point.x_mm - prior.x_mm);
        const float dy = static_cast<float>(point.y_mm - prior.y_mm);
        const float distance = sqrtf(dx * dx + dy * dy);
        const float heading_delta =
            HeadingDeltaDeg(point.heading_cdeg, prior.heading_cdeg);
        const float bearing_deg = atan2f(dy, dx) * 57.2957795f;

        const bool duplicate = distance < kReplayDuplicateDistanceMm &&
                               heading_delta < kReplayDuplicateHeadingDeg;
        const bool segment_too_long = distance > kReplayMaxSegmentMm;
        if (duplicate || segment_too_long || !std::isfinite(distance) ||
            !std::isfinite(bearing_deg)) {
            valid = false;
        }
        summed_mm += static_cast<uint32_t>(lroundf(distance));
        ESP_LOGI(kTag,
                 "ROUTE,REPLAY_PLAN,WP=%u/%u,DIST_MM=%lu,BEARING_DEG=%.1f,H_DEG=%.1f,H_DELTA=%.1f,FLAGS=0x%02X,VALID=%u",
                 static_cast<unsigned>(index + 1U), static_cast<unsigned>(count),
                 static_cast<unsigned long>(lroundf(distance)), bearing_deg,
                 static_cast<float>(point.heading_cdeg) / 100.0f,
                 heading_delta, static_cast<unsigned>(point.flags),
                 (duplicate || segment_too_long) ? 0U : 1U);
    }

    const uint32_t stored_mm = loaded_route_.header.route_length_mm;
    const uint32_t length_error = summed_mm > stored_mm
        ? summed_mm - stored_mm : stored_mm - summed_mm;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY_PLAN,SUM_MM=%lu,STORED_MM=%lu,LENGTH_ERR_MM=%lu",
             static_cast<unsigned long>(summed_mm),
             static_cast<unsigned long>(stored_mm),
             static_cast<unsigned long>(length_error));

    replay_plan_valid_ = valid;
    replay_wp_index_ = valid ? 2U : 0U;
    replay_wp_total_ = valid ? count : 0U;
    if (valid && count >= 2U) {
        const RouteWaypoint& first = loaded_route_.waypoints[0];
        const RouteWaypoint& second = loaded_route_.waypoints[1];
        const float dx = static_cast<float>(second.x_mm - first.x_mm);
        const float dy = static_cast<float>(second.y_mm - first.y_mm);
        replay_target_mm_ = static_cast<uint32_t>(lroundf(sqrtf(dx * dx + dy * dy)));
    } else {
        replay_target_mm_ = 0U;
    }
    replay_travel_mm_ = 0U;
    replay_error_mm_ = 0U;
    mode_ = valid ? Mode::REPLAY_CHECKED : Mode::LOADED;
    odometry_valid_ = false;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=DRY_RUN,PASS=%u,SLOT=%u,POINTS=%u,MOTOR=0,DRIVE=OFF,NEXT=%s",
             valid ? 1U : 0U, static_cast<unsigned>(selected_slot_),
             static_cast<unsigned>(count),
             valid ? "B1_FIRST_SEGMENT" : "NONE");
    Notify(valid ? "REPLAY DRY PASS: START=B1" : "REPLAY DRY FAIL", 4000);
    UpdateMapStatus();
    return valid;
}

bool TeachRoute::CheckReplaySafety(const char* stage, RobotState& state,
                                   RobotObstacleStatus& obstacle,
                                   const char*& reason) const {
    state = {};
    obstacle = {};
    reason = "OK";
    const bool mission = mission_manager_ != nullptr && mission_manager_->IsActive();
    const bool ai_hold = mission_manager_ != nullptr &&
                         mission_manager_->IsAiObstacleHoldActive();
    const bool ps2 = robot_uart_ != nullptr && robot_uart_->Ps2OverrideActive();
    bool pass = true;
    if (replay_cancel_requested_.load()) { reason = "X_CANCEL"; pass = false; }
    else if (mission) { reason = "MISSION_ACTIVE"; pass = false; }
    else if (ai_hold) { reason = "AI_OBSTACLE_HOLD"; pass = false; }
    else if (ps2) { reason = "PS2_OVERRIDE"; pass = false; }
    else if (robot_uart_ == nullptr || !robot_uart_->GetState(state, 500)) {
        reason = "STATE_RX"; pass = false;
    } else if (!state.valid) { reason = "STATE_INVALID"; pass = false; }
    else if (state.moving || state.left != 0 || state.right != 0) {
        reason = "MOVING"; pass = false;
    } else if (state.brake_enabled) { reason = "BRAKE"; pass = false; }
    else if (!robot_uart_->GetObstacle(obstacle, 500)) {
        reason = "OBS_RX"; pass = false;
    } else if (!obstacle.valid) { reason = "OBS_INVALID"; pass = false; }
    else if (!obstacle.fresh) { reason = "OBS_STALE"; pass = false; }
    else if (strcmp(obstacle.zone, "CAUTION") == 0) {
        reason = "CAUTION"; pass = false;
    } else if (strcmp(obstacle.zone, "BLOCKED") == 0) {
        reason = "BLOCKED"; pass = false;
    } else if (strcmp(obstacle.zone, "EMERGENCY") == 0) {
        reason = "EMERGENCY"; pass = false;
    } else if (strcmp(obstacle.zone, "CLEAR") != 0) {
        reason = "OBS_INVALID"; pass = false;
    }
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=SAFETY_CHECK,STAGE=%s,PASS=%u,REASON=%s,MISSION=%u,AI_HOLD=%u,PS2_OVERRIDE=%u,STATE_VALID=%u,MOVING=%u,BRAKE=%u,OBS_VALID=%u,OBS_FRESH=%u,OBS_ECHO=%u,OBS_DIST_CM=%.1f,OBS_ZONE=%s",
             stage != nullptr ? stage : "UNKNOWN", pass ? 1U : 0U, reason,
             mission ? 1U : 0U, ai_hold ? 1U : 0U, ps2 ? 1U : 0U,
             state.valid ? 1U : 0U, state.moving ? 1U : 0U,
             state.brake_enabled ? 1U : 0U, obstacle.valid ? 1U : 0U,
             obstacle.fresh ? 1U : 0U, obstacle.echo_valid ? 1U : 0U,
             obstacle.distance_cm,
             obstacle.zone[0] != '\0' ? obstacle.zone : "UNKNOWN");
    return pass;
}

bool TeachRoute::StartReplayFirstSegment() {
    if (mode_ != Mode::REPLAY_CHECKED) {
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B1_REJECT,REASON=MODE,MODE=%u,EXPECTED=REPLAY_CHECKED,MOTOR=0",
                 static_cast<unsigned>(mode_));
        return false;
    }
    if (!replay_plan_valid_ ||
        replay_motion_running_.load() || robot_uart_ == nullptr ||
        mission_manager_ == nullptr) {
        return false;
    }
    const uint16_t count = loaded_route_.header.waypoint_count;
    if (count < 2U) return false;

    const RouteWaypoint& start = loaded_route_.waypoints[0];
    const RouteWaypoint& target = loaded_route_.waypoints[1];
    const float dx = static_cast<float>(target.x_mm - start.x_mm);
    const float dy = static_cast<float>(target.y_mm - start.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    const float target_heading = fabsf(static_cast<float>(target.heading_cdeg) / 100.0f);
    if (!std::isfinite(distance) || distance < kReplayB1MinSegmentMm ||
        distance > kReplayB1MaxSegmentMm || target_heading > kReplayB1MaxHeadingDeg) {
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B1_REJECT,REASON=SEGMENT,DIST_MM=%.1f,H_DEG=%.1f,MIN_MM=%.0f,MAX_MM=%.0f,MAX_H=%.0f",
                 distance, target_heading, kReplayB1MinSegmentMm,
                 kReplayB1MaxSegmentMm, kReplayB1MaxHeadingDeg);
        Notify("B1 REJECT: FIRST SEGMENT", 4000);
        replay_plan_valid_ = false;
        UpdateMapStatus();
        return false;
    }

    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    if (!CheckReplaySafety("START_GATE", state, obstacle, reason)) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=B1_REJECT,REASON=%s", reason);
        Notify("B1 REJECT: SAFETY", 4000);
        replay_plan_valid_ = false;
        UpdateMapStatus();
        return false;
    }

    replay_cancel_requested_.store(false);
    replay_motion_running_.store(true);
    replay_plan_valid_ = false;
    replay_wp_index_ = 2U;
    replay_wp_total_ = count;
    replay_target_mm_ = static_cast<uint32_t>(lroundf(distance));
    replay_travel_mm_ = 0U;
    replay_error_mm_ = replay_target_mm_;
    mode_ = Mode::REPLAY_RUNNING;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=B1_START,SLOT=%u,WP=2/%u,TARGET_MM=%lu,SPEED=%d,TURN=OFF,ALIGN_REQUIRED=MANUAL",
             static_cast<unsigned>(selected_slot_), static_cast<unsigned>(count),
             static_cast<unsigned long>(lroundf(distance)), kReplayB1Speed);
    Notify("B1 GO WP2: X/R3 STOP", 3500);
    UpdateMapStatus();

    if (xTaskCreatePinnedToCore(ReplayTaskEntry, "map_replay_b1", 4096, this, 3,
                                &replay_task_, 1) != pdPASS) {
        replay_task_ = nullptr;
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGE(kTag, "ROUTE,REPLAY=B1_REJECT,REASON=TASK_CREATE");
        Notify("B1 REJECT: NO TASK", 4000);
        UpdateMapStatus();
        return false;
    }
    return true;
}

void TeachRoute::ReplayTaskEntry(void* context) {
    if (context != nullptr) {
        static_cast<TeachRoute*>(context)->RunReplayFirstSegment();
        static_cast<TeachRoute*>(context)->replay_task_ = nullptr;
    }
    vTaskDelete(nullptr);
}

void TeachRoute::RunReplayFirstSegment() {
    const uint16_t count = loaded_route_.header.waypoint_count;
    const RouteWaypoint& start = loaded_route_.waypoints[0];
    const RouteWaypoint& target = loaded_route_.waypoints[1];
    const float dx = static_cast<float>(target.x_mm - start.x_mm);
    const float dy = static_cast<float>(target.y_mm - start.y_mm);
    const int target_mm = static_cast<int>(lroundf(sqrtf(dx * dx + dy * dy)));

    bool success = false;
    const char* reason = "MOVE_FAIL";
    RobotDistanceResult result;

    RobotState state;
    RobotObstacleStatus obstacle;
    const char* safety_reason = "OK";
    const bool safe_before = CheckReplaySafety(
        "WORKER_PRECHECK", state, obstacle, safety_reason);

    if (!safe_before) {
        reason = safety_reason;
    } else if (!robot_uart_->SetMode(true, 700)) {
        reason = "AI_MODE";
    } else {
        const bool moved = robot_uart_->MoveDistance(
            true, target_mm, kReplayB1Speed, result, kReplayB1TimeoutMs);

        if (replay_cancel_requested_.load()) {
            reason = "X_CANCEL";
        } else if (robot_uart_->Ps2OverrideActive()) {
            reason = "PS2_OVERRIDE";
        } else if (mission_manager_->IsAiObstacleHoldActive()) {
            reason = "AI_OBSTACLE_HOLD";
        } else {
            RobotState after;
            if (robot_uart_->GetState(after, 500) && after.valid &&
                after.brake_enabled) {
                reason = "R3_BRAKE";
            } else if (moved && result.completed) {
                success = true;
                reason = "DONE";
            }
        }
    }

    // Do not race a PS2 takeover with MODE,MANUAL. Otherwise restore MANUAL
    // after the closed-loop B1 lease finishes or fails.
    if (!robot_uart_->Ps2OverrideActive()) {
        (void)robot_uart_->SetMode(false, 700);
    }

    replay_travel_mm_ = success ? static_cast<uint32_t>(lroundf(result.travelled_mm))
                                : 0U;
    replay_error_mm_ = success
        ? static_cast<uint32_t>(lroundf(fabsf(result.travelled_mm - result.target_mm)))
        : replay_target_mm_;
    if (success) {
        mode_ = Mode::REPLAY_COMPLETE;
        const float error_mm = fabsf(result.travelled_mm - result.target_mm);
        ESP_LOGI(kTag,
                 "ROUTE,REPLAY=B1_DONE,WP=2/%u,TARGET_MM=%.1f,TRAVEL_MM=%.1f,ERR_MM=%.1f,CONTINUE=NO,MOTOR=0",
                 static_cast<unsigned>(count), result.target_mm,
                 result.travelled_mm, error_mm);
        Notify("B1 WP2 DONE: STOPPED", 4000);
    } else {
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B1_STOP,REASON=%s,WP=2/%u,TARGET_MM=%d,CONTINUE=NO,MOTOR=0",
                 reason, static_cast<unsigned>(count), target_mm);
        Notify("B1 STOPPED: NO CONTINUE", 4000);
    }

    replay_cancel_requested_.store(false);
    replay_motion_running_.store(false);
    replay_plan_valid_ = false;
    odometry_valid_ = false;
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
    replay_plan_valid_ = false;
    mode_ = Mode::READY;
    char notification[40];
    snprintf(notification, sizeof(notification), "%s DELETED",
             RouteStore::SlotName(selected_slot_));
    Notify(notification);
    UpdateMapStatus();
}

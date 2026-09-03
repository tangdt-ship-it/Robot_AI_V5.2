#include "teach_route.h"

#include "application.h"
#include "mission_manager.h"
#include "robot_uart.h"
#include "safety_blackbox.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr uint32_t kReplaySafetyRxTimeoutMs = 900;
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
constexpr float kReplayB3MinSegmentMm = 20.0f;
constexpr float kReplayB3MaxSegmentMm = 500.0f;
constexpr uint32_t kReplayB3TimeoutMs = 20000U;
enum class ReplayMode : uint8_t { SHORT_SAFETY_TEST, FULL_PRODUCTION };
#ifndef ROBOT_V5_FULL_REPLAY_PRODUCTION
#define ROBOT_V5_FULL_REPLAY_PRODUCTION 0
#endif
// The full route engine is compiled and tested behind an explicit gate, but
// the safe one-segment test is the alpha default until HIL commissioning.
constexpr ReplayMode kReplayMode = ROBOT_V5_FULL_REPLAY_PRODUCTION
                                        ? ReplayMode::FULL_PRODUCTION
                                        : ReplayMode::SHORT_SAFETY_TEST;
constexpr uint32_t kReplayHrShortMaxMm = 150U;

float NormalizeTurnDelta(float delta_deg) {
    while (delta_deg > 180.0f) delta_deg -= 360.0f;
    while (delta_deg < -180.0f) delta_deg += 360.0f;
    return delta_deg;
}

const char* ReplayTerminalStatus(const char* reason) {
    if (reason == nullptr) return "STOPPED";
    if (strcmp(reason, "X_CANCEL") == 0) return "CANCELLED";
    if (strcmp(reason, "PLAN_INVALID") == 0 ||
        strcmp(reason, "INVALID_ROUTE") == 0) return "INVALID_ROUTE";
    if (strstr(reason, "RESET_BOUNDARY") != nullptr) {
        return "ABORTED_RESET_BOUNDARY";
    }
    if (strstr(reason, "ENCODER") != nullptr ||
        strstr(reason, "ODOMETRY") != nullptr ||
        strstr(reason, "POSE") != nullptr ||
        strstr(reason, "HEADING") != nullptr) {
        return "ABORTED_POSE_UNRELIABLE";
    }
    if (strstr(reason, "LINK") != nullptr ||
        strcmp(reason, "STATE_RX") == 0 || strcmp(reason, "OBS_RX") == 0) {
        return "ABORTED_LINK_LOSS";
    }
    if (strstr(reason, "SENSOR") != nullptr ||
        strstr(reason, "OBS_") != nullptr) {
        return "ABORTED_SENSOR_FAULT";
    }
    return "STOPPED";
}
}
TeachRoute::TeachRoute(RobotUart* robot_uart, MissionManager* mission_manager)
    : robot_uart_(robot_uart), mission_manager_(mission_manager) {}

void TeachRoute::SetDisplay(Display* display) {
    // Compatibility shim only. MAP V1 must never touch the ESP32 TFT/LVGL
    // path. The STM32 20x4 LCD owns page/slot presentation.
    (void)display;
}

bool TeachRoute::Begin() {
#if ROBOT_MAP_OWNER_STM32
    ESP_LOGI(kTag, "ROUTE,OWNER=STM32,LEGACY_ESP32_MAP_RUNTIME=DORMANT");
    return true;
#endif
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
             "ROUTE,REPLAY_V1=FULL,N_WAYPOINT=%u,AUTO_TURN=1,AUTO_MOVE=1,AUTO_RESUME=0,AI_DETOUR=0,SPEED=%d",
             static_cast<unsigned>(kMaxWaypointsPerMap), kReplayB1Speed);
    ESP_LOGI(kTag,
             "ROUTE,REPLAY_B1_OBS_POLICY=VALID_FRESH_CLEAR,ECHO0_CLEAR_ALLOWED=1");
    ESP_LOGI(kTag, "ROUTE,REPLAY_RESUME_PERSIST=NO");
    ESP_LOGI(kTag,
             "ROUTE,REPLAY_MODE=%s,MAX_SHORT_MM=%lu,FULL_DEFAULT=%u,AUTO_RESUME=OFF",
             kReplayMode == ReplayMode::SHORT_SAFETY_TEST
                 ? "SHORT_SAFETY_TEST" : "FULL_PRODUCTION",
             static_cast<unsigned long>(kReplayHrShortMaxMm),
             ROBOT_V5_FULL_REPLAY_PRODUCTION ? 1U : 0U);
    ESP_LOGI(kTag,
             "ROUTE,MAP_UI_PROTO=3,V2_COMPAT=1,REPLAY_OP=ON,REPLAY_MODES=0-8,LCD_REPLAY_STATUS=ON");

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
#if ROBOT_MAP_OWNER_STM32
    ESP_LOGI(kTag, "ROUTE,INPUT_OWNER=STM32,LEGACY_INPUT_TASK=DISABLED");
    return true;
#endif
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
#if ROBOT_MAP_OWNER_STM32
    (void)action;
    (void)slot;
    return;
#endif
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
    // LOADED + dry-run PASS -> full route replay.
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
            StartFullReplay();
        } else if (mode_ == Mode::REPLAY_HOLD && resume_valid_) {
            AttemptResumeFullReplay();
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
#if ROBOT_MAP_OWNER_STM32
    return;
#endif
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
            "MAP,UI,%u,%u,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu,%u",
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
            static_cast<unsigned long>(replay_error_mm_),
            static_cast<unsigned>(replay_operation_));
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
    replay_operation_ = 0U;
    replay_bearing12_deg_ = replay_bearing23_deg_ = replay_turn_delta_deg_ = 0.0f;
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

    ClearResumeContext();
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
        bool stop_ok = false;
        if (robot_uart_ != nullptr && !robot_uart_->Ps2OverrideActive()) {
            stop_ok = robot_uart_->Stop(700);
        }
        RobotState after;
        const bool state_ok = robot_uart_ != nullptr &&
                              robot_uart_->GetState(after, 700);
        ESP_LOGW(kTag,
                 "ROUTE,HR_CANCEL,STOP_REQUEST=1,STOP_RESULT=%s,STATE_MOVING_AFTER=%u,L_AFTER=%d,R_AFTER=%d",
                 stop_ok && state_ok && !after.moving && after.left == 0 &&
                         after.right == 0
                     ? "PASS" : "FAIL",
                 state_ok ? (after.moving ? 1U : 0U) : 1U,
                 state_ok ? after.left : -1, state_ok ? after.right : -1);
        if (state_ok && !after.moving && after.left == 0 && after.right == 0) {
            replay_motion_running_.store(false);
        }
        return;
    }
    if (mode_ != Mode::REPLAY_READY && mode_ != Mode::REPLAY_HOLD &&
        !replay_plan_valid_) return;
    ClearResumeContext();
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
        const float bearing_deg = atan2f(dy, dx) * 57.2957795f;
        float turn_before = 0.0f;
        if (index >= 2U) {
            const RouteWaypoint& before = loaded_route_.waypoints[index - 2U];
            const float prior_bearing = atan2f(
                static_cast<float>(prior.y_mm - before.y_mm),
                static_cast<float>(prior.x_mm - before.x_mm)) * 57.2957795f;
            turn_before = NormalizeTurnDelta(bearing_deg - prior_bearing);
        }
        const bool segment_invalid = !std::isfinite(distance) ||
            !std::isfinite(bearing_deg) || distance < 20.0f ||
            distance > kReplayMaxSegmentMm;
        const bool turn_invalid = index >= 2U &&
            (!std::isfinite(turn_before) || fabsf(turn_before) > 120.0f);
        if (segment_invalid || turn_invalid) {
            valid = false;
        }
        if (std::isfinite(distance)) {
            summed_mm += static_cast<uint32_t>(lroundf(distance));
        }
        ESP_LOGI(kTag,
                 "ROUTE,FULL_PLAN,SEG=%u/%u,FROM=%u,TO=%u,DIST_MM=%lu,TURN_BEFORE_DEG=%.1f,VALID=%u",
                 static_cast<unsigned>(index), static_cast<unsigned>(count - 1U),
                 static_cast<unsigned>(index), static_cast<unsigned>(index + 1U),
                 static_cast<unsigned long>(lroundf(distance)), turn_before,
                 (segment_invalid || turn_invalid) ? 0U : 1U);
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
    replay_operation_ = 0U;
    mode_ = valid ? Mode::REPLAY_CHECKED : Mode::LOADED;
    odometry_valid_ = false;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=DRY_RUN,PASS=%u,SLOT=%u,POINTS=%u,MOTOR=0,DRIVE=OFF,NEXT=%s",
             valid ? 1U : 0U, static_cast<unsigned>(selected_slot_),
             static_cast<unsigned>(count),
             valid ? "FULL_REPLAY" : "NONE");
    Notify(valid ? "REPLAY DRY PASS: START FULL" : "REPLAY DRY FAIL", 4000);
    UpdateMapStatus();
    return valid;
}

bool TeachRoute::CheckReplaySafety(const char* stage, RobotState& state,
                                   RobotObstacleStatus& obstacle,
                                   const char*& reason, bool require_turn) {
    GetSafetyBlackBox().Record(SafetyEventType::PREFLIGHT_BEGIN,
                               robot_uart_ != nullptr ? robot_uart_->MotionSessionId() : 0,
                               0, stage);
    state = {};
    obstacle = {};
    reason = "OK";
    const char* link_status = "UNAVAILABLE";
    const char* sensor_status = "UNAVAILABLE";
    const char* encoder_status = "UNAVAILABLE";
    const char* odometry_status = "UNAVAILABLE";
    const char* heading_status = "UNAVAILABLE";
    const char* owner_status = "UNAVAILABLE";
    const char* lease_status = "UNAVAILABLE";
    const char* stop_status = "UNAVAILABLE";
    const char* route_status = "UNAVAILABLE";
    const char* camera_status = "WARN";  // Basic replay never requires camera.
    const bool mission = mission_manager_ != nullptr && mission_manager_->IsActive();
    const bool ai_hold = mission_manager_ != nullptr &&
                         mission_manager_->IsAiObstacleHoldActive();
    const bool ps2 = robot_uart_ != nullptr && robot_uart_->Ps2OverrideActive();
    bool pass = true;
    auto fail = [&](const char* value) {
        if (pass) reason = value;
        pass = false;
    };

    if (loaded_route_.header.waypoint_count < 2U ||
        loaded_route_.header.waypoint_count > kMaxWaypointsPerMap) {
        route_status = "FAIL";
        fail("INVALID_ROUTE");
    } else route_status = "PASS";
    stop_status = "PASS";
    owner_status = "PASS";
    lease_status = "PASS";
    if (replay_cancel_requested_.load()) { stop_status = "FAIL"; fail("X_CANCEL"); }
    else if (mission) { owner_status = "FAIL"; fail("MISSION_ACTIVE"); }
    else if (ai_hold) { stop_status = "FAIL"; fail("AI_OBSTACLE_HOLD"); }
    else if (ps2) { owner_status = "FAIL"; fail("PS2_OVERRIDE"); }
    if (pass && (robot_uart_ == nullptr ||
                 !robot_uart_->GetState(state, kReplaySafetyRxTimeoutMs))) {
        link_status = "FAIL"; fail("STATE_RX");
    } else if (pass && !state.valid) { link_status = "FAIL"; fail("STATE_INVALID"); }
    else if (pass) link_status = "PASS";
    if (pass && (state.moving || state.left != 0 || state.right != 0)) {
        stop_status = "FAIL"; fail("MOVING");
    } else if (pass && state.brake_enabled) { stop_status = "FAIL"; fail("BRAKE"); }
    else if (pass && !robot_uart_->SessionReady()) { link_status = "FAIL"; fail("LINK_STALE"); }
    else if (pass && strcmp(state.motion_owner, "NONE") != 0 &&
             strcmp(state.motion_owner, "MCP") != 0) {
        owner_status = "FAIL"; fail("OWNER_INVALID");
    } else if (pass && robot_uart_->MotionLeaseActive()) {
        lease_status = "FAIL"; fail("LEASE_ACTIVE");
    }
    if (pass && !robot_uart_->GetObstacle(obstacle, kReplaySafetyRxTimeoutMs)) {
        sensor_status = "FAIL"; fail("OBS_RX");
    } else if (pass && !obstacle.valid) { sensor_status = "FAIL"; fail("OBS_INVALID"); }
    else if (pass && !obstacle.fresh) { sensor_status = "FAIL"; fail("OBS_STALE"); }
    else if (pass && strcmp(obstacle.health, "HEALTHY") != 0) {
        sensor_status = "FAIL"; fail("SENSOR_HEALTH");
    } else if (pass && (strcmp(obstacle.front_left_health, "HEALTHY") != 0 ||
               strcmp(obstacle.front_right_health, "HEALTHY") != 0 ||
               obstacle.front_left_age_ms > 400U ||
               obstacle.front_right_age_ms > 400U)) {
        sensor_status = "FAIL"; fail("SENSOR_CHANNEL_HEALTH");
    }
    else if (pass && strcmp(obstacle.zone, "CAUTION") == 0) {
        sensor_status = "FAIL"; fail("CAUTION");
    } else if (pass && strcmp(obstacle.zone, "BLOCKED") == 0) {
        sensor_status = "FAIL"; fail("BLOCKED");
    } else if (pass && strcmp(obstacle.zone, "EMERGENCY") == 0) {
        sensor_status = "FAIL"; fail("EMERGENCY");
    } else if (pass && strcmp(obstacle.zone, "CLEAR") != 0) {
        sensor_status = "FAIL"; fail("OBS_INVALID");
    } else if (pass) sensor_status = "PASS";

    RobotEncoderStatus encoder;
    RobotOdometry odometry;
    if (pass && !robot_uart_->GetEncoderStatus(encoder, kReplaySafetyRxTimeoutMs)) {
        encoder_status = "FAIL"; fail("ENCODER_RX");
    } else if (pass && (!encoder.valid || !encoder.ready ||
                        strcmp(encoder.health, "OK") != 0)) {
        encoder_status = "FAIL"; fail("ENCODER_UNRELIABLE");
    } else if (pass) encoder_status = "PASS";
    if (pass && !robot_uart_->GetOdometry(odometry, kReplaySafetyRxTimeoutMs)) {
        odometry_status = "FAIL"; fail("ODOMETRY_RX");
    } else if (pass && !odometry.valid) {
        odometry_status = "FAIL"; fail("ODOMETRY_UNRELIABLE");
    } else if (pass && replay_reset_generation_ != 0U &&
               odometry.reset_generation != replay_reset_generation_) {
        odometry_status = "FAIL"; fail("RESET_BOUNDARY_UNRESOLVED");
    } else if (pass) odometry_status = "PASS";
    if (pass && require_turn) {
        RobotFusionStatus fusion;
        if (!robot_uart_->GetFusionStatus(fusion, kReplaySafetyRxTimeoutMs)) {
            heading_status = "FAIL"; fail("HEADING_RX");
        } else if (!fusion.valid || !fusion.ready ||
                   strcmp(fusion.health, "FUSED") != 0 ||
                   fusion.confidence_pct < 70.0f) {
            heading_status = "FAIL"; fail("HEADING_UNRELIABLE");
        } else heading_status = "PASS";
    }
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=PREFLIGHT,STAGE=%s,PASS=%u,REASON=%s,LINK=%s,SENSORS=%s,ENCODER=%s,ODOMETRY=%s,HEADING=%s,OWNER=%s,LEASE=%s,STOP=%s,ROUTE=%s,CAMERA=%s,MISSION=%u,AI_HOLD=%u,PS2_OVERRIDE=%u,STATE_VALID=%u,OWNER_VALUE=%s,MOVING=%u,BRAKE=%u,OBS_VALID=%u,OBS_FRESH=%u,OBS_ECHO=%u,OBS_HEALTH=%s,OBS_LH=%s,OBS_RH=%s,OBS_LAGE=%lu,OBS_RAGE=%lu,OBS_DIST_CM=%.1f,OBS_ZONE=%s,RESET_GEN=%lu",
             stage != nullptr ? stage : "UNKNOWN", pass ? 1U : 0U, reason,
             link_status, sensor_status, encoder_status, odometry_status,
             heading_status, owner_status, lease_status, stop_status,
             route_status, camera_status,
             mission ? 1U : 0U, ai_hold ? 1U : 0U, ps2 ? 1U : 0U,
             state.valid ? 1U : 0U,
             state.motion_owner[0] != '\0' ? state.motion_owner : "UNKNOWN",
             state.moving ? 1U : 0U,
             state.brake_enabled ? 1U : 0U, obstacle.valid ? 1U : 0U,
             obstacle.fresh ? 1U : 0U, obstacle.echo_valid ? 1U : 0U,
             obstacle.health[0] != '\0' ? obstacle.health : "UNKNOWN",
             obstacle.front_left_health[0] != '\0'
                 ? obstacle.front_left_health : "UNKNOWN",
             obstacle.front_right_health[0] != '\0'
                 ? obstacle.front_right_health : "UNKNOWN",
             static_cast<unsigned long>(obstacle.front_left_age_ms),
             static_cast<unsigned long>(obstacle.front_right_age_ms),
             obstacle.distance_cm,
             obstacle.zone[0] != '\0' ? obstacle.zone : "UNKNOWN",
             static_cast<unsigned long>(odometry.reset_generation));
    GetSafetyBlackBox().Record(pass ? SafetyEventType::PREFLIGHT_PASS
                                    : SafetyEventType::PREFLIGHT_FAIL,
                               robot_uart_ != nullptr ? robot_uart_->MotionSessionId() : 0,
                               0, reason, odometry.reset_generation);
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

bool TeachRoute::StartReplayTurnAtWp2() {
    if (mode_ != Mode::REPLAY_CHECKED) {
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B2_REJECT,REASON=MODE,MODE=%u,EXPECTED=REPLAY_CHECKED,MOTOR=0",
                 static_cast<unsigned>(mode_));
        return false;
    }
    if (!replay_plan_valid_ || replay_motion_running_.load() ||
        robot_uart_ == nullptr || mission_manager_ == nullptr) {
        return false;
    }
    const float abs_delta = fabsf(replay_turn_delta_deg_);
    if (!std::isfinite(abs_delta) || abs_delta > 120.0f) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=B2_REJECT,REASON=TURN_RANGE,MOTOR=0");
        mode_ = Mode::REPLAY_HOLD;
        UpdateMapStatus();
        return false;
    }

    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    if (!CheckReplaySafety("B2_START_GATE", state, obstacle, reason, true)) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=B2_REJECT,REASON=%s,MOTOR=0", reason);
        mode_ = Mode::REPLAY_HOLD;
        UpdateMapStatus();
        return false;
    }

    replay_cancel_requested_.store(false);
    replay_motion_running_.store(true);
    replay_plan_valid_ = false;
    replay_wp_index_ = 2U;
    replay_wp_total_ = loaded_route_.header.waypoint_count;
    replay_target_mm_ = static_cast<uint32_t>(lroundf(abs_delta));
    replay_travel_mm_ = 0U;
    replay_error_mm_ = replay_target_mm_;
    mode_ = Mode::REPLAY_RUNNING;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=B2_START,WP=2/%u,TARGET_DEG=%.1f,SPEED=%d,DIR=%s,MOVE=0",
             static_cast<unsigned>(replay_wp_total_), replay_turn_delta_deg_,
             kReplayB1Speed, replay_turn_delta_deg_ > 0.0f ? "LEFT" :
             replay_turn_delta_deg_ < 0.0f ? "RIGHT" : "ALIGNED");
    Notify("B2 TURN WP2: X/R3 STOP", 3500);
    UpdateMapStatus();
    if (xTaskCreatePinnedToCore(ReplayTurnTaskEntry, "map_replay_b2", 4096,
                                this, 3, &replay_task_, 1) != pdPASS) {
        replay_task_ = nullptr;
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGE(kTag, "ROUTE,REPLAY=B2_TURN_STOP,REASON=TASK_CREATE,CONTINUE=NO");
        UpdateMapStatus();
        return false;
    }
    return true;
}

bool TeachRoute::StartReplayFinalSegment() {
    if (mode_ != Mode::REPLAY_CHECKED) {
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B3_REJECT,REASON=MODE,MODE=%u,EXPECTED=REPLAY_CHECKED,MOTOR=0",
                 static_cast<unsigned>(mode_));
        return false;
    }
    if (!replay_plan_valid_ || replay_motion_running_.load() ||
        robot_uart_ == nullptr || mission_manager_ == nullptr) {
        return false;
    }
    const uint16_t count = loaded_route_.header.waypoint_count;
    if (count < 3U) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=B3_REJECT,REASON=POINTS,MOTOR=0");
        mode_ = Mode::REPLAY_HOLD;
        UpdateMapStatus();
        return false;
    }
    const RouteWaypoint& wp2 = loaded_route_.waypoints[1];
    const RouteWaypoint& wp3 = loaded_route_.waypoints[2];
    const float dx = static_cast<float>(wp3.x_mm - wp2.x_mm);
    const float dy = static_cast<float>(wp3.y_mm - wp2.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    ESP_LOGI(kTag, "ROUTE,REPLAY=B3_GEOMETRY,WP=2->3,DIST_MM=%.1f", distance);
    if (!std::isfinite(distance) || distance < kReplayB3MinSegmentMm ||
        distance > kReplayB3MaxSegmentMm) {
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B3_REJECT,REASON=SEGMENT_RANGE,DIST_MM=%.1f,MOTOR=0",
                 distance);
        mode_ = Mode::REPLAY_HOLD;
        UpdateMapStatus();
        return false;
    }

    ESP_LOGI(kTag,
             "ROUTE,REPLAY=B3_PREREQ,POSE=WP2_ALIGNED,USER_CONFIRM_REQUIRED=1");
    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    if (!CheckReplaySafety("B3_START_GATE", state, obstacle, reason)) {
        ESP_LOGW(kTag, "ROUTE,REPLAY=B3_STOP,REASON=%s,CONTINUE=NO,MOTOR=0",
                 reason);
        mode_ = Mode::REPLAY_HOLD;
        UpdateMapStatus();
        return false;
    }

    replay_cancel_requested_.store(false);
    replay_motion_running_.store(true);
    replay_plan_valid_ = false;
    replay_wp_index_ = 3U;
    replay_wp_total_ = count;
    replay_target_mm_ = static_cast<uint32_t>(lroundf(distance));
    replay_travel_mm_ = 0U;
    replay_error_mm_ = replay_target_mm_;
    mode_ = Mode::REPLAY_RUNNING;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=B3_START,WP=3/%u,TARGET_MM=%lu,SPEED=%d,TURN=OFF,MOVE=0",
             static_cast<unsigned>(count),
             static_cast<unsigned long>(replay_target_mm_), kReplayB1Speed);
    Notify("B3 FINAL WP3: X/R3 STOP", 3500);
    UpdateMapStatus();
    if (xTaskCreatePinnedToCore(ReplayFinalTaskEntry, "map_replay_b3", 4096,
                                this, 3, &replay_task_, 1) != pdPASS) {
        replay_task_ = nullptr;
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGE(kTag,
                 "ROUTE,REPLAY=B3_STOP,REASON=TASK_CREATE,CONTINUE=NO,MOTOR=0");
        UpdateMapStatus();
        return false;
    }
    return true;
}

bool TeachRoute::StartFullReplay() {
    if (mode_ != Mode::REPLAY_CHECKED) {
        ESP_LOGW(kTag,
                 "ROUTE,FULL_REPLAY=REJECT,REASON=MODE,MODE=%u,MOTOR=0",
                 static_cast<unsigned>(mode_));
        return false;
    }
    if (!replay_plan_valid_ || replay_motion_running_.load() ||
        robot_uart_ == nullptr || mission_manager_ == nullptr) {
        return false;
    }
    const uint16_t count = loaded_route_.header.waypoint_count;
    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    if (!CheckReplaySafety("FULL_START_GATE", state, obstacle, reason)) {
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGW(kTag,
                 "ROUTE,FULL_REPLAY=STOP,REASON=%s,CONTINUE=NO,MOTOR=0",
                 reason);
        UpdateMapStatus();
        return false;
    }
    ClearResumeContext();
    replay_reset_generation_ = obstacle.encoder_reset_generation;
    replay_cancel_requested_.store(false);
    replay_motion_running_.store(true);
    replay_plan_valid_ = false;
    replay_wp_index_ = 2U;
    replay_wp_total_ = count;
    replay_target_mm_ = 0U;
    replay_travel_mm_ = 0U;
    replay_error_mm_ = 0U;
    replay_operation_ = 0U;
    mode_ = Mode::REPLAY_RUNNING;
    ESP_LOGI(kTag,
             "ROUTE,FULL_REPLAY=START,SLOT=%u,POINTS=%u,SEGMENTS=%u,MOTOR=0",
             static_cast<unsigned>(selected_slot_), static_cast<unsigned>(count),
             static_cast<unsigned>(count - 1U));
    if ((kReplayMode == ReplayMode::SHORT_SAFETY_TEST)) {
        ESP_LOGI(kTag,
                 "ROUTE,HR_SHORT=START,MAX_MM=%lu,TURN=OFF,NEXT_WP=OFF",
                 static_cast<unsigned long>(kReplayHrShortMaxMm));
    }
    Notify("FULL REPLAY START: X/R3 STOP", 3500);
    UpdateMapStatus();
    if (xTaskCreatePinnedToCore(FullReplayTaskEntry, "map_replay_full", 6144,
                                this, 3, &replay_task_, 1) != pdPASS) {
        replay_task_ = nullptr;
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGE(kTag,
                 "ROUTE,FULL_REPLAY=STOP,REASON=TASK_CREATE,CONTINUE=NO,MOTOR=0");
        UpdateMapStatus();
        return false;
    }
    return true;
}

bool TeachRoute::AttemptResumeFullReplay() {
    if (mode_ != Mode::REPLAY_HOLD || !resume_valid_ ||
        replay_motion_running_.load() || robot_uart_ == nullptr ||
        mission_manager_ == nullptr) {
        return false;
    }
    ESP_LOGI(kTag,
             "ROUTE,FULL_REPLAY=RESUME_REQUEST,WP=%u/%u,REMAIN_MM=%lu",
             static_cast<unsigned>(resume_wp_index_),
             static_cast<unsigned>(replay_wp_total_),
             static_cast<unsigned long>(resume_remaining_mm_));
    replay_cancel_requested_.store(false);
    replay_motion_running_.store(true);
    if (xTaskCreatePinnedToCore(FullReplayTaskEntry, "map_replay_resume",
                                6144, this, 3, &replay_task_, 1) != pdPASS) {
        replay_task_ = nullptr;
        replay_motion_running_.store(false);
        ESP_LOGW(kTag,
                 "ROUTE,FULL_REPLAY=RESUME_STOP,REASON=TASK_CREATE,MOTOR=0");
        UpdateMapStatus();
        return false;
    }
    UpdateMapStatus();
    return true;
}

void TeachRoute::ReplayTurnTaskEntry(void* context) {
    if (context != nullptr) {
        static_cast<TeachRoute*>(context)->RunReplayTurnAtWp2();
        static_cast<TeachRoute*>(context)->replay_task_ = nullptr;
    }
    vTaskDelete(nullptr);
}

void TeachRoute::ReplayFinalTaskEntry(void* context) {
    if (context != nullptr) {
        static_cast<TeachRoute*>(context)->RunReplayFinalSegment();
        static_cast<TeachRoute*>(context)->replay_task_ = nullptr;
    }
    vTaskDelete(nullptr);
}

void TeachRoute::RunReplayTurnAtWp2() {
    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    bool safety_ready = false;
    unsigned int precheck_attempt = 0;
    for (precheck_attempt = 1; precheck_attempt <= 3; ++precheck_attempt) {
        char stage[32];
        snprintf(stage, sizeof(stage), "B2_WORKER_PRECHECK_%u",
                 precheck_attempt);
        if (CheckReplaySafety(stage, state, obstacle, reason, true)) {
            safety_ready = true;
            ESP_LOGI(kTag,
                     "ROUTE,REPLAY=B2_PRECHECK_READY,ATTEMPT=%u/3,REASON=OK",
                     precheck_attempt);
            break;
        }
        const bool retryable = strcmp(reason, "STATE_RX") == 0 ||
                               strcmp(reason, "OBS_RX") == 0;
        if (!retryable) break;
        if (precheck_attempt < 3) {
            ESP_LOGW(kTag,
                     "ROUTE,REPLAY=B2_PRECHECK_RETRY,ATTEMPT=%u/3,REASON=%s",
                     precheck_attempt, reason);
            vTaskDelay(pdMS_TO_TICKS(precheck_attempt == 1 ? 100 : 150));
        }
    }
    if (!safety_ready) {
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        const bool exhausted = precheck_attempt >= 3 &&
                               (strcmp(reason, "STATE_RX") == 0 ||
                                strcmp(reason, "OBS_RX") == 0);
        const char* stop_reason = exhausted
            ? (strcmp(reason, "STATE_RX") == 0
                   ? "STATE_RX_RETRY_EXHAUSTED"
                   : "OBS_RX_RETRY_EXHAUSTED")
            : reason;
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B2_TURN_STOP,REASON=%s,CONTINUE=NO,MOVE=0",
                 stop_reason);
        UpdateMapStatus();
        return;
    }

    const float abs_delta = fabsf(replay_turn_delta_deg_);
    RobotTurnResult result;
    bool ok = true;
    bool ai_mode = false;
    if (abs_delta >= 2.0f) {
        ai_mode = robot_uart_ != nullptr && robot_uart_->SetMode(true, 700);
        if (!ai_mode) {
            replay_motion_running_.store(false);
            mode_ = Mode::REPLAY_HOLD;
            ESP_LOGW(kTag, "ROUTE,REPLAY=B2_TURN_STOP,REASON=AI_MODE_SESSION,CONTINUE=NO,MOVE=0");
            UpdateMapStatus();
            return;
        }
        ESP_LOGI(kTag, "ROUTE,REPLAY=B2_SESSION,PASS=1,MODE=AI");
        ok = robot_uart_ != nullptr && robot_uart_->TurnRelative(
            replay_turn_delta_deg_ > 0.0f, static_cast<int>(lroundf(abs_delta)),
            kReplayB1Speed, result, 25000);
    } else {
        result.completed = true;
        result.target_deg = 0.0f;
        result.heading_deg = 0.0f;
        result.error_deg = 0.0f;
    }
    if (ai_mode && robot_uart_ != nullptr && !robot_uart_->Ps2OverrideActive()) {
        const bool restored = robot_uart_->SetMode(false, 700);
        ESP_LOGI(kTag, "ROUTE,REPLAY=B2_SESSION,MODE=MANUAL,RESTORE=%s",
                 restored ? "PASS" : "FAIL");
    }
    replay_motion_running_.store(false);
    if (!ok || !result.completed) {
        mode_ = Mode::REPLAY_HOLD;
        const char* fail = ok ? "TURN_FAIL" : "TURN_STOP";
        ESP_LOGW(kTag, "ROUTE,REPLAY=B2_TURN_STOP,REASON=%s,CONTINUE=NO", fail);
        UpdateMapStatus();
        return;
    }
    replay_travel_mm_ = 0U;
    replay_error_mm_ = static_cast<uint32_t>(lroundf(fabsf(result.error_deg)));
    mode_ = Mode::REPLAY_COMPLETE;
    ESP_LOGI(kTag,
             "ROUTE,REPLAY=B2_TURN_DONE,WP=2/3,DELTA_DEG=%.1f,TARGET_DEG=%.1f,FINAL_H_DEG=%.1f,ERR_DEG=%.1f,CONTINUE=NO,MOVE=0",
             replay_turn_delta_deg_, result.target_deg, result.heading_deg,
             result.error_deg);
    Notify("B2 TURN DONE: STOPPED", 3500);
    UpdateMapStatus();
}

void TeachRoute::ClearResumeContext() {
    resume_valid_ = false;
    resume_wp_index_ = 0U;
    resume_original_target_mm_ = 0U;
    resume_completed_mm_ = 0U;
    resume_remaining_mm_ = 0U;
    resume_count_ = 0U;
    resume_hold_x_mm_ = 0.0f;
    resume_hold_y_mm_ = 0.0f;
    resume_hold_heading_rad_ = 0.0f;
    resume_reset_generation_ = 0;
}

void TeachRoute::RunReplayFinalSegment() {
    const RouteWaypoint& wp2 = loaded_route_.waypoints[1];
    const RouteWaypoint& wp3 = loaded_route_.waypoints[2];
    const float dx = static_cast<float>(wp3.x_mm - wp2.x_mm);
    const float dy = static_cast<float>(wp3.y_mm - wp2.y_mm);
    const int target_mm = static_cast<int>(lroundf(sqrtf(dx * dx + dy * dy)));

    RobotState state;
    RobotObstacleStatus obstacle;
    const char* reason = "OK";
    bool safety_ready = false;
    unsigned int precheck_attempt = 0;
    for (precheck_attempt = 1; precheck_attempt <= 3; ++precheck_attempt) {
        char stage[32];
        snprintf(stage, sizeof(stage), "B3_WORKER_PRECHECK_%u",
                 precheck_attempt);
        if (CheckReplaySafety(stage, state, obstacle, reason)) {
            safety_ready = true;
            ESP_LOGI(kTag,
                     "ROUTE,REPLAY=B3_PRECHECK_READY,ATTEMPT=%u/3,REASON=OK",
                     precheck_attempt);
            break;
        }
        const bool retryable = strcmp(reason, "STATE_RX") == 0 ||
                               strcmp(reason, "OBS_RX") == 0;
        if (!retryable) break;
        if (precheck_attempt < 3) {
            ESP_LOGW(kTag,
                     "ROUTE,REPLAY=B3_PRECHECK_RETRY,ATTEMPT=%u/3,REASON=%s",
                     precheck_attempt, reason);
            vTaskDelay(pdMS_TO_TICKS(precheck_attempt == 1 ? 100 : 150));
        }
    }

    if (!safety_ready) {
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        const bool exhausted = precheck_attempt >= 3 &&
                               (strcmp(reason, "STATE_RX") == 0 ||
                                strcmp(reason, "OBS_RX") == 0);
        const char* stop_reason = exhausted
            ? (strcmp(reason, "STATE_RX") == 0
                   ? "STATE_RX_RETRY_EXHAUSTED"
                   : "OBS_RX_RETRY_EXHAUSTED")
            : reason;
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B3_STOP,REASON=%s,TRAVEL_MM=0,CONTINUE=NO,MOTOR=0",
                 stop_reason);
        UpdateMapStatus();
        return;
    }

    const bool ai_mode = robot_uart_->SetMode(true, 700);
    if (!ai_mode) {
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B3_STOP,REASON=AI_MODE_SESSION,CONTINUE=NO,MOTOR=0");
        UpdateMapStatus();
        return;
    }
    ESP_LOGI(kTag, "ROUTE,REPLAY=B3_SESSION,PASS=1,MODE=AI");

    RobotDistanceResult result;
    const bool moved = robot_uart_->MoveDistance(true, target_mm,
                                                  kReplayB1Speed, result,
                                                  kReplayB3TimeoutMs);
    bool success = false;
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
        } else {
            reason = moved ? "MOVE_INCOMPLETE" : "MOVE_FAIL";
        }
    }

    if (!robot_uart_->Ps2OverrideActive()) {
        const bool restored = robot_uart_->SetMode(false, 700);
        ESP_LOGI(kTag, "ROUTE,REPLAY=B3_SESSION,MODE=MANUAL,RESTORE=%s",
                 restored ? "PASS" : "FAIL");
    }

    replay_motion_running_.store(false);
    replay_travel_mm_ = static_cast<uint32_t>(lroundf(result.travelled_mm));
    replay_error_mm_ = static_cast<uint32_t>(lroundf(
        fabsf(result.travelled_mm - result.target_mm)));
    if (success) {
        mode_ = Mode::REPLAY_COMPLETE;
        ESP_LOGI(kTag,
                 "ROUTE,REPLAY=B3_DONE,WP=3/3,TARGET_MM=%.1f,TRAVEL_MM=%.1f,ERR_MM=%.1f,CONTINUE=NO,TURN=0,MOTOR=0",
                 result.target_mm, result.travelled_mm,
                 fabsf(result.travelled_mm - result.target_mm));
        Notify("B3 COMPLETE: ROUTE FINISHED", 4000);
    } else {
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGW(kTag,
                 "ROUTE,REPLAY=B3_STOP,REASON=%s,TRAVEL_MM=%.1f,CONTINUE=NO,MOTOR=0",
                 reason, result.travelled_mm);
        Notify("B3 STOPPED: NO CONTINUE", 4000);
    }
    replay_cancel_requested_.store(false);
    replay_plan_valid_ = false;
    odometry_valid_ = false;
    UpdateMapStatus();
}

void TeachRoute::FullReplayTaskEntry(void* context) {
    if (context != nullptr) {
        static_cast<TeachRoute*>(context)->RunFullReplay();
        static_cast<TeachRoute*>(context)->replay_task_ = nullptr;
    }
    vTaskDelete(nullptr);
}

void TeachRoute::RunFullReplay() {
    const uint16_t count = loaded_route_.header.waypoint_count;
    bool ai_mode = false;
    uint32_t total_target = 0U;
    uint32_t total_travel = 0U;

    auto stop_hold = [&](const char* reason) {
        if (ai_mode && !robot_uart_->Ps2OverrideActive()) {
            (void)robot_uart_->SetMode(false, 700);
        }
        replay_operation_ = 0U;
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        GetSafetyBlackBox().Record(SafetyEventType::HOLD,
                                   robot_uart_->MotionSessionId(), 0, reason);
        ESP_LOGW(kTag,
                 "ROUTE,FULL_REPLAY=STOP,TERMINAL=%s,WP=%u/%u,REASON=%s,CONTINUE=NO,MOTOR=0",
                 ReplayTerminalStatus(reason),
                 static_cast<unsigned>(replay_wp_index_),
                 static_cast<unsigned>(count), reason);
        Notify("FULL REPLAY HOLD: X CANCEL", 4000);
        UpdateMapStatus();
    };

    auto ensure_ai_mode = [&]() -> bool {
        if (ai_mode) return true;
        ai_mode = robot_uart_->SetMode(true, 700);
        if (ai_mode) {
            GetSafetyBlackBox().Record(SafetyEventType::OWNER_ACQUIRE,
                                       robot_uart_->MotionSessionId(), 0,
                                       "REPLAY");
            ESP_LOGI(kTag, "ROUTE,FULL_REPLAY=SESSION,PASS=1,MODE=AI");
        } else {
            GetSafetyBlackBox().Record(SafetyEventType::OWNER_REJECT,
                                       robot_uart_->MotionSessionId(), 0,
                                       "AI_MODE");
        }
        return ai_mode;
    };

    auto hold_obstacle = [&](uint16_t index, uint32_t target_mm,
                             const RobotDistanceResult& result,
                             uint32_t prior_target, uint32_t prior_travel) {
        if (resume_count_ >= 3U) {
            replay_operation_ = 0U;
            replay_motion_running_.store(false);
            mode_ = Mode::REPLAY_HOLD;
            ESP_LOGW(kTag,
                     "ROUTE,FULL_REPLAY=HOLD,WP=%u/%u,REASON=RESUME_LIMIT,CONTINUE=NO,MOTOR=0",
                     static_cast<unsigned>(index + 1U),
                     static_cast<unsigned>(count));
            UpdateMapStatus();
            return;
        }
        RobotOdometry hold_pose;
        if (!robot_uart_->GetOdometry(hold_pose, 700) || !hold_pose.valid) {
            stop_hold("ODOMETRY");
            return;
        }
        const uint32_t completed = static_cast<uint32_t>(std::max(
            0.0f, std::min(static_cast<float>(target_mm), result.travelled_mm)));
        const uint32_t remaining = target_mm > completed ? target_mm - completed : 0U;
        resume_valid_ = remaining > 0U;
        GetSafetyBlackBox().Record(SafetyEventType::HOLD,
                                   robot_uart_->MotionSessionId(),
                                   result.operation_id, "OBSTACLE",
                                   hold_pose.reset_generation, 0, index);
        resume_wp_index_ = static_cast<uint16_t>(index + 1U);
        resume_original_target_mm_ = target_mm;
        resume_completed_mm_ = completed;
        resume_remaining_mm_ = remaining;
        resume_count_++;
        resume_hold_x_mm_ = hold_pose.x_mm;
        resume_hold_y_mm_ = hold_pose.y_mm;
        resume_hold_heading_rad_ = hold_pose.heading_rad;
        resume_reset_generation_ = hold_pose.reset_generation;
        replay_wp_index_ = resume_wp_index_;
        replay_wp_total_ = count;
        replay_operation_ = 3U;
        replay_target_mm_ = target_mm;
        replay_travel_mm_ = completed;
        replay_error_mm_ = remaining;
        if (ai_mode && !robot_uart_->Ps2OverrideActive()) {
            (void)robot_uart_->SetMode(false, 700);
            ai_mode = false;
        }
        replay_motion_running_.store(false);
        mode_ = Mode::REPLAY_HOLD;
        ESP_LOGW(kTag,
                 "ROUTE,FULL_REPLAY=HOLD,WP=%u/%u,OP=MOVE,REASON=OBSTACLE,TARGET_MM=%lu,TRAVEL_MM=%lu,REMAIN_MM=%lu,RESUMABLE=%u,HOLD_X_MM=%.1f,HOLD_Y_MM=%.1f,HOLD_H_RAD=%.3f,PRIOR_TARGET_MM=%lu,PRIOR_TRAVEL_MM=%lu,MOTOR=0",
                 static_cast<unsigned>(resume_wp_index_),
                 static_cast<unsigned>(count),
                 static_cast<unsigned long>(target_mm),
                 static_cast<unsigned long>(completed),
                 static_cast<unsigned long>(remaining), resume_valid_ ? 1U : 0U,
                 hold_pose.x_mm, hold_pose.y_mm, hold_pose.heading_rad,
                 static_cast<unsigned long>(prior_target),
                 static_cast<unsigned long>(prior_travel));
        Notify(resume_valid_ ? "MAP HOLD: START RESUME" : "MAP HOLD: REPLAY STOPPED", 4000);
        UpdateMapStatus();
    };

    uint16_t first_index = 1U;
    if (resume_valid_) {
        const uint16_t resume_index = resume_wp_index_;
        const uint32_t original_target = resume_original_target_mm_;
        const uint32_t completed_before = resume_completed_mm_;
        const uint32_t remaining = resume_remaining_mm_;
        RobotState resume_state;
        RobotObstacleStatus resume_obstacle;
        bool clear_samples = true;
        ESP_LOGI(kTag,
                 "ROUTE,FULL_REPLAY=RESUME_REQUEST,WP=%u/%u,REMAIN_MM=%lu",
                 static_cast<unsigned>(resume_index),
                 static_cast<unsigned>(count),
                 static_cast<unsigned long>(remaining));
        if (!robot_uart_->GetState(resume_state, kReplaySafetyRxTimeoutMs) ||
            !resume_state.valid || resume_state.moving ||
            resume_state.left != 0 || resume_state.right != 0 ||
            resume_state.brake_enabled || robot_uart_->Ps2OverrideActive() ||
            mission_manager_->IsActive()) {
            stop_hold("RESUME_STATE");
            return;
        }
        for (unsigned int sample = 1U; sample <= 3U; ++sample) {
            if (!robot_uart_->GetObstacle(resume_obstacle,
                                          kReplaySafetyRxTimeoutMs) ||
                !resume_obstacle.valid || !resume_obstacle.fresh ||
                strcmp(resume_obstacle.zone, "CLEAR") != 0) {
                clear_samples = false;
                ESP_LOGW(kTag,
                         "ROUTE,RESUME_CLEAR,SAMPLE=%u/3,ZONE=%s,PASS=0",
                         sample, resume_obstacle.zone[0] != '\0'
                                    ? resume_obstacle.zone : "UNKNOWN");
                break;
            }
            ESP_LOGI(kTag, "ROUTE,RESUME_CLEAR,SAMPLE=%u/3,ZONE=CLEAR",
                     sample);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!clear_samples) {
            stop_hold("RESUME_OBSTACLE");
            return;
        }
        RobotOdometry current_pose;
        if (!robot_uart_->GetOdometry(current_pose, 700) || !current_pose.valid) {
            stop_hold("POSE_CHANGED");
            return;
        }
        if (current_pose.reset_generation != resume_reset_generation_) {
            GetSafetyBlackBox().Record(SafetyEventType::RESET_BOUNDARY,
                                       robot_uart_->MotionSessionId(), 0,
                                       "RESUME_RESET", current_pose.reset_generation);
            ESP_LOGW(kTag,
                     "ROUTE,RESET_BOUNDARY_UNRESOLVED,HOLD_GEN=%lu,CURRENT_GEN=%lu,RESUME=0",
                     static_cast<unsigned long>(resume_reset_generation_),
                     static_cast<unsigned long>(current_pose.reset_generation));
            stop_hold("RESET_BOUNDARY_UNRESOLVED");
            return;
        }
        const float drift_mm = hypotf(current_pose.x_mm - resume_hold_x_mm_,
                                      current_pose.y_mm - resume_hold_y_mm_);
        const float drift_deg = fabsf(NormalizeTurnDelta(
            (current_pose.heading_rad - resume_hold_heading_rad_) * 57.2957795f));
        ESP_LOGI(kTag, "ROUTE,RESUME_POSE,DRIFT_MM=%.1f,DRIFT_DEG=%.1f,PASS=%u",
                 drift_mm, drift_deg, (drift_mm <= 30.0f && drift_deg <= 5.0f) ? 1U : 0U);
        if (drift_mm > 30.0f || drift_deg > 5.0f) {
            stop_hold("POSE_CHANGED");
            return;
        }
        if (!mission_manager_->ReleaseAiObstacleHoldForReplay()) {
            stop_hold("MISSION_ACTIVE");
            return;
        }
        RobotState gate_state;
        RobotObstacleStatus gate_obstacle;
        const char* gate_reason = "OK";
        if (!CheckReplaySafety("FULL_RESUME_GATE", gate_state, gate_obstacle,
                               gate_reason)) {
            stop_hold(gate_reason);
            return;
        }
        GetSafetyBlackBox().Record(SafetyEventType::RESUME,
                                   robot_uart_->MotionSessionId(), 0,
                                   "MANUAL");
        if (!ensure_ai_mode()) {
            stop_hold("AI_MODE_SESSION");
            return;
        }
        replay_operation_ = 1U;
        mode_ = Mode::REPLAY_RUNNING;
        RobotDistanceResult resume_result;
        const bool resumed = robot_uart_->MoveDistance(
            true, static_cast<int>(remaining), kReplayB1Speed, resume_result,
            kReplayB1TimeoutMs);
        if (resume_result.code == RobotDistanceResult::Code::OBSTACLE) {
            hold_obstacle(static_cast<uint16_t>(resume_index - 1U),
                          original_target, resume_result, original_target,
                          completed_before);
            return;
        }
        if (!resumed || !resume_result.completed) {
            stop_hold(resume_result.code == RobotDistanceResult::Code::ENCODER_FAULT
                          ? "ENCODER_FAULT"
                          : resume_result.code == RobotDistanceResult::Code::TIMEOUT
                                ? "TIMEOUT"
                                : resume_result.code == RobotDistanceResult::Code::LINK_ERROR
                                      ? "LINK_LOSS" : "MOVE_FAIL");
            return;
        }
        total_target += original_target;
        total_travel += completed_before +
                        static_cast<uint32_t>(lroundf(resume_result.travelled_mm));
        ESP_LOGI(kTag,
                 "ROUTE,FULL_REPLAY=RESUME_MOVE_DONE,WP=%u/%u,RUN_MM=%.1f,SEG_TOTAL_MM=%lu",
                 static_cast<unsigned>(resume_index), static_cast<unsigned>(count),
                 resume_result.travelled_mm,
                 static_cast<unsigned long>(completed_before +
                     static_cast<uint32_t>(lroundf(resume_result.travelled_mm))));
        if ((kReplayMode == ReplayMode::SHORT_SAFETY_TEST)) {
            if (ai_mode && !robot_uart_->Ps2OverrideActive()) {
                (void)robot_uart_->SetMode(false, 700);
            }
            replay_operation_ = 3U;
            replay_motion_running_.store(false);
            replay_travel_mm_ = completed_before +
                static_cast<uint32_t>(lroundf(resume_result.travelled_mm));
            replay_error_mm_ = replay_travel_mm_ > original_target
                ? replay_travel_mm_ - original_target
                : original_target - replay_travel_mm_;
            mode_ = Mode::REPLAY_COMPLETE;
            ESP_LOGI(kTag,
                     "ROUTE,HR_SHORT=DONE,TERMINAL=TEST_COMPLETE,TARGET_MM=%lu,TRAVEL_MM=%lu,MOTOR=0,CONTINUE=NO",
                     static_cast<unsigned long>(original_target),
                     static_cast<unsigned long>(replay_travel_mm_));
            Notify("MAP TEST DONE: STOPPED", 4000);
            ClearResumeContext();
            replay_cancel_requested_.store(false);
            replay_plan_valid_ = false;
            odometry_valid_ = false;
            UpdateMapStatus();
            return;
        }
        ClearResumeContext();
        first_index = resume_index;
    }

    for (uint16_t index = first_index; index < count; ++index) {
        if (replay_cancel_requested_.load()) {
            stop_hold("X_CANCEL");
            return;
        }

        const RouteWaypoint& prior = loaded_route_.waypoints[index - 1U];
        const RouteWaypoint& point = loaded_route_.waypoints[index];
        const float dx = static_cast<float>(point.x_mm - prior.x_mm);
        const float dy = static_cast<float>(point.y_mm - prior.y_mm);
        const float distance = sqrtf(dx * dx + dy * dy);
        const uint32_t test_target_mm = (kReplayMode == ReplayMode::SHORT_SAFETY_TEST) && index == 1U
            ? std::min(static_cast<uint32_t>(lroundf(distance)),
                       kReplayHrShortMaxMm)
            : static_cast<uint32_t>(lroundf(distance));
        float turn_delta = 0.0f;
        if (index >= 2U) {
            const RouteWaypoint& before = loaded_route_.waypoints[index - 2U];
            const float previous_bearing = atan2f(
                static_cast<float>(prior.y_mm - before.y_mm),
                static_cast<float>(prior.x_mm - before.x_mm)) * 57.2957795f;
            const float next_bearing = atan2f(dy, dx) * 57.2957795f;
            turn_delta = NormalizeTurnDelta(next_bearing - previous_bearing);
        }
        if (!std::isfinite(distance) || distance < 20.0f ||
            distance > kReplayMaxSegmentMm ||
            !std::isfinite(turn_delta) || fabsf(turn_delta) > 120.0f) {
            stop_hold("PLAN_INVALID");
            return;
        }

        if (index >= 2U && fabsf(turn_delta) >= 2.0f) {
            replay_wp_index_ = index;
            replay_operation_ = 2U;
            replay_target_mm_ = static_cast<uint32_t>(lroundf(fabsf(turn_delta)));
            replay_travel_mm_ = 0U;
            replay_error_mm_ = replay_target_mm_;
            UpdateMapStatus();

            RobotState state;
            RobotObstacleStatus obstacle;
            const char* reason = "OK";
            bool ready = false;
            for (unsigned int attempt = 1; attempt <= 3; ++attempt) {
                char stage[32];
                snprintf(stage, sizeof(stage), "FULL_TURN_PRECHECK_%u", attempt);
                if (CheckReplaySafety(stage, state, obstacle, reason, true)) {
                    ready = true;
                    break;
                }
                if (strcmp(reason, "STATE_RX") != 0 &&
                    strcmp(reason, "OBS_RX") != 0) break;
                if (attempt < 3) {
                    ESP_LOGW(kTag,
                             "ROUTE,FULL_REPLAY=PRECHECK_RETRY,OP=TURN,ATTEMPT=%u/3,REASON=%s",
                             attempt, reason);
                    vTaskDelay(pdMS_TO_TICKS(attempt == 1 ? 100 : 150));
                }
            }
            if (!ready) {
                stop_hold(reason);
                return;
            }
            if (!ensure_ai_mode()) {
                stop_hold("AI_MODE_SESSION");
                return;
            }

            RobotTurnResult turn_result;
            const bool turned = robot_uart_->TurnRelative(
                turn_delta > 0.0f, static_cast<int>(lroundf(fabsf(turn_delta))),
                kReplayB1Speed, turn_result, 25000);
            if (!turned || !turn_result.completed) {
                stop_hold("TURN_FAIL");
                return;
            }
            if (fabsf(turn_result.error_deg) > 8.0f) {
                stop_hold("TURN_ERROR");
                return;
            }
            replay_error_mm_ = static_cast<uint32_t>(lroundf(
                fabsf(turn_result.error_deg)));
            ESP_LOGI(kTag,
                     "ROUTE,FULL_TURN_DONE,WP=%u/%u,DELTA_DEG=%.1f,TARGET_DEG=%.1f,FINAL_H_DEG=%.1f,ERR_DEG=%.1f",
                     static_cast<unsigned>(index), static_cast<unsigned>(count),
                     turn_delta, turn_result.target_deg, turn_result.heading_deg,
                     turn_result.error_deg);
        }

        replay_wp_index_ = index + 1U;
        replay_operation_ = 1U;
        replay_target_mm_ = test_target_mm;
        replay_travel_mm_ = 0U;
        replay_error_mm_ = replay_target_mm_;
        UpdateMapStatus();
        RobotState state;
        RobotObstacleStatus obstacle;
        const char* reason = "OK";
        bool ready = false;
        for (unsigned int attempt = 1; attempt <= 3; ++attempt) {
            char stage[32];
            snprintf(stage, sizeof(stage), "FULL_MOVE_PRECHECK_%u", attempt);
            if (CheckReplaySafety(stage, state, obstacle, reason)) {
                ready = true;
                break;
            }
            if (strcmp(reason, "STATE_RX") != 0 &&
                strcmp(reason, "OBS_RX") != 0) break;
            if (attempt < 3) {
                ESP_LOGW(kTag,
                         "ROUTE,FULL_REPLAY=PRECHECK_RETRY,OP=MOVE,ATTEMPT=%u/3,REASON=%s",
                         attempt, reason);
                vTaskDelay(pdMS_TO_TICKS(attempt == 1 ? 100 : 150));
            }
        }
        if (!ready) {
            stop_hold(reason);
            return;
        }
        if (!ensure_ai_mode()) {
            stop_hold("AI_MODE_SESSION");
            return;
        }

        RobotDistanceResult move_result;
        const bool moved = robot_uart_->MoveDistance(
            true, static_cast<int>(replay_target_mm_), kReplayB1Speed,
            move_result, kReplayB1TimeoutMs);
        if (replay_cancel_requested_.load()) {
            stop_hold("X_CANCEL");
            return;
        }
        if (robot_uart_->Ps2OverrideActive()) {
            stop_hold("PS2_OVERRIDE");
            return;
        }
        RobotState after;
        if (robot_uart_->GetState(after, 500) && after.valid &&
            after.brake_enabled) {
            stop_hold("R3_BRAKE");
            return;
        }
        if (!moved || !move_result.completed) {
            if (move_result.code == RobotDistanceResult::Code::OBSTACLE) {
                hold_obstacle(index, replay_target_mm_, move_result,
                              total_target, total_travel);
            } else if (move_result.code == RobotDistanceResult::Code::ENCODER_FAULT) {
                stop_hold("ENCODER_FAULT");
            } else if (move_result.code == RobotDistanceResult::Code::TIMEOUT) {
                stop_hold("TIMEOUT");
            } else if (move_result.code == RobotDistanceResult::Code::LINK_ERROR) {
                stop_hold("LINK_LOSS");
            } else {
                stop_hold("MOVE_FAIL");
            }
            return;
        }
        replay_travel_mm_ = static_cast<uint32_t>(lroundf(move_result.travelled_mm));
        replay_error_mm_ = static_cast<uint32_t>(lroundf(
            fabsf(move_result.travelled_mm - move_result.target_mm)));
        total_target += static_cast<uint32_t>(lroundf(move_result.target_mm));
        total_travel += replay_travel_mm_;
        ESP_LOGI(kTag,
                 "ROUTE,FULL_MOVE_DONE,WP=%u/%u,TARGET_MM=%.1f,TRAVEL_MM=%.1f,ERR_MM=%.1f",
                 static_cast<unsigned>(index + 1U), static_cast<unsigned>(count),
                 move_result.target_mm, move_result.travelled_mm,
                 fabsf(move_result.travelled_mm - move_result.target_mm));
        if ((kReplayMode == ReplayMode::SHORT_SAFETY_TEST) && index == 1U) {
            if (ai_mode && !robot_uart_->Ps2OverrideActive()) {
                (void)robot_uart_->SetMode(false, 700);
            }
            replay_operation_ = 3U;
            replay_motion_running_.store(false);
            replay_travel_mm_ = static_cast<uint32_t>(lroundf(
                move_result.travelled_mm));
            replay_error_mm_ = replay_travel_mm_ > replay_target_mm_
                ? replay_travel_mm_ - replay_target_mm_
                : replay_target_mm_ - replay_travel_mm_;
            mode_ = Mode::REPLAY_COMPLETE;
            ESP_LOGI(kTag,
                     "ROUTE,HR_SHORT=DONE,TERMINAL=TEST_COMPLETE,TARGET_MM=%lu,TRAVEL_MM=%.1f,MOTOR=0,CONTINUE=NO",
                     static_cast<unsigned long>(replay_target_mm_),
                     move_result.travelled_mm);
            Notify("MAP TEST DONE: STOPPED", 4000);
            replay_cancel_requested_.store(false);
            replay_plan_valid_ = false;
            odometry_valid_ = false;
            UpdateMapStatus();
            return;
        }
    }

    if (ai_mode && !robot_uart_->Ps2OverrideActive()) {
        (void)robot_uart_->SetMode(false, 700);
    }
    replay_operation_ = 0U;
    replay_motion_running_.store(false);
    replay_travel_mm_ = total_travel;
    replay_error_mm_ = total_travel > total_target
        ? total_travel - total_target : total_target - total_travel;
    mode_ = Mode::REPLAY_COMPLETE;
    ESP_LOGI(kTag,
             "ROUTE,FULL_REPLAY=COMPLETE,TERMINAL=ROUTE_COMPLETE,SLOT=%u,POINTS=%u,SEGMENTS=%u,TOTAL_TARGET_MM=%lu,TOTAL_TRAVEL_MM=%lu,MOTOR=0",
             static_cast<unsigned>(selected_slot_), static_cast<unsigned>(count),
             static_cast<unsigned>(count - 1U),
             static_cast<unsigned long>(total_target),
             static_cast<unsigned long>(total_travel));
    Notify("FULL REPLAY COMPLETE", 4000);
    replay_cancel_requested_.store(false);
    replay_plan_valid_ = false;
    odometry_valid_ = false;
    UpdateMapStatus();
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

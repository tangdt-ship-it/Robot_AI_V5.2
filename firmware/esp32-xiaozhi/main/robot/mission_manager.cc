#include "mission_manager.h"

#include "robot_uart.h"
#include "application.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_rom_crc.h>
#include <nvs.h>

#include <freertos/idf_additions.h>

#include "cJSON.h"

// STM32 emergency braking and command-link fail-safe remain active
// independently. ESP32 automatic detour is a commissioning flag and defaults
// off so an autonomous mission stops at an obstacle until HIL validation.
#ifndef CONFIG_OBSTACLE_SHADOW_MODE
#define CONFIG_OBSTACLE_SHADOW_MODE 0
#endif
#ifndef CONFIG_AUTOMATIC_DETOUR
#define CONFIG_AUTOMATIC_DETOUR 0
#endif

namespace {
constexpr const char* kTag = "MissionManager";
constexpr const char* kShadowTag = "ObstacleShadow";
constexpr float kPi = 3.14159265358979323846f;

// Shadow-mode local path planner weights (kept in one place so tuning does not
// require hunting through the file).
constexpr float kWObstacle = 1.00f;   // HC-SR04 distance/zone dominance.
constexpr float kWVision = 0.60f;     // Vision risk/clear contribution.
constexpr float kWHeading = 0.15f;    // Prefer the side closer to H0.
constexpr float kWTurn = 0.10f;       // No servo: both sides cost one fixed turn.
constexpr float kInvalidDirectionCost = 1000.0f;
constexpr float kBlockedDirectionCost = 500.0f;
constexpr float kCautionDistanceCm = 30.0f;
constexpr float kEmergencyDistanceCm = 9.0f;
constexpr float kVisionUncertaintyPenalty = 40.0f;
constexpr float kMaxCleanDistanceCm = 180.0f;
constexpr int kShadowScanSpeed = 12;  // Smooth but faster closed-loop body turns.
constexpr uint32_t kHeadingSettleMs = 100;
constexpr float kEstimatedSegmentCm = 7.0f;
constexpr float kFastCommitDistanceCm = 55.0f;
constexpr float kDiagonalAngleDeg = 35.0f;
constexpr float kBypassExtraCm = 10.0f;
constexpr float kMinDiagonalCm = 30.0f;
constexpr float kMaxDiagonalCm = 70.0f;
constexpr int kMaxApproachSegments = 8;
constexpr int kMinLateralSegments = 6;  // 42 cm: body width + 7 cm margin.
constexpr int kMaxLateralSegments = 10; // 70 cm, adapt to the observed box.
// Field tuning with the current box found the return corridor only after 12
// segments (about 84 cm). Do not perform earlier inward probes near its edge.
constexpr int kMinPassSegments = 12;
constexpr int kMaxPassSegments = 16;    // 112 cm for a longer box.
constexpr float kReturnClearanceMarginCm = 12.0f;
constexpr uint32_t kSemanticVisionIntervalMs = 6000;
// NVS flash operations temporarily disable the flash cache. They must not be
// started while the internal/DMA heaps are nearly exhausted, otherwise a
// concurrent audio/camera allocation can turn a recoverable write failure into
// a panic. The mission task itself uses a PSRAM stack, so persistence is
// deferred to the internal-stack Application task below.
constexpr size_t kHomePersistMinInternalFree = 16 * 1024;
constexpr size_t kHomePersistMinInternalLargest = 8 * 1024;
constexpr size_t kHomePersistMinDmaFree = 12 * 1024;
constexpr size_t kHomePersistMinDmaLargest = 8 * 1024;
// Return Home performs UART transactions, sensor checks and bounded replans,
// but its large temporary buffers are heap-backed. Keep the task stack in
// PSRAM so a normal Xiaozhi/audio/camera runtime cannot exhaust internal SRAM.
constexpr uint32_t kReturnHomeTaskStackBytes = 32768;
constexpr size_t kMaxPersistedBreadcrumbs = 128;
constexpr const char* kHomeNvsNamespace = "mission_home";
constexpr const char* kHomeNvsKey = "route";
constexpr uint32_t kHomeStorageMagic = 0x4D485231U;  // "MHR1"
constexpr uint16_t kHomeStorageVersion = 1;

struct PersistentHomeHeader {
    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t breadcrumb_count = 0;
    float home_heading_deg = 0.0f;
    uint32_t crc32 = 0;
};

struct PersistentBreadcrumb {
    float x_cm = 0.0f;
    float y_cm = 0.0f;
    float heading_deg = 0.0f;
};

static_assert(sizeof(PersistentHomeHeader) == 16,
              "persistent HOME header layout changed");
static_assert(sizeof(PersistentBreadcrumb) == 12,
              "persistent breadcrumb layout changed");

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

class LockGuard {
public:
    explicit LockGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
        locked_ = mutex_ != nullptr &&
                  xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) == pdTRUE;
    }
    ~LockGuard() {
        if (locked_) xSemaphoreGive(mutex_);
    }
    bool locked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_ = nullptr;
    bool locked_ = false;
};
}  // namespace

MissionManager::MissionManager(RobotUart* robot_uart)
    : robot_uart_(robot_uart), mutex_(xSemaphoreCreateMutex()) {
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            ResetMapLocked();
            planner_self_test_ok_ = PlannerSelfTest();
        }
    }
    LoadPersistentHome();
    ESP_LOGI(kShadowTag, "Planner self-test: %s; shadow-only=%d",
             planner_self_test_ok_ ? "PASS" : "FAIL", ShadowModeEnabled());
}

MissionManager::~MissionManager() {
    CancelMission();
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

void MissionManager::SetCamera(Camera* camera) {
    LockGuard lock(mutex_);
    if (lock.locked()) camera_ = camera;
}

float MissionManager::NormalizeHeading(float heading_deg) {
    while (heading_deg > 180.0f) heading_deg -= 360.0f;
    while (heading_deg <= -180.0f) heading_deg += 360.0f;
    return heading_deg;
}

bool MissionManager::ShadowModeEnabled() {
    return CONFIG_OBSTACLE_SHADOW_MODE != 0;
}

const char* MissionManager::TypeName(MissionType type) {
    switch (type) {
        case MissionType::NONE: return "none";
        case MissionType::AUTONOMOUS_FORWARD: return "autonomous_forward";
        case MissionType::AVOID_SCAN: return "avoid_scan";
        case MissionType::BYPASS_ONCE: return "bypass_once";
        case MissionType::RETURN_HOME: return "return_home";
    }
    return "unknown";
}

const char* MissionManager::StateName(MissionState state) {
    switch (state) {
        case MissionState::IDLE: return "idle";
        case MissionState::STARTING: return "starting";
        case MissionState::NAVIGATING: return "navigating";
        case MissionState::OBSTACLE_STOP: return "obstacle_stop";
        case MissionState::SCAN_LEFT: return "scan_left";
        case MissionState::SCAN_RIGHT: return "scan_right";
        case MissionState::CHOOSE_PATH: return "choose_path";
        case MissionState::WAIT_NO_PATH: return "wait_no_path";
        case MissionState::DETOUR_OUT: return "detour_out";
        case MissionState::DETOUR_PASS: return "detour_pass";
        case MissionState::DETOUR_RETURN: return "detour_return";
        case MissionState::RESUMING: return "resuming";
        case MissionState::RETURN_STARTING: return "return_starting";
        case MissionState::RETURN_TURNING: return "return_turning";
        case MissionState::RETURN_MOVING: return "return_moving";
        case MissionState::RETURN_BLOCKED: return "return_blocked";
        case MissionState::RETURN_REPLAN: return "return_replan";
        case MissionState::RETURN_FINAL_ALIGN: return "return_final_align";
        case MissionState::RETURN_COMPLETED: return "return_completed";
        case MissionState::COMPLETED: return "completed";
        case MissionState::FAILED: return "failed";
        case MissionState::CANCELLED: return "cancelled";
    }
    return "unknown";
}

bool MissionManager::IsBlockedZone(const char* zone) {
    return zone == nullptr || strcmp(zone, "BLOCKED") == 0 ||
           strcmp(zone, "EMERGENCY") == 0 || strcmp(zone, "STALE") == 0;
}

bool MissionManager::CameraPathBlocked(const DirectionSample& sample) {
    if (!sample.vision.valid) return false;
    
    // Multi-level collision detection:
    // Level 1: Imminent collision from high edge density or high contrast near camera
    if (sample.vision.collision_imminent && sample.vision.confidence_pct >= 40.0f) {
        ESP_LOGW(kTag, "COLLISION_IMMINENT ttc=%.0f ms flow=%.0f",
                 sample.vision.time_to_collision_ms, 
                 sample.vision.motion_flow_magnitude);
        return true;
    }
    
    // Level 2: Blocked center or high edge density (obstacle present)
    if (sample.vision.confidence_pct >= 45.0f) {
        const bool blocked_center = sample.vision.center_open_pct < 35.0f;
        const bool high_edge_density = sample.vision.edge_density_pct > 60.0f;
        const bool dark_block = sample.vision.dark_obstacle && 
                                sample.vision.confidence_pct >= 55.0f;
        const bool sharp_edge = sample.vision.high_contrast && 
                                sample.vision.center_open_pct < 50.0f;
        
        if (blocked_center || high_edge_density || dark_block || sharp_edge) {
            ESP_LOGI(kTag, "CAMERA_BLOCKED center=%.0f%% edge=%.0f%% dark=%d sharp=%d",
                     sample.vision.center_open_pct, sample.vision.edge_density_pct,
                     sample.vision.dark_obstacle, sample.vision.high_contrast);
            return true;
        }
    }
    
    return false;
}

bool MissionManager::SemanticPathBlocked(
    const VisionObstacleResult& vision) {
    if (!vision.valid || vision.confidence < 0.55f) return false;
    const bool vulnerable = strcmp(vision.object_type, "person") == 0 ||
           strcmp(vision.object_type, "animal") == 0 ||
           strcmp(vision.object_type, "vehicle") == 0;
    const bool known_object = strcmp(vision.object_type, "unknown") != 0;
    // Never steer solely from an "unknown/high" model response. It remains
    // recorded, but must be corroborated by local camera/SR04. Recognized
    // people/animals/vehicles retain an immediate semantic veto.
    return vulnerable ||
           (known_object && strcmp(vision.danger_level, "high") == 0) ||
           (known_object && vision.risk_score >= 0.70f);
}

void MissionManager::ResetMapLocked() {
    memset(occupancy_, 0, sizeof(occupancy_));
    context_.pose = {};
}

bool MissionManager::StartAutonomousForward(int speed,
                                             uint32_t runtime_seconds,
                                             bool camera_guidance) {
    if (robot_uart_ == nullptr || mutex_ == nullptr) return false;
    if (ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=AUTONOMOUS_FORWARD,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(ai_obstacle_hold_event_id_.load()));
        return false;
    }
    if (ShadowModeEnabled()) {
        ESP_LOGW(kShadowTag,
                 "Live autonomous detour rejected: shadow-only mode enabled");
        return false;
    }
    speed = std::clamp(speed, 10, 20);
    runtime_seconds = std::clamp<uint32_t>(runtime_seconds, 1, 180);
    // Production forward motion always uses the camera. Keep the argument for
    // API compatibility, but callers cannot disable road monitoring.
    (void)camera_guidance;
    camera_guidance = true;

    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return false;
        if (active_) {
            if (context_.type != MissionType::AUTONOMOUS_FORWARD) return false;
            const uint32_t elapsed = NowMs() - context_.started_ms;
            const uint32_t requested_end = elapsed + runtime_seconds * 1000U;
            context_.runtime_limit_ms =
                std::max(context_.runtime_limit_ms, requested_end);
            context_.speed = speed;
            context_.camera_guidance = camera_guidance;
            context_.last_update_ms = NowMs();
            ESP_LOGI(kTag,
                     "Mission %lu remembered/extended: speed=%d limit_ms=%lu",
                     static_cast<unsigned long>(context_.mission_id), speed,
                     static_cast<unsigned long>(context_.runtime_limit_ms));
            return true;
        }
        context_ = {};
        context_.mission_id = next_mission_id_++;
        context_.type = MissionType::AUTONOMOUS_FORWARD;
        context_.state = MissionState::STARTING;
        context_.speed = speed;
        context_.camera_guidance = camera_guidance;
        context_.shadow_mode = false;
        context_.started_ms = NowMs();
        context_.last_update_ms = context_.started_ms;
        context_.runtime_limit_ms = runtime_seconds * 1000U;
        ResetMapLocked();
        cancel_requested_.store(false);
        active_ = true;
    }

    if (xTaskCreatePinnedToCoreWithCaps(
            TaskEntry, "robot_navigation", 10240, this, 3, &task_, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        SetFailure("cannot_start_navigation_task");
        Finish(MissionState::FAILED);
        return false;
    }
    return true;
}

bool MissionManager::SetHome() {
    if (robot_uart_ == nullptr || mutex_ == nullptr || IsActive()) return false;
    if (!InitializeOdometryReference(true)) return false;
    const bool persisted = PersistHomeRoute();
    persistent_home_ok_ = persisted;
    if (!persisted) ESP_LOGE(kTag, "HOME_PERSIST_FAILED");
    return persisted;
}

bool MissionManager::StartReturnHome(int speed, bool camera_guidance) {
    if (robot_uart_ == nullptr || mutex_ == nullptr) return false;
    if (ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=RETURN_HOME,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(ai_obstacle_hold_event_id_.load()));
        return false;
    }
    speed = std::clamp(speed, 10, 20);
    bool needs_reference_restore = false;
    bool home_available = false;
    bool mission_active = false;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            mission_active = active_;
            home_available = home_valid_ && !breadcrumbs_.empty();
            // Keep the live odometry reference after an operator move. Rebase
            // only when invalidated by an ESP32/STM32 reboot.
            needs_reference_restore = home_available && !odom_reference_valid_;
        }
    }
    if (mission_active) {
        SetFailure("mission_already_active");
        return false;
    }
    if (!home_available) {
        SetFailure("home_not_available");
        return false;
    }
    const bool odom_ready = needs_reference_restore
                                ? RestoreOdometryReference()
                                : SyncPoseFromOdometry(false);
    if (!odom_ready) {
        SetFailure("return_home_odometry_restore_failed");
        return false;
    }
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || active_ || !home_valid_ ||
            breadcrumbs_.empty()) {
            return false;
        }
        context_ = {};
        context_.mission_id = next_mission_id_++;
        context_.type = MissionType::RETURN_HOME;
        context_.state = MissionState::RETURN_STARTING;
        context_.speed = speed;
        context_.camera_guidance = camera_guidance;
        context_.shadow_mode = false;
        context_.started_ms = NowMs();
        context_.last_update_ms = context_.started_ms;
        context_.route_heading_deg = home_heading_deg_;
        memset(occupancy_, 0, sizeof(occupancy_));
        cancel_requested_.store(false);
        active_ = true;
    }
    if (xTaskCreatePinnedToCoreWithCaps(
            ReturnHomeTaskEntry, "robot_return_home",
            kReturnHomeTaskStackBytes, this, 3, &task_, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        SetFailure("cannot_start_return_home_task");
        Finish(MissionState::FAILED);
        return false;
    }
    return true;
}

bool MissionManager::CancelMission() {
    cancel_requested_.store(true);
    const bool was_active = IsActive();
    if (was_active && robot_uart_ != nullptr) robot_uart_->Stop(700);
    if (ai_obstacle_hold_.exchange(false)) {
        const uint32_t event_id = ai_obstacle_hold_event_id_.exchange(0);
        ESP_LOGI("AI_OBS", "HOLD_CLEAR=EXPLICIT_CANCEL");
        ESP_LOGI("AI_OBS", "HOLD_EVENT_ID_CLEAR=%lu", static_cast<unsigned long>(event_id));
        ESP_LOGI("AI_OBS", "HOLD=0");
    }
    return true;
}

void MissionManager::HoldForAiObstacle() {
    const uint32_t event_id = ai_obstacle_hold_event_id_.fetch_add(1) + 1;
    ai_obstacle_hold_.store(true);
    cancel_requested_.store(true);
    ESP_LOGI("AI_OBS", "MISSION_HOLD=1,HOLD=1,HOLD_EVENT_ID=%lu,SUPPRESS_DIAGONAL_BYPASS=1",
             static_cast<unsigned long>(event_id));
}

bool MissionManager::StartShadowScan(float scan_angle_deg) {
    if (robot_uart_ == nullptr || mutex_ == nullptr ||
        !planner_self_test_ok_) return false;
    if (ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=SHADOW_SCAN,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(ai_obstacle_hold_event_id_.load()));
        return false;
    }
    scan_angle_deg = std::clamp(scan_angle_deg, 10.0f, 90.0f);
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || active_) return false;
        context_ = {};
        context_.mission_id = next_mission_id_++;
        context_.type = MissionType::AVOID_SCAN;
        context_.state = MissionState::STARTING;
        context_.speed = kShadowScanSpeed;
        context_.camera_guidance = true;
        context_.shadow_mode = true;
        context_.scan_angle_deg = scan_angle_deg;
        context_.started_ms = NowMs();
        context_.last_update_ms = context_.started_ms;
        ResetMapLocked();
        cancel_requested_.store(false);
        active_ = true;
    }
    if (xTaskCreatePinnedToCoreWithCaps(
            TaskEntry, "robot_shadow_scan", 8192, this, 3, &task_, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        SetFailure("cannot_start_shadow_scan_task");
        Finish(MissionState::FAILED);
        return false;
    }
    return true;
}

bool MissionManager::StartBypassOnce() {
    if (robot_uart_ == nullptr || mutex_ == nullptr ||
        !planner_self_test_ok_) return false;
    if (ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=BYPASS_ONCE,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(ai_obstacle_hold_event_id_.load()));
        return false;
    }
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || active_) return false;
        context_ = {};
        context_.mission_id = next_mission_id_++;
        context_.type = MissionType::BYPASS_ONCE;
        context_.state = MissionState::STARTING;
        context_.speed = 10;
        context_.camera_guidance = true;
        context_.shadow_mode = false;
        context_.scan_angle_deg = 35.0f;
        context_.started_ms = NowMs();
        context_.last_update_ms = context_.started_ms;
        ResetMapLocked();
        cancel_requested_.store(false);
        active_ = true;
    }
    if (xTaskCreatePinnedToCoreWithCaps(
            TaskEntry, "robot_bypass_once", 12288, this, 3, &task_, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        SetFailure("cannot_start_bypass_task");
        Finish(MissionState::FAILED);
        return false;
    }
    return true;
}

// Shadow-mode body turn: switch STM32 to AI mode, use closed-loop fused heading
// (encoder + gyro when the optional compass is unavailable), then wait for
// heading settle. Never timed turn.
bool MissionManager::TurnToShadow(float heading_deg, int speed) {
    if (CancelRequested()) return false;
    RobotTurnResult result;
    const int target = static_cast<int>(std::lround(NormalizeHeading(heading_deg)));
    const bool mode_ok = robot_uart_->SetMode(true, 700);
    const bool turn_ok = mode_ok &&
        robot_uart_->TurnAbsolute(target, speed, result, 13000);
    if (!turn_ok) {
        robot_uart_->Stop(700);
        // STM32 intentionally has a tighter completion tolerance than the
        // Shadow observer. If it times out already stopped very near target,
        // accept only after active state and fused-heading verification below.
        RobotState state;
        float timeout_heading = 0.0f;
        const bool near_target = mode_ok &&
            robot_uart_->GetState(state, 700) && !state.moving &&
            robot_uart_->GetHeading(timeout_heading, 700) &&
            std::fabs(NormalizeHeading(timeout_heading - heading_deg)) <= 8.0f;
        if (!near_target) return false;
        result.heading_deg = timeout_heading;
        ESP_LOGW(kShadowTag,
                 "Accepted stopped near-target turn after timeout: target=%.1f actual=%.1f",
                 heading_deg, timeout_heading);
    }
    UpdateHeading(result.heading_deg);
    vTaskDelay(pdMS_TO_TICKS(kHeadingSettleMs));
    // Verify we actually reached the target after settling.
    float h = 0.0f;
    if (!robot_uart_->GetHeading(h, 700)) {
        robot_uart_->Stop(700);
        return false;
    }
    const float err = std::fabs(NormalizeHeading(NormalizeHeading(h) -
                                                 NormalizeHeading(heading_deg)));
    if (err > 8.0f) {
        ESP_LOGW(kShadowTag, "Heading settle error too large: %.2f deg", err);
        robot_uart_->Stop(700);
        return false;
    }
    return !CancelRequested();
}

// Shadow-mode environment sample. Uses the same RobotUart::GetObstacle data the
// live mission uses; never invents a distance. Fresh timeout => 400 cm.
bool MissionManager::SampleShadow(bool use_camera, DirectionScan& scan) {
    (void)use_camera;
    scan = {};
    // STM32 already applies a five-sample median and asymmetric low-pass
    // filter at 60 ms. One fresh read avoids duplicating that filter here and
    // removes roughly 140 ms plus two UART round trips from every decision.
    std::vector<float> distances;
    RobotObstacleStatus latest;
    RobotObstacleStatus reading;
    if (robot_uart_->GetObstacle(reading, 500) && reading.fresh) {
        latest = reading;
        scan.sensor_fresh = true;
        if (reading.echo_valid) distances.push_back(reading.distance_cm);
    }
    if (!scan.sensor_fresh) return false;

    scan.echo_valid = !distances.empty();
    if (scan.echo_valid) {
        std::sort(distances.begin(), distances.end());
        scan.distance_cm = distances[distances.size() / 2];
    } else {
        // A fresh timeout means no object within the HC-SR04 working range.
        scan.distance_cm = 400.0f;
    }
    snprintf(scan.zone, sizeof(scan.zone), "%s",
             latest.fresh ? latest.zone : "STALE");
    scan.approach_rate_cm_s = latest.approach_rate_cm_s;
    scan.blocked = IsBlockedZone(scan.zone) || latest.limited ||
                   (scan.echo_valid && scan.distance_cm < kEmergencyDistanceCm);
    if (!robot_uart_->GetHeading(scan.heading_deg, 700)) return false;
    scan.heading_deg = NormalizeHeading(scan.heading_deg);
    UpdateHeading(scan.heading_deg);
    // Shadow Mode does not translate the chassis, but the three heading-tagged
    // rays are still valid local occupancy observations around the current pose.
    ObserveRay(scan.heading_deg, scan.distance_cm, scan.echo_valid);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) context_.last_obstacle_cm = scan.distance_cm;
    }
    scan.valid = true;
    return true;
}

bool MissionManager::ScanDirectionShadow(float heading_deg, MissionState state,
                                         DirectionScan& scan) {
    SetState(state);
    return TurnToShadow(heading_deg, kShadowScanSpeed) &&
           SampleShadow(true, scan);
}

void MissionManager::ParseVisionJson(const char* json,
                                     VisionObstacleResult& vision) {
    vision = {};
    if (json == nullptr || *json == '\0') return;
    cJSON* root = cJSON_Parse(json);
    if (root == nullptr) {
        ESP_LOGW(kShadowTag, "Vision JSON parse failed");
        return;
    }
    cJSON* nested = nullptr;
    cJSON* payload = root;
    const cJSON* wrapped_text = cJSON_GetObjectItem(root, "text");
    if (cJSON_IsString(wrapped_text)) {
        nested = cJSON_Parse(wrapped_text->valuestring);
        if (cJSON_IsObject(nested)) payload = nested;
    }
    auto read_string = [payload](const char* key, char* out, size_t out_size) {
        const cJSON* item = cJSON_GetObjectItem(payload, key);
        if (cJSON_IsString(item)) {
            snprintf(out, out_size, "%s", item->valuestring);
            return true;
        }
        return false;
    };
    auto read_number = [payload](const char* key, float* out) {
        const cJSON* item = cJSON_GetObjectItem(payload, key);
        if (cJSON_IsNumber(item)) {
            *out = static_cast<float>(item->valuedouble);
            return true;
        }
        return false;
    };
    const bool has_object =
        read_string("object", vision.object_type, sizeof(vision.object_type));
    const bool has_danger =
        read_string("danger", vision.danger_level, sizeof(vision.danger_level));
    const bool has_clear = read_number("clear_score", &vision.clear_score);
    const bool has_risk = read_number("risk_score", &vision.risk_score);
    const bool has_confidence =
        read_number("confidence", &vision.confidence);

    auto allow_token = [](char* value, size_t size,
                          const char* const* allowed, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            if (strcmp(value, allowed[i]) == 0) return;
        }
        snprintf(value, size, "%s", "unknown");
    };
    static const char* const kObjects[] = {
        "person", "animal", "wall", "chair", "box", "vehicle", "unknown"};
    static const char* const kDanger[] = {
        "low", "medium", "high", "unknown"};
    allow_token(vision.object_type, sizeof(vision.object_type), kObjects,
                sizeof(kObjects) / sizeof(kObjects[0]));
    allow_token(vision.danger_level, sizeof(vision.danger_level), kDanger,
                sizeof(kDanger) / sizeof(kDanger[0]));
    vision.clear_score = std::clamp(vision.clear_score, 0.0f, 1.0f);
    vision.risk_score = std::clamp(vision.risk_score, 0.0f, 1.0f);
    vision.confidence = std::clamp(vision.confidence, 0.0f, 1.0f);
    // A partial/model-mangled object must not receive a favorable score.
    vision.valid = cJSON_IsObject(payload) && has_object && has_danger &&
                   has_clear && has_risk && has_confidence;
    if (vision.valid) {
        snprintf(vision.source, sizeof(vision.source), "%s", "semantic");
    }
    if (nested != nullptr) cJSON_Delete(nested);
    cJSON_Delete(root);
}

void MissionManager::CaptureVision(const char* direction,
                                   VisionObstacleResult& vision) {
    vision = {};
    Camera* camera = nullptr;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) camera = camera_;
    }
    if (camera == nullptr) return;  // No camera => stay invalid (no guessing).

    auto use_local_fallback = [camera, &vision, direction]() {
        CameraNavigationMetrics metrics;
        if (!camera->AnalyzeNavigation(metrics) || !metrics.valid) return;
        const float clear = std::clamp(metrics.center_open_pct / 100.0f,
                                       0.0f, 1.0f);
        vision = {};
        vision.valid = true;
        vision.clear_score = clear;
        vision.risk_score = 1.0f - clear;
        // Local RGB565 texture is useful but deliberately less authoritative
        // than a structured semantic result or HC-SR04.
        vision.confidence = std::clamp(
            metrics.confidence_pct / 100.0f * 0.55f, 0.10f, 0.55f);
        snprintf(vision.source, sizeof(vision.source), "%s", "local");
        ESP_LOGI(kShadowTag,
                 "VISION %s local fallback clear=%.2f risk=%.2f conf=%.2f",
                 direction, vision.clear_score, vision.risk_score,
                 vision.confidence);
    };
    try {
        // The vision service prompt asks for a compact JSON object. The service
        // response may contain extra prose; we extract the first {...} block.
        if (!camera->Capture()) {
            ESP_LOGW(kShadowTag, "Vision capture failed (%s)", direction);
            use_local_fallback();
            return;
        }
        const std::string raw = camera->Explain(
            "Analyze this image for robot obstacle avoidance. Return ONLY this "
            "JSON (no markdown): {\"object\":\"person|animal|wall|chair|box|"
            "vehicle|unknown\",\"danger\":\"low|medium|high|unknown\","
            "\"clear_score\":0.0..1.0,\"risk_score\":0.0..1.0,"
            "\"confidence\":0.0..1.0}");
        // Locate the first JSON object in the response.
        size_t start = raw.find('{');
        size_t end = raw.rfind('}');
        if (start == std::string::npos || end == std::string::npos ||
            end <= start) {
            ESP_LOGW(kShadowTag, "Vision no JSON block (%s)", direction);
            use_local_fallback();
            return;
        }
        const std::string json = raw.substr(start, end - start + 1);
        ParseVisionJson(json.c_str(), vision);
        if (!vision.valid) use_local_fallback();
        ESP_LOGI(kShadowTag,
                 "VISION %s valid=%d source=%s object=%s danger=%s clear=%.2f risk=%.2f conf=%.2f",
                 direction, vision.valid, vision.source, vision.object_type,
                 vision.danger_level, vision.clear_score, vision.risk_score,
                 vision.confidence);
    } catch (const std::exception& e) {
        ESP_LOGW(kShadowTag, "Vision exception (%s): %s", direction, e.what());
        use_local_fallback();
    }
}

void MissionManager::CaptureVisionLocal(const char* direction,
                                        VisionObstacleResult& vision) {
    vision = {};
    Camera* camera = nullptr;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) camera = camera_;
    }
    if (camera == nullptr) return;

    CameraNavigationMetrics metrics;
    if (!camera->AnalyzeNavigation(metrics) || !metrics.valid) {
        ESP_LOGW(kShadowTag, "VISION %s local analysis failed", direction);
        return;
    }
    const float clear = std::clamp(metrics.center_open_pct / 100.0f,
                                   0.0f, 1.0f);
    vision.valid = true;
    vision.clear_score = clear;
    vision.risk_score = 1.0f - clear;
    vision.confidence = std::clamp(metrics.confidence_pct / 100.0f * 0.55f,
                                   0.10f, 0.55f);
    snprintf(vision.object_type, sizeof(vision.object_type), "%s", "unknown");
    snprintf(vision.danger_level, sizeof(vision.danger_level), "%s", "unknown");
    snprintf(vision.source, sizeof(vision.source), "%s", "local");
    ESP_LOGI(kShadowTag,
             "VISION %s local clear=%.2f risk=%.2f conf=%.2f L=%.0f C=%.0f R=%.0f",
             direction, vision.clear_score, vision.risk_score,
             vision.confidence, metrics.left_open_pct,
             metrics.center_open_pct, metrics.right_open_pct);
}

void MissionManager::SemanticTaskEntry(void* context) {
    static_cast<MissionManager*>(context)->RunSemanticMonitor();
    vTaskDelete(nullptr);
}

bool MissionManager::StartSemanticMonitor() {
    bool expected = false;
    if (!semantic_task_running_.compare_exchange_strong(expected, true)) {
        return false;
    }
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.last_ai_vision_ms = NowMs();
            ++context_.ai_vision_attempt_count;
        }
    }
    if (xTaskCreatePinnedToCore(SemanticTaskEntry, "road_ai_upload", 4608,
                                this, 1, nullptr, 0) != pdPASS) {
        semantic_task_running_.store(false);
        return false;
    }
    return true;
}

void MissionManager::RunSemanticMonitor() {
    VisionObstacleResult semantic;
    CaptureVision("forward", semantic);
    const bool blocked = SemanticPathBlocked(semantic);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            ++context_.ai_vision_count;
            if (strcmp(semantic.source, "semantic") == 0) {
                ++context_.ai_vision_upload_count;
            }
            context_.last_ai_vision_ms = NowMs();
            context_.last_ai_vision = semantic;
        }
    }
    ESP_LOGI(kTag,
             "AI_ROAD_MONITOR source=%s blocked=%d object=%s danger=%s risk=%.2f stack_free=%u",
             semantic.source, blocked, semantic.object_type,
             semantic.danger_level, semantic.risk_score,
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    semantic_task_running_.store(false);
}

float MissionManager::CostDistance(const DirectionScan& scan) {
    if (!scan.valid || !scan.sensor_fresh) return kInvalidDirectionCost;
    if (scan.blocked) return kBlockedDirectionCost;
    // CLEAR beyond max working range => minimal cost.
    if (!scan.echo_valid) return 0.0f;
    if (scan.distance_cm >= kMaxCleanDistanceCm) return 0.0f;
    if (scan.distance_cm <= kEmergencyDistanceCm) return kBlockedDirectionCost;
    // Linear cost from high (close) to 0 (>= kMaxCleanDistanceCm).
    const float scaled = (kMaxCleanDistanceCm - scan.distance_cm) /
                         kMaxCleanDistanceCm;
    return std::max(0.0f, scaled * 100.0f);
}

float MissionManager::CostVision(const DirectionScan& scan) {
    if (!scan.valid) return kVisionUncertaintyPenalty;
    if (!scan.vision.valid) return kVisionUncertaintyPenalty;
    // Lower cost for high clear_score / low risk_score. Blend toward the
    // uncertainty penalty as confidence falls; confidence=0 must never make
    // an incomplete vision result look safer than having no vision at all.
    const float risk = std::clamp(scan.vision.risk_score, 0.0f, 1.0f);
    const float clear = std::clamp(scan.vision.clear_score, 0.0f, 1.0f);
    const float conf = std::clamp(scan.vision.confidence, 0.0f, 1.0f);
    float risk_cost = risk * 100.0f;
    if (strcmp(scan.vision.danger_level, "high") == 0) {
        risk_cost = std::max(risk_cost, 95.0f);
    }
    if (strcmp(scan.vision.object_type, "person") == 0 ||
        strcmp(scan.vision.object_type, "animal") == 0 ||
        strcmp(scan.vision.object_type, "vehicle") == 0) {
        risk_cost = std::max(risk_cost, 85.0f);
    }
    const float clear_cost = (1.0f - clear) * 100.0f;
    const float learned_cost = risk_cost * 0.6f + clear_cost * 0.4f;
    return conf * learned_cost +
           (1.0f - conf) * kVisionUncertaintyPenalty;
}

float MissionManager::CostHeading(float scan_angle_deg) {
    // Scan happens to the left (+) and right (-) by the same angle, so the
    // heading preference is symmetric; keep it intentional and small.
    return 0.0f;
}

float MissionManager::CostTurn() {
    // No servo: each candidate requires one body turn of the same angle.
    return 1.0f;
}

float MissionManager::DirectionCost(const DirectionScan& scan,
                                    float scan_angle_deg, bool is_left) {
    (void)is_left;
    return kWObstacle * CostDistance(scan) +
           kWVision * CostVision(scan) +
           kWHeading * CostHeading(scan_angle_deg) +
           kWTurn * CostTurn();
}

bool MissionManager::IsTraversable(const DirectionScan& scan) {
    if (!scan.valid || !scan.sensor_fresh || scan.blocked ||
        (scan.echo_valid && scan.distance_cm < kOpenDirectionCm)) {
        return false;
    }
    // A semantic danger veto is stronger than an otherwise-open ultrasonic
    // ray.  The camera may see a person/animal/vehicle that the narrow
    // HC-SR04 cone misses.
    if (scan.vision.valid && scan.vision.confidence >= 0.55f &&
        (strcmp(scan.vision.danger_level, "high") == 0 ||
         strcmp(scan.vision.object_type, "person") == 0 ||
         strcmp(scan.vision.object_type, "animal") == 0 ||
         strcmp(scan.vision.object_type, "vehicle") == 0)) {
        return false;
    }
    return true;
}

PathPlanResult MissionManager::ComputePlan(const DirectionScan& left,
                                           const DirectionScan& right,
                                           float scan_angle_deg) {
    PathPlanResult plan;
    plan.valid = true;
    plan.left_cost = DirectionCost(left, scan_angle_deg, true);
    plan.right_cost = DirectionCost(right, scan_angle_deg, false);
    const bool left_ok = IsTraversable(left);
    const bool right_ok = IsTraversable(right);
    constexpr float kChoiceThreshold = 8.0f;

    if (left_ok && right_ok) {
        if (plan.left_cost + kChoiceThreshold < plan.right_cost) {
            plan.choice = PathChoice::LEFT;
            plan.confidence = 0.5f + 0.5f *
                (plan.right_cost - plan.left_cost) /
                std::max(plan.right_cost, 1.0f);
        } else if (plan.right_cost + kChoiceThreshold < plan.left_cost) {
            plan.choice = PathChoice::RIGHT;
            plan.confidence = 0.5f + 0.5f *
                (plan.left_cost - plan.right_cost) /
                std::max(plan.left_cost, 1.0f);
        } else {
            // Near-tie: both rays are physically traversable, so use the
            // camera risk as the first tie-breaker, then ultrasonic clearance.
            // This avoids freezing in front of a simple box while preserving
            // the semantic danger veto in IsTraversable().
            const float left_risk = left.vision.valid
                ? left.vision.risk_score : 0.5f;
            const float right_risk = right.vision.valid
                ? right.vision.risk_score : 0.5f;
            if (std::fabs(left_risk - right_risk) >= 0.15f) {
                plan.choice = left_risk < right_risk
                    ? PathChoice::LEFT : PathChoice::RIGHT;
                plan.confidence = 0.55f + 0.25f *
                    std::min(std::fabs(left_risk - right_risk), 1.0f);
            } else {
                const float left_clearance = left.echo_valid
                    ? left.distance_cm : kMaxCleanDistanceCm;
                const float right_clearance = right.echo_valid
                    ? right.distance_cm : kMaxCleanDistanceCm;
                plan.choice = left_clearance >= right_clearance
                    ? PathChoice::LEFT : PathChoice::RIGHT;
                plan.confidence = 0.55f;
            }
        }
        snprintf(plan.reason, sizeof(plan.reason),
                 "left=%.1f right=%.1f (both traversable)",
                 plan.left_cost, plan.right_cost);
    } else if (left_ok) {
        plan.choice = PathChoice::LEFT;
        plan.confidence = 0.75f;
        snprintf(plan.reason, sizeof(plan.reason),
                 "left traversable; right blocked/invalid %.1f/%.1f",
                 plan.left_cost, plan.right_cost);
    } else if (right_ok) {
        plan.choice = PathChoice::RIGHT;
        plan.confidence = 0.75f;
        snprintf(plan.reason, sizeof(plan.reason),
                 "right traversable; left blocked/invalid %.1f/%.1f",
                 plan.left_cost, plan.right_cost);
    } else {
        plan.choice = PathChoice::NONE;
        plan.confidence = 0.0f;
        snprintf(plan.reason, sizeof(plan.reason),
                 "no traversable direction %.1f/%.1f",
                 plan.left_cost, plan.right_cost);
    }
    plan.confidence = std::clamp(plan.confidence, 0.0f, 1.0f);
    return plan;
}

bool MissionManager::PlannerSelfTest() {
    DirectionScan clear_left;
    clear_left.valid = true;
    clear_left.sensor_fresh = true;
    clear_left.echo_valid = true;
    clear_left.distance_cm = 150.0f;
    snprintf(clear_left.zone, sizeof(clear_left.zone), "%s", "CLEAR");

    DirectionScan clear_right = clear_left;
    clear_right.distance_cm = 45.0f;
    DirectionScan blocked = clear_left;
    blocked.blocked = true;
    blocked.distance_cm = 15.0f;
    snprintf(blocked.zone, sizeof(blocked.zone), "%s", "BLOCKED");
    DirectionScan invalid;

    const PathPlanResult prefer_left =
        ComputePlan(clear_left, clear_right, 35.0f);
    const PathPlanResult only_right =
        ComputePlan(blocked, clear_right, 35.0f);
    const PathPlanResult no_path = ComputePlan(blocked, invalid, 35.0f);
    return prefer_left.choice == PathChoice::LEFT &&
           only_right.choice == PathChoice::RIGHT &&
           no_path.choice == PathChoice::NONE &&
           !IsTraversable(blocked);
}

void MissionManager::RunPlanner() {
    DirectionScan left;
    DirectionScan right;
    float scan_angle = 35.0f;
    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return;
        left = context_.scan_left;
        right = context_.scan_right;
        scan_angle = context_.scan_angle_deg;
    }
    const PathPlanResult plan = ComputePlan(left, right, scan_angle);

    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.plan = plan;
            snprintf(context_.chosen_side, sizeof(context_.chosen_side),
                     "%s", plan.choice == PathChoice::LEFT ? "left" :
                           (plan.choice == PathChoice::RIGHT ? "right" : "none"));
        }
    }
    const char* choice_str = plan.choice == PathChoice::LEFT
                                 ? "LEFT" : (plan.choice == PathChoice::RIGHT
                                                 ? "RIGHT" : "NONE");
    ESP_LOGI(kShadowTag,
             "PLAN,LEFT_DIST=%.1f,RIGHT_DIST=%.1f,LEFT_VISION=%.2f,RIGHT_VISION=%.2f,LEFT_RISK=%.2f,RIGHT_RISK=%.2f,LEFT_COST=%.1f,RIGHT_COST=%.1f,CHOICE=%s,CONF=%.2f",
             left.distance_cm, right.distance_cm,
             left.vision.valid ? left.vision.clear_score : -1.0f,
             right.vision.valid ? right.vision.clear_score : -1.0f,
             left.vision.valid ? left.vision.risk_score : -1.0f,
             right.vision.valid ? right.vision.risk_score : -1.0f,
             plan.left_cost, plan.right_cost, choice_str, plan.confidence);
}

void MissionManager::RunShadowScan() {
    ESP_LOGI(kShadowTag, "Shadow scan task started");
    RobotState state;
    float original_heading = 0.0f;
    float scan_angle = 35.0f;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) scan_angle = context_.scan_angle_deg;
    }
    // GetState is the authoritative active link probe. Do not reject from the
    // short cached IsConnected window after an otherwise healthy quiet UART.
    if (!robot_uart_->GetState(state, 700) || state.moving ||
        !robot_uart_->GetHeading(original_heading, 700)) {
        SetFailure("shadow_preflight_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(MissionState::FAILED);
        return;
    }
    if (!state.compass_ok) {
        ESP_LOGW(kShadowTag,
                 "Compass unavailable; shadow scan uses fused encoder/gyro heading");
    }
    original_heading = NormalizeHeading(original_heading);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.original_heading_deg = original_heading;
            context_.pose.heading_deg = original_heading;
        }
    }
    ESP_LOGI(kShadowTag, "H0=%.1f scan_angle=%.1f",
             original_heading, scan_angle);

    // 1. Center: stop, measure, capture.
    SetState(MissionState::OBSTACLE_STOP);
    robot_uart_->Stop(700);
    DirectionScan center;
    if (!ScanDirectionShadow(original_heading, MissionState::CHOOSE_PATH,
                             center)) {
        SetFailure("center_scan_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(CancelRequested() ? MissionState::CANCELLED
                                 : MissionState::FAILED);
        return;
    }
    CaptureVision("center", center.vision);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) context_.scan_center = center;
    }
    ESP_LOGI(kShadowTag,
             "SHADOW_CENTER DIST=%.1f ZONE=%s ECHO=%d VISION=%d",
             center.distance_cm, center.zone, center.echo_valid,
             center.vision.valid);

    // 2. Left body scan.
    DirectionScan left;
    if (ScanDirectionShadow(original_heading + scan_angle,
                            MissionState::SCAN_LEFT, left)) {
        CaptureVision("left", left.vision);
        ESP_LOGI(kShadowTag,
                 "SHADOW_LEFT DIST=%.1f ZONE=%s ECHO=%d VISION=%d",
                 left.distance_cm, left.zone, left.echo_valid,
                 left.vision.valid);
    }
    {
        LockGuard lock(mutex_);
        if (lock.locked()) context_.scan_left = left;
    }
    // MUST return to H0 before continuing (closed-loop Compass, not timed).
    const bool returned_left = TurnToShadow(original_heading, kShadowScanSpeed);
    if (!returned_left) {
        SetFailure("left_scan_return_h0_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(CancelRequested() ? MissionState::CANCELLED
                                 : MissionState::FAILED);
        return;
    }

    // 3. Right body scan.
    DirectionScan right;
    if (ScanDirectionShadow(original_heading - scan_angle,
                            MissionState::SCAN_RIGHT, right)) {
        CaptureVision("right", right.vision);
        ESP_LOGI(kShadowTag,
                 "SHADOW_RIGHT DIST=%.1f ZONE=%s ECHO=%d VISION=%d",
                 right.distance_cm, right.zone, right.echo_valid,
                 right.vision.valid);
    }
    {
        LockGuard lock(mutex_);
        if (lock.locked()) context_.scan_right = right;
    }
    // MUST return to H0.
    const bool returned_right = TurnToShadow(original_heading, kShadowScanSpeed);
    if (!returned_right) {
        SetFailure("right_scan_return_h0_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(CancelRequested() ? MissionState::CANCELLED
                                 : MissionState::FAILED);
        return;
    }

    // 4. Local path planner.
    SetState(MissionState::CHOOSE_PATH);
    RunPlanner();

    // 5. Shadow result: robot stays stopped at H0. No PASS_OBSTACLE.
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            const char* choice_str =
                context_.plan.choice == PathChoice::LEFT
                    ? "LEFT" : (context_.plan.choice == PathChoice::RIGHT
                                    ? "RIGHT" : "NONE");
            ESP_LOGI(kShadowTag,
                     "SHADOW_RESULT H0=%.1f CHOICE=%s CONF=%.2f REASON=%s",
                     original_heading, choice_str,
                     context_.plan.confidence, context_.plan.reason);
        }
    }
    robot_uart_->Stop(700);
    robot_uart_->SetMode(false, 700);
    Finish(MissionState::COMPLETED);
}

bool MissionManager::IsActive() const {
    LockGuard lock(mutex_);
    return lock.locked() && active_;
}

bool MissionManager::IsAiObstacleHoldActive() const {
    return ai_obstacle_hold_.load();
}

bool MissionManager::ReleaseAiObstacleHoldForReplay() {
    if (IsActive()) return false;
    if (!ai_obstacle_hold_.exchange(false)) return true;
    ai_obstacle_hold_event_id_.store(0);
    cancel_requested_.store(false);
    ESP_LOGI("AI_OBS", "HOLD_CLEAR=MAP_REPLAY_RESUME");
    ESP_LOGI("AI_OBS", "HOLD=0");
    return true;
}

bool MissionManager::CancelRequested() {
    if (cancel_requested_.load()) return true;
    if (robot_uart_ != nullptr && robot_uart_->Ps2OverrideActive()) {
        // A PS2 takeover is terminal for this mission.  Do not mistake its
        // interrupted MOVE for an obstacle and start an AI replan.
        cancel_requested_.store(true);
        if (ai_obstacle_hold_.exchange(false)) {
            ai_obstacle_hold_event_id_.store(0);
            ESP_LOGI("AI_OBS", "HOLD_CLEAR=PS2_OVERRIDE");
            ESP_LOGI("AI_OBS", "HOLD=0");
        }
        ESP_LOGI(kTag, "RH,CANCEL_REASON=PS2_OVERRIDE");
        return true;
    }
    return false;
}

void MissionManager::SetState(MissionState state) {
    LockGuard lock(mutex_);
    if (!lock.locked()) return;
    context_.state = state;
    context_.last_update_ms = NowMs();
    ESP_LOGI(kTag, "Mission %lu -> %s",
             static_cast<unsigned long>(context_.mission_id), StateName(state));
}

void MissionManager::SetFailure(const char* failure) {
    LockGuard lock(mutex_);
    if (!lock.locked()) return;
    snprintf(context_.failure, sizeof(context_.failure), "%s",
             failure == nullptr ? "unknown" : failure);
}

void MissionManager::Finish(MissionState terminal_state) {
    LockGuard lock(mutex_);
    if (!lock.locked()) return;
    context_.state = terminal_state;
    context_.last_update_ms = NowMs();
    active_ = false;
    task_ = nullptr;
}

void MissionManager::UpdateHeading(float heading_deg) {
    LockGuard lock(mutex_);
    if (!lock.locked()) return;
    context_.pose.heading_deg = NormalizeHeading(heading_deg);
    context_.last_update_ms = NowMs();
}

bool MissionManager::LoadPersistentHome() {
    nvs_handle_t handle = 0;
    const esp_err_t open_error =
        nvs_open(kHomeNvsNamespace, NVS_READONLY, &handle);
    if (open_error == ESP_ERR_NVS_NOT_FOUND) return false;
    if (open_error != ESP_OK) {
        ESP_LOGE(kTag, "HOME storage open failed: %s",
                 esp_err_to_name(open_error));
        return false;
    }

    size_t size = 0;
    esp_err_t error = nvs_get_blob(handle, kHomeNvsKey, nullptr, &size);
    if (error != ESP_OK || size < sizeof(PersistentHomeHeader) ||
        size > sizeof(PersistentHomeHeader) +
                    kMaxPersistedBreadcrumbs * sizeof(PersistentBreadcrumb)) {
        nvs_close(handle);
        if (error != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kTag, "HOME storage unavailable or invalid size=%u",
                     static_cast<unsigned>(size));
        }
        return false;
    }

    std::vector<uint8_t> bytes(size);
    error = nvs_get_blob(handle, kHomeNvsKey, bytes.data(), &size);
    nvs_close(handle);
    if (error != ESP_OK || size != bytes.size()) {
        ESP_LOGW(kTag, "HOME storage read failed: %s", esp_err_to_name(error));
        return false;
    }

    PersistentHomeHeader header;
    memcpy(&header, bytes.data(), sizeof(header));
    const size_t expected_size = sizeof(header) +
        static_cast<size_t>(header.breadcrumb_count) *
            sizeof(PersistentBreadcrumb);
    if (header.magic != kHomeStorageMagic ||
        header.version != kHomeStorageVersion ||
        header.breadcrumb_count == 0 ||
        header.breadcrumb_count > kMaxPersistedBreadcrumbs ||
        expected_size != bytes.size() || !std::isfinite(header.home_heading_deg)) {
        ESP_LOGW(kTag, "HOME storage header validation failed");
        return false;
    }

    const uint32_t expected_crc = header.crc32;
    header.crc32 = 0;
    uint32_t crc = esp_rom_crc32_le(
        0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    crc = esp_rom_crc32_le(
        crc, bytes.data() + sizeof(header),
        static_cast<uint32_t>(bytes.size() - sizeof(header)));
    if (crc != expected_crc) {
        ESP_LOGW(kTag, "HOME storage CRC validation failed");
        return false;
    }

    BreadcrumbList restored;
    restored.reserve(header.breadcrumb_count);
    for (size_t i = 0; i < header.breadcrumb_count; ++i) {
        PersistentBreadcrumb point;
        memcpy(&point, bytes.data() + sizeof(header) +
                   i * sizeof(point), sizeof(point));
        if (!std::isfinite(point.x_cm) || !std::isfinite(point.y_cm) ||
            !std::isfinite(point.heading_deg)) {
            ESP_LOGW(kTag, "HOME storage point validation failed index=%u",
                     static_cast<unsigned>(i));
            return false;
        }
        restored.push_back({point.x_cm, point.y_cm,
                            NormalizeHeading(point.heading_deg)});
    }
    if (std::fabs(restored.front().x_cm) > 1.0f ||
        std::fabs(restored.front().y_cm) > 1.0f) {
        ESP_LOGW(kTag, "HOME storage origin validation failed");
        return false;
    }

    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return false;
        home_valid_ = true;
        persistent_home_ok_ = true;
        odom_reference_valid_ = false;
        home_heading_deg_ = NormalizeHeading(header.home_heading_deg);
        breadcrumbs_ = std::move(restored);
        context_.pose.x_cm = breadcrumbs_.back().x_cm;
        context_.pose.y_cm = breadcrumbs_.back().y_cm;
        context_.pose.heading_deg = breadcrumbs_.back().heading_deg;
        context_.pose.confidence_pct = 80.0f;
    }
    ESP_LOGI(kTag, "HOME_RESTORED breadcrumbs=%u heading=%.1f",
             static_cast<unsigned>(header.breadcrumb_count),
             home_heading_deg_);
    return true;
}

bool MissionManager::SyncAfterExternalMotion() {
    if (robot_uart_ == nullptr || mutex_ == nullptr) return false;

    bool needs_reference_restore = false;
    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return false;
        // Manual distance commands must never race an autonomous or
        // return-home task. The UART layer serializes packets, but it cannot
        // decide which motion owner is allowed to update the route.
        if (active_) return false;
        if (!home_valid_) return true;
        needs_reference_restore = !odom_reference_valid_;
    }
    if (needs_reference_restore && !RestoreOdometryReference()) return false;

    RobotOdometry odom;
    if (!robot_uart_->GetOdometry(odom, 700) || !odom.valid) return false;
    const float heading_deg = NormalizeHeading(odom.heading_rad * 180.0f / kPi);
    bool breadcrumb_added = false;
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || !home_valid_ || !odom_reference_valid_) return false;
        context_.pose.x_cm = (odom.x_mm - odom_origin_x_mm_) / 10.0f;
        context_.pose.y_cm = (odom.y_mm - odom_origin_y_mm_) / 10.0f;
        context_.pose.heading_deg = heading_deg;
        context_.pose.confidence_pct = 95.0f;
        context_.last_update_ms = NowMs();

        // External moves may be shorter than the autonomous breadcrumb
        // interval. Record a meaningful operator move so a 10 cm test move is
        // still available after reboot and can be followed back to HOME.
        constexpr float kExternalMoveRecordCm = 1.5f;
        constexpr float kExternalHeadingRecordDeg = 2.0f;
        const Breadcrumb current{context_.pose.x_cm, context_.pose.y_cm,
                                 context_.pose.heading_deg};
        if (breadcrumbs_.empty()) {
            breadcrumbs_.push_back(current);
            breadcrumb_added = true;
        } else {
            const Breadcrumb& last = breadcrumbs_.back();
            const float dx = current.x_cm - last.x_cm;
            const float dy = current.y_cm - last.y_cm;
            const float distance = std::sqrt(dx * dx + dy * dy);
            const float heading_delta = std::fabs(
                NormalizeHeading(current.heading_deg - last.heading_deg));
            if (distance >= kExternalMoveRecordCm ||
                heading_delta >= kExternalHeadingRecordDeg) {
                constexpr size_t kMaxBreadcrumbs = 512;
                if (breadcrumbs_.size() >= kMaxBreadcrumbs) {
                    breadcrumbs_.erase(breadcrumbs_.begin() + 1);
                }
                breadcrumbs_.push_back(current);
                breadcrumb_added = true;
            }
        }
    }
    if (breadcrumb_added) RequestHomePersistence();
    return true;
}

void MissionManager::RequestHomePersistence() {
    const size_t internal_free =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t internal_largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    const size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    if (internal_free < kHomePersistMinInternalFree ||
        internal_largest < kHomePersistMinInternalLargest ||
        dma_free < kHomePersistMinDmaFree ||
        dma_largest < kHomePersistMinDmaLargest) {
        ESP_LOGW(kTag,
                 "HOME_PERSIST_DEFER_LOW_MEMORY INTERNAL_FREE=%u INTERNAL_LARGEST=%u DMA_FREE=%u DMA_LARGEST=%u",
                 static_cast<unsigned>(internal_free),
                 static_cast<unsigned>(internal_largest),
                 static_cast<unsigned>(dma_free),
                 static_cast<unsigned>(dma_largest));
        return;
    }

    bool expected = false;
    if (!home_persist_scheduled_.compare_exchange_strong(expected, true)) {
        return;
    }
    try {
        // Application::Run executes callbacks on the main task, whose stack is
        // internal RAM. This is required because nvs_set_blob/nvs_commit
        // disable the flash cache and cannot safely run on the PSRAM-backed
        // navigation task stack.
        Application::GetInstance().Schedule([this]() {
            const bool persisted = PersistHomeRoute();
            if (!persisted) {
                LockGuard lock(mutex_);
                if (lock.locked()) persistent_home_ok_ = false;
            } else {
                LockGuard lock(mutex_);
                if (lock.locked()) persistent_home_ok_ = true;
            }
            home_persist_scheduled_.store(false);
        });
    } catch (const std::exception& e) {
        home_persist_scheduled_.store(false);
        ESP_LOGW(kTag, "HOME_PERSIST_SCHEDULE_FAILED: %s", e.what());
    }
}

bool MissionManager::PersistHomeRoute() {
    const size_t internal_free =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t internal_largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    const size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    if (internal_free < kHomePersistMinInternalFree ||
        internal_largest < kHomePersistMinInternalLargest ||
        dma_free < kHomePersistMinDmaFree ||
        dma_largest < kHomePersistMinDmaLargest) {
        ESP_LOGW(kTag,
                 "HOME_PERSIST_SKIP_LOW_MEMORY INTERNAL_FREE=%u INTERNAL_LARGEST=%u DMA_FREE=%u DMA_LARGEST=%u",
                 static_cast<unsigned>(internal_free),
                 static_cast<unsigned>(internal_largest),
                 static_cast<unsigned>(dma_free),
                 static_cast<unsigned>(dma_largest));
        return false;
    }
    BreadcrumbList route;
    float home_heading = 0.0f;
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || !home_valid_ || breadcrumbs_.empty()) {
            return false;
        }
        route = breadcrumbs_;
        home_heading = home_heading_deg_;
    }

    // Keep HOME at index zero and retain the newest points if the in-memory
    // route exceeds the compact NVS record. DriveToBreadcrumb already limits
    // each physical leg and rechecks obstacles, so this bounded thinning is
    // safe while keeping the record below the 16 KiB NVS partition budget.
    if (route.size() > kMaxPersistedBreadcrumbs) {
        BreadcrumbList compact;
        compact.reserve(kMaxPersistedBreadcrumbs);
        compact.push_back(route.front());
        const size_t first_newest = route.size() -
                                     (kMaxPersistedBreadcrumbs - 1U);
        compact.insert(compact.end(), route.begin() + first_newest,
                       route.end());
        route = std::move(compact);
    }

    PersistentHomeHeader header;
    header.magic = kHomeStorageMagic;
    header.version = kHomeStorageVersion;
    header.breadcrumb_count = static_cast<uint16_t>(route.size());
    header.home_heading_deg = NormalizeHeading(home_heading);
    header.crc32 = 0;

    std::vector<uint8_t> bytes(
        sizeof(header) + route.size() * sizeof(PersistentBreadcrumb));
    memcpy(bytes.data(), &header, sizeof(header));
    for (size_t i = 0; i < route.size(); ++i) {
        const PersistentBreadcrumb point{
            route[i].x_cm, route[i].y_cm, NormalizeHeading(route[i].heading_deg)};
        memcpy(bytes.data() + sizeof(header) + i * sizeof(point), &point,
               sizeof(point));
    }
    uint32_t crc = esp_rom_crc32_le(0, bytes.data(), sizeof(header));
    crc = esp_rom_crc32_le(crc, bytes.data() + sizeof(header),
                           static_cast<uint32_t>(bytes.size() - sizeof(header)));
    header.crc32 = crc;
    memcpy(bytes.data(), &header, sizeof(header));

    nvs_handle_t handle = 0;
    esp_err_t error = nvs_open(kHomeNvsNamespace, NVS_READWRITE, &handle);
    if (error == ESP_OK) {
        error = nvs_set_blob(handle, kHomeNvsKey, bytes.data(), bytes.size());
    }
    if (error == ESP_OK) error = nvs_commit(handle);
    if (handle != 0) nvs_close(handle);
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "HOME storage write failed: %s", esp_err_to_name(error));
        return false;
    }
    return true;
}

bool MissionManager::RestoreOdometryReference() {
    if (robot_uart_ == nullptr) return false;
    RobotOdometry odom;
    if (!robot_uart_->GetOdometry(odom, 700) || !odom.valid) return false;
    LockGuard lock(mutex_);
    if (!lock.locked() || !home_valid_ || breadcrumbs_.empty()) return false;
    const Breadcrumb& last = breadcrumbs_.back();
    // Rebase against the last durable home-relative pose. This works both
    // when STM32 rebooted and reset its odometry and when only ESP32 rebooted.
    odom_origin_x_mm_ = odom.x_mm - last.x_cm * 10.0f;
    odom_origin_y_mm_ = odom.y_mm - last.y_cm * 10.0f;
    odom_reference_valid_ = true;
    context_.pose.x_cm = last.x_cm;
    context_.pose.y_cm = last.y_cm;
    context_.pose.heading_deg = NormalizeHeading(
        odom.heading_rad * 180.0f / kPi);
    context_.pose.confidence_pct = 80.0f;
    return true;
}

bool MissionManager::InitializeOdometryReference(bool reset_breadcrumbs) {
    if (robot_uart_ == nullptr) return false;
    RobotOdometry odom;
    if (!robot_uart_->GetOdometry(odom, 700) || !odom.valid) return false;
    const float heading_deg = NormalizeHeading(odom.heading_rad * 180.0f / kPi);
    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return false;
        odom_origin_x_mm_ = odom.x_mm;
        odom_origin_y_mm_ = odom.y_mm;
        odom_reference_valid_ = true;
        home_valid_ = true;
        home_heading_deg_ = heading_deg;
        context_.pose.x_cm = 0.0f;
        context_.pose.y_cm = 0.0f;
        context_.pose.heading_deg = heading_deg;
        context_.pose.confidence_pct = 95.0f;
        if (reset_breadcrumbs) {
            breadcrumbs_.clear();
            breadcrumbs_.push_back({0.0f, 0.0f, heading_deg});
        }
        context_.last_update_ms = NowMs();
    }
    ESP_LOGI(kTag, "HOME_SET odom=(%.1f,%.1f) heading=%.1f",
             odom_origin_x_mm_, odom_origin_y_mm_, home_heading_deg_);
    return true;
}

bool MissionManager::RecordBreadcrumbLocked() {
    constexpr float kBreadcrumbDistanceCm = 12.5f;
    constexpr float kBreadcrumbHeadingDeg = 12.0f;
    constexpr size_t kMaxBreadcrumbs = 512;
    const Breadcrumb current{context_.pose.x_cm, context_.pose.y_cm,
                             context_.pose.heading_deg};
    if (breadcrumbs_.empty()) {
        breadcrumbs_.push_back(current);
        return true;
    }
    const Breadcrumb& last = breadcrumbs_.back();
    const float dx = current.x_cm - last.x_cm;
    const float dy = current.y_cm - last.y_cm;
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float heading_delta = std::fabs(
        NormalizeHeading(current.heading_deg - last.heading_deg));
    if (distance < kBreadcrumbDistanceCm &&
        heading_delta < kBreadcrumbHeadingDeg) {
        return false;
    }
    if (breadcrumbs_.size() >= kMaxBreadcrumbs) {
        // Preserve HOME at index 0 while thinning the oldest travelled points.
        breadcrumbs_.erase(breadcrumbs_.begin() + 1);
    }
    breadcrumbs_.push_back(current);
    return true;
}

bool MissionManager::SyncPoseFromOdometry(bool record_breadcrumb) {
    if (robot_uart_ == nullptr) return false;
    RobotOdometry odom;
    if (!robot_uart_->GetOdometry(odom, 700) || !odom.valid) return false;
    const float heading_deg = NormalizeHeading(odom.heading_rad * 180.0f / kPi);
    bool breadcrumb_added = false;
    {
        LockGuard lock(mutex_);
        if (!lock.locked() || !odom_reference_valid_) return false;
        context_.pose.x_cm = (odom.x_mm - odom_origin_x_mm_) / 10.0f;
        context_.pose.y_cm = (odom.y_mm - odom_origin_y_mm_) / 10.0f;
        context_.pose.heading_deg = heading_deg;
        context_.pose.confidence_pct = 95.0f;
        context_.last_update_ms = NowMs();
        if (record_breadcrumb && context_.type != MissionType::RETURN_HOME) {
            breadcrumb_added = RecordBreadcrumbLocked();
        }
    }
    if (breadcrumb_added) RequestHomePersistence();
    return true;
}

void MissionManager::ObserveRay(float heading_deg, float distance_cm,
                                bool echo_valid) {
    LockGuard lock(mutex_);
    if (!lock.locked()) return;
    const float radians = NormalizeHeading(heading_deg) * kPi / 180.0f;
    const float ray_cm = std::clamp(distance_cm, 0.0f, kMapMaxRayCm);
    const int center = kMapSize / 2;
    auto mark = [this](int x, int y, int delta) {
        if (x < 0 || x >= kMapSize || y < 0 || y >= kMapSize) return;
        occupancy_[y][x] = static_cast<int8_t>(std::clamp(
            static_cast<int>(occupancy_[y][x]) + delta, -8, 8));
    };
    for (float along = kMapResolutionCm * 0.5f;
         along + kMapResolutionCm * 0.5f < ray_cm;
         along += kMapResolutionCm * 0.5f) {
        const float world_x = context_.pose.x_cm + along * std::cos(radians);
        const float world_y = context_.pose.y_cm + along * std::sin(radians);
        const int x = center + static_cast<int>(std::lround(world_x /
                                                            kMapResolutionCm));
        const int y = center - static_cast<int>(std::lround(world_y /
                                                            kMapResolutionCm));
        mark(x, y, -1);
    }
    if (echo_valid && distance_cm <= kMapMaxRayCm) {
        const float world_x = context_.pose.x_cm + ray_cm * std::cos(radians);
        const float world_y = context_.pose.y_cm + ray_cm * std::sin(radians);
        const int x = center + static_cast<int>(std::lround(world_x /
                                                            kMapResolutionCm));
        const int y = center - static_cast<int>(std::lround(world_y /
                                                            kMapResolutionCm));
        mark(x, y, 4);
    }
}

bool MissionManager::SampleEnvironment(bool use_camera,
                                       DirectionSample& sample) {
    sample = {};
    // The authoritative STM32 value is already filtered from five ultrasonic
    // samples. Do not filter it a second time on ESP32.
    std::vector<float> distances;
    RobotObstacleStatus latest;
    RobotObstacleStatus reading;
    if (robot_uart_->GetObstacle(reading, 500) && reading.fresh) {
        latest = reading;
        sample.sensor_fresh = true;
        if (reading.echo_valid) distances.push_back(reading.distance_cm);
    }
    if (!sample.sensor_fresh) return false;

    sample.echo_valid = !distances.empty();
    if (sample.echo_valid) {
        std::sort(distances.begin(), distances.end());
        sample.distance_cm = distances[distances.size() / 2];
    } else {
        // A fresh timeout means no object within the HC-SR04 working range.
        sample.distance_cm = 400.0f;
    }
    sample.blocked = IsBlockedZone(latest.zone) || latest.limited ||
                     (sample.echo_valid &&
                      sample.distance_cm < kEmergencyDistanceCm);
    if (!robot_uart_->GetHeading(sample.heading_deg, 700)) return false;
    sample.heading_deg = NormalizeHeading(sample.heading_deg);
    UpdateHeading(sample.heading_deg);
    ObserveRay(sample.heading_deg, sample.distance_cm, sample.echo_valid);

    Camera* camera = nullptr;
    {
        LockGuard lock(mutex_);
        if (lock.locked() && context_.camera_guidance && use_camera) {
            camera = camera_;
        }
    }
    if (camera != nullptr) camera->AnalyzeNavigation(sample.vision);

    {
        LockGuard lock(mutex_);
        if (lock.locked()) context_.last_obstacle_cm = sample.distance_cm;
    }
    return true;
}

bool MissionManager::TurnTo(float heading_deg) {
    if (CancelRequested()) return false;
    RobotTurnResult result;
    const int target = static_cast<int>(std::lround(NormalizeHeading(heading_deg)));
    int speed = 15;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) speed = context_.speed;
    }
    const bool ok = robot_uart_->SetMode(true, 700) &&
                    robot_uart_->TurnAbsolute(target, speed, result, 13000);
    if (!ok) {
        // STM32 has already transferred motor authority to PS2.  A STOP here
        // would race the operator and reclaim that authority.
        if (robot_uart_->Ps2OverrideActive()) return false;
        robot_uart_->Stop(700);
        return false;
    }
    UpdateHeading(result.heading_deg);
    vTaskDelay(pdMS_TO_TICKS(100));
    return !CancelRequested();
}

bool MissionManager::DriveSegment(bool forward, uint32_t duration_ms,
                                  bool use_camera) {
    if (CancelRequested()) return false;
    int speed = 10;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) speed = context_.speed;
    }

    DirectionSample before;
    if (forward && (!SampleEnvironment(use_camera, before) || before.blocked ||
                    CameraPathBlocked(before))) {
        return false;
    }
    float heading = before.sensor_fresh ? before.heading_deg : 0.0f;
    if (!before.sensor_fresh && !robot_uart_->GetHeading(heading, 700)) {
        return false;
    }
    // Mission entry points may begin in MANUAL (notably the bounded field-test
    // bypass). Make every movement primitive self-contained; the mode ACK is
    // normally one UART round trip and prevents a silent NACK,MODE stop.
    if (!robot_uart_->SetMode(true, 500) ||
        !robot_uart_->StartContinuous(forward, speed, 500)) {
        return false;
    }

    const uint32_t started = NowMs();
    bool blocked = false;
    while ((NowMs() - started) < duration_ms) {
        if (CancelRequested()) {
            blocked = true;
            break;
        }
        RobotState live_state;
        if (forward && (!robot_uart_->GetState(live_state, 300) ||
                        !live_state.moving)) {
            blocked = true;
            break;
        }
        float current_heading = 0.0f;
        if (forward && robot_uart_->GetHeading(current_heading, 300)) {
            const float delta = std::fabs(NormalizeHeading(
                current_heading - heading));
            if (delta > 18.0f) {
                ESP_LOGW(kTag,
                         "Compass hazard during forward drive: delta=%.1f deg",
                         delta);
                blocked = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(90));
        RobotObstacleStatus cached;
        if (forward && robot_uart_->GetCachedObstacle(cached) && cached.fresh &&
            (IsBlockedZone(cached.zone) || cached.limited)) {
            blocked = true;
            break;
        }
    }
    const bool stopped = robot_uart_->Stop(700);
    if (stopped) {
        // V4.2 position is authoritative STM32 encoder odometry. Timed drive
        // remains only a bounded motion primitive; it no longer estimates X/Y.
        (void)SyncPoseFromOdometry(!blocked);
    }
    return !blocked && stopped && !CancelRequested();
}

bool MissionManager::AdvanceSegments(int count, MissionState state) {
    SetState(state);
    for (int i = 0; i < count; ++i) {
        if (!DriveSegment(true, kDriveSegmentMs)) return false;
    }
    return true;
}

MissionManager::MotionResult MissionManager::DriveDistanceCm(
    float distance_cm, MissionState state) {
    if (robot_uart_->Ps2OverrideActive()) return MotionResult::PS2_PREEMPTED;
    if (cancel_requested_.load()) return MotionResult::CANCELLED;
    int speed = 10;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) speed = context_.speed;
    }
    distance_cm = std::clamp(distance_cm, 2.0f, 120.0f);
    DirectionSample before;
    if (!SampleEnvironment(true, before) || before.blocked ||
        CameraPathBlocked(before)) {
        return MotionResult::OBSTACLE_BLOCKED;
    }
    SetState(state);
    RobotDistanceResult result;
    const int distance_mm = static_cast<int>(std::lround(distance_cm * 10.0f));
    ESP_LOGI(kTag, "DRIVE_DISTANCE target=%.1fcm encoder_controlled speed=%d",
             distance_cm, speed);
    const bool ok = robot_uart_->SetMode(true, 700) &&
                    robot_uart_->MoveDistance(true, distance_mm, speed, result,
                                              31000);
    (void)SyncPoseFromOdometry(ok);
    if (robot_uart_->Ps2OverrideActive()) return MotionResult::PS2_PREEMPTED;
    if (cancel_requested_.load()) return MotionResult::CANCELLED;
    return ok && result.completed ? MotionResult::SUCCESS : MotionResult::ERROR;
}

bool MissionManager::DriveToBreadcrumb(float x_cm, float y_cm,
                                        int max_replans) {
    // A 12 cm tolerance made a 10 cm operator test move look like HOME and
    // skipped the only return leg. Keep this below the smallest commissioned
    // move while allowing normal encoder settling.
    constexpr float kWaypointToleranceCm = 4.0f;
    constexpr float kMaxReturnStepCm = 60.0f;
    int replans = 0;
    for (int advance = 0; advance < 10 && !CancelRequested(); ++advance) {
        if (!SyncPoseFromOdometry(false)) return false;
        NavigationPose pose;
        {
            LockGuard lock(mutex_);
            if (!lock.locked()) return false;
            pose = context_.pose;
        }
        const float dx = x_cm - pose.x_cm;
        const float dy = y_cm - pose.y_cm;
        const float distance_cm = std::sqrt(dx * dx + dy * dy);
        if (distance_cm <= kWaypointToleranceCm) return true;
        const float target_heading = NormalizeHeading(
            std::atan2(dy, dx) * 180.0f / kPi);
        {
            LockGuard lock(mutex_);
            if (lock.locked()) context_.route_heading_deg = target_heading;
        }
        const float turn_error = std::fabs(
            NormalizeHeading(target_heading - pose.heading_deg));
        if (turn_error > 4.0f) {
            SetState(MissionState::RETURN_TURNING);
            if (!TurnTo(target_heading)) return false;
        }

        DirectionSample ahead;
        if (!SampleEnvironment(true, ahead)) return false;
        if (ahead.blocked || CameraPathBlocked(ahead)) {
            SetState(MissionState::RETURN_BLOCKED);
            if (replans++ >= max_replans) return false;
            SetState(MissionState::RETURN_REPLAN);
            if (!AvoidObstacle()) return false;
            continue;
        }

        const float step_cm = std::min(distance_cm, kMaxReturnStepCm);
        const MotionResult motion =
            DriveDistanceCm(step_cm, MissionState::RETURN_MOVING);
        if (motion == MotionResult::PS2_PREEMPTED) {
            (void)CancelRequested();
            return false;
        }
        if (motion == MotionResult::CANCELLED || motion == MotionResult::ERROR) {
            return false;
        }
        if (motion == MotionResult::OBSTACLE_BLOCKED) {
            if (replans++ >= max_replans) return false;
            SetState(MissionState::RETURN_REPLAN);
            if (!AvoidObstacle()) return false;
        }
    }
    return false;
}

bool MissionManager::ScanDirection(float heading_deg, MissionState state,
                                   DirectionSample& sample) {
    SetState(state);
    return TurnTo(heading_deg) && SampleEnvironment(true, sample);
}

bool MissionManager::AvoidObstacle() {
    if (CONFIG_AUTOMATIC_DETOUR == 0) {
        ESP_LOGW(kTag,
                 "AUTO_DETOUR_DISABLED obstacle remains latched; stopping");
        SetFailure("automatic_detour_disabled");
        return false;
    }
    if (ai_obstacle_hold_.load()) return false;
    float base_heading = 0.0f;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) base_heading = context_.route_heading_deg;
    }
    // A single scan can legitimately find both diagonal rays occupied even
    // in an otherwise navigable room (for example while close to a chair
    // corner). Backing away changes both the ultrasonic cone and camera
    // perspective, so re-plan automatically instead of terminating the
    // remembered movement command or asking the user what to do.
    constexpr int kMaxNoPathReplans = 3;
    for (int attempt = 0; attempt < kMaxNoPathReplans; ++attempt) {
        if (BypassObstacleDiagonal(base_heading)) {
            LockGuard lock(mutex_);
            if (lock.locked()) context_.failure[0] = '\0';
            return true;
        }
        if (CancelRequested()) return false;

        bool no_safe_side = false;
        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                no_safe_side =
                    strcmp(context_.failure, "diagonal_no_safe_side") == 0;
                if (no_safe_side && attempt + 1 < kMaxNoPathReplans) {
                    context_.failure[0] = '\0';
                }
            }
        }
        if (!no_safe_side || attempt + 1 >= kMaxNoPathReplans) {
            // Avoidance failed after all attempts
            if (attempt + 1 >= kMaxNoPathReplans) {
                ESP_LOGE(kTag, "AVOIDANCE_FAILED: No safe path found after %d attempts",
                         kMaxNoPathReplans);
                Application::GetInstance().Schedule([]() {
                    Application::GetInstance().Alert(
                        "Không thể tránh vật cản", 
                        "Không tìm thấy đường đi an toàn. Robot dừng lại để chờ hỗ trợ.",
                        "sad");
                });
            }
            return false;
        }
        ESP_LOGW(kTag, "NO_PATH_REPLAN attempt=%d/%d after automatic backup",
                 attempt + 1, kMaxNoPathReplans);
        vTaskDelay(pdMS_TO_TICKS(180));
    }
    return false;

#if 0  // Retained temporarily as field-reference for the former square detour.
    SetState(MissionState::OBSTACLE_STOP);
    robot_uart_->Stop(700);

    float base_heading = 0.0f;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) base_heading = context_.route_heading_deg;
    }

    for (int attempt = 0; attempt < 2 && !CancelRequested(); ++attempt) {
        DirectionSample left;
        DirectionSample right;
        if (!ScanDirection(base_heading + kScanAngleDeg,
                           MissionState::SCAN_LEFT, left) ||
            !ScanDirection(base_heading - kScanAngleDeg,
                           MissionState::SCAN_RIGHT, right)) {
            SetFailure("scan_failed_or_compass_lost");
            return false;
        }

        SetState(MissionState::CHOOSE_PATH);
        const bool left_open = !left.blocked &&
                               left.distance_cm >= kOpenDirectionCm;
        const bool right_open = !right.blocked &&
                                right.distance_cm >= kOpenDirectionCm;
        const float left_vision = left.vision.valid
                                      ? left.vision.center_open_pct : 50.0f;
        const float right_vision = right.vision.valid
                                       ? right.vision.center_open_pct : 50.0f;
        const float left_score = std::min(left.distance_cm, 180.0f) * 0.75f +
                                 left_vision * 0.45f;
        const float right_score = std::min(right.distance_cm, 180.0f) * 0.75f +
                                  right_vision * 0.45f;
        const bool choose_left = left_open &&
                                 (!right_open || left_score >= right_score);
        const bool choose_right = right_open && !choose_left;

        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                context_.left_clearance_cm = left.distance_cm;
                context_.right_clearance_cm = right.distance_cm;
                context_.left_vision_score = left_vision;
                context_.right_vision_score = right_vision;
                snprintf(context_.chosen_side, sizeof(context_.chosen_side),
                         "%s", choose_left ? "left" :
                               (choose_right ? "right" : "none"));
            }
        }

        if (!choose_left && !choose_right) {
            SetState(MissionState::WAIT_NO_PATH);
            if (!TurnTo(base_heading) ||
                !DriveSegment(false, kDriveSegmentMs)) {
                SetFailure("no_safe_path");
                return false;
            }
            continue;
        }

        const float side = choose_left ? 1.0f : -1.0f;
        if (!TurnTo(base_heading + side * kScanAngleDeg) ||
            !AdvanceSegments(kDetourOutSegments,
                             MissionState::DETOUR_OUT) ||
            !TurnTo(base_heading) ||
            !AdvanceSegments(kDetourPassSegments,
                             MissionState::DETOUR_PASS) ||
            !TurnTo(base_heading - side * kScanAngleDeg) ||
            !AdvanceSegments(kDetourReturnSegments,
                             MissionState::DETOUR_RETURN) ||
            !TurnTo(base_heading)) {
            SetFailure("detour_blocked_or_cancelled");
            return false;
        }
        {
            LockGuard lock(mutex_);
            if (lock.locked()) ++context_.avoidance_count;
        }
        SetState(MissionState::RESUMING);
        return true;
    }

    SetFailure("no_safe_path_after_backup");
    return false;
#endif
}

bool MissionManager::BypassObstacleDiagonal(float base_heading) {
    SetState(MissionState::OBSTACLE_STOP);
    robot_uart_->Stop(500);

    DirectionSample center;
    if (!TurnTo(base_heading) || !SampleEnvironment(true, center)) {
        SetFailure("diagonal_center_sample_failed");
        return false;
    }
    const float obstacle_cm = center.echo_valid
                                  ? center.distance_cm : kAvoidTriggerCm;
    const float detour_cm = std::clamp(obstacle_cm + kBypassExtraCm,
                                       kMinDiagonalCm, kMaxDiagonalCm);

    // Camera-based obstacle position detection (NEW)
    // obstacle_center_x: -1=none, 0-1 position (0=left, 0.5=center, 1=right)
    bool prefer_left = true;
    if (center.vision.valid && center.vision.obstacle_center_x >= 0.0f) {
        // If obstacle is strongly on right (x > 0.65), prefer left to bypass
        // If obstacle is strongly on left (x < 0.35), prefer right to bypass
        if (center.vision.obstacle_center_x > 0.65f) {
            prefer_left = true;
        } else if (center.vision.obstacle_center_x < 0.35f) {
            prefer_left = false;
        } else {
            // Obstacle in center: use left/right openness heuristic
            prefer_left = center.vision.left_open_pct >= center.vision.right_open_pct;
        }
        ESP_LOGI(kTag, "OBSTACLE_POSITION x=%.2f prefer=%s",
                 center.vision.obstacle_center_x, prefer_left ? "LEFT" : "RIGHT");
    } else {
        // Fallback to traditional heuristic
        prefer_left = !center.vision.valid ||
            center.vision.left_open_pct >= center.vision.right_open_pct;
    }
    DirectionSample left;
    DirectionSample right;
    bool left_sampled = false;
    bool right_sampled = false;

    auto scan_side = [&](bool left_side) -> bool {
        DirectionSample& sample = left_side ? left : right;
        const MissionState state = left_side ? MissionState::SCAN_LEFT
                                             : MissionState::SCAN_RIGHT;
        const float heading = base_heading +
                              (left_side ? kDiagonalAngleDeg
                                         : -kDiagonalAngleDeg);
        const bool ok = ScanDirection(heading, state, sample);
        if (left_side) left_sampled = ok;
        else right_sampled = ok;
        return ok;
    };
    auto open = [](const DirectionSample& sample) {
        const bool camera_open = !sample.vision.valid ||
                                 sample.vision.center_open_pct >= 40.0f;
        return sample.sensor_fresh && !sample.blocked &&
               (!sample.echo_valid || sample.distance_cm >= kOpenDirectionCm) &&
               camera_open;
    };
    auto strongly_open = [&](const DirectionSample& sample) {
        const bool camera_strong = !sample.vision.valid ||
                                   sample.vision.center_open_pct >= 55.0f;
        return open(sample) &&
               (!sample.echo_valid ||
                sample.distance_cm >= kFastCommitDistanceCm) &&
               camera_strong;
    };
    auto score = [](const DirectionSample& sample) {
        const float vision = sample.vision.valid
                                 ? sample.vision.center_open_pct : 50.0f;
        // Boost score for high contrast edges (confident obstacle detection)
        const float confidence_boost = sample.vision.high_contrast ? 1.15f : 1.0f;
        return (std::min(sample.distance_cm, kMaxCleanDistanceCm) * 0.75f +
                vision * 0.45f) * confidence_boost;
    };

    const bool first_left = prefer_left;
    if (!scan_side(first_left)) {
        SetFailure("diagonal_first_scan_failed");
        return false;
    }
    DirectionSample& first = first_left ? left : right;
    bool choose_left = false;
    bool side_chosen = false;
    if (strongly_open(first)) {
        choose_left = first_left;
        side_chosen = true;
        ESP_LOGI(kTag, "FAST_COMMIT side=%s distance=%.1f camera=%.1f",
                 choose_left ? "LEFT" : "RIGHT", first.distance_cm,
                 first.vision.valid ? first.vision.center_open_pct : -1.0f);
    } else {
        if (!scan_side(!first_left)) {
            if (!open(first)) {
                SetFailure("diagonal_second_scan_failed");
                return false;
            }
            choose_left = first_left;
            side_chosen = true;
        } else {
            const bool left_open = open(left);
            const bool right_open = open(right);
            if (left_open || right_open) {
                choose_left = left_open &&
                              (!right_open || score(left) >= score(right));
                side_chosen = true;
            }
        }
    }
    if (!side_chosen) {
        SetState(MissionState::WAIT_NO_PATH);
        if (TurnTo(base_heading)) DriveSegment(false, kDriveSegmentMs);
        SetFailure("diagonal_no_safe_side");
        return false;
    }

    const float side = choose_left ? 1.0f : -1.0f;
    const float diagonal_heading = base_heading + side * kDiagonalAngleDeg;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.left_clearance_cm = left_sampled ? left.distance_cm : 0.0f;
            context_.right_clearance_cm = right_sampled ? right.distance_cm : 0.0f;
            context_.left_vision_score = left_sampled && left.vision.valid
                                             ? left.vision.center_open_pct : 0.0f;
            context_.right_vision_score = right_sampled && right.vision.valid
                                              ? right.vision.center_open_pct : 0.0f;
            context_.scan_angle_deg = kDiagonalAngleDeg;
            snprintf(context_.chosen_side, sizeof(context_.chosen_side), "%s",
                     choose_left ? "left" : "right");
        }
    }

    ESP_LOGI(kTag,
             "DIAGONAL_BYPASS side=%s obstacle=%.1fcm detour=%.1fcm angle=%.1f",
             choose_left ? "LEFT" : "RIGHT", obstacle_cm, detour_cm,
             kDiagonalAngleDeg);
    if (!TurnTo(diagonal_heading) ||
        DriveDistanceCm(detour_cm, MissionState::DETOUR_OUT) !=
            MotionResult::SUCCESS) {
        SetFailure("diagonal_out_blocked");
        return false;
    }

    bool route_clear = false;
    int extra_diagonal_segments = 0;
    for (int probe = 0; probe < 3 && !CancelRequested(); ++probe) {
        if (!TurnTo(base_heading)) {
            SetFailure("diagonal_heading_restore_failed");
            return false;
        }
        DirectionSample corridor;
        if (!SampleEnvironment(true, corridor)) {
            SetFailure("diagonal_corridor_sample_failed");
            return false;
        }
        route_clear = open(corridor);
        if (route_clear) break;
        if (probe == 2 || !TurnTo(diagonal_heading) ||
            DriveDistanceCm(2.0f * kEstimatedSegmentCm,
                            MissionState::DETOUR_OUT) != MotionResult::SUCCESS) {
            break;
        }
        extra_diagonal_segments += 2;
    }
    if (!route_clear) {
        SetFailure("diagonal_cannot_clear_obstacle");
        return false;
    }

    // Continue parallel to the requested route long enough to pass the object.
    // At 20 cm this produces a 30 cm pass, matching the requested behavior.
    const float pass_cm = std::clamp(obstacle_cm + kBypassExtraCm,
                                     kMinDiagonalCm, 60.0f);
    if (DriveDistanceCm(pass_cm, MissionState::DETOUR_PASS) !=
            MotionResult::SUCCESS ||
        !TurnTo(base_heading)) {
        SetFailure("diagonal_pass_blocked");
        return false;
    }
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.lateral_segments = static_cast<uint8_t>(std::min(
                255, static_cast<int>(std::ceil(detour_cm /
                                                kEstimatedSegmentCm)) +
                         extra_diagonal_segments));
            context_.pass_segments = static_cast<uint8_t>(std::min(
                255, static_cast<int>(std::ceil(pass_cm /
                                                kEstimatedSegmentCm))));
            ++context_.avoidance_count;
        }
    }
    SetState(MissionState::RESUMING);
    
    // NOTIFICATION: Alert user that obstacle bypassed and resuming
    Application::GetInstance().Schedule([]() {
        Application::GetInstance().Alert(
            "Tránh vật cản thành công", 
            "Đã lách qua vật cản. Tiếp tục đi theo hướng ban đầu...",
            "happy");
    });
    
    return true;
}

bool MissionManager::BypassObstacleAdaptive(float base_heading) {
    return BypassObstacleDiagonal(base_heading);

#if 0  // Former 90-degree square bypass retained as tuning reference.
    SetState(MissionState::OBSTACLE_STOP);
    robot_uart_->Stop(700);

    DirectionScan left;
    DirectionScan right;
    const float scan_angle = 35.0f;
    if (!ScanDirectionShadow(base_heading + scan_angle,
                             MissionState::SCAN_LEFT, left)) {
        SetFailure("bypass_left_scan_failed");
        return false;
    }
    CaptureVisionLocal("left", left.vision);
    if (!TurnToShadow(base_heading, kShadowScanSpeed) ||
        !ScanDirectionShadow(base_heading - scan_angle,
                             MissionState::SCAN_RIGHT, right)) {
        SetFailure("bypass_right_scan_failed");
        return false;
    }
    CaptureVisionLocal("right", right.vision);
    if (!TurnToShadow(base_heading, kShadowScanSpeed)) {
        SetFailure("bypass_scan_return_h0_failed");
        return false;
    }

    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.scan_left = left;
            context_.scan_right = right;
            context_.scan_angle_deg = scan_angle;
        }
    }
    RunPlanner();

    PathPlanResult plan;
    {
        LockGuard lock(mutex_);
        if (!lock.locked()) return false;
        plan = context_.plan;
    }
    if (!plan.valid || plan.choice == PathChoice::NONE) {
        SetFailure("bypass_no_safe_side");
        return false;
    }

    const bool choose_left = plan.choice == PathChoice::LEFT;
    const float side = choose_left ? 1.0f : -1.0f;
    const float outward_heading = base_heading + side * 90.0f;
    const float inward_heading = base_heading - side * 90.0f;
    ESP_LOGI(kShadowTag,
             "BYPASS_START side=%s H0=%.1f out=%.1f in=%.1f",
             choose_left ? "LEFT" : "RIGHT", base_heading,
             NormalizeHeading(outward_heading), NormalizeHeading(inward_heading));

    if (!TurnToShadow(outward_heading, kShadowScanSpeed)) {
        SetFailure("bypass_outward_turn_failed");
        return false;
    }

    int lateral_segments = 0;
    bool forward_corridor_clear = false;
    while (lateral_segments < kMaxLateralSegments && !CancelRequested()) {
        if (!DriveSegment(true, kDriveSegmentMs)) {
            SetFailure("bypass_lateral_blocked");
            return false;
        }
        ++lateral_segments;
        if (lateral_segments < kMinLateralSegments) continue;

        if (!TurnToShadow(base_heading, kShadowScanSpeed)) {
            SetFailure("bypass_corridor_turn_failed");
            return false;
        }
        DirectionSample corridor;
        if (!SampleEnvironment(true, corridor)) {
            SetFailure("bypass_corridor_sample_failed");
            return false;
        }
        const bool camera_clear = !corridor.vision.valid ||
            (corridor.vision.center_open_pct >= 50.0f &&
             (choose_left ? corridor.vision.right_open_pct
                          : corridor.vision.left_open_pct) >= 35.0f);
        forward_corridor_clear = !corridor.blocked &&
            (!corridor.echo_valid || corridor.distance_cm >= kOpenDirectionCm) &&
            camera_clear;
        ESP_LOGI(kShadowTag,
                 "BYPASS_LATERAL segments=%d est=%.1fcm front=%.1f camera=%d clear=%d",
                 lateral_segments, lateral_segments * kEstimatedSegmentCm,
                 corridor.distance_cm, camera_clear, forward_corridor_clear);
        if (forward_corridor_clear) break;
        if (!TurnToShadow(outward_heading, kShadowScanSpeed)) {
            SetFailure("bypass_lateral_resume_turn_failed");
            return false;
        }
    }
    if (!forward_corridor_clear) {
        SetFailure("bypass_cannot_clear_obstacle_width");
        return false;
    }

    int pass_segments = 0;
    bool return_path_clear = false;
    auto continue_in_offset_lane = [&](const char* reason) -> bool {
        // The box may still be beside the chassis when an inward probe is
        // attempted. Returning to the original lane is optional; recovering
        // H0 and continuing in the verified offset corridor is safer.
        robot_uart_->Stop(700);
        if (!TurnToShadow(base_heading, kShadowScanSpeed)) {
            SetFailure("bypass_offset_heading_recovery_failed");
            return false;
        }
        DirectionSample resume;
        if (!SampleEnvironment(true, resume)) {
            SetFailure("bypass_offset_resume_sample_failed");
            return false;
        }
        const bool camera_clear = !resume.vision.valid ||
            resume.vision.center_open_pct >= 40.0f;
        const bool path_clear = !resume.blocked &&
            (!resume.echo_valid || resume.distance_cm >= kOpenDirectionCm) &&
            camera_clear;
        if (!path_clear) {
            SetFailure("bypass_offset_resume_not_clear");
            return false;
        }
        // Two guarded segments demonstrate that the journey can resume. The
        // one-shot commissioning mission then stops and returns to MANUAL.
        if (!AdvanceSegments(2, MissionState::RESUMING)) {
            SetFailure("bypass_offset_resume_blocked");
            return false;
        }
        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                context_.lateral_segments =
                    static_cast<uint8_t>(lateral_segments);
                context_.pass_segments = static_cast<uint8_t>(pass_segments);
                ++context_.avoidance_count;
            }
        }
        ESP_LOGI(kShadowTag,
                 "BYPASS_COMPLETE_OFFSET reason=%s lateral=%d pass=%d heading=%.1f",
                 reason, lateral_segments, pass_segments, base_heading);
        SetState(MissionState::RESUMING);
        return true;
    };
    while (pass_segments < kMaxPassSegments && !CancelRequested()) {
        if (!DriveSegment(true, kDriveSegmentMs)) {
            SetFailure("bypass_pass_blocked");
            return false;
        }
        ++pass_segments;
        if (pass_segments < kMinPassSegments) continue;

        if (!TurnToShadow(inward_heading, kShadowScanSpeed)) {
            return continue_in_offset_lane("inward_turn_blocked");
        }
        DirectionSample inward;
        if (!SampleEnvironment(false, inward)) {
            SetFailure("bypass_inward_probe_failed");
            return false;
        }
        const float required = lateral_segments * kEstimatedSegmentCm +
                               kReturnClearanceMarginCm;
        return_path_clear = !inward.blocked &&
            (!inward.echo_valid || inward.distance_cm >= required);
        ESP_LOGI(kShadowTag,
                 "BYPASS_PASS segments=%d est=%.1fcm inward=%.1f required=%.1f clear=%d",
                 pass_segments, pass_segments * kEstimatedSegmentCm,
                 inward.distance_cm, required, return_path_clear);
        if (return_path_clear) break;
        if (!TurnToShadow(base_heading, kShadowScanSpeed)) {
            SetFailure("bypass_pass_resume_turn_failed");
            return false;
        }
    }
    if (!return_path_clear) {
        return continue_in_offset_lane("return_path_not_verified");
    }

    if (!AdvanceSegments(lateral_segments, MissionState::DETOUR_RETURN)) {
        // A distant side obstacle can enter the HC-SR04 cone near the end of
        // the lane return. Never keep driving toward it: restore H0 and
        // continue in the remaining offset lane if the forward corridor is
        // verified. This still satisfies return-to-original-heading without
        // forcing an unsafe return-to-original-track.
        return continue_in_offset_lane("lane_return_blocked");
    }
    if (!TurnToShadow(base_heading, kShadowScanSpeed)) {
        SetFailure("bypass_return_heading_failed");
        return false;
    }

    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.lateral_segments = static_cast<uint8_t>(lateral_segments);
            context_.pass_segments = static_cast<uint8_t>(pass_segments);
            ++context_.avoidance_count;
        }
    }
    ESP_LOGI(kShadowTag,
             "BYPASS_COMPLETE side=%s lateral=%d pass=%d heading=%.1f",
             choose_left ? "LEFT" : "RIGHT", lateral_segments,
             pass_segments, base_heading);
    SetState(MissionState::RESUMING);
    return true;
#endif
}

void MissionManager::RunBypassOnce() {
    ESP_LOGI(kShadowTag, "One-obstacle bypass task started");
    RobotState state;
    float route_heading = 0.0f;
    if (!robot_uart_->GetState(state, 700) || !state.compass_ok ||
        state.brake_enabled || state.moving ||
        !robot_uart_->GetHeading(route_heading, 700)) {
        SetFailure(state.brake_enabled ? "brake_locked" :
                                          "bypass_preflight_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(MissionState::FAILED);
        return;
    }
    route_heading = NormalizeHeading(route_heading);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.route_heading_deg = route_heading;
            context_.original_heading_deg = route_heading;
            context_.pose.heading_deg = route_heading;
        }
    }

    bool obstacle_found = false;
    for (int segment = 0;
         segment < kMaxApproachSegments && !CancelRequested(); ++segment) {
        SetState(MissionState::NAVIGATING);
        DirectionSample ahead;
        if (!SampleEnvironment(false, ahead)) {
            SetFailure("bypass_approach_sensor_failed");
            break;
        }
        ESP_LOGI(kShadowTag,
                 "BYPASS_APPROACH segment=%d distance=%.1f blocked=%d",
                 segment, ahead.distance_cm, ahead.blocked);
        if (ahead.blocked ||
            (ahead.echo_valid && ahead.distance_cm <= kAvoidTriggerCm)) {
            obstacle_found = true;
            break;
        }
        if (!DriveSegment(true, kDriveSegmentMs)) {
            DirectionSample stopped_ahead;
            obstacle_found = SampleEnvironment(false, stopped_ahead) &&
                (stopped_ahead.blocked ||
                 (stopped_ahead.echo_valid &&
                  stopped_ahead.distance_cm <= kAvoidTriggerCm));
            if (!obstacle_found) SetFailure("bypass_approach_drive_failed");
            break;
        }
        // A segment can cross the 30 cm CAUTION boundary without being hard
        // limited by STM32. Sample once after motion so the last allowed
        // approach segment cannot fall through as "no obstacle".
        DirectionSample after;
        if (!SampleEnvironment(false, after)) {
            SetFailure("bypass_approach_post_sample_failed");
            break;
        }
        if (after.blocked ||
            (after.echo_valid && after.distance_cm <= kAvoidTriggerCm)) {
            obstacle_found = true;
            ESP_LOGI(kShadowTag,
                     "BYPASS_APPROACH trigger_after_segment=%d distance=%.1f",
                     segment, after.distance_cm);
            break;
        }
    }

    bool completed = false;
    if (!CancelRequested() && obstacle_found) {
        completed = BypassObstacleAdaptive(route_heading);
    } else if (!CancelRequested()) {
        LockGuard lock(mutex_);
        if (lock.locked() && context_.failure[0] == '\0') {
            snprintf(context_.failure, sizeof(context_.failure), "%s",
                     "no_obstacle_within_approach_limit");
        }
    }

    robot_uart_->Stop(700);
    robot_uart_->SetMode(false, 700);
    if (CancelRequested()) {
        Finish(MissionState::CANCELLED);
    } else {
        Finish(completed ? MissionState::COMPLETED : MissionState::FAILED);
    }
    ESP_LOGI(kShadowTag, "One-obstacle bypass task ended completed=%d",
             completed);
}

void MissionManager::RunReturnHome() {
    ESP_LOGI(kTag, "Return-home task started");
    RobotState state;
    if (!robot_uart_->GetState(state, 700) || state.brake_enabled ||
        !robot_uart_->SetMode(true, 700) || !SyncPoseFromOdometry(false)) {
        SetFailure(state.brake_enabled ? "brake_locked" :
                                          "return_home_preflight_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(MissionState::FAILED);
        return;
    }

    BreadcrumbList trail;
    float final_heading = 0.0f;
    bool home_available = false;
    {
        LockGuard lock(mutex_);
        if (lock.locked() && home_valid_ && !breadcrumbs_.empty()) {
            trail = breadcrumbs_;
            final_heading = home_heading_deg_;
            home_available = true;
        }
    }
    if (!home_available) {
        SetFailure("home_not_available");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(MissionState::FAILED);
        return;
    }

    bool failed = false;
    // Walk the trail backwards. Duplicate/nearby points are automatically
    // skipped by DriveToBreadcrumb's tolerance.
    for (auto it = trail.rbegin(); it != trail.rend() && !CancelRequested();
         ++it) {
        if (!DriveToBreadcrumb(it->x_cm, it->y_cm, 3)) {
            failed = true;
            SetFailure("return_waypoint_failed");
            break;
        }
    }
    if (!failed && !CancelRequested()) {
        // Explicit final approach covers a thinned trail and accumulated
        // waypoint tolerance before restoring the original HOME heading.
        if (!DriveToBreadcrumb(0.0f, 0.0f, 3)) {
            failed = true;
            SetFailure("return_home_position_failed");
        }
    }
    if (!failed && !CancelRequested()) {
        SetState(MissionState::RETURN_FINAL_ALIGN);
        if (!TurnTo(final_heading)) {
            failed = true;
            SetFailure("return_final_heading_failed");
        }
    }

    const bool ps2_preempted = robot_uart_->Ps2OverrideActive();
    if (!ps2_preempted) {
        robot_uart_->Stop(700);
        (void)SyncPoseFromOdometry(false);
        robot_uart_->SetMode(false, 700);
    }
    if (CancelRequested()) {
        Finish(MissionState::CANCELLED);
    } else if (failed) {
        Finish(MissionState::FAILED);
    } else {
        Finish(MissionState::RETURN_COMPLETED);
    }
    MissionState final_state = MissionState::FAILED;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) final_state = context_.state;
    }
    ESP_LOGI(kTag, "Return-home task ended: %s", StateName(final_state));
}

void MissionManager::ReturnHomeTaskEntry(void* context) {
    MissionManager* manager = static_cast<MissionManager*>(context);
    manager->RunReturnHome();
    // The stack was allocated from PSRAM by the WithCaps API and must be
    // released by its matching deleter.
    vTaskDeleteWithCaps(nullptr);
}

void MissionManager::TaskEntry(void* context) {
    MissionManager* manager = static_cast<MissionManager*>(context);
    if (manager->ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=TASK_ENTRY,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(manager->ai_obstacle_hold_event_id_.load()));
        manager->Finish(MissionState::CANCELLED);
        vTaskDeleteWithCaps(nullptr);
        return;
    }
    MissionType type = MissionType::NONE;
    {
        LockGuard lock(manager->mutex_);
        if (lock.locked()) type = manager->context_.type;
    }
    if (type == MissionType::AVOID_SCAN) {
        manager->RunShadowScan();
    } else if (type == MissionType::BYPASS_ONCE) {
        manager->RunBypassOnce();
    } else {
        manager->RunAutonomous();
    }
    vTaskDeleteWithCaps(nullptr);
}

void MissionManager::RunAutonomous() {
    ESP_LOGI(kTag, "Autonomous navigation task started");
    if (ai_obstacle_hold_.load()) {
        ESP_LOGI("AI_OBS", "SUPPRESS_MISSION_START=AUTONOMOUS_TASK,HOLD_EVENT_ID=%lu",
                 static_cast<unsigned long>(ai_obstacle_hold_event_id_.load()));
        Finish(MissionState::CANCELLED);
        return;
    }
    RobotState state;
    float route_heading = 0.0f;
    int motion_speed = 10;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) motion_speed = context_.speed;
    }
    bool need_home = true;
    bool need_reference_restore = false;
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            need_home = !home_valid_;
            need_reference_restore = home_valid_ && !odom_reference_valid_;
        }
    }
    const bool odom_ready = need_home
                                ? InitializeOdometryReference(true)
                                : (need_reference_restore
                                       ? RestoreOdometryReference()
                                       : SyncPoseFromOdometry(true));
    if (!odom_ready) {
        SetFailure("odometry_not_ready");
        Finish(MissionState::FAILED);
        return;
    }
    // The navigation task uses a PSRAM-backed stack. Do not perform the NVS
    // write here; queue it onto the internal-stack application task instead.
    if (need_home) RequestHomePersistence();
    // An active GetState response proves the link; the cached heartbeat age can
    // legitimately be stale before a mission starts.
    if (!robot_uart_->GetState(state, 700) || state.brake_enabled ||
        !robot_uart_->GetHeading(route_heading, 700) ||
        !robot_uart_->SetMode(true, 700) ||
        !robot_uart_->StartContinuous(true, motion_speed, 700)) {
        SetFailure(state.brake_enabled ? "brake_locked" :
                                      "preflight_failed");
        robot_uart_->Stop(700);
        robot_uart_->SetMode(false, 700);
        Finish(MissionState::FAILED);
        return;
    }

    route_heading = NormalizeHeading(route_heading);
    {
        LockGuard lock(mutex_);
        if (lock.locked()) {
            context_.route_heading_deg = route_heading;
            context_.pose.heading_deg = route_heading;
        }
    }

    bool failed = false;
    while (!CancelRequested()) {
        uint32_t elapsed = 0;
        uint32_t limit = 0;
        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                elapsed = NowMs() - context_.started_ms;
                limit = context_.runtime_limit_ms;
            }
        }
        if (elapsed >= limit) break;

        SetState(MissionState::NAVIGATING);
        if (!SyncPoseFromOdometry(true)) {
            SetFailure("odometry_sync_failed");
            failed = true;
            break;
        }
        const uint32_t observation_started = NowMs();
        DirectionSample ahead;
        if (!SampleEnvironment(true, ahead)) {
            SetFailure("ultrasonic_stale_or_heading_lost");
            failed = true;
            break;
        }
        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                context_.runtime_limit_ms += NowMs() - observation_started;
            }
        }

        bool semantic_blocked = false;
        uint32_t last_semantic_ms = 0;
        VisionObstacleResult latest_semantic;
        {
            LockGuard lock(mutex_);
            if (lock.locked()) {
                last_semantic_ms = context_.last_ai_vision_ms;
                latest_semantic = context_.last_ai_vision;
            }
        }
        semantic_blocked = last_semantic_ms != 0 &&
            (NowMs() - last_semantic_ms) <= 15000U &&
            SemanticPathBlocked(latest_semantic);
        if (last_semantic_ms == 0 ||
            (NowMs() - last_semantic_ms) >= kSemanticVisionIntervalMs) {
            StartSemanticMonitor();
        }

        const bool camera_blocked = CameraPathBlocked(ahead);
        if (ahead.blocked || ahead.distance_cm < kAvoidTriggerCm ||
            camera_blocked || semantic_blocked) {
            ESP_LOGI(kTag,
                     "AVOID_TRIGGER sr04=%d camera=%d semantic=%d dist=%.1f center=%.0f",
                     ahead.blocked || ahead.distance_cm < kAvoidTriggerCm,
                     camera_blocked, semantic_blocked, ahead.distance_cm,
                     ahead.vision.center_open_pct);
            
            // NOTIFICATION: Alert user about obstacle detection and avoidance activation
            Application::GetInstance().Schedule(
                [dist = ahead.distance_cm, blocked = ahead.blocked, 
                 cam_block = camera_blocked]() {
                    char message[256];
                    if (blocked || cam_block) {
                        snprintf(message, sizeof(message),
                                 "Phát hiện vật cản phía trước (%.0f cm). Kích hoạt chế độ tránh vật cản tự động...",
                                 dist);
                    } else {
                        snprintf(message, sizeof(message),
                                 "Vật cản phía trước quá gần (%.0f cm). Đang tìm đường khác để né...",
                                 dist);
                    }
                    Application::GetInstance().Alert(
                        "Phát hiện vật cản", message, "thinking");
                });
            
            const uint32_t avoidance_started = NowMs();
            if (!AvoidObstacle()) {
                failed = true;
                break;
            }
            int resumed_speed = 10;
            {
                LockGuard lock(mutex_);
                if (lock.locked()) resumed_speed = context_.speed;
            }
            if (!robot_uart_->StartContinuous(true, resumed_speed, 700)) {
                SetFailure("resume_forward_failed");
                failed = true;
                break;
            }
            // The requested forward runtime is remembered; autonomous
            // obstacle handling does not consume it.
            {
                LockGuard lock(mutex_);
                if (lock.locked()) {
                    context_.runtime_limit_ms += NowMs() - avoidance_started;
                }
            }
            continue;
        }
        // Correct accumulated wheel/motor asymmetry before it turns a long
        // forward command into an arc. Avoidance may intentionally use a
        // diagonal heading, but normal navigation always converges back to
        // the remembered route heading without consuming requested run time.
        float route_heading = 0.0f;
        {
            LockGuard lock(mutex_);
            if (lock.locked()) route_heading = context_.route_heading_deg;
        }
        const float heading_error =
            NormalizeHeading(route_heading - ahead.heading_deg);
        if (std::fabs(heading_error) > 7.0f) {
            ESP_LOGI(kTag,
                     "HEADING_CORRECTION current=%.1f route=%.1f error=%.1f",
                     ahead.heading_deg, route_heading, heading_error);
            const uint32_t correction_started = NowMs();
            if (!TurnTo(route_heading)) {
                SetFailure("route_heading_correction_failed");
                failed = true;
                break;
            }
            if (!robot_uart_->StartContinuous(true, motion_speed, 700)) {
                SetFailure("resume_after_heading_correction_failed");
                failed = true;
                break;
            }
            {
                LockGuard lock(mutex_);
                if (lock.locked()) {
                    context_.runtime_limit_ms += NowMs() - correction_started;
                }
            }
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    robot_uart_->Stop(700);
    (void)SyncPoseFromOdometry(true);
    robot_uart_->SetMode(false, 700);
    if (CancelRequested()) {
        Finish(MissionState::CANCELLED);
    } else if (failed) {
        Finish(MissionState::FAILED);
    } else {
        Finish(MissionState::COMPLETED);
    }
    ESP_LOGI(kTag, "Autonomous navigation task ended: %s",
             StateName(context_.state));
}

std::string MissionManager::StatusJson() const {
    LockGuard lock(mutex_);
    if (!lock.locked()) return "{\"ok\":false,\"error\":\"lock_timeout\"}";
    const char* choice_str = context_.plan.choice == PathChoice::LEFT
                                 ? "LEFT" : (context_.plan.choice == PathChoice::RIGHT
                                                 ? "RIGHT" : "NONE");
    // Keep large diagnostic JSON off small FreeRTOS task stacks.
    std::vector<char> json(2048);
    const uint32_t elapsed = context_.started_ms == 0
                                 ? 0 : NowMs() - context_.started_ms;
    snprintf(
        json.data(), json.size(),
        "{\"ok\":true,\"active\":%s,\"mission_id\":%lu,\"mission_type\":\"%s\",\"state\":\"%s\",\"shadow_only\":%s,\"shadow_mode\":%s,\"planner_self_test_ok\":%s,\"pose\":{\"x_cm\":%.1f,\"y_cm\":%.1f,\"heading_deg\":%.1f,\"confidence_pct\":%.0f},\"route_heading_deg\":%.1f,\"original_heading_deg\":%.1f,\"scan_angle_deg\":%.1f,\"speed\":%d,\"elapsed_ms\":%lu,\"avoidance_count\":%u,\"lateral_segments\":%u,\"pass_segments\":%u,\"last_obstacle_cm\":%.1f,\"camera_always_on\":%s,\"ai_vision\":{\"checks\":%u,\"attempts\":%u,\"uploads\":%u,\"source\":\"%s\",\"object\":\"%s\",\"danger\":\"%s\",\"risk\":%.2f},\"scan\":{\"center_cm\":%.1f,\"left_cm\":%.1f,\"right_cm\":%.1f,\"left_vision_source\":\"%s\",\"right_vision_source\":\"%s\"},\"plan\":{\"valid\":%s,\"choice\":\"%s\",\"confidence\":%.2f},\"failure\":\"%s\",\"localization\":\"stm32_encoder_plus_fused_heading\"}",
        active_ ? "true" : "false",
        static_cast<unsigned long>(context_.mission_id),
        TypeName(context_.type), StateName(context_.state),
        ShadowModeEnabled() ? "true" : "false",
        context_.shadow_mode ? "true" : "false",
        planner_self_test_ok_ ? "true" : "false",
        context_.pose.x_cm, context_.pose.y_cm,
        context_.pose.heading_deg, context_.pose.confidence_pct,
        context_.route_heading_deg, context_.original_heading_deg,
        context_.scan_angle_deg, context_.speed,
        static_cast<unsigned long>(elapsed),
        static_cast<unsigned>(context_.avoidance_count),
        static_cast<unsigned>(context_.lateral_segments),
        static_cast<unsigned>(context_.pass_segments),
        context_.last_obstacle_cm,
        context_.camera_guidance ? "true" : "false",
        static_cast<unsigned>(context_.ai_vision_count),
        static_cast<unsigned>(context_.ai_vision_attempt_count),
        static_cast<unsigned>(context_.ai_vision_upload_count),
        context_.last_ai_vision.source,
        context_.last_ai_vision.object_type,
        context_.last_ai_vision.danger_level,
        context_.last_ai_vision.risk_score,
        context_.scan_center.distance_cm,
        context_.scan_left.distance_cm, context_.scan_right.distance_cm,
        context_.scan_left.vision.source, context_.scan_right.vision.source,
        context_.plan.valid ? "true" : "false", choice_str,
        context_.plan.confidence, context_.failure);
    std::string result(json.data());
    if (!result.empty() && result.back() == '}') {
        char extra[160];
        snprintf(extra, sizeof(extra),
                 ",\"home_valid\":%s,\"home_persisted\":%s,\"breadcrumbs\":%u,\"home_heading_deg\":%.1f",
                 home_valid_ ? "true" : "false",
                 persistent_home_ok_ ? "true" : "false",
                 static_cast<unsigned>(breadcrumbs_.size()), home_heading_deg_);
        result.insert(result.size() - 1U, extra);
    }
    return result;
}

std::string MissionManager::HomeJson() const {
    LockGuard lock(mutex_);
    if (!lock.locked()) return "{\"ok\":false,\"error\":\"lock_timeout\"}";
    char json[320];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"home_valid\":%s,\"home_persisted\":%s,\"home_heading_deg\":%.1f,\"breadcrumbs\":%u,\"pose\":{\"x_cm\":%.1f,\"y_cm\":%.1f,\"heading_deg\":%.1f},\"localization\":\"stm32_encoder_plus_fused_heading\"}",
             home_valid_ ? "true" : "false",
             persistent_home_ok_ ? "true" : "false", home_heading_deg_,
             static_cast<unsigned>(breadcrumbs_.size()), context_.pose.x_cm,
             context_.pose.y_cm, context_.pose.heading_deg);
    return std::string(json);
}

std::string MissionManager::PlanJson() const {
    LockGuard lock(mutex_);
    if (!lock.locked()) return "{\"ok\":false,\"error\":\"lock_timeout\"}";
    const char* choice_str = context_.plan.choice == PathChoice::LEFT
                                 ? "LEFT" : (context_.plan.choice == PathChoice::RIGHT
                                                 ? "RIGHT" : "NONE");
    // Semantic scan details can be large; allocate the formatting buffer on
    // the heap so MCP/diagnostic tasks do not need multi-kilobyte stack frames.
    std::vector<char> json(4096);
    snprintf(json.data(), json.size(),
             "{\"ok\":true,\"shadow_only\":%s,\"shadow_mode\":%s,\"planner_self_test_ok\":%s,\"valid\":%s,\"choice\":\"%s\",\"confidence\":%.2f,\"left_cost\":%.1f,\"right_cost\":%.1f,\"reason\":\"%s\",\"scan\":{\"center\":{\"distance_cm\":%.1f,\"zone\":\"%s\",\"echo_valid\":%s,\"fresh\":%s,\"vision_source\":\"%s\"},\"left\":{\"distance_cm\":%.1f,\"zone\":\"%s\",\"echo_valid\":%s,\"fresh\":%s,\"vision_valid\":%s,\"vision_source\":\"%s\",\"vision_clear\":%.2f,\"vision_risk\":%.2f,\"vision_object\":\"%s\",\"vision_danger\":\"%s\"},\"right\":{\"distance_cm\":%.1f,\"zone\":\"%s\",\"echo_valid\":%s,\"fresh\":%s,\"vision_valid\":%s,\"vision_source\":\"%s\",\"vision_clear\":%.2f,\"vision_risk\":%.2f,\"vision_object\":\"%s\",\"vision_danger\":\"%s\"}}}",
             ShadowModeEnabled() ? "true" : "false",
             context_.shadow_mode ? "true" : "false",
             planner_self_test_ok_ ? "true" : "false",
             context_.plan.valid ? "true" : "false", choice_str,
             context_.plan.confidence, context_.plan.left_cost,
             context_.plan.right_cost, context_.plan.reason,
             context_.scan_center.distance_cm, context_.scan_center.zone,
             context_.scan_center.echo_valid ? "true" : "false",
             context_.scan_center.sensor_fresh ? "true" : "false",
             context_.scan_center.vision.source,
             context_.scan_left.distance_cm, context_.scan_left.zone,
             context_.scan_left.echo_valid ? "true" : "false",
             context_.scan_left.sensor_fresh ? "true" : "false",
             context_.scan_left.vision.valid ? "true" : "false",
             context_.scan_left.vision.source,
             context_.scan_left.vision.clear_score,
             context_.scan_left.vision.risk_score,
             context_.scan_left.vision.object_type,
             context_.scan_left.vision.danger_level,
             context_.scan_right.distance_cm, context_.scan_right.zone,
             context_.scan_right.echo_valid ? "true" : "false",
             context_.scan_right.sensor_fresh ? "true" : "false",
             context_.scan_right.vision.valid ? "true" : "false",
             context_.scan_right.vision.source,
             context_.scan_right.vision.clear_score,
             context_.scan_right.vision.risk_score,
             context_.scan_right.vision.object_type,
             context_.scan_right.vision.danger_level);
    return std::string(json.data());
}

std::string MissionManager::MapJson() const {
    LockGuard lock(mutex_);
    if (!lock.locked()) return "{\"ok\":false,\"error\":\"lock_timeout\"}";
    std::string json;
    json.reserve(1024);
    char header[256];
    snprintf(header, sizeof(header),
             "{\"ok\":true,\"resolution_cm\":%.0f,\"size\":%d,\"pose\":{\"x_cm\":%.1f,\"y_cm\":%.1f,\"heading_deg\":%.1f},\"legend\":\"? unknown, . free, # occupied, R robot\",\"rows\":[",
             kMapResolutionCm, kMapSize, context_.pose.x_cm,
             context_.pose.y_cm, context_.pose.heading_deg);
    json += header;
    const int center = kMapSize / 2;
    const int robot_x = center + static_cast<int>(std::lround(
                                    context_.pose.x_cm / kMapResolutionCm));
    const int robot_y = center - static_cast<int>(std::lround(
                                    context_.pose.y_cm / kMapResolutionCm));
    for (int y = 0; y < kMapSize; ++y) {
        if (y != 0) json += ',';
        json += '"';
        for (int x = 0; x < kMapSize; ++x) {
            char cell = occupancy_[y][x] >= 2 ? '#'
                        : (occupancy_[y][x] < 0 ? '.' : '?');
            if (x == robot_x && y == robot_y) cell = 'R';
            json += cell;
        }
        json += '"';
    }
    json += "]}";
    return json;
}

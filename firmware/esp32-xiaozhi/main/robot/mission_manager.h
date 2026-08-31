#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <string>
#include <vector>

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "camera.h"

class RobotUart;

template <typename T>
class RobotPsramAllocator {
public:
    using value_type = T;

    RobotPsramAllocator() noexcept = default;

    template <typename U>
    RobotPsramAllocator(const RobotPsramAllocator<U>&) noexcept {}

    T* allocate(std::size_t count) {
        if (count > static_cast<std::size_t>(-1) / sizeof(T)) {
            throw std::bad_alloc();
        }
        void* memory = heap_caps_malloc(
            count * sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (memory == nullptr) throw std::bad_alloc();
        return static_cast<T*>(memory);
    }

    void deallocate(T* memory, std::size_t) noexcept {
        heap_caps_free(memory);
    }

    template <typename U>
    struct rebind {
        using other = RobotPsramAllocator<U>;
    };
};

template <typename T, typename U>
bool operator==(const RobotPsramAllocator<T>&,
                const RobotPsramAllocator<U>&) noexcept {
    return true;
}

template <typename T, typename U>
bool operator!=(const RobotPsramAllocator<T>&,
                const RobotPsramAllocator<U>&) noexcept {
    return false;
}

enum class MissionType : uint8_t {
    NONE,
    AUTONOMOUS_FORWARD,
    AVOID_SCAN,  // Shadow-mode obstacle scan (never drives around the object).
    BYPASS_ONCE, // Commissioning: approach and bypass one verified obstacle.
    RETURN_HOME, // Follow STM32 odometry breadcrumbs back to the saved home pose.
};

enum class MissionState : uint8_t {
    IDLE,
    STARTING,
    NAVIGATING,
    OBSTACLE_STOP,
    SCAN_LEFT,
    SCAN_RIGHT,
    CHOOSE_PATH,
    WAIT_NO_PATH,
    DETOUR_OUT,
    DETOUR_PASS,
    DETOUR_RETURN,
    RESUMING,
    RETURN_STARTING,
    RETURN_TURNING,
    RETURN_MOVING,
    RETURN_BLOCKED,
    RETURN_REPLAN,
    RETURN_FINAL_ALIGN,
    RETURN_COMPLETED,
    COMPLETED,
    FAILED,
    CANCELLED,
};

struct NavigationPose {
    float x_cm = 0.0f;
    float y_cm = 0.0f;
    float heading_deg = 0.0f;
    float confidence_pct = 100.0f;
};

// Structured vision result for one scan direction. The vision service prompt
// returns JSON; parse failures leave valid=false without inventing data.
struct VisionObstacleResult {
    bool valid = false;
    float clear_score = 0.0f;
    float risk_score = 0.0f;
    char source[12] = "none";
    char object_type[24] = "unknown";
    char danger_level[16] = "unknown";
    float confidence = 0.0f;
};

enum class PathChoice : uint8_t {
    NONE,
    LEFT,
    RIGHT,
};

struct PathPlanResult {
    bool valid = false;
    PathChoice choice = PathChoice::NONE;
    float left_cost = 0.0f;
    float right_cost = 0.0f;
    float confidence = 0.0f;
    char reason[80] = {};
};

// Per-direction ultrasonic + vision snapshot kept independently so the planner
// can combine sensor and vision without one overriding the HC-SR04 safety.
struct DirectionScan {
    bool valid = false;
    bool sensor_fresh = false;
    bool echo_valid = false;
    bool blocked = false;
    float distance_cm = 0.0f;
    float approach_rate_cm_s = 0.0f;
    char zone[12] = {};
    float heading_deg = 0.0f;
    VisionObstacleResult vision;
};

struct MissionContext {
    uint32_t mission_id = 0;
    MissionType type = MissionType::NONE;
    MissionState state = MissionState::IDLE;
    NavigationPose pose;
    int speed = 10;
    bool camera_guidance = true;
    uint32_t started_ms = 0;
    uint32_t last_update_ms = 0;
    uint32_t runtime_limit_ms = 0;
    uint16_t avoidance_count = 0;
    uint8_t lateral_segments = 0;
    uint8_t pass_segments = 0;
    float route_heading_deg = 0.0f;
    float last_obstacle_cm = 0.0f;
    float left_clearance_cm = 0.0f;
    float right_clearance_cm = 0.0f;
    float left_vision_score = 0.0f;
    float right_vision_score = 0.0f;
    char chosen_side[8] = "none";
    char failure[64] = {};
    uint16_t ai_vision_count = 0;
    uint16_t ai_vision_attempt_count = 0;
    uint16_t ai_vision_upload_count = 0;
    uint32_t last_ai_vision_ms = 0;
    VisionObstacleResult last_ai_vision;
    // Shadow-mode scan fields.
    bool shadow_mode = true;
    float original_heading_deg = 0.0f;
    float scan_angle_deg = 35.0f;
    DirectionScan scan_center;
    DirectionScan scan_left;
    DirectionScan scan_right;
    PathPlanResult plan;
};

// High-level navigation runs on ESP32. STM32 remains the only motor owner and
// applies HC-SR04 braking, PS2 override and link fail-safe independently.
class MissionManager {
public:
    explicit MissionManager(RobotUart* robot_uart);
    ~MissionManager();

    void SetCamera(Camera* camera);
    // Starts a remembered navigation mission. Calling this again while an
    // autonomous mission is active updates its speed/camera policy and extends
    // the remaining requested run time instead of rejecting the command.
    static bool ShadowModeEnabled();
    bool StartAutonomousForward(int speed, uint32_t runtime_seconds,
                                bool camera_guidance);
    // Shadow-mode obstacle scan: stop, body scan left/right, capture camera,
    // run path planner, log LEFT/RIGHT/NONE. Never drives around the object.
    bool StartShadowScan(float scan_angle_deg = 35.0f);
    // Explicitly commissioned one-obstacle bypass. Unlike general autonomous
    // navigation, this has bounded approach/lateral/pass limits.
    bool StartBypassOnce();
    // Save the current fused STM32 odometry pose as HOME and clear the old
    // breadcrumb trail. This is safe while idle only.
    bool SetHome();
    // Navigate back through recorded odometry breadcrumbs. The robot turns
    // toward each waypoint and drives forward; STM32 safety remains supreme.
    bool StartReturnHome(int speed = 12, bool camera_guidance = true);
    // Synchronize a completed operator-requested move into the HOME route.
    // This does not start or stop motion; it only records the resulting pose.
    bool SyncAfterExternalMotion();
    bool CancelMission();
    // AI Obstacle Assist V1 owns a confirmed STM32 STOP event. This only
    // suppresses the legacy automatic bypass; it never resumes motion.
    void HoldForAiObstacle();
    bool IsActive() const;
    // Read-only integration gate for Teach Route. It does not release the
    // commissioned AI obstacle hold and cannot cause motion.
    bool IsAiObstacleHoldActive() const;
    // Replay resume may release only the latched AI obstacle hold, and only
    // while no mission owns the robot. It never starts motion.
    bool ReleaseAiObstacleHoldForReplay();
    std::string StatusJson() const;
    std::string HomeJson() const;
    std::string MapJson() const;
    // Planner result for the most recent shadow scan (or empty if none).
    std::string PlanJson() const;

private:
    static constexpr int kMapSize = 21;
    static constexpr float kMapResolutionCm = 25.0f;
    static constexpr float kMapMaxRayCm = 240.0f;
    static constexpr float kScanAngleDeg = 55.0f;
    // A candidate side ray must provide at least 40 cm in the measured
    // direction before the 35 cm-wide chassis may use it for a detour.
    static constexpr float kOpenDirectionCm = 40.0f;
    // Start at the CAUTION boundary. With a 40 cm-long body this preserves
    // room for the corner sweep while remaining close to the box.
    static constexpr float kAvoidTriggerCm = 30.0f;
    // Longer segments remove the stop-go feel while still re-checking the
    // environment before each forward advance and stopping immediately on any
    // dangerous sensor condition.
    static constexpr uint32_t kDriveSegmentMs = 2000;
    static constexpr int kDetourOutSegments = 3;
    static constexpr int kDetourPassSegments = 4;
    static constexpr int kDetourReturnSegments = 3;

    struct DirectionSample {
        bool sensor_fresh = false;
        bool echo_valid = false;
        bool blocked = true;
        float distance_cm = 0.0f;
        float heading_deg = 0.0f;
        CameraNavigationMetrics vision;
    };

    static void TaskEntry(void* context);
    static void ReturnHomeTaskEntry(void* context);
    static void SemanticTaskEntry(void* context);
    void RunSemanticMonitor();
    bool StartSemanticMonitor();
    void RunAutonomous();
    void RunBypassOnce();
    void RunShadowScan();
    void RunReturnHome();
    void CaptureVision(const char* direction, VisionObstacleResult& vision);
    void CaptureVisionLocal(const char* direction,
                            VisionObstacleResult& vision);
    static void ParseVisionJson(const char* json, VisionObstacleResult& vision);
    static float DirectionCost(const DirectionScan& scan,
                               float scan_angle_deg, bool is_left);
    static float CostDistance(const DirectionScan& scan);
    static float CostVision(const DirectionScan& scan);
    static float CostHeading(float scan_angle_deg);
    static float CostTurn();
    static bool IsTraversable(const DirectionScan& scan);
    static PathPlanResult ComputePlan(const DirectionScan& left,
                                      const DirectionScan& right,
                                      float scan_angle_deg);
    static bool PlannerSelfTest();
    void RunPlanner();
    bool AvoidObstacle();
    bool BypassObstacleAdaptive(float base_heading);
    bool BypassObstacleDiagonal(float base_heading);
    bool ScanDirection(float heading_deg, MissionState state,
                       DirectionSample& sample);
    bool ScanDirectionShadow(float heading_deg, MissionState state,
                             DirectionScan& scan);
    bool TurnToShadow(float heading_deg, int speed);
    bool SampleShadow(bool use_camera, DirectionScan& scan);
    bool SampleEnvironment(bool use_camera, DirectionSample& sample);
    bool TurnTo(float heading_deg);
    bool DriveSegment(bool forward, uint32_t duration_ms,
                      bool use_camera = true);
    enum class MotionResult : uint8_t {
        SUCCESS,
        PS2_PREEMPTED,
        OBSTACLE_BLOCKED,
        CANCELLED,
        ERROR,
    };
    MotionResult DriveDistanceCm(float distance_cm, MissionState state);
    bool AdvanceSegments(int count, MissionState state);
    bool CancelRequested();

    void SetState(MissionState state);
    void SetFailure(const char* failure);
    void Finish(MissionState terminal_state);
    void ResetMapLocked();
    void UpdateHeading(float heading_deg);
    void RequestHomePersistence();
    bool LoadPersistentHome();
    bool PersistHomeRoute();
    bool RestoreOdometryReference();
    bool InitializeOdometryReference(bool reset_breadcrumbs);
    bool SyncPoseFromOdometry(bool record_breadcrumb);
    bool RecordBreadcrumbLocked();
    bool DriveToBreadcrumb(float x_cm, float y_cm, int max_replans = 3);
    void ObserveRay(float heading_deg, float distance_cm, bool echo_valid);

    static float NormalizeHeading(float heading_deg);
    static const char* TypeName(MissionType type);
    static const char* StateName(MissionState state);
    static bool IsBlockedZone(const char* zone);
    static bool CameraPathBlocked(const DirectionSample& sample);
    static bool SemanticPathBlocked(const VisionObstacleResult& vision);

    RobotUart* robot_uart_ = nullptr;
    Camera* camera_ = nullptr;
    mutable SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t task_ = nullptr;
    std::atomic_bool cancel_requested_{false};
    std::atomic_bool ai_obstacle_hold_{false};
    std::atomic_uint32_t ai_obstacle_hold_event_id_{0};
    std::atomic_bool semantic_task_running_{false};
    std::atomic_bool home_persist_scheduled_{false};
    bool active_ = false;
    bool planner_self_test_ok_ = false;
    uint32_t next_mission_id_ = 1;
    struct Breadcrumb {
        float x_cm = 0.0f;
        float y_cm = 0.0f;
        float heading_deg = 0.0f;
    };

    MissionContext context_;
    bool odom_reference_valid_ = false;
    float odom_origin_x_mm_ = 0.0f;
    float odom_origin_y_mm_ = 0.0f;
    bool home_valid_ = false;
    bool persistent_home_ok_ = false;
    float home_heading_deg_ = 0.0f;
    using BreadcrumbList =
        std::vector<Breadcrumb, RobotPsramAllocator<Breadcrumb>>;
    BreadcrumbList breadcrumbs_;
    int8_t occupancy_[kMapSize][kMapSize] = {};
};

#ifndef XIAOZHI_TEACH_ROUTE_H
#define XIAOZHI_TEACH_ROUTE_H

#include "route_store.h"

#include <atomic>
#include <cstdint>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Display;
class MissionManager;
class RobotUart;
struct RobotState;
struct RobotObstacleStatus;

class TeachRoute {
public:
    TeachRoute(RobotUart* robot_uart, MissionManager* mission_manager);

    // Compatibility shim only. MAP V1 never renders on the ESP32 TFT.
    void SetDisplay(Display* display);
    bool Begin();

    // Kept for board-source compatibility. No raw PS2 polling task is created.
    bool StartInputTask();

    // High-level Map events are emitted by STM32 after it owns/debounces PS2.
    static void OnMapEvent(void* context, const char* action, uint8_t slot);
    void HandleMapEvent(const char* action, uint8_t slot);

private:
    // Numeric values are part of the ESP32 -> STM32 MAP,UI wire contract.
    // Keep these synchronized with the STM32 LCD parser/presentation.
    enum class Mode : uint8_t {
        READY = 0,
        TEACHING = 1,
        LOADED = 2,
        DELETE_CONFIRM = 3,
        REPLAY_READY = 4,
        REPLAY_CHECKED = 5,
        REPLAY_RUNNING = 6,
        REPLAY_HOLD = 7,
        REPLAY_COMPLETE = 8,
    };
    enum class WaypointSource : uint8_t {
        MANUAL,
        AUTO_DISTANCE,
        AUTO_CORNER,
        ENDPOINT,
    };

    static void AutoTimerEntry(void* context);
    void AutoTimerTick();
    bool StartAutoTimer();
    void StopAutoTimer();

    bool StartTeach();
    void CancelTeach();
    void UpdateAutoWaypoint();
    void AddWaypoint(bool manual_mark);
    bool AppendWaypoint(const RouteWaypoint& point, WaypointSource source);
    bool ReadCurrentWaypoint(RouteWaypoint& point) const;
    void TrackSample(const RouteWaypoint& point);
    void ResetSmartTracking();
    void ResetCornerTracking();
    void UndoWaypoint();
    void SaveTeach();
    void LoadSelected();
    void RequestDelete();
    void ConfirmDelete();

    // Replay V1 is commissioned in safety gates. Phase A validates without
    // motion. Phase B1 then permits exactly the first saved segment, with no
    // automatic turn and no continuation to the next waypoint.
    bool ArmReplay();
    void CancelReplay();
    bool RunReplayDryRun();
    bool StartReplayFirstSegment();
    bool StartReplayTurnAtWp2();
    bool StartReplayFinalSegment();
    bool StartFullReplay();
    bool AttemptResumeFullReplay();
    static void ReplayTaskEntry(void* context);
    static void ReplayTurnTaskEntry(void* context);
    static void ReplayFinalTaskEntry(void* context);
    static void FullReplayTaskEntry(void* context);
    void RunReplayFirstSegment();
    void RunReplayTurnAtWp2();
    void RunReplayFinalSegment();
    void RunFullReplay();
    void ClearResumeContext();
    bool CheckReplaySafety(const char* stage, RobotState& state,
                           RobotObstacleStatus& obstacle,
                           const char*& reason) const;

    void SetSelectedSlot(uint8_t slot);
    void Notify(const char* message, int duration_ms = 2500) const;
    void UpdateMapStatus() const;
    bool ReadOdometry();
    static int16_t HeadingCdeg(float heading_rad);
    static float HeadingDeltaDeg(int16_t first, int16_t second);
    static const char* WaypointSourceName(WaypointSource source);
    static uint8_t WaypointSourceFlags(WaypointSource source);

    RobotUart* robot_uart_ = nullptr;
    MissionManager* mission_manager_ = nullptr;
    RouteStore store_;
    RouteSlot selected_slot_ = RouteSlot::MAP_1;
    Mode mode_ = Mode::READY;
    RouteData working_route_{};
    RouteData loaded_route_{};
    float start_x_mm_ = 0.0f;
    float start_y_mm_ = 0.0f;
    float start_heading_rad_ = 0.0f;
    bool odometry_valid_ = false;

    // Smart Waypoint V1 samples odometry frequently but stores only manual
    // points, sparse safety checkpoints and one stabilized point per meaningful
    // corner. The persistent V1 format and 128-point cap remain unchanged.
    RouteWaypoint last_sample_{};
    bool last_sample_valid_ = false;
    bool corner_pending_ = false;
    uint8_t corner_stable_samples_ = 0;

    bool replay_plan_valid_ = false;
    std::atomic_bool replay_motion_running_{false};
    std::atomic_bool replay_cancel_requested_{false};
    TaskHandle_t replay_task_ = nullptr;
    uint16_t replay_wp_index_ = 0;
    uint16_t replay_wp_total_ = 0;
    uint32_t replay_target_mm_ = 0;
    uint32_t replay_travel_mm_ = 0;
    uint32_t replay_error_mm_ = 0;
    float replay_bearing12_deg_ = 0.0f;
    float replay_bearing23_deg_ = 0.0f;
    float replay_turn_delta_deg_ = 0.0f;
    uint8_t replay_operation_ = 0;  // 0 NONE, 1 MOVE, 2 TURN, 3 HOLD.

    // Volatile obstacle-interrupted MOVE context. Never persisted to NVS.
    bool resume_valid_ = false;
    uint16_t resume_wp_index_ = 0;
    uint32_t resume_original_target_mm_ = 0;
    uint32_t resume_completed_mm_ = 0;
    uint32_t resume_remaining_mm_ = 0;
    uint8_t resume_count_ = 0;
    float resume_hold_x_mm_ = 0.0f;
    float resume_hold_y_mm_ = 0.0f;
    float resume_hold_heading_rad_ = 0.0f;

    esp_timer_handle_t auto_timer_ = nullptr;
    std::atomic_bool auto_timer_enabled_{false};
    std::atomic_bool auto_tick_pending_{false};
};

#endif  // XIAOZHI_TEACH_ROUTE_H

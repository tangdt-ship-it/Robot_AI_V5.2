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
    // Numeric values are mirrored by the STM32 LCD MAP,UI parser. Phase B1
    // keeps the wire UI in READY/TEACH/LOADED/DELETE only; REPLAY_READY is an
    // internal/log state until the LCD protocol is extended in a later phase.
    enum class Mode : uint8_t {
        READY = 0,
        TEACHING = 1,
        LOADED = 2,
        DELETE_CONFIRM = 3,
        REPLAY_READY = 4,
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
    static void ReplayTaskEntry(void* context);
    void RunReplayFirstSegment();

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

    esp_timer_handle_t auto_timer_ = nullptr;
    std::atomic_bool auto_timer_enabled_{false};
    std::atomic_bool auto_tick_pending_{false};
};

#endif  // XIAOZHI_TEACH_ROUTE_H

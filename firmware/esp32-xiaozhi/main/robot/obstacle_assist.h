#pragma once

#include <atomic>
#include <cstdint>

#include "camera.h"
#include "robot_uart.h"

// Informational only: this service never sends a motor command.
class ObstacleAssist {
public:
    explicit ObstacleAssist(RobotUart* uart) : uart_(uart) {}
    void SetCamera(Camera* camera) { camera_ = camera; }
    void SetMissionManager(class MissionManager* manager) { mission_manager_ = manager; }
    static void OnStopped(void* context, const RobotObstacleStatus& event,
                          bool was_motion_active);

private:
    enum class State : uint8_t { IDLE, DETECTED, WAIT_CAMERA, CAPTURING,
                                 ANALYZING, RESULT_READY, COOLDOWN, FAILED };
    struct Result {
        char object[32] = "unknown";
        char left[10] = "UNKNOWN";
        char right[10] = "UNKNOWN";
        char action[8] = "WAIT";
        char confidence[10] = "UNKNOWN";
        char description[96] = {};
        char final_suggestion[8] = "WAIT";
        char hazard[12] = "UNKNOWN";
    };
    void Run(RobotObstacleStatus event);
    static bool Parse(const std::string& text, Result& result);
    static bool Unsafe(const char* zone);
    static void CopyField(const std::string& text, const char* name,
                          char* output, size_t output_size);
    RobotUart* uart_ = nullptr;
    Camera* camera_ = nullptr;
    class MissionManager* mission_manager_ = nullptr;
    std::atomic_bool active_{false};
    uint32_t last_event_ms_ = 0;
    char last_zone_[12] = {};
    bool raw_diagnostic_pending_ = true;
};

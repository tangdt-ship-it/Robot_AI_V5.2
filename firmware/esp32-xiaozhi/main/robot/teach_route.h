#ifndef XIAOZHI_TEACH_ROUTE_H
#define XIAOZHI_TEACH_ROUTE_H

#include "route_store.h"

#include <cstdint>

class Display;
class MissionManager;
class RobotUart;

class TeachRoute {
public:
    TeachRoute(RobotUart* robot_uart, MissionManager* mission_manager);

    void SetDisplay(Display* display);
    bool Begin();
    bool StartInputTask();

private:
    enum class Mode : uint8_t { READY, TEACHING, LOADED, DELETE_CONFIRM };

    static void InputTaskEntry(void* context);
    void InputTask();
    void Update();
    void HandleButtons(uint16_t buttons, uint32_t now_ms);
    struct ButtonTracker {
        const char* name = "";
        uint16_t mask = 0;
        bool press_active = false;
        uint32_t press_started_ms = 0;
        bool long_fired = false;
    };
    void UpdateButton(ButtonTracker& button, bool raw_down, uint32_t now_ms);
    void LogPollTiming() const;
    void OnButtonPressed(ButtonTracker& button);
    void OnButtonReleased(ButtonTracker& button, uint32_t duration_ms);
    void OnButtonLong(ButtonTracker& button, uint32_t duration_ms);
    bool StartTeach();
    void CancelTeach();
    void UpdateAutoWaypoint();
    void AddWaypoint(bool manual_mark);
    void UndoWaypoint();
    void SaveTeach();
    void LoadSelected();
    void RequestDelete();
    void ConfirmDelete();
    void TogglePage();
    void SelectNextSlot();
    void Notify(const char* message, int duration_ms = 2500) const;
    void UpdateMapStatus() const;
    bool ReadOdometry();
    static int16_t HeadingCdeg(float heading_rad);
    static float HeadingDeltaDeg(int16_t first, int16_t second);

    RobotUart* robot_uart_ = nullptr;
    MissionManager* mission_manager_ = nullptr;
    Display* display_ = nullptr;
    RouteStore store_;
    RouteSlot selected_slot_ = RouteSlot::MAP_1;
    Mode mode_ = Mode::READY;
    bool map_page_ = false;
    ButtonTracker select_button_{"SELECT", 0x0001};
    ButtonTracker l3_button_{"L3", 0x0002};
    ButtonTracker triangle_button_{"TRIANGLE", 0x1000};
    ButtonTracker circle_button_{"CIRCLE", 0x2000};
    ButtonTracker square_button_{"SQUARE", 0x8000};
    uint32_t last_ps2_poll_ms_ = 0;
    uint32_t last_ps2_poll_dt_ms_ = 0;
    uint32_t ps2_poll_min_ms_ = UINT32_MAX;
    uint32_t ps2_poll_max_ms_ = 0;
    RouteData working_route_{};
    RouteData loaded_route_{};
    float start_x_mm_ = 0.0f;
    float start_y_mm_ = 0.0f;
    float start_heading_rad_ = 0.0f;
    bool odometry_valid_ = false;
};

#endif  // XIAOZHI_TEACH_ROUTE_H

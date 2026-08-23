#include "teach_route.h"

#include "display.h"
#include "mission_manager.h"
#include "robot_uart.h"

#include <cmath>
#include <cstdio>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
constexpr const char* kTag = "TeachRoute";
constexpr uint16_t kPs2Select = 0x0001;
constexpr uint16_t kPs2L3 = 0x0002;
constexpr uint16_t kPs2Triangle = 0x1000;
constexpr uint16_t kPs2Circle = 0x2000;
constexpr uint16_t kPs2Square = 0x8000;
constexpr uint32_t kSelectLongMs = 1500;
constexpr uint32_t kSquareLongMs = 2000;
constexpr uint32_t kMapInputPollIntervalMs = 50;
constexpr uint32_t kIdleInputPollIntervalMs = 125;
constexpr float kAutoDistanceMm = 125.0f;
constexpr float kAutoHeadingDeg = 12.0f;
constexpr float kDuplicateDistanceMm = 10.0f;
constexpr float kDuplicateHeadingDeg = 1.0f;

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}
}

TeachRoute::TeachRoute(RobotUart* robot_uart, MissionManager* mission_manager)
    : robot_uart_(robot_uart), mission_manager_(mission_manager) {}

void TeachRoute::SetDisplay(Display* display) {
    display_ = display;
}

bool TeachRoute::Begin() {
    if (!store_.Begin()) {
        Notify("ROUTE STORAGE ERROR", 4000);
        return false;
    }
    for (RouteSlot slot : {RouteSlot::MAP_1, RouteSlot::MAP_2}) {
        const RouteSlotMetadata metadata = store_.GetMetadata(slot);
        ESP_LOGI(kTag, "ROUTE,BOOT,SLOT=%u,STATE=%s,POINTS=%u",
                 static_cast<unsigned>(slot), RouteStore::StateName(metadata.state),
                 metadata.waypoint_count);
    }
    return true;
}

bool TeachRoute::StartInputTask() {
    return xTaskCreate(InputTaskEntry, "teach_route", 3072, this, 2, nullptr) == pdPASS;
}

void TeachRoute::InputTaskEntry(void* context) {
    static_cast<TeachRoute*>(context)->InputTask();
}

void TeachRoute::InputTask() {
    // RobotLink HELLO establishes the fresh STM32 sequence window during boot.
    // Do not issue PS2 polling before that handshake has completed.
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (true) {
        const uint32_t started_ms = NowMs();
        Update();
        const uint32_t interval_ms = (map_page_ || mode_ == Mode::TEACHING)
                                         ? kMapInputPollIntervalMs
                                         : kIdleInputPollIntervalMs;
        const uint32_t elapsed_ms = NowMs() - started_ms;
        if (elapsed_ms < interval_ms) {
            vTaskDelay(pdMS_TO_TICKS(interval_ms - elapsed_ms));
        }
    }
}

void TeachRoute::Notify(const char* message, int duration_ms) const {
    ESP_LOGI(kTag, "%s", message);
    if (display_ != nullptr) display_->ShowNotification(message, duration_ms);
}

void TeachRoute::UpdateMapStatus() const {
    if (!map_page_ || display_ == nullptr) return;
    const RouteSlotMetadata metadata = store_.GetMetadata(selected_slot_);
    const char* mode = mode_ == Mode::TEACHING ? "TEACH" :
                       mode_ == Mode::LOADED ? "LOADED" :
                       mode_ == Mode::DELETE_CONFIRM ? "DELETE?" : "READY";
    char status[96];
    snprintf(status, sizeof(status), "%s %s %u/%u %s", RouteStore::SlotName(selected_slot_),
             RouteStore::StateName(metadata.state), metadata.waypoint_count,
             static_cast<unsigned>(kMaxWaypointsPerMap), mode);
    display_->SetStatus(status);
}

void TeachRoute::TogglePage() {
    map_page_ = !map_page_;
    if (map_page_) {
        Notify("MAP PAGE", 1500);
        UpdateMapStatus();
    } else {
        Notify("ROBOT PAGE", 1500);
    }
}

void TeachRoute::SelectNextSlot() {
    selected_slot_ = selected_slot_ == RouteSlot::MAP_1 ? RouteSlot::MAP_2 :
                                                          RouteSlot::MAP_1;
    char notification[40];
    snprintf(notification, sizeof(notification), "%s SELECTED",
             RouteStore::SlotName(selected_slot_));
    Notify(notification);
    UpdateMapStatus();
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
    RobotPs2Status ps2;
    if (!robot_uart_->GetPs2Status(ps2, 250) || !ps2.fresh || !ps2.buttons_valid) {
        Notify("TEACH REJECT: PS2", 3500);
        return false;
    }
    if (!ReadOdometry()) {
        Notify("TEACH REJECT: ODOM", 3500);
        return false;
    }
    working_route_ = {};
    working_route_.header.waypoint_count = 1;
    working_route_.waypoints[0] = {0, 0, 0, 0, 0};
    mode_ = Mode::TEACHING;
    char notification[48];
    snprintf(notification, sizeof(notification), "TEACH %s START",
             RouteStore::SlotName(selected_slot_));
    Notify(notification, 3500);
    UpdateMapStatus();
    return true;
}

void TeachRoute::CancelTeach() {
    if (mode_ != Mode::TEACHING) return;
    working_route_ = {};
    odometry_valid_ = false;
    mode_ = Mode::READY;
    Notify("TEACH CANCELLED");
    UpdateMapStatus();
}

void TeachRoute::AddWaypoint(bool manual_mark) {
    if (mode_ != Mode::TEACHING || !odometry_valid_) return;
    RobotOdometry odometry;
    if (robot_uart_ == nullptr || !robot_uart_->GetOdometry(odometry, 250) || !odometry.valid) {
        Notify("ODOMETRY ERROR");
        return;
    }
    const int32_t x = static_cast<int32_t>(lroundf(odometry.x_mm - start_x_mm_));
    const int32_t y = static_cast<int32_t>(lroundf(odometry.y_mm - start_y_mm_));
    const int16_t heading = HeadingCdeg(odometry.heading_rad - start_heading_rad_);
    const uint16_t count = working_route_.header.waypoint_count;
    RouteWaypoint& last = working_route_.waypoints[count - 1];
    const float dx = static_cast<float>(x - last.x_mm);
    const float dy = static_cast<float>(y - last.y_mm);
    const float distance = sqrtf(dx * dx + dy * dy);
    const float heading_delta = HeadingDeltaDeg(heading, last.heading_cdeg);
    if (manual_mark && distance < kDuplicateDistanceMm &&
        heading_delta < kDuplicateHeadingDeg) {
        last.flags |= kRouteWaypointManualMark;
        Notify("POINT MARKED");
        return;
    }
    if (!manual_mark && distance < kAutoDistanceMm && heading_delta < kAutoHeadingDeg) {
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
    working_route_.header.route_length_mm += static_cast<uint32_t>(lroundf(distance));
    char notification[48];
    snprintf(notification, sizeof(notification), "POINT %u %s", count + 1,
             manual_mark ? "MARKED" : "ADDED");
    Notify(notification);
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
    const uint32_t segment = static_cast<uint32_t>(lroundf(sqrtf(dx * dx + dy * dy)));
    working_route_.header.route_length_mm =
        working_route_.header.route_length_mm > segment ?
        working_route_.header.route_length_mm - segment : 0;
    working_route_.header.waypoint_count = count - 1;
    char notification[36];
    snprintf(notification, sizeof(notification), "UNDO POINT %u", count);
    Notify(notification);
    UpdateMapStatus();
}

void TeachRoute::SaveTeach() {
    if (mode_ != Mode::TEACHING) return;
    if (!store_.Save(selected_slot_, working_route_)) {
        Notify("STORAGE ERROR", 4000);
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
    RouteData route{};
    if (!store_.Load(selected_slot_, route)) {
        Notify("MAP NOT SAVED", 3000);
        return;
    }
    loaded_route_ = route;
    mode_ = Mode::LOADED;
    char notification[56];
    snprintf(notification, sizeof(notification), "%s LOADED: %u POINTS",
             RouteStore::SlotName(selected_slot_), route.header.waypoint_count);
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

void TeachRoute::UpdateButton(ButtonTracker& button, bool raw_down,
                              uint32_t now_ms) {
    if (raw_down && !button.press_active) {
        LogPollTiming();
        button.press_active = true;
        button.press_started_ms = now_ms;
        button.long_fired = false;
        ESP_LOGI(kTag, "MAP_BTN,%s,DOWN", button.name);
        OnButtonPressed(button);
    } else if (!raw_down && button.press_active) {
        LogPollTiming();
        const uint32_t duration_ms = now_ms - button.press_started_ms;
        ESP_LOGI(kTag, "MAP_BTN,%s,UP,DURATION=%lu", button.name,
                 static_cast<unsigned long>(duration_ms));
        OnButtonReleased(button, duration_ms);
        button.press_active = false;
        button.press_started_ms = 0;
        button.long_fired = false;
    }
    const uint32_t threshold = button.mask == kPs2Select ? kSelectLongMs :
                               button.mask == kPs2Square ? kSquareLongMs : 0;
    if (raw_down && button.press_active && threshold != 0 && !button.long_fired &&
        now_ms - button.press_started_ms >= threshold) {
        button.long_fired = true;
        OnButtonLong(button, now_ms - button.press_started_ms);
    }
}

void TeachRoute::LogPollTiming() const {
    ESP_LOGI(kTag, "MAP_BTN,POLL_DT=%lu,MIN=%lu,MAX=%lu",
             static_cast<unsigned long>(last_ps2_poll_dt_ms_),
             static_cast<unsigned long>(ps2_poll_min_ms_),
             static_cast<unsigned long>(ps2_poll_max_ms_));
}

void TeachRoute::OnButtonPressed(ButtonTracker& button) {
    if (button.mask == kPs2L3) {
        TogglePage();
        return;
    }
    if (!map_page_) return;
    if (button.mask == kPs2Triangle) {
        if (mode_ == Mode::TEACHING) AddWaypoint(true);
        else if (mode_ == Mode::READY || mode_ == Mode::LOADED) StartTeach();
    } else if (button.mask == kPs2Circle) {
        if (mode_ == Mode::TEACHING) SaveTeach();
        else if (mode_ == Mode::DELETE_CONFIRM) ConfirmDelete();
        else if (mode_ == Mode::READY || mode_ == Mode::LOADED) LoadSelected();
    }
}

void TeachRoute::OnButtonReleased(ButtonTracker& button, uint32_t duration_ms) {
    if (button.long_fired) {
        ESP_LOGI(kTag, "MAP_BTN,%s,SHORT_SUPPRESSED_AFTER_LONG", button.name);
        return;
    }
    if (!map_page_) return;
    if (button.mask == kPs2Select) {
        ESP_LOGI(kTag, "MAP_BTN,SELECT,SHORT");
        if (mode_ == Mode::DELETE_CONFIRM) {
            mode_ = Mode::READY;
            Notify("DELETE CANCELLED");
            UpdateMapStatus();
        } else if (mode_ != Mode::TEACHING) {
            SelectNextSlot();
        }
    } else if (button.mask == kPs2Square && mode_ == Mode::TEACHING) {
        ESP_LOGI(kTag, "MAP_BTN,SQUARE,SHORT,DURATION=%lu",
                 static_cast<unsigned long>(duration_ms));
        UndoWaypoint();
    }
}

void TeachRoute::OnButtonLong(ButtonTracker& button, uint32_t duration_ms) {
    if (!map_page_) return;
    ESP_LOGI(kTag, "MAP_BTN,%s,LONG,DURATION=%lu", button.name,
             static_cast<unsigned long>(duration_ms));
    if (button.mask == kPs2Select && mode_ == Mode::TEACHING) {
        CancelTeach();
    } else if (button.mask == kPs2Square &&
               (mode_ == Mode::READY || mode_ == Mode::LOADED)) {
        RequestDelete();
    }
}

void TeachRoute::HandleButtons(uint16_t buttons, uint32_t now_ms) {
    UpdateButton(l3_button_, (buttons & kPs2L3) == 0, now_ms);
    UpdateButton(select_button_, (buttons & kPs2Select) == 0, now_ms);
    UpdateButton(triangle_button_, (buttons & kPs2Triangle) == 0, now_ms);
    UpdateButton(circle_button_, (buttons & kPs2Circle) == 0, now_ms);
    UpdateButton(square_button_, (buttons & kPs2Square) == 0, now_ms);
}

void TeachRoute::Update() {
    if (robot_uart_ == nullptr) return;
    RobotPs2Status ps2;
    if (!robot_uart_->GetPs2Status(ps2, 250) || !ps2.fresh || !ps2.buttons_valid) {
        return;
    }
    const uint32_t now_ms = NowMs();
    if (last_ps2_poll_ms_ != 0) {
        last_ps2_poll_dt_ms_ = now_ms - last_ps2_poll_ms_;
        if (last_ps2_poll_dt_ms_ < ps2_poll_min_ms_) {
            ps2_poll_min_ms_ = last_ps2_poll_dt_ms_;
        }
        if (last_ps2_poll_dt_ms_ > ps2_poll_max_ms_) {
            ps2_poll_max_ms_ = last_ps2_poll_dt_ms_;
        }
    }
    last_ps2_poll_ms_ = now_ms;
    HandleButtons(ps2.buttons, now_ms);
    UpdateAutoWaypoint();
}

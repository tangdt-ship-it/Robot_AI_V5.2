#include "obstacle_assist.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "cJSON.h"

#include "application.h"
#include "board.h"
#include "display.h"
#include "mission_manager.h"

namespace {
constexpr const char* kTag = "AI_OBS";
uint32_t NowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL); }
void Mem(const char* stage) {
    ESP_LOGI(kTag, "MEM_%s,INTERNAL_FREE=%u,INTERNAL_LARGEST=%u,PSRAM_FREE=%u",
             stage, static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}
}

void ObstacleAssist::OnStopped(void* context, const RobotObstacleStatus& event,
                               bool was_motion_active) {
    auto* self = static_cast<ObstacleAssist*>(context);
    if (self == nullptr || !was_motion_active || self->camera_ == nullptr ||
        strcmp(event.zone, "CLEAR") == 0) return;
    const uint32_t now = NowMs();
    if (self->active_.exchange(true) ||
        (strcmp(self->last_zone_, event.zone) == 0 && now - self->last_event_ms_ < 10000)) return;
    self->last_event_ms_ = now;
    snprintf(self->last_zone_, sizeof(self->last_zone_), "%s", event.zone);
    ESP_LOGI(kTag, "OBSTACLE_EVENT=STOPPED,ZONE=%s", event.zone);
    ESP_LOGI(kTag, "RAW_SR04,L=%.1f,R=%.1f", event.front_left_distance_cm,
             event.front_right_distance_cm);
    ESP_LOGI(kTag, "ROBOT_SR04,LEFT=%.1f,RIGHT=%.1f,LZ=%s,RZ=%s",
             event.RobotLeftDistanceCm(), event.RobotRightDistanceCm(),
             event.RobotLeftZone(), event.RobotRightZone());
    if (self->mission_manager_ != nullptr) self->mission_manager_->HoldForAiObstacle();
    Application::GetInstance().Schedule([self, event]() {
        self->Run(event);
        self->active_.store(false);
    });
}

bool ObstacleAssist::Unsafe(const char* zone) {
    return zone == nullptr || strcmp(zone, "BLOCKED") == 0 ||
           strcmp(zone, "EMERGENCY") == 0 || strcmp(zone, "STALE") == 0;
}

void ObstacleAssist::CopyField(const std::string& text, const char* name,
                               char* output, size_t output_size) {
    std::string wanted(name);
    std::transform(wanted.begin(), wanted.end(), wanted.begin(), ::toupper);
    size_t line_start = 0;
    while (line_start < text.size()) {
        const size_t line_end = text.find_first_of("\r\n", line_start);
        std::string line = text.substr(line_start, line_end - line_start);
        while (!line.empty() && (std::isspace(static_cast<unsigned char>(line.front())) ||
                                 line.front() == '-' || line.front() == '*' || line.front() == '`')) line.erase(0, 1);
        const size_t separator = line.find_first_of("=:");
        if (separator != std::string::npos) {
            std::string key = line.substr(0, separator);
            while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back()))) key.pop_back();
            std::transform(key.begin(), key.end(), key.begin(), ::toupper);
            if (key == wanted) {
                std::string value = line.substr(separator + 1);
                while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) value.erase(0, 1);
                while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) value.pop_back();
                snprintf(output, output_size, "%s", value.c_str());
                return;
            }
        }
        line_start = line_end == std::string::npos ? text.size() : line_end + 1;
    }
}

bool ObstacleAssist::Parse(const std::string& text, Result& result) {
    std::string payload = text;
    cJSON* root = cJSON_Parse(text.c_str());
    if (root != nullptr) {
        const cJSON* wrapped = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(wrapped) && wrapped->valuestring != nullptr) payload = wrapped->valuestring;
    }
    CopyField(payload, "OBJECT", result.object, sizeof(result.object));
    CopyField(payload, "LEFT", result.left, sizeof(result.left));
    CopyField(payload, "RIGHT", result.right, sizeof(result.right));
    CopyField(payload, "ACTION", result.action, sizeof(result.action));
    CopyField(payload, "CONFIDENCE", result.confidence, sizeof(result.confidence));
    CopyField(payload, "DESCRIPTION", result.description, sizeof(result.description));
    if (root != nullptr) cJSON_Delete(root);
    for (char* value : {result.left, result.right, result.action, result.confidence}) {
        for (char* p = value; *p != '\0'; ++p) *p = static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
    }
    const bool valid = (strcmp(result.left, "CLEAR") == 0 || strcmp(result.left, "BLOCKED") == 0 || strcmp(result.left, "UNKNOWN") == 0) &&
        (strcmp(result.right, "CLEAR") == 0 || strcmp(result.right, "BLOCKED") == 0 || strcmp(result.right, "UNKNOWN") == 0) &&
        (strcmp(result.action, "LEFT") == 0 || strcmp(result.action, "RIGHT") == 0 || strcmp(result.action, "BACK") == 0 || strcmp(result.action, "WAIT") == 0 || strcmp(result.action, "ABORT") == 0) &&
        (strcmp(result.confidence, "HIGH") == 0 || strcmp(result.confidence, "MEDIUM") == 0 || strcmp(result.confidence, "LOW") == 0);
    if (!valid) { snprintf(result.action, sizeof(result.action), "WAIT"); snprintf(result.confidence, sizeof(result.confidence), "UNKNOWN"); }
    return valid;
}

void ObstacleAssist::Run(RobotObstacleStatus event) {
    ESP_LOGI(kTag, "EVENT=DETECTED");
    RobotState state;
    const bool stopped = uart_ != nullptr && uart_->GetState(state, 700) && !state.moving && state.left == 0 && state.right == 0;
    const bool stopped_event = event.valid && strcmp(event.zone, "CLEAR") != 0;
    if (!stopped_event || !stopped) {
        ESP_LOGE(kTag, "FAIL_STAGE=STOP_OR_SENSOR FINAL=WAIT"); return;
    }
    ESP_LOGI(kTag, "MOTION_BLOCKED=1,MOTOR_STOPPED=1");
    ESP_LOGI(kTag, "STOP_OR_SENSOR=PASS,EVENT_STOPPED=1,MOTOR_STOPPED=1");
    Application::GetInstance().Schedule([] { Board::GetInstance().GetDisplay()->ShowNotification("OBSTACLE DETECTED"); });
    Mem("BEFORE");
    bool acquired = camera_->TryAcquireVision(50);
    if (!acquired) { ESP_LOGI(kTag, "STATE=WAIT_CAMERA"); acquired = camera_->TryAcquireVision(30000); }
    if (!acquired) { ESP_LOGE(kTag, "FAIL_STAGE=CAMERA_BUSY FINAL=WAIT"); return; }
    struct Release { Camera* c; ~Release() { c->ReleaseVision(); } } release{camera_};
    Result result;
    try {
        ESP_LOGI(kTag, "STATE=CAPTURING");
        if (!camera_->Capture()) { ESP_LOGE(kTag, "FAIL_STAGE=CAPTURE FINAL=WAIT"); return; }
        ESP_LOGI(kTag, "CAPTURE=PASS,STATE=ANALYZING");
        const std::string reply = camera_->Explain(
            "You are viewing the forward image of a mobile robot. Identify the main obstacle and free space in ROBOT coordinates. Return exactly:\nOBJECT=<object>\nLEFT=<CLEAR|BLOCKED|UNKNOWN>\nRIGHT=<CLEAR|BLOCKED|UNKNOWN>\nACTION=<LEFT|RIGHT|BACK|WAIT|ABORT>\nCONFIDENCE=<HIGH|MEDIUM|LOW>\nDESCRIPTION=<short>. Do not control the robot; do not give PWM, speed, or angles.");
        Mem("DURING");
        if (raw_diagnostic_pending_) {
            std::string sanitized;
            sanitized.reserve(std::min<size_t>(reply.size(), 384));
            for (char c : reply) {
                if (sanitized.size() >= 384) break;
                sanitized.push_back((c == '\r' || c == '\n' || c == '\t') ? ' ' : c);
            }
            ESP_LOGI(kTag, "VISION_RAW=%s", sanitized.c_str());
            raw_diagnostic_pending_ = false;
        }
        if (!Parse(reply, result)) { ESP_LOGE(kTag, "FAIL_STAGE=PARSE FINAL=WAIT"); return; }
    } catch (const std::exception& e) { ESP_LOGE(kTag, "FAIL_STAGE=VISION FINAL=WAIT ERR=%s", e.what()); return; }
    RobotObstacleStatus sr04 = event;
    RobotObstacleStatus fresh_sr04;
    const bool fresh_sr04_ok = uart_ != nullptr && uart_->GetObstacle(fresh_sr04, 700) &&
                               fresh_sr04.valid && fresh_sr04.fresh;
    if (fresh_sr04_ok) {
        sr04 = fresh_sr04;
    }
    ESP_LOGI(kTag, "VISION_ACTION=%s", result.action);
    ESP_LOGI(kTag, "RAW_SR04,L=%.1f,R=%.1f", sr04.front_left_distance_cm, sr04.front_right_distance_cm);
    ESP_LOGI(kTag, "ROBOT_SR04,LEFT=%.1f,RIGHT=%.1f,LZ=%s,RZ=%s", sr04.RobotLeftDistanceCm(), sr04.RobotRightDistanceCm(), sr04.RobotLeftZone(), sr04.RobotRightZone());
    snprintf(result.final_suggestion, sizeof(result.final_suggestion), "%s", result.action);
    const bool left_selected = strcmp(result.action, "LEFT") == 0;
    const bool right_selected = strcmp(result.action, "RIGHT") == 0;
    const char* selected_zone = left_selected ? sr04.RobotLeftZone() : sr04.RobotRightZone();
    const bool selected_direction_clear = fresh_sr04_ok &&
        ((left_selected && strcmp(sr04.RobotLeftZone(), "CLEAR") == 0) ||
         (right_selected && strcmp(sr04.RobotRightZone(), "CLEAR") == 0));
    if (strcmp(result.confidence, "LOW") == 0) {
        snprintf(result.final_suggestion, sizeof(result.final_suggestion), "WAIT");
        ESP_LOGI(kTag, "VETO=%s,REASON=CONFIDENCE_LOW", result.action);
    } else if ((left_selected || right_selected) && !selected_direction_clear) {
        snprintf(result.final_suggestion, sizeof(result.final_suggestion), "WAIT");
        ESP_LOGI(kTag, "VETO=%s,REASON=SR04_%s", result.action,
                 fresh_sr04_ok ? selected_zone : "INVALID");
    } else {
        ESP_LOGI(kTag, "VETO=NONE");
    }
    ESP_LOGI(kTag, "FINAL=%s", result.final_suggestion);
    ESP_LOGI(kTag, "VISION=PASS,OBJECT=%s,VLEFT=%s,VRIGHT=%s,ACTION=%s,CONF=%s,FINAL=%s", result.object, result.left, result.right, result.action, result.confidence, result.final_suggestion);
    Application::GetInstance().Schedule([result, sr04] { char msg[192]; snprintf(msg, sizeof(msg), "AI OBSTACLE %s | L:%s R:%s | SUGGEST:%s | SR04 L:%.0f R:%.0f", result.object, result.left, result.right, result.final_suggestion, sr04.RobotLeftDistanceCm(), sr04.RobotRightDistanceCm()); Board::GetInstance().GetDisplay()->ShowNotification(msg, 10000); });
    Mem("AFTER");
    ESP_LOGI(kTag, "STATE=RESULT_READY");
    ESP_LOGI(kTag, "MISSION_RESUME=0");
}

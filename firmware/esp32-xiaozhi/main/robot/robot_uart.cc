#include "robot_uart.h"
#include "application.h"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr const char* kTag = "RobotUart";
constexpr size_t kDriverBufferSize = 1024;

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

uint16_t Crc16Ccitt(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
}  // namespace

bool RobotUart::Begin() {
    if (started_) return true;

    transaction_mutex_ = xSemaphoreCreateMutex();
    tx_mutex_ = xSemaphoreCreateMutex();
    state_mutex_ = xSemaphoreCreateMutex();
    response_events_ = xEventGroupCreate();
    if (transaction_mutex_ == nullptr || tx_mutex_ == nullptr ||
        state_mutex_ == nullptr ||
        response_events_ == nullptr) {
        ESP_LOGE(kTag, "Cannot allocate synchronization objects");
        return false;
    }

    const uart_config_t config = {
        .baud_rate = baud_rate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };
    esp_err_t error = uart_driver_install(port_, kDriverBufferSize, 0, 0,
                                          nullptr, 0);
    if (error == ESP_OK) error = uart_param_config(port_, &config);
    if (error == ESP_OK) {
        error = uart_set_pin(port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE,
                             UART_PIN_NO_CHANGE);
    }
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "UART init failed: %s", esp_err_to_name(error));
        return false;
    }

    uart_flush_input(port_);
    BaseType_t created = xTaskCreatePinnedToCore(
        RxTaskEntry, "robot_uart_rx", 4096, this, 4, &rx_task_, 0);
    if (created != pdPASS) {
        ESP_LOGE(kTag, "Cannot start RX task");
        uart_driver_delete(port_);
        return false;
    }
    started_ = true;
    ESP_LOGI(kTag, "Ready: UART%d RX=%d TX=%d baud=%d",
             static_cast<int>(port_), static_cast<int>(rx_pin_),
             static_cast<int>(tx_pin_), baud_rate_);
    xTaskCreate(LinkTestTaskEntry, "robot_link_test", 3072, this, 2, nullptr);
    xTaskCreatePinnedToCore(HeartbeatTaskEntry, "robot_heartbeat", 2048, this,
                            3, &heartbeat_task_, 0);
    return true;
}

int RobotUart::ClampSpeed(int speed) {
    return std::max(10, std::min(speed, 20));
}

bool RobotUart::SendFrame(const char* body) {
    if (!started_ || body == nullptr) return false;
    if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(30)) != pdTRUE) return false;

    const uint16_t sequence = tx_sequence_++;
    char protected_body[160];
    // RobotLink parser keeps TYPE and PAYLOAD as separate CSV fields. Commands
    // without a payload (PING/STOP/HB) therefore retain an empty trailing
    // payload field, matching the STM32 RobotLink encoder.
    const int body_length = strchr(body, ',') == nullptr
        ? snprintf(protected_body, sizeof(protected_body),
                   "RAI,3,%u,%s,", sequence, body)
        : snprintf(protected_body, sizeof(protected_body),
                   "RAI,3,%u,%s", sequence, body);
    if (body_length <= 0 ||
        static_cast<size_t>(body_length) >= sizeof(protected_body)) {
        xSemaphoreGive(tx_mutex_);
        return false;
    }
    const uint16_t crc = Crc16Ccitt(protected_body,
                                    static_cast<size_t>(body_length));
    char frame[192];
    const int length = snprintf(frame, sizeof(frame), "$%s*%04X\r\n",
                                protected_body, crc);
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(frame)) {
        xSemaphoreGive(tx_mutex_);
        return false;
    }
    const int written = uart_write_bytes(port_, frame, length);
    xSemaphoreGive(tx_mutex_);
    if (written == length) {
        if (strcmp(body, "HB") != 0) {
            ESP_LOGI(kTag, "ROBOT TX V3 seq=%u: %s", sequence, body);
        }
        return true;
    }
    ESP_LOGE(kTag, "UART write failed");
    return false;
}

bool RobotUart::SendAndWait(const char* body, EventBits_t expected,
                            uint32_t timeout_ms) {
    if (!started_ ||
        xSemaphoreTake(transaction_mutex_, pdMS_TO_TICKS(timeout_ms)) !=
            pdTRUE) {
        return false;
    }
    xEventGroupClearBits(response_events_, expected | kResponseNack);
    const bool sent = SendFrame(body);
    EventBits_t bits = 0;
    if (sent) {
        bits = xEventGroupWaitBits(response_events_,
                                   expected | kResponseNack, pdTRUE, pdFALSE,
                                   pdMS_TO_TICKS(timeout_ms));
    }
    xSemaphoreGive(transaction_mutex_);
    return sent && (bits & expected) != 0 &&
           (bits & kResponseNack) == 0;
}

bool RobotUart::Ping(uint32_t timeout_ms) {
    return SendAndWait("PING", kResponsePong, timeout_ms);
}

bool RobotUart::CheckProtocol(uint32_t timeout_ms) {
    protocol_compatible_ = false;
    if (!SendAndWait("HELLO,PROTO,3", kResponseHello, timeout_ms)) {
        return false;
    }
    // HELLO is the new-session boundary; PING completes negotiation before
    // any MODE or motion command may reach the STM32.
    const bool negotiated = Ping(timeout_ms);
    protocol_compatible_ = negotiated;
    return negotiated;
}

bool RobotUart::SetMode(bool ai_mode, uint32_t timeout_ms) {
    if (ai_mode && !protocol_compatible_ && !CheckProtocol(timeout_ms)) {
        ESP_LOGE(kTag, "Protocol mismatch: motion inhibited");
        return false;
    }
    if (!ai_mode) motion_lease_active_ = false;
    const bool ok = SendAndWait(ai_mode ? "MODE,AI" : "MODE,MANUAL",
                                kResponseAck, timeout_ms);
    // An explicitly accepted subsequent AI session is allowed after PS2 has
    // been released; don't retain the prior session's cancellation forever.
    if (ok && ai_mode) ps2_override_active_.store(false);
    return ok;
}

bool RobotUart::MoveForward(int speed, uint32_t timeout_ms) {
    char command[32];
    snprintf(command, sizeof(command), "CMD,FWD,%d", ClampSpeed(speed));
    return SendAndWait(command, kResponseAck, timeout_ms);
}

bool RobotUart::MoveBackward(int speed, uint32_t timeout_ms) {
    char command[32];
    snprintf(command, sizeof(command), "CMD,BACK,%d", ClampSpeed(speed));
    return SendAndWait(command, kResponseAck, timeout_ms);
}

bool RobotUart::TurnLeft(int speed, uint32_t timeout_ms) {
    char command[32];
    snprintf(command, sizeof(command), "CMD,LEFT,%d", ClampSpeed(speed));
    return SendAndWait(command, kResponseAck, timeout_ms);
}

bool RobotUart::TurnRight(int speed, uint32_t timeout_ms) {
    char command[32];
    snprintf(command, sizeof(command), "CMD,RIGHT,%d", ClampSpeed(speed));
    return SendAndWait(command, kResponseAck, timeout_ms);
}

bool RobotUart::StartContinuous(bool forward, int speed,
                                uint32_t timeout_ms) {
    char command[48];
    snprintf(command, sizeof(command), "MOVE,%s,%d,CONT",
             forward ? "FWD" : "BACK", ClampSpeed(speed));
    const bool started = SendAndWait(command, kResponseAck, timeout_ms);
    motion_lease_active_ = started;
    return started;
}

bool RobotUart::StartContinuousRotation(bool left, int speed,
                                        uint32_t timeout_ms) {
    char command[48];
    snprintf(command, sizeof(command), "MOVE,%s,%d,CONT",
             left ? "LEFT" : "RIGHT", ClampSpeed(speed));
    const bool started = SendAndWait(command, kResponseAck, timeout_ms);
    motion_lease_active_ = started;
    return started;
}

bool RobotUart::MoveDistance(bool forward, int distance_mm, int speed,
                             RobotDistanceResult& result,
                             uint32_t timeout_ms) {
    distance_mm = std::max(1, std::min(distance_mm, 5000));
    char command[56];
    snprintf(command, sizeof(command), "MOVE,%s,%d,%d",
             forward ? "FWD" : "BACK", distance_mm, ClampSpeed(speed));
    xEventGroupClearBits(response_events_,
                         kResponseDistanceDone | kResponseDistanceError);
    distance_waiting_ = true;
    const bool started = SendAndWait(command, kResponseAck, 700);
    if (!started) {
        distance_waiting_ = false;
        return false;
    }
    motion_lease_active_ = true;
    const EventBits_t bits = xEventGroupWaitBits(
        response_events_, kResponseDistanceDone | kResponseDistanceError,
        pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    motion_lease_active_ = false;
    distance_waiting_ = false;
    if ((bits & kResponseDistanceDone) == 0 ||
        (bits & kResponseDistanceError) != 0) {
        // STM32 has already handed motor ownership to PS2; don't race the
        // operator with a follow-up STOP.
        if (!Ps2OverrideActive()) Stop(700);
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    result = distance_result_;
    xSemaphoreGive(state_mutex_);
    return result.completed;
}

bool RobotUart::TurnRelative(bool left, int angle_deg, int speed,
                             RobotTurnResult& result,
                             uint32_t timeout_ms) {
    angle_deg = std::max(1, std::min(angle_deg, 180));
    char command[64];
    snprintf(command, sizeof(command), "TURN,REL,%s,%d,%d",
             left ? "LEFT" : "RIGHT", angle_deg, ClampSpeed(speed));
    xEventGroupClearBits(response_events_,
                         kResponseTurnDone | kResponseTurnError);
    turn_waiting_ = true;
    const bool started = SendAndWait(command, kResponseAck, 700);
    if (!started) {
        turn_waiting_ = false;
        return false;
    }
    motion_lease_active_ = true;
    const EventBits_t bits = xEventGroupWaitBits(
        response_events_, kResponseTurnDone | kResponseTurnError, pdTRUE,
        pdFALSE, pdMS_TO_TICKS(timeout_ms));
    motion_lease_active_ = false;
    turn_waiting_ = false;
    if ((bits & kResponseTurnDone) == 0 ||
        (bits & kResponseTurnError) != 0) {
        if (!Ps2OverrideActive()) Stop(700);
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    result = turn_result_;
    xSemaphoreGive(state_mutex_);
    return result.completed;
}

bool RobotUart::TurnAbsolute(int heading_deg, int speed,
                             RobotTurnResult& result,
                             uint32_t timeout_ms) {
    heading_deg = std::max(-180, std::min(heading_deg, 180));
    char command[56];
    snprintf(command, sizeof(command), "TURN,ABS,%d,%d", heading_deg,
             ClampSpeed(speed));
    xEventGroupClearBits(response_events_,
                         kResponseTurnDone | kResponseTurnError);
    turn_waiting_ = true;
    const bool started = SendAndWait(command, kResponseAck, 700);
    if (!started) {
        turn_waiting_ = false;
        return false;
    }
    motion_lease_active_ = true;
    const EventBits_t bits = xEventGroupWaitBits(
        response_events_, kResponseTurnDone | kResponseTurnError, pdTRUE,
        pdFALSE, pdMS_TO_TICKS(timeout_ms));
    motion_lease_active_ = false;
    turn_waiting_ = false;
    if ((bits & kResponseTurnDone) == 0 ||
        (bits & kResponseTurnError) != 0) {
        if (!Ps2OverrideActive()) Stop(700);
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    result = turn_result_;
    xSemaphoreGive(state_mutex_);
    return result.completed;
}

bool RobotUart::Stop(uint32_t timeout_ms) {
    motion_lease_active_ = false;
    return SendAndWait("STOP", kResponseStopDone, timeout_ms);
}

bool RobotUart::GetState(RobotState& state, uint32_t timeout_ms) {
    if (!SendAndWait("GET,STATE", kResponseState, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    state = state_;
    xSemaphoreGive(state_mutex_);
    return state.valid;
}

bool RobotUart::GetSpeed(int& speed, uint32_t timeout_ms) {
    if (!SendAndWait("GET,SPEED", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    speed = value_speed_;
    xSemaphoreGive(state_mutex_);
    return true;
}

bool RobotUart::SetSpeed(int speed, uint32_t timeout_ms) {
    if (speed < 10 || speed > 255) return false;
    char command[32];
    snprintf(command, sizeof(command), "SET,SPEED,%d", speed);
    return SendAndWait(command, kResponseAck, timeout_ms);
}

bool RobotUart::GetBrake(bool& enabled, uint32_t timeout_ms) {
    if (!SendAndWait("GET,BRAKE", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    enabled = value_brake_;
    xSemaphoreGive(state_mutex_);
    return true;
}

bool RobotUart::SetBrake(bool enabled, uint32_t timeout_ms) {
    return SendAndWait(enabled ? "SET,BRAKE,ON" : "SET,BRAKE,OFF",
                       kResponseAck, timeout_ms);
}

bool RobotUart::GetRamp(bool& enabled, uint32_t timeout_ms) {
    if (!SendAndWait("GET,RAMP", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    enabled = value_ramp_;
    xSemaphoreGive(state_mutex_);
    return true;
}

bool RobotUart::SetRamp(bool enabled, uint32_t timeout_ms) {
    return SendAndWait(enabled ? "SET,RAMP,ON" : "SET,RAMP,OFF",
                       kResponseAck, timeout_ms);
}

bool RobotUart::GetHeading(float& heading_deg, uint32_t timeout_ms) {
    if (!SendAndWait("GET,HEADING", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    heading_deg = value_heading_deg_;
    xSemaphoreGive(state_mutex_);
    return true;
}

bool RobotUart::GetCompassStatus(RobotCompassStatus& status,
                                 uint32_t timeout_ms) {
    if (!SendAndWait("GET,COMPASS_STATUS", kResponseValue, timeout_ms)) {
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = compass_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::ResetCompass(uint32_t timeout_ms) {
    if (!started_ ||
        xSemaphoreTake(transaction_mutex_, pdMS_TO_TICKS(timeout_ms)) !=
            pdTRUE) {
        return false;
    }
    xEventGroupClearBits(response_events_,
                         kResponseAck | kResponseNack |
                             kResponseCompassZeroed);
    bool success = SendFrame("COMPASS,RESET");
    EventBits_t bits = 0;
    if (success) {
        bits = xEventGroupWaitBits(response_events_,
                                   kResponseAck | kResponseNack, pdTRUE,
                                   pdFALSE, pdMS_TO_TICKS(700));
        success = (bits & kResponseAck) != 0 &&
                  (bits & kResponseNack) == 0;
    }
    // The STM32 applies the same resetZero()/heading.reset() operation used
    // by the physical L2 button before it emits the ACK.  A disconnected
    // compass cannot produce the optional ZEROED sample event and may later
    // emit COMPASS,LOST, but that must not turn an already acknowledged reset
    // into a voice-tool failure.  ACK is the authoritative result here.
    if (success) {
        (void)xEventGroupWaitBits(response_events_,
                                  kResponseCompassZeroed | kResponseNack,
                                  pdTRUE, pdFALSE,
                                  pdMS_TO_TICKS(120));
    }
    xSemaphoreGive(transaction_mutex_);
    return success;
}

bool RobotUart::ResetEncoders(uint32_t timeout_ms) {
    return SendAndWait("ENCODER,RESET", kResponseAck, timeout_ms);
}

bool RobotUart::GetPs2Status(RobotPs2Status& status, uint32_t timeout_ms) {
    if (!SendAndWait("PS2,STATUS", kResponsePs2, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = ps2_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::GetObstacle(RobotObstacleStatus& status,
                            uint32_t timeout_ms) {
    if (!SendAndWait("GET,OBSTACLE", kResponseObstacle, timeout_ms)) {
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = obstacle_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::GetOdometry(RobotOdometry& odometry, uint32_t timeout_ms) {
    if (!SendAndWait("GET,ODOMETRY", kResponseOdometry, timeout_ms)) {
        return false;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    odometry = odometry_;
    xSemaphoreGive(state_mutex_);
    return odometry.valid;
}

bool RobotUart::GetEncoderStatus(RobotEncoderStatus& status,
                                 uint32_t timeout_ms) {
    if (!SendAndWait("GET,ENCODER", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = encoder_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::GetImuStatus(RobotImuStatus& status, uint32_t timeout_ms) {
    if (!SendAndWait("GET,IMU", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = imu_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::GetFusionStatus(RobotFusionStatus& status,
                                uint32_t timeout_ms) {
    if (!SendAndWait("GET,FUSION", kResponseValue, timeout_ms)) return false;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    status = fusion_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::GetCachedObstacle(RobotObstacleStatus& status) {
    if (!started_ ||
        xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    status = obstacle_status_;
    xSemaphoreGive(state_mutex_);
    return status.valid;
}

bool RobotUart::Ps2OverrideActive() const {
    return ps2_override_active_.load();
}

bool RobotUart::IsConnected() const {
    const uint32_t last_rx = last_rx_ms_;
    return last_rx != 0 && (NowMs() - last_rx) <= 1500;
}

void RobotUart::HandleFrame(const char* frame) {
    last_rx_ms_ = NowMs();
    ESP_LOGI(kTag, "ROBOT RX: <%s>", frame);
    if (strcmp(frame, "PONG") == 0) {
        xEventGroupSetBits(response_events_, kResponsePong);
        return;
    }
    if (strncmp(frame, "HELLO,STM32,ROBOT_AI,PROTO,3", sizeof("HELLO,STM32,ROBOT_AI,PROTO,3") - 1U) == 0) {
        protocol_compatible_ = true;
        xEventGroupSetBits(response_events_, kResponseHello);
        return;
    }
    if (strncmp(frame, "ACK,", 4) == 0) {
        xEventGroupSetBits(response_events_, kResponseAck);
        return;
    }
    if (strcmp(frame, "DONE,STOP") == 0) {
        xEventGroupSetBits(response_events_, kResponseStopDone);
        return;
    }
    float distance_target = 0.0f;
    float distance_travelled = 0.0f;
    if (sscanf(frame, "DONE,MOVE,TARGET,%f,TRAVEL,%f", &distance_target,
               &distance_travelled) == 2) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            distance_result_.completed = true;
            distance_result_.target_mm = distance_target;
            distance_result_.travelled_mm = distance_travelled;
            xSemaphoreGive(state_mutex_);
        }
        motion_lease_active_ = false;
        xEventGroupSetBits(response_events_, kResponseDistanceDone);
        return;
    }
    if (sscanf(frame, "ERR,MOVE,%*[^,],TRAVEL,%f", &distance_travelled) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            distance_result_.completed = false;
            distance_result_.travelled_mm = distance_travelled;
            xSemaphoreGive(state_mutex_);
        }
        motion_lease_active_ = false;
        xEventGroupSetBits(response_events_, kResponseDistanceError);
        return;
    }
    if (strncmp(frame, "ERR,MOVE,", 9) == 0) {
        motion_lease_active_ = false;
        xEventGroupSetBits(response_events_, kResponseDistanceError);
        return;
    }
    float turn_heading = 0.0f;
    float turn_target = 0.0f;
    float turn_error = 0.0f;
    if (sscanf(frame, "DONE,TURN,H,%f,TGT,%f,ERR,%f", &turn_heading,
               &turn_target, &turn_error) == 3) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            turn_result_.completed = true;
            turn_result_.heading_deg = turn_heading;
            turn_result_.target_deg = turn_target;
            turn_result_.error_deg = turn_error;
            xSemaphoreGive(state_mutex_);
        }
        motion_lease_active_ = false;
        xEventGroupSetBits(response_events_, kResponseTurnDone);
        return;
    }
    if (strncmp(frame, "PROGRESS,TURN,", 14) == 0) return;
    char obstacle_zone[12] = {};
    float obstacle_distance = 0.0f;
    const bool obstacle_detected =
        sscanf(frame, "EVENT,OBSTACLE,DETECTED,ZONE,%11[^,],DIST,%f",
               obstacle_zone, &obstacle_distance) == 2;
    const bool obstacle_stopped = !obstacle_detected &&
        sscanf(frame, "EVENT,OBSTACLE,STOPPED,ZONE,%11[^,],DIST,%f",
               obstacle_zone, &obstacle_distance) == 2;
    if (obstacle_detected || obstacle_stopped) {
        const bool was_motion_active = motion_lease_active_;
        // DETECTED is telemetry, not a motion-lease cancellation. In
        // particular, the front sensor can report CAUTION while an avoidance
        // turn is sweeping past an object. Stopping heartbeat here makes the
        // STM32 abort that turn with MOTION_LEASE_TIMEOUT. A genuine STOPPED event
        // ends a translational lease, but rotation remains safe and must keep
        // its heartbeat until DONE,TURN or ERR,TURN.
        if (obstacle_stopped && !turn_waiting_) {
            motion_lease_active_ = false;
            if (distance_waiting_) {
                xEventGroupSetBits(response_events_, kResponseDistanceError);
            }
        }
        const bool urgent = strcmp(obstacle_zone, "BLOCKED") == 0 ||
                            strcmp(obstacle_zone, "EMERGENCY") == 0;
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            obstacle_status_.valid = true;
            obstacle_status_.fresh = true;
            obstacle_status_.echo_valid = true;
            obstacle_status_.distance_cm = obstacle_distance;
            strncpy(obstacle_status_.zone, obstacle_zone,
                    sizeof(obstacle_status_.zone) - 1);
            obstacle_status_.zone[sizeof(obstacle_status_.zone) - 1] = '\0';
            obstacle_status_.limited = urgent;
            xSemaphoreGive(state_mutex_);
        }
        if (urgent) {
            Application::GetInstance().Schedule(
                [obstacle_distance, zone = std::string(obstacle_zone)]() {
                    char message[128];
                    snprintf(message, sizeof(message),
                             "Robot đã tự hãm: vật cản phía trước cách %.0f cm (%s).",
                             obstacle_distance, zone.c_str());
                    Application::GetInstance().Alert(
                        "Cảnh báo vật cản", message, "surprised");
                });
        }
        if (obstacle_stopped && obstacle_stopped_callback_ != nullptr) {
            obstacle_stopped_callback_(obstacle_stopped_context_, obstacle_status_, was_motion_active);
        }
        return;
    }
    if (sscanf(frame, "EVENT,OBSTACLE,CLEAR,DIST,%f",
               &obstacle_distance) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            obstacle_status_.valid = true;
            obstacle_status_.fresh = true;
            obstacle_status_.echo_valid = true;
            obstacle_status_.distance_cm = obstacle_distance;
            snprintf(obstacle_status_.zone, sizeof(obstacle_status_.zone),
                     "%s", "CLEAR");
            obstacle_status_.limited = false;
            xSemaphoreGive(state_mutex_);
        }
        return;
    }
    if (strncmp(frame, "EVENT,ENCODER,FAULT,", 20) == 0) {
        motion_lease_active_ = false;
        if (turn_waiting_) {
            xEventGroupSetBits(response_events_, kResponseTurnError);
        }
        if (distance_waiting_) {
            xEventGroupSetBits(response_events_, kResponseDistanceError);
        }
        return;
    }
    const bool motion_aborted =
        strcmp(frame, "EVENT,AI_CANCELLED,PS2_OVERRIDE") == 0 ||
        strcmp(frame, "EVENT,STOP,MOTION_LEASE_TIMEOUT") == 0;
    if (motion_aborted) {
        if (strcmp(frame, "EVENT,AI_CANCELLED,PS2_OVERRIDE") == 0) {
            ps2_override_active_.store(true);
        }
        motion_lease_active_ = false;
        if (turn_waiting_)
            xEventGroupSetBits(response_events_, kResponseTurnError);
        if (distance_waiting_)
            xEventGroupSetBits(response_events_, kResponseDistanceError);
        return;
    }
    if (strncmp(frame, "ERR,TURN,", 9) == 0 ||
        (turn_waiting_ && (strcmp(frame, "ERR,COMPASS,LOST") == 0 ||
                           strcmp(frame, "ERR,HEADING,LOST") == 0))) {
        motion_lease_active_ = false;
        xEventGroupSetBits(response_events_, kResponseTurnError);
        return;
    }
    if (strncmp(frame, "DONE,", 5) == 0) {
        xEventGroupSetBits(response_events_, kResponseDone);
        return;
    }
    if (strncmp(frame, "NACK,", 5) == 0 ||
        strncmp(frame, "ERR,", 4) == 0) {
        xEventGroupSetBits(response_events_, kResponseNack);
        return;
    }
    float zero_heading = 0.0f;
    if (sscanf(frame, "EVENT,COMPASS,ZEROED,H,%f", &zero_heading) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            value_heading_deg_ = zero_heading;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseCompassZeroed);
        return;
    }

    int int_value = 0;
    float float_value = 0.0f;
    char text_value[8] = {};
    if (sscanf(frame, "VALUE,SPEED,%d", &int_value) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            value_speed_ = int_value;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    if (sscanf(frame, "VALUE,BRAKE,%7s", text_value) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            value_brake_ = strcmp(text_value, "ON") == 0;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    if (sscanf(frame, "VALUE,RAMP,%7s", text_value) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            value_ramp_ = strcmp(text_value, "ON") == 0;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    if (sscanf(frame, "VALUE,HEADING,%f", &float_value) == 1) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            value_heading_deg_ = float_value;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    int calibrating = 0;
    if (sscanf(frame, "VALUE,COMPASS,%7[^,],CAL,%d,H,%f", text_value,
               &calibrating, &float_value) == 3) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            compass_status_.valid = true;
            compass_status_.connected = strcmp(text_value, "OK") == 0;
            compass_status_.calibrating = calibrating != 0;
            compass_status_.heading_deg = float_value;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    int encoder_ready = 0;
    char encoder_health[20] = {};
    float left_velocity = 0.0f;
    float right_velocity = 0.0f;
    if (sscanf(frame,
               "VALUE,ENCODER,READY,%d,HEALTH,%19[^,],LV,%f,RV,%f",
               &encoder_ready, encoder_health, &left_velocity,
               &right_velocity) == 4) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            encoder_status_.valid = true;
            encoder_status_.ready = encoder_ready != 0;
            strncpy(encoder_status_.health, encoder_health,
                    sizeof(encoder_status_.health) - 1);
            encoder_status_.health[sizeof(encoder_status_.health) - 1] = '\0';
            encoder_status_.left_velocity_mm_s = left_velocity;
            encoder_status_.right_velocity_mm_s = right_velocity;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    int imu_ready = 0;
    int imu_calibrated = 0;
    char imu_health[20] = {};
    float imu_gz = 0.0f;
    float imu_ax = 0.0f;
    float imu_ay = 0.0f;
    float imu_az = 0.0f;
    if (sscanf(frame,
               "VALUE,IMU,READY,%d,CAL,%d,HEALTH,%19[^,],GZ,%f,AX,%f,AY,%f,AZ,%f",
               &imu_ready, &imu_calibrated, imu_health, &imu_gz, &imu_ax,
               &imu_ay, &imu_az) == 7) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            imu_status_.valid = true;
            imu_status_.ready = imu_ready != 0;
            imu_status_.calibrated = imu_calibrated != 0;
            strncpy(imu_status_.health, imu_health,
                    sizeof(imu_status_.health) - 1);
            imu_status_.health[sizeof(imu_status_.health) - 1] = '\0';
            imu_status_.gyro_z_dps = imu_gz;
            imu_status_.accel_x_g = imu_ax;
            imu_status_.accel_y_g = imu_ay;
            imu_status_.accel_z_g = imu_az;
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    int fusion_ready = 0;
    char fusion_health[20] = {};
    char fusion_source[20] = {};
    float fusion_heading = 0.0f;
    float fusion_rate = 0.0f;
    float fusion_confidence = 0.0f;
    if (sscanf(frame,
               "VALUE,FUSION,READY,%d,HEALTH,%19[^,],H,%f,RATE,%f,CONF,%f,SRC,%19s",
               &fusion_ready, fusion_health, &fusion_heading, &fusion_rate,
               &fusion_confidence, fusion_source) == 6) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            fusion_status_.valid = true;
            fusion_status_.ready = fusion_ready != 0;
            strncpy(fusion_status_.health, fusion_health,
                    sizeof(fusion_status_.health) - 1);
            fusion_status_.health[sizeof(fusion_status_.health) - 1] = '\0';
            fusion_status_.heading_deg = fusion_heading;
            fusion_status_.yaw_rate_dps = fusion_rate;
            fusion_status_.confidence_pct = fusion_confidence;
            strncpy(fusion_status_.source, fusion_source,
                    sizeof(fusion_status_.source) - 1);
            fusion_status_.source[sizeof(fusion_status_.source) - 1] = '\0';
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseValue);
        return;
    }
    int enabled = 0;
    int fresh = 0;
    unsigned long age = 0;
    unsigned int buttons = 0;
    const int ps2_fields = sscanf(
        frame,
        "PS2,STATE,%7[^,],ENABLED,%d,FRESH,%d,AGE,%lu,MODE,%*x,BTN,%x",
        text_value, &enabled, &fresh, &age, &buttons);
    if (ps2_fields >= 4) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            ps2_status_.valid = true;
            strncpy(ps2_status_.state, text_value,
                    sizeof(ps2_status_.state) - 1);
            ps2_status_.state[sizeof(ps2_status_.state) - 1] = '\0';
            ps2_status_.enabled = enabled != 0;
            ps2_status_.fresh = fresh != 0;
            ps2_status_.age_ms = static_cast<uint32_t>(age);
            ps2_status_.buttons_valid = ps2_fields == 5;
            ps2_status_.buttons = static_cast<uint16_t>(buttons);
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponsePs2);
        return;
    }
    int echo_valid = 0;
    int limited = 0;
    float front_left = 0.0f;
    float front_right = 0.0f;
    char front_left_zone[12] = {};
    char front_right_zone[12] = {};
    char suggested[8] = {};
    if (sscanf(frame,
               "VALUE,OBSTACLE,FRESH,%d,ECHO,%d,DIST,%f,RATE,%f,ZONE,%11[^,],LIMIT,%d,LEFT,%f,RIGHT,%f,LZ,%11[^,],RZ,%11[^,],NEAREST,%*f,SUG,%7s",
               &fresh, &echo_valid, &float_value,
               &obstacle_distance, obstacle_zone, &limited, &front_left,
               &front_right, front_left_zone, front_right_zone, suggested) >= 6) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            obstacle_status_.valid = true;
            obstacle_status_.fresh = fresh != 0;
            obstacle_status_.echo_valid = echo_valid != 0;
            obstacle_status_.distance_cm = float_value;
            obstacle_status_.approach_rate_cm_s = obstacle_distance;
            strncpy(obstacle_status_.zone, obstacle_zone,
                    sizeof(obstacle_status_.zone) - 1);
            obstacle_status_.zone[sizeof(obstacle_status_.zone) - 1] = '\0';
            obstacle_status_.limited = limited != 0;
            obstacle_status_.front_left_distance_cm = front_left;
            obstacle_status_.front_right_distance_cm = front_right;
            strncpy(obstacle_status_.front_left_zone, front_left_zone,
                    sizeof(obstacle_status_.front_left_zone) - 1);
            strncpy(obstacle_status_.front_right_zone, front_right_zone,
                    sizeof(obstacle_status_.front_right_zone) - 1);
            strncpy(obstacle_status_.suggested_avoidance, suggested,
                    sizeof(obstacle_status_.suggested_avoidance) - 1);
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseObstacle);
        return;
    }
    char odometry_distance_text[24] = {};
    char odometry_x_text[24] = {};
    char odometry_y_text[24] = {};
    char odometry_heading_text[24] = {};
    char left_ticks_text[24] = {};
    char right_ticks_text[24] = {};
    if (sscanf(frame,
               "VALUE,ODOMETRY,DIST,%23[^,],X,%23[^,],Y,%23[^,],H,%23[^,],LT,%23[^,],RT,%23s",
               odometry_distance_text, odometry_x_text, odometry_y_text,
               odometry_heading_text, left_ticks_text, right_ticks_text) == 6) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            odometry_.valid = true;
            odometry_.distance_mm = strtof(odometry_distance_text, nullptr);
            odometry_.x_mm = strtof(odometry_x_text, nullptr);
            odometry_.y_mm = strtof(odometry_y_text, nullptr);
            odometry_.heading_rad = strtof(odometry_heading_text, nullptr);
            odometry_.left_ticks = strtoll(left_ticks_text, nullptr, 10);
            odometry_.right_ticks = strtoll(right_ticks_text, nullptr, 10);
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseOdometry);
        return;
    }
    if (strncmp(frame, "STATE,", 6) != 0) return;

    char mode[8] = {};
    char brake[4] = {};
    char ramp[4] = {};
    char ps2[8] = {};
    char compass[8] = {};
    char ai_link[8] = {};
    int speed = 0;
    int left = 0;
    int right = 0;
    int moving = 0;
    float heading = 0.0f;
    if (sscanf(frame,
               "STATE,MODE,%7[^,],SPEED,%d,BRAKE,%3[^,],RAMP,%3[^,],H,%f,L,%d,R,%d,MOVE,%d,PS2,%7[^,],COMPASS,%7[^,],AI_LINK,%7s",
               mode, &speed, brake, ramp, &heading, &left, &right, &moving,
               ps2, compass, ai_link) == 11) {
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            state_.valid = true;
            state_.ai_mode = strcmp(mode, "AI") == 0;
            state_.heading_deg = heading;
            state_.speed = speed;
            state_.left = left;
            state_.right = right;
            state_.moving = moving != 0;
            state_.brake_enabled = strcmp(brake, "ON") == 0;
            state_.ramp_enabled = strcmp(ramp, "ON") == 0;
            state_.ps2_ok = strcmp(ps2, "LOST") != 0;
            state_.compass_ok = strcmp(compass, "OK") == 0;
            state_.received_at_ms = NowMs();
            xSemaphoreGive(state_mutex_);
        }
        xEventGroupSetBits(response_events_, kResponseState);
        return;
    }

    int legacy_heading = 0;
    if (sscanf(frame, "STATE,%7[^,],H,%d,S,%d,L,%d,R,%d,MOVE,%d", mode,
               &legacy_heading, &speed, &left, &right, &moving) != 6) {
        ESP_LOGW(kTag, "Invalid STATE frame");
        return;
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        state_.valid = true;
        state_.ai_mode = strcmp(mode, "AI") == 0;
        state_.heading_deg = static_cast<float>(legacy_heading);
        state_.speed = speed;
        state_.left = left;
        state_.right = right;
        state_.moving = moving != 0;
        state_.received_at_ms = NowMs();
        xSemaphoreGive(state_mutex_);
    }
    xEventGroupSetBits(response_events_, kResponseState);
}

void RobotUart::Update() {
    uint8_t bytes[64];
    const int count = uart_read_bytes(port_, bytes, sizeof(bytes), 0);
    for (int i = 0; i < count; ++i) {
        const char value = static_cast<char>(bytes[i]);
        if (value == '<') {
            receiving_ = true;
            rx_length_ = 0;
            continue;
        }
        if (!receiving_) continue;
        if (value == '>') {
            rx_frame_[rx_length_] = '\0';
            receiving_ = false;
            HandleFrame(rx_frame_);
            continue;
        }
        if (value == '\r' || value == '\n') continue;
        if (rx_length_ >= sizeof(rx_frame_) - 1) {
            receiving_ = false;
            rx_length_ = 0;
            continue;
        }
        rx_frame_[rx_length_++] = value;
    }
}

void RobotUart::RxTaskEntry(void* context) {
    static_cast<RobotUart*>(context)->RxTask();
}

void RobotUart::RxTask() {
    while (true) {
        Update();
        // pdMS_TO_TICKS(2) can round down to zero when the RTOS tick is 10 ms,
        // starving IDLE0 and triggering the task watchdog. One tick always
        // yields while keeping UART latency below a scheduler tick.
        vTaskDelay(1);
    }
}

void RobotUart::HeartbeatTaskEntry(void* context) {
    static_cast<RobotUart*>(context)->HeartbeatTask();
}

void RobotUart::HeartbeatTask() {
    uint32_t last_heartbeat_ms = 0;
    while (true) {
        const uint32_t now = NowMs();
        if (motion_lease_active_ &&
            (now - last_heartbeat_ms) >= 200U) {
            if (SendFrame("HB")) last_heartbeat_ms = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void RobotUart::LinkTestTaskEntry(void* context) {
    static_cast<RobotUart*>(context)->RunLinkTest();
    vTaskDelete(nullptr);
}

void RobotUart::RunLinkTest() {
    vTaskDelay(pdMS_TO_TICKS(800));
    const bool protocol_ok = CheckProtocol(700);
    ESP_LOGI(kTag, "SELFTEST PROTOCOL V3: %s",
             protocol_ok ? "PASS" : "FAIL");
    if (!protocol_ok) return;
    ESP_LOGI(kTag, "SELFTEST HELLO/PING: PASS");
    RobotState state;
    const bool state_ok = GetState(state, 700);
    ESP_LOGI(kTag, "SELFTEST GET/STATE: %s", state_ok ? "PASS" : "FAIL");
    if (!state_ok) return;

    int speed = 0;
    const bool speed_get = GetSpeed(speed, 700);
    const bool speed_set = speed_get && SetSpeed(speed, 700);
    int speed_verify = 0;
    const bool speed_ok = speed_set && GetSpeed(speed_verify, 700) &&
                          speed_verify == speed;
    ESP_LOGI(kTag, "SELFTEST SPEED GET/SET: %s value=%d",
             speed_ok ? "PASS" : "FAIL", speed_verify);

    bool brake = false;
    const bool brake_get = GetBrake(brake, 700);
    const bool brake_test_value = !brake;
    const bool brake_set = brake_get && SetBrake(brake_test_value, 700);
    vTaskDelay(pdMS_TO_TICKS(180));
    bool brake_verify = false;
    const bool brake_toggled = brake_set && GetBrake(brake_verify, 700) &&
                               brake_verify == brake_test_value;
    const bool brake_restore = brake_toggled && SetBrake(brake, 700);
    bool brake_restored_value = !brake;
    const bool brake_ok = brake_restore && GetBrake(brake_restored_value, 700) &&
                          brake_restored_value == brake;
    ESP_LOGI(kTag, "SELFTEST BRAKE LOCK/RESTORE: %s tested=%s restored=%s",
             brake_ok ? "PASS" : "FAIL", brake_verify ? "ON" : "OFF",
             brake_restored_value ? "ON" : "OFF");

    bool ramp = false;
    const bool ramp_get = GetRamp(ramp, 700);
    const bool ramp_set = ramp_get && SetRamp(ramp, 700);
    bool ramp_verify = false;
    const bool ramp_ok = ramp_set && GetRamp(ramp_verify, 700) &&
                         ramp_verify == ramp;
    ESP_LOGI(kTag, "SELFTEST RAMP GET/SET: %s value=%s",
             ramp_ok ? "PASS" : "FAIL", ramp_verify ? "ON" : "OFF");

    float heading = 0.0f;
    const bool heading_ok = GetHeading(heading, 700);
    ESP_LOGI(kTag, "SELFTEST GET HEADING: %s value=%.1f",
             heading_ok ? "PASS" : "FAIL", heading);

    RobotCompassStatus compass;
    const bool compass_ok = GetCompassStatus(compass, 700);
    ESP_LOGI(kTag, "SELFTEST COMPASS STATUS: %s connected=%d cal=%d",
             compass_ok ? "PASS" : "FAIL", compass.connected,
             compass.calibrating);

    RobotPs2Status ps2;
    const bool ps2_ok = GetPs2Status(ps2, 700);
    ESP_LOGI(kTag,
             "SELFTEST PS2 STATUS: %s state=%s enabled=%d fresh=%d age=%lu",
             ps2_ok ? "PASS" : "FAIL", ps2.state, ps2.enabled, ps2.fresh,
             static_cast<unsigned long>(ps2.age_ms));

    RobotObstacleStatus obstacle;
    const bool obstacle_ok = GetObstacle(obstacle, 700);
    ESP_LOGI(kTag,
             "SELFTEST ULTRASONIC: %s fresh=%d echo=%d distance=%.1f zone=%s",
             obstacle_ok && obstacle.fresh ? "PASS" : "FAIL",
             obstacle.fresh, obstacle.echo_valid, obstacle.distance_cm,
             obstacle.zone);
    if (obstacle_ok && obstacle.fresh &&
        (strcmp(obstacle.zone, "BLOCKED") == 0 ||
         strcmp(obstacle.zone, "EMERGENCY") == 0)) {
        const bool ai_guard_mode = SetMode(true, 700);
        const bool forward_rejected = ai_guard_mode && !MoveForward(10, 700);
        RobotState guarded_state;
        const bool still_stopped = GetState(guarded_state, 700) &&
                                   !guarded_state.moving &&
                                   guarded_state.left == 0 &&
                                   guarded_state.right == 0;
        const bool guard_restore = SetMode(false, 700);
        ESP_LOGI(kTag,
                 "SELFTEST OBSTACLE FORWARD GUARD: %s rejected=%d stopped=%d",
                 forward_rejected && still_stopped && guard_restore
                     ? "PASS" : "FAIL",
                 forward_rejected, still_stopped);
    } else {
        ESP_LOGI(kTag,
                 "SELFTEST OBSTACLE FORWARD GUARD: SKIP (path clear/no echo)");
    }

    const bool compass_reset_ok = compass_ok && compass.connected &&
                                  ResetCompass(2200);
    ESP_LOGI(kTag, "SELFTEST COMPASS RESET/ZERO EVENT: %s",
             compass_reset_ok ? "PASS" : "FAIL");

    const bool mode_ack = SetMode(true, 700);
    RobotState ai_state;
    const bool mode_state = mode_ack && GetState(ai_state, 700) &&
                            ai_state.ai_mode && !ai_state.moving;
    ESP_LOGI(kTag, "SELFTEST MODE AI (stopped): %s",
             mode_state ? "PASS" : "FAIL");

    const bool stop_ack = Stop(700);
    vTaskDelay(pdMS_TO_TICKS(50));
    RobotState stopped_state;
    const bool stop_state = stop_ack && GetState(stopped_state, 700) &&
                            !stopped_state.moving &&
                            stopped_state.left == 0 &&
                            stopped_state.right == 0;
    ESP_LOGI(kTag, "SELFTEST STOP: %s", stop_state ? "PASS" : "FAIL");

    const bool manual_ok = SetMode(false, 700);
    ESP_LOGI(kTag, "SELFTEST RESTORE MANUAL: %s",
             manual_ok ? "PASS" : "FAIL");
}

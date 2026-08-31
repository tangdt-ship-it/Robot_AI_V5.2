#ifndef XIAOZHI_ROBOT_UART_H
#define XIAOZHI_ROBOT_UART_H

#include "safety_blackbox.h"

#include <cstddef>
#include <atomic>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

struct RobotState {
    bool valid = false;
    bool ai_mode = false;
    float heading_deg = 0.0f;
    int speed = 0;
    int left = 0;
    int right = 0;
    bool moving = false;
    bool brake_enabled = false;
    bool ramp_enabled = false;
    bool compass_ok = false;
    bool ps2_ok = false;
    char motion_owner[12] = "UNKNOWN";
    uint32_t received_at_ms = 0;
};

struct RobotCompassStatus {
    bool valid = false;
    bool connected = false;
    bool calibrating = false;
    float heading_deg = 0.0f;
};

struct RobotPs2Status {
    bool valid = false;
    char state[8] = {};
    bool enabled = false;
    bool fresh = false;
    uint32_t age_ms = 0;
    bool buttons_valid = false;
    uint16_t buttons = 0xFFFF;
};

struct RobotObstacleStatus {
    bool valid = false;
    bool fresh = false;
    bool echo_valid = false;
    char health[24] = "UNKNOWN";
    float distance_cm = 0.0f;
    float approach_rate_cm_s = 0.0f;
    char zone[12] = {};
    bool limited = false;
    float front_left_distance_cm = 0.0f;
    float front_right_distance_cm = 0.0f;
    char front_left_zone[12] = {};
    char front_right_zone[12] = {};
    char front_left_health[24] = "UNKNOWN";
    char front_right_health[24] = "UNKNOWN";
    uint32_t front_left_age_ms = 0;
    uint32_t front_right_age_ms = 0;
    uint32_t encoder_reset_generation = 0;
    char suggested_avoidance[8] = {};
    float RobotLeftDistanceCm() const { return front_right_distance_cm; }
    float RobotRightDistanceCm() const { return front_left_distance_cm; }
    const char* RobotLeftZone() const { return front_right_zone; }
    const char* RobotRightZone() const { return front_left_zone; }
};

struct RobotTurnResult {
    bool completed = false;
    uint32_t session_id = 0;
    uint32_t operation_id = 0;
    float heading_deg = 0.0f;
    float target_deg = 0.0f;
    float error_deg = 0.0f;
};

struct RobotDistanceResult {
    enum class Code : uint8_t {
        NONE,
        DONE,
        TIMEOUT,
        OBSTACLE,
        ENCODER_FAULT,
        LINK_ERROR,
    };
    Code code = Code::NONE;
    bool completed = false;
    uint32_t session_id = 0;
    uint32_t operation_id = 0;
    float target_mm = 0.0f;
    float travelled_mm = 0.0f;
};

struct RobotOdometry {
    bool valid = false;
    float distance_mm = 0.0f;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float heading_rad = 0.0f;
    int64_t left_ticks = 0;
    int64_t right_ticks = 0;
    uint32_t reset_generation = 0;
};

struct RobotEncoderStatus {
    bool valid = false;
    bool ready = false;
    char health[20] = {};
    float left_velocity_mm_s = 0.0f;
    float right_velocity_mm_s = 0.0f;
};

struct RobotImuStatus {
    bool valid = false;
    bool ready = false;
    bool calibrated = false;
    char health[20] = {};
    float gyro_z_dps = 0.0f;
    float accel_x_g = 0.0f;
    float accel_y_g = 0.0f;
    float accel_z_g = 0.0f;
};

struct RobotFusionStatus {
    bool valid = false;
    bool ready = false;
    char health[20] = {};
    float heading_deg = 0.0f;
    float yaw_rate_dps = 0.0f;
    float confidence_pct = 0.0f;
    char source[20] = {};
};

class RobotUart {
public:
    using ObstacleStoppedCallback = void (*)(void*, const RobotObstacleStatus&, bool);
    using MapEventCallback = void (*)(void*, const char*, uint8_t);

    RobotUart(gpio_num_t rx_pin, gpio_num_t tx_pin,
              uart_port_t port = UART_NUM_1, int baud_rate = 115200)
        : rx_pin_(rx_pin), tx_pin_(tx_pin), port_(port), baud_rate_(baud_rate) {
        // Board instances are created lazily from app_main, after the FreeRTOS
        // scheduler is running.  Start a low-priority recovery observer now;
        // it waits for Begin() and the one-shot startup link test before doing
        // anything.  The task never emits a motion command.
        if (xTaskCreatePinnedToCore(ProtocolRecoveryTaskEntry,
                                    "robot_proto_recovery", 3072, this, 2,
                                    &protocol_recovery_task_, 0) != pdPASS) {
            protocol_recovery_task_ = nullptr;
            ESP_LOGE("RobotUart", "Cannot start protocol recovery task");
        }
    }
    ~RobotUart() = default;

    bool Begin();
    void Update();

    bool Ping(uint32_t timeout_ms = 500);
    bool CheckProtocol(uint32_t timeout_ms = 500);
    bool SetMode(bool ai_mode, uint32_t timeout_ms = 500);
    bool MoveForward(int speed, uint32_t timeout_ms = 500);
    bool MoveBackward(int speed, uint32_t timeout_ms = 500);
    bool TurnLeft(int speed, uint32_t timeout_ms = 500);
    bool TurnRight(int speed, uint32_t timeout_ms = 500);
    bool StartContinuous(bool forward, int speed,
                         uint32_t timeout_ms = 700);
    bool StartContinuousRotation(bool left, int speed,
                                 uint32_t timeout_ms = 700);
    bool MoveDistance(bool forward, int distance_mm, int speed,
                      RobotDistanceResult& result,
                      uint32_t timeout_ms = 31000);
    bool TurnRelative(bool left, int angle_deg, int speed,
                      RobotTurnResult& result,
                      uint32_t timeout_ms = 13000);
    bool TurnAbsolute(int heading_deg, int speed, RobotTurnResult& result,
                      uint32_t timeout_ms = 13000);

    // Public STOP is cancellation-aware. The transport-level uint32_t overload
    // remains private so normal callers cannot accidentally bypass waiter
    // release. Physical STOP confirmation is authoritative: a finite MOVE/TURN
    // waiter is released only after the STM32 has returned DONE,STOP.
    bool Stop(int timeout_ms = 500) {
        const bool wake_turn = turn_waiting_;
        const bool wake_distance = distance_waiting_;
        stop_in_progress_ = true;
        const uint32_t bounded_timeout =
            timeout_ms > 0 ? static_cast<uint32_t>(timeout_ms) : 0U;
        const bool stopped = Stop(bounded_timeout);
        if (stopped) {
            EventBits_t wake_bits = 0;
            if (wake_turn) wake_bits |= kResponseTurnError;
            if (wake_distance) wake_bits |= kResponseDistanceError;
            if (wake_bits != 0) {
                xEventGroupSetBits(response_events_, wake_bits);
            }
        }
        stop_in_progress_ = false;
        return stopped;
    }

    bool GetState(RobotState& state, uint32_t timeout_ms = 500);
    bool GetSpeed(int& speed, uint32_t timeout_ms = 500);
    bool SetSpeed(int speed, uint32_t timeout_ms = 500);
    bool GetBrake(bool& enabled, uint32_t timeout_ms = 500);
    bool SetBrake(bool enabled, uint32_t timeout_ms = 500);
    bool GetRamp(bool& enabled, uint32_t timeout_ms = 500);
    bool SetRamp(bool enabled, uint32_t timeout_ms = 500);
    bool GetHeading(float& heading_deg, uint32_t timeout_ms = 500);
    bool GetCompassStatus(RobotCompassStatus& status,
                          uint32_t timeout_ms = 500);
    bool ResetCompass(uint32_t timeout_ms = 2200);
    bool ResetEncoders(uint32_t timeout_ms = 700);
    bool GetPs2Status(RobotPs2Status& status, uint32_t timeout_ms = 500);
    bool GetObstacle(RobotObstacleStatus& status,
                     uint32_t timeout_ms = 500);
    bool GetOdometry(RobotOdometry& odometry, uint32_t timeout_ms = 500);
    bool GetEncoderStatus(RobotEncoderStatus& status,
                          uint32_t timeout_ms = 500);
    bool GetImuStatus(RobotImuStatus& status, uint32_t timeout_ms = 500);
    bool GetFusionStatus(RobotFusionStatus& status,
                         uint32_t timeout_ms = 500);
    bool GetCachedObstacle(RobotObstacleStatus& status);
    void SetObstacleStoppedCallback(ObstacleStoppedCallback callback, void* context) {
        obstacle_stopped_callback_ = callback;
        obstacle_stopped_context_ = context;
    }
    void SetMapEventCallback(MapEventCallback callback, void* context) {
        map_event_callback_ = callback;
        map_event_context_ = context;
    }

    bool Ps2OverrideActive() const;
    bool IsConnected() const;
    bool MotionLeaseActive() const { return motion_lease_active_; }
    bool SessionReady() const {
        return protocol_compatible_ && IsConnected() && !stop_in_progress_;
    }
    uint32_t MotionSessionId() const { return motion_session_id_; }

private:
    friend class TeachRoute;

    static constexpr EventBits_t kResponsePong = BIT0;
    static constexpr EventBits_t kResponseAck = BIT1;
    static constexpr EventBits_t kResponseState = BIT2;
    static constexpr EventBits_t kResponseDone = BIT3;
    static constexpr EventBits_t kResponseNack = BIT4;
    static constexpr EventBits_t kResponseHello = BIT5;
    static constexpr EventBits_t kResponseValue = BIT6;
    static constexpr EventBits_t kResponsePs2 = BIT7;
    static constexpr EventBits_t kResponseCompassZeroed = BIT8;
    static constexpr EventBits_t kResponseTurnDone = BIT9;
    static constexpr EventBits_t kResponseTurnError = BIT10;
    static constexpr EventBits_t kResponseStopDone = BIT11;
    static constexpr EventBits_t kResponseObstacle = BIT12;
    static constexpr EventBits_t kResponseDistanceDone = BIT13;
    static constexpr EventBits_t kResponseDistanceError = BIT14;
    static constexpr EventBits_t kResponseOdometry = BIT15;

    static uint32_t RandomCorrelationSeed() {
        uint32_t value = 0U;
        do {
            value = esp_random();
        } while (value == 0U || value == 0xFFFFFFFFU);
        return value;
    }

    static void RxTaskEntry(void* context);
    static void LinkTestTaskEntry(void* context);
    static void HeartbeatTaskEntry(void* context);
    static void ProtocolRecoveryTaskEntry(void* context) {
        static_cast<RobotUart*>(context)->ProtocolRecoveryTask();
    }
    void RxTask();
    void HeartbeatTask();
    void RunLinkTest();

    // Alpha.9 recovery runs outside the UART RX task so CheckProtocol() can
    // wait for HELLO/PONG while the RX task remains free to parse them.  It
    // observes only STM32 boot-epoch changes (plus an initial failed startup
    // negotiation), waits for all motion/cancellation state to be idle, then
    // retries HELLO/PING until the session is restored.  No MODE, MOVE, TURN,
    // STOP or lease acquisition is issued here.
    void ProtocolRecoveryTask() {
        while (!started_) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // The normal one-shot RunLinkTest starts after 800 ms and may consume
        // up to 1.4 s for HELLO+PING. Establish the recovery baseline only
        // after that path has had time to finish, avoiding duplicate startup
        // negotiations and extra SID increments.
        vTaskDelay(pdMS_TO_TICKS(2500));
        uint32_t observed_epoch = GetStm32BootEpoch();
        bool recovery_needed = !protocol_compatible_;

        while (true) {
            const uint32_t current_epoch = GetStm32BootEpoch();
            if (current_epoch != observed_epoch) {
                observed_epoch = current_epoch;
                recovery_needed = true;
                ESP_LOGW("RobotUart",
                         "ROBOT_SESSION=RECOVERY,STATE=REQUESTED,STM32_EPOCH=%lu",
                         static_cast<unsigned long>(current_epoch));
                // Let STM32 finish setup()/UART initialization after BOOT.
                vTaskDelay(pdMS_TO_TICKS(250));
            }

            if (protocol_compatible_) {
                recovery_needed = false;
            } else if (recovery_needed &&
                       !motion_lease_active_ && !stop_in_progress_ &&
                       !motion_correlation_active_ && !turn_waiting_ &&
                       !distance_waiting_) {
                const bool recovered = CheckProtocol(700);
                if (recovered) {
                    recovery_needed = false;
                    ESP_LOGI("RobotUart",
                             "ROBOT_SESSION=RECOVERY,STATE=PASS,SID=%lu,STM32_EPOCH=%lu",
                             static_cast<unsigned long>(motion_session_id_),
                             static_cast<unsigned long>(observed_epoch));
                } else {
                    ESP_LOGW("RobotUart",
                             "ROBOT_SESSION=RECOVERY,STATE=RETRY,STM32_EPOCH=%lu",
                             static_cast<unsigned long>(observed_epoch));
                }
            }

            // Keep retry traffic bounded if STM32 is absent. Successful
            // recovery returns this task to a passive observer state.
            vTaskDelay(pdMS_TO_TICKS(750));
        }
    }

    void HandleFrame(const char* frame);
    void DispatchMapEvent(const char* action, uint8_t slot);
    bool SendFrame(const char* body);
    bool SendAndWait(const char* body, EventBits_t expected,
                     uint32_t timeout_ms);
    // Raw STOP transport primitive. Use the public int overload so finite
    // operation waiters are released only after physical STOP confirmation.
    bool Stop(uint32_t timeout_ms);
    bool BeginMotionCorrelation(uint32_t& session_id, uint32_t& operation_id);
    void EndMotionCorrelation();
    void InvalidateMotionCorrelation(const char* reason);
    bool MatchMotionCorrelation(uint32_t session_id,
                                uint32_t operation_id) const;
    static int ClampSpeed(int speed);
    static int ClampDriveSpeed(int speed);

    SemaphoreHandle_t transaction_mutex_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    SemaphoreHandle_t state_mutex_ = nullptr;
    EventGroupHandle_t response_events_ = nullptr;
    TaskHandle_t rx_task_ = nullptr;
    TaskHandle_t heartbeat_task_ = nullptr;
    TaskHandle_t protocol_recovery_task_ = nullptr;
    RobotState state_;
    int value_speed_ = 0;
    bool value_brake_ = false;
    bool value_ramp_ = false;
    float value_heading_deg_ = 0.0f;
    RobotCompassStatus compass_status_;
    RobotPs2Status ps2_status_;
    RobotObstacleStatus obstacle_status_;
    RobotTurnResult turn_result_;
    RobotDistanceResult distance_result_;
    RobotOdometry odometry_;
    RobotEncoderStatus encoder_status_;
    RobotImuStatus imu_status_;
    RobotFusionStatus fusion_status_;
    char rx_frame_[256] = {};
    size_t rx_length_ = 0;
    bool receiving_ = false;
    bool started_ = false;
    volatile uint32_t last_rx_ms_ = 0;
    volatile bool protocol_compatible_ = false;
    volatile bool motion_lease_active_ = false;
    volatile bool stop_in_progress_ = false;
    volatile bool turn_waiting_ = false;
    volatile bool distance_waiting_ = false;
    volatile bool motion_ack_waiting_ = false;
    volatile bool motion_correlation_active_ = false;
    // Alpha.7 seeds both halves of the correlation pair from the ESP32
    // hardware RNG at construction. CheckProtocol() still advances SID on
    // each successful negotiation, preserving the existing session-boundary
    // semantics while preventing deterministic (SID=1, OP=1) reuse after an
    // ESP32 reboot. Independent random seeds reduce pair-collision risk across
    // reboots without adding NVS writes or changing the STM32 protocol.
    volatile uint32_t motion_session_id_ = RandomCorrelationSeed();
    volatile uint32_t next_operation_id_ = RandomCorrelationSeed();
    volatile uint32_t waiting_session_id_ = 0;
    volatile uint32_t waiting_operation_id_ = 0;
    volatile uint32_t terminal_operation_id_ = 0;
    std::atomic_bool ps2_override_active_{false};
    ObstacleStoppedCallback obstacle_stopped_callback_ = nullptr;
    void* obstacle_stopped_context_ = nullptr;
    MapEventCallback map_event_callback_ = nullptr;
    void* map_event_context_ = nullptr;
    gpio_num_t rx_pin_;
    gpio_num_t tx_pin_;
    uart_port_t port_;
    int baud_rate_;
    uint16_t tx_sequence_ = 0;
};

#endif

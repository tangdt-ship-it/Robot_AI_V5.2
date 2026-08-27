#ifndef XIAOZHI_SAFETY_BLACKBOX_H
#define XIAOZHI_SAFETY_BLACKBOX_H

#include <array>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// RAM-only forensic breadcrumbs.  Recording is bounded, allocation-free and
// deliberately has no actuator dependency; it must remain usable during a
// safety failure without changing the robot's behaviour.
enum class SafetyEventType : uint8_t {
    PREFLIGHT_BEGIN,
    PREFLIGHT_PASS,
    PREFLIGHT_FAIL,
    OWNER_ACQUIRE,
    OWNER_REJECT,
    LEASE_ACQUIRE,
    LEASE_RELEASE,
    COMMAND_SEND,
    ACK_ACCEPT,
    ACK_STALE,
    RESULT_ACCEPT,
    RESULT_STALE,
    HOLD,
    RESUME,
    RESET_BOUNDARY,
    STOP,
    CANCEL,
    LINK_LOSS,
    SESSION_CHANGE,
    TERMINAL_RESULT,
    DIAGNOSTIC,
};

struct SafetyBlackBoxEvent {
    uint32_t timestamp_ms = 0;
    uint32_t sequence = 0;
    uint32_t session_id = 0;
    uint32_t operation_id = 0;
    uint32_t reset_generation = 0;
    uint32_t stm32_boot_epoch = 0;
    uint16_t route_index = 0;
    uint16_t segment_index = 0;
    SafetyEventType type = SafetyEventType::DIAGNOSTIC;
    char reason[16] = {};
};

class SafetyBlackBox {
public:
    static constexpr size_t kCapacity = 48;

    void Record(SafetyEventType type, uint32_t session_id = 0,
                uint32_t operation_id = 0, const char* reason = nullptr,
                uint32_t reset_generation = 0, uint16_t route_index = 0,
                uint16_t segment_index = 0);
    size_t CopyRecent(SafetyBlackBoxEvent* output, size_t maximum) const;
    size_t size() const;

private:
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    std::array<SafetyBlackBoxEvent, kCapacity> events_{};
    uint32_t next_sequence_ = 1;
    size_t next_slot_ = 0;
    size_t size_ = 0;
};

enum class MotionDiagnostic : uint8_t {
    OK,
    NO_ENCODER_PROGRESS,
    IMPLAUSIBLE_ENCODER_JUMP,
    ENCODER_ODOMETRY_MISMATCH,
    ODOMETRY_UNRELIABLE,
    HEADING_UNRELIABLE,
    RESET_BOUNDARY,
    UNCALIBRATED,
    UNAVAILABLE,
};

struct MotionDiagnosticSample {
    bool available = false;
    bool encoder_ready = false;
    bool odometry_valid = false;
    bool heading_reliable = false;
    bool reset_boundary = false;
    bool thresholds_calibrated = false;
    bool command_expects_progress = false;
    float encoder_delta_mm = 0.0f;
    float odometry_delta_mm = 0.0f;
    float maximum_jump_mm = 0.0f;
    float maximum_mismatch_mm = 0.0f;
};

// Classification only.  Callers must never treat this helper as a motor or
// calibration controller; thresholds are only honoured when explicitly known.
MotionDiagnostic ClassifyMotionDiagnostic(const MotionDiagnosticSample& sample);
SafetyBlackBox& GetSafetyBlackBox();

// ESP32-local epoch of observed STM32 BOOT frames. It is deliberately volatile:
// an ESP32 reboot also destroys all volatile Replay/HOLD resume contexts. While
// the ESP32 remains alive, every STM32 reboot advances this value even when the
// STM32's raw RESET_GEN restarts at the same numeric value.
uint32_t GetStm32BootEpoch();

#endif

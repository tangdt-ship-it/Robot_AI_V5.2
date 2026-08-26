#include "safety_blackbox.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr const char* kTag = "SafetyBlackBox";

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

const char* EventTypeName(SafetyEventType type) {
    switch (type) {
        case SafetyEventType::PREFLIGHT_BEGIN: return "PREFLIGHT_BEGIN";
        case SafetyEventType::PREFLIGHT_PASS: return "PREFLIGHT_PASS";
        case SafetyEventType::PREFLIGHT_FAIL: return "PREFLIGHT_FAIL";
        case SafetyEventType::OWNER_ACQUIRE: return "OWNER_ACQUIRE";
        case SafetyEventType::OWNER_REJECT: return "OWNER_REJECT";
        case SafetyEventType::LEASE_ACQUIRE: return "LEASE_ACQUIRE";
        case SafetyEventType::LEASE_RELEASE: return "LEASE_RELEASE";
        case SafetyEventType::COMMAND_SEND: return "COMMAND_SEND";
        case SafetyEventType::ACK_ACCEPT: return "ACK_ACCEPT";
        case SafetyEventType::ACK_STALE: return "ACK_STALE";
        case SafetyEventType::RESULT_ACCEPT: return "RESULT_ACCEPT";
        case SafetyEventType::RESULT_STALE: return "RESULT_STALE";
        case SafetyEventType::HOLD: return "HOLD";
        case SafetyEventType::RESUME: return "RESUME";
        case SafetyEventType::RESET_BOUNDARY: return "RESET_BOUNDARY";
        case SafetyEventType::STOP: return "STOP";
        case SafetyEventType::CANCEL: return "CANCEL";
        case SafetyEventType::LINK_LOSS: return "LINK_LOSS";
        case SafetyEventType::SESSION_CHANGE: return "SESSION_CHANGE";
        case SafetyEventType::TERMINAL_RESULT: return "TERMINAL_RESULT";
        case SafetyEventType::DIAGNOSTIC: return "DIAGNOSTIC";
    }
    return "UNKNOWN";
}
}  // namespace

void SafetyBlackBox::Record(SafetyEventType type, uint32_t session_id,
                            uint32_t operation_id, const char* reason,
                            uint32_t reset_generation, uint16_t route_index,
                            uint16_t segment_index) {
    SafetyBlackBoxEvent event{};
    event.timestamp_ms = NowMs();
    event.session_id = session_id;
    event.operation_id = operation_id;
    event.reset_generation = reset_generation;
    event.route_index = route_index;
    event.segment_index = segment_index;
    event.type = type;
    if (reason != nullptr) {
        std::strncpy(event.reason, reason, sizeof(event.reason) - 1U);
    }
    portENTER_CRITICAL(&lock_);
    event.sequence = next_sequence_++;
    if (next_sequence_ == 0U) next_sequence_ = 1U;
    events_[next_slot_] = event;
    next_slot_ = (next_slot_ + 1U) % kCapacity;
    size_ = std::min(size_ + 1U, kCapacity);
    portEXIT_CRITICAL(&lock_);

    // Alpha.6 H2 observability: emit each forensic breadcrumb after it has
    // been committed to the bounded RAM ring. This is telemetry only; it does
    // not call RobotUart, acquire a motion lease or change actuator state.
    ESP_LOGI(kTag,
             "ROBOT_BLACKBOX SEQ=%lu MS=%lu TYPE=%s SID=%lu OP=%lu RESET_GEN=%lu ROUTE=%u SEG=%u REASON=%s",
             static_cast<unsigned long>(event.sequence),
             static_cast<unsigned long>(event.timestamp_ms),
             EventTypeName(event.type),
             static_cast<unsigned long>(event.session_id),
             static_cast<unsigned long>(event.operation_id),
             static_cast<unsigned long>(event.reset_generation),
             static_cast<unsigned>(event.route_index),
             static_cast<unsigned>(event.segment_index),
             event.reason[0] != '\0' ? event.reason : "-");

    // H0/H1 retain the compact negotiated-session marker used by existing
    // hardware procedures. The black-box line above is the richer H2 trace.
    if (type == SafetyEventType::SESSION_CHANGE && session_id != 0U &&
        operation_id == 0U && reason != nullptr &&
        std::strcmp(reason, "HELLO") == 0) {
        ESP_LOGI(kTag, "ROBOT_SESSION=READY,SID=%lu",
                 static_cast<unsigned long>(session_id));
    }
}

size_t SafetyBlackBox::CopyRecent(SafetyBlackBoxEvent* output,
                                  size_t maximum) const {
    if (output == nullptr || maximum == 0U) return 0U;
    portENTER_CRITICAL(&lock_);
    const size_t count = std::min(size_, maximum);
    const size_t first = (next_slot_ + kCapacity - size_) % kCapacity;
    for (size_t i = 0; i < count; ++i) {
        output[i] = events_[(first + i) % kCapacity];
    }
    portEXIT_CRITICAL(&lock_);
    return count;
}

size_t SafetyBlackBox::size() const {
    portENTER_CRITICAL(&lock_);
    const size_t value = size_;
    portEXIT_CRITICAL(&lock_);
    return value;
}

MotionDiagnostic ClassifyMotionDiagnostic(const MotionDiagnosticSample& sample) {
    if (!sample.available) return MotionDiagnostic::UNAVAILABLE;
    if (sample.reset_boundary) return MotionDiagnostic::RESET_BOUNDARY;
    if (!sample.encoder_ready || !sample.odometry_valid) {
        return MotionDiagnostic::ODOMETRY_UNRELIABLE;
    }
    if (!sample.heading_reliable) return MotionDiagnostic::HEADING_UNRELIABLE;
    if (sample.command_expects_progress &&
        std::fabs(sample.encoder_delta_mm) < 0.01f) {
        return MotionDiagnostic::NO_ENCODER_PROGRESS;
    }
    if (!sample.thresholds_calibrated) return MotionDiagnostic::UNCALIBRATED;
    if (sample.maximum_jump_mm <= 0.0f || sample.maximum_mismatch_mm <= 0.0f) {
        return MotionDiagnostic::UNCALIBRATED;
    }
    if (std::fabs(sample.encoder_delta_mm) > sample.maximum_jump_mm) {
        return MotionDiagnostic::IMPLAUSIBLE_ENCODER_JUMP;
    }
    if (std::fabs(sample.encoder_delta_mm - sample.odometry_delta_mm) >
        sample.maximum_mismatch_mm) {
        return MotionDiagnostic::ENCODER_ODOMETRY_MISMATCH;
    }
    return MotionDiagnostic::OK;
}

SafetyBlackBox& GetSafetyBlackBox() {
    static SafetyBlackBox blackbox;
    return blackbox;
}

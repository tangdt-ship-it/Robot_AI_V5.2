#include "safety_blackbox.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_timer.h>

namespace {
uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
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

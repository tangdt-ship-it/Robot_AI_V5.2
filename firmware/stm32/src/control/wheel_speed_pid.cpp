#include <control/wheel_speed_pid.h>

#include <math.h>

namespace {
constexpr float kMinDtS = 0.001f;
constexpr float kMaxDtS = 0.100f;
constexpr float kIntegralLimit = 500.0f;
constexpr float kCommandMin = -255.0f;
constexpr float kCommandMax = 255.0f;
}  // namespace

void WheelSpeedPid::begin(bool enabled, float kp, float ki, float kd,
                          float targetMmSPerCommand) {
  enabled_ = enabled && targetMmSPerCommand > 0.0f;
  kp_ = max(0.0f, kp);
  ki_ = max(0.0f, ki);
  kd_ = max(0.0f, kd);
  targetMmSPerCommand_ = max(0.0f, targetMmSPerCommand);
  reset();
}

void WheelSpeedPid::reset() {
  integral_ = 0.0f;
  previousErrorMmS_ = 0.0f;
  targetMmS_ = 0.0f;
  errorMmS_ = 0.0f;
  outputCommand_ = 0;
  previousCommandSign_ = 0;
  lastUpdateMs_ = 0;
}

float WheelSpeedPid::clamp(float value, float low, float high) {
  return value < low ? low : value > high ? high : value;
}

int8_t WheelSpeedPid::signOf(int16_t value) {
  return value < 0 ? -1 : value > 0 ? 1 : 0;
}

int16_t WheelSpeedPid::update(int16_t command, float measuredMmS,
                              uint32_t nowMs) {
  command = constrain(command, -255, 255);
  if (!enabled_ || command == 0) {
    reset();
    return command;
  }

  const int8_t commandSign = signOf(command);
  // A direction change is a new control session. Do not carry integral energy
  // from forward into reverse (or vice versa).
  if (previousCommandSign_ != 0 && commandSign != previousCommandSign_) {
    integral_ = 0.0f;
    previousErrorMmS_ = 0.0f;
    lastUpdateMs_ = 0;
  }
  previousCommandSign_ = commandSign;

  float dt = lastUpdateMs_ == 0U
                 ? 0.005f
                 : static_cast<float>(nowMs - lastUpdateMs_) / 1000.0f;
  lastUpdateMs_ = nowMs;
  dt = clamp(dt, kMinDtS, kMaxDtS);

  targetMmS_ = static_cast<float>(command) * targetMmSPerCommand_;
  errorMmS_ = targetMmS_ - measuredMmS;
  const float derivative = (errorMmS_ - previousErrorMmS_) / dt;
  const float candidateIntegral = clamp(
      integral_ + errorMmS_ * dt, -kIntegralLimit, kIntegralLimit);

  // Existing command is the feed-forward term. The PID correction is in the
  // same signed command units, so the old speed/ramp/limit semantics remain
  // intact when the candidate profile is enabled.
  // A speed correction may reduce a wheel to zero, but it must never reverse
  // that wheel while the owner is still commanding the original direction.
  const float outputMin = command > 0 ? 0.0f : kCommandMin;
  const float outputMax = command > 0 ? kCommandMax : 0.0f;
  float unsaturated = static_cast<float>(command) +
                      kp_ * errorMmS_ +
                      ki_ * candidateIntegral +
                      kd_ * derivative;
  const bool saturatingHigh = unsaturated > outputMax && errorMmS_ > 0.0f;
  const bool saturatingLow = unsaturated < outputMin && errorMmS_ < 0.0f;
  if (!saturatingHigh && !saturatingLow) integral_ = candidateIntegral;

  unsaturated = static_cast<float>(command) +
                kp_ * errorMmS_ +
                ki_ * integral_ +
                kd_ * derivative;
  const float output = clamp(unsaturated, outputMin, outputMax);
  outputCommand_ = static_cast<int16_t>(roundf(output));
  previousErrorMmS_ = errorMmS_;
  return outputCommand_;
}

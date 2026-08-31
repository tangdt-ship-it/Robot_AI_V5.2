#ifndef WHEEL_SPEED_PID_H
#define WHEEL_SPEED_PID_H

#include <Arduino.h>

// Small, allocation-free signed wheel-speed PID. The command remains the
// public RobotController setpoint (the existing -255..255 units); the PID
// adjusts only the applied motor command using encoder velocity feedback.
class WheelSpeedPid {
 public:
  void begin(bool enabled, float kp, float ki, float kd,
             float targetMmSPerCommand);
  void reset();
  int16_t update(int16_t command, float measuredMmS, uint32_t nowMs);

  bool enabled() const { return enabled_; }
  int16_t outputCommand() const { return outputCommand_; }
  float targetMmS() const { return targetMmS_; }
  float errorMmS() const { return errorMmS_; }

 private:
  static float clamp(float value, float low, float high);
  static int8_t signOf(int16_t value);

  bool enabled_ = false;
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float targetMmSPerCommand_ = 0.0f;
  float integral_ = 0.0f;
  float previousErrorMmS_ = 0.0f;
  float targetMmS_ = 0.0f;
  float errorMmS_ = 0.0f;
  int16_t outputCommand_ = 0;
  int8_t previousCommandSign_ = 0;
  uint32_t lastUpdateMs_ = 0;
};

#endif

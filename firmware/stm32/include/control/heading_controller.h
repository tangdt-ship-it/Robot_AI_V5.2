#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <Arduino.h>

class HeadingController {
 public:
  void begin();
  void capture(float currentHeading);
  void reset();
  int16_t update(float currentHeading, int16_t commandMagnitude);

  bool isActive() const { return active_; }
  float targetHeading() const { return targetHeading_; }
  float error() const { return error_; }
  float output() const { return output_; }
  float yawRate() const { return yawRateFiltered_; }
  float desiredYawRate() const { return desiredYawRate_; }
  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }
  bool isDisturbed() const { return disturbed_; }
  uint32_t disturbanceCount() const { return disturbanceCount_; }
  void setTunings(float kp, float ki, float kd);

 private:
  static float shortestError(float target, float current);

  bool active_ = false;
  float targetHeading_ = 0.0f;
  float error_ = 0.0f;
  float integral_ = 0.0f;
  float lastHeading_ = 0.0f;
  float yawRateFiltered_ = 0.0f;
  float desiredYawRate_ = 0.0f;
  float output_ = 0.0f;
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  uint32_t lastUpdateMs_ = 0;
  bool headingReady_ = false;
  bool headingLocked_ = true;
  bool disturbed_ = false;
  float wrongResponseTimeS_ = 0.0f;
  uint32_t disturbanceCount_ = 0;
};

#endif

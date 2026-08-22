#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <Arduino.h>
#include <stm32f1xx_hal.h>

enum class EncoderHealth : uint8_t {
  DISABLED = 0,
  OK = 1,
  INIT_FAILED = 2,
  LEFT_STALL = 3,
  RIGHT_STALL = 4,
  BOTH_STALL = 5,
};

struct WheelOdometryData {
  int64_t leftTicks = 0;
  int64_t rightTicks = 0;
  float leftDistanceMm = 0.0f;
  float rightDistanceMm = 0.0f;
  float distanceMm = 0.0f;
  float xMm = 0.0f;
  float yMm = 0.0f;
  // Navigation heading used for X/Y integration. V4.2 feeds this from the
  // fused Encoder + MPU6050 + Compass estimator.
  float headingRad = 0.0f;
  // Independent encoder-only yaw is kept for fusion and diagnostics.
  float encoderHeadingRad = 0.0f;
  float leftVelocityMmS = 0.0f;
  float rightVelocityMmS = 0.0f;
  float linearVelocityMmS = 0.0f;
  float angularVelocityRadS = 0.0f;
};

class WheelOdometry {
 public:
  bool begin();
  // Phase 1: sample encoder timers and compute travel/yaw increments.
  void update(int16_t leftCommand = 0, int16_t rightCommand = 0);
  // Phase 2: integrate the pending translation using the best available
  // heading. Call once after HeadingFusion::update().
  void integratePose(float headingDeg, bool externalHeadingValid);
  void reset();
  // Zero the two wheel encoder readouts without changing the navigation pose,
  // fused-heading reference, or accumulated centre travel.
  void resetWheelCounts();
  const WheelOdometryData& data() const { return data_; }

  bool ready() const { return initialized_; }
  bool healthy() const { return initialized_ && health_ == EncoderHealth::OK; }
  bool stallFault() const {
    return health_ == EncoderHealth::LEFT_STALL ||
           health_ == EncoderHealth::RIGHT_STALL ||
           health_ == EncoderHealth::BOTH_STALL;
  }
  EncoderHealth health() const { return health_; }
  const char* healthText() const;

 private:
  static int32_t counterDelta(uint32_t current, uint32_t previous);
  static float normalizeRad(float radians);
  void updateStallHealth(uint32_t nowMs, int32_t leftDelta,
                         int32_t rightDelta, int16_t leftCommand,
                         int16_t rightCommand);

  TIM_HandleTypeDef leftTimer_ = {};
  TIM_HandleTypeDef rightTimer_ = {};
  uint32_t leftPreviousCount_ = 0;
  uint32_t rightPreviousCount_ = 0;
  uint32_t lastUpdateMs_ = 0;
  uint32_t leftCommandStartMs_ = 0;
  uint32_t rightCommandStartMs_ = 0;
  uint32_t leftLastMotionMs_ = 0;
  uint32_t rightLastMotionMs_ = 0;
  bool initialized_ = false;
  EncoderHealth health_ = EncoderHealth::DISABLED;
  float pendingTravelMm_ = 0.0f;
  WheelOdometryData data_;
};

#endif

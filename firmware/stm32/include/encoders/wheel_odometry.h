#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <Arduino.h>
#include <stm32f1xx_hal.h>
#include <robot_config.h>

enum class EncoderHealth : uint8_t {
  DISABLED = 0,
  OK = 1,
  INIT_FAILED = 2,
  LEFT_STALL = 3,
  RIGHT_STALL = 4,
  BOTH_STALL = 5,
};

enum class EncoderResetReason : uint8_t {
  UNKNOWN,
  BOOT,
  PS2_R2,
  ROBOTLINK,
};

enum class WheelCalibrationPhase : uint8_t {
  NONE = 0,
  STRAIGHT = 1,
  TURN = 2,
};

struct WheelCalibrationStatus {
  WheelCalibrationPhase phase = WheelCalibrationPhase::NONE;
  bool valid = false;
  bool persisted = false;
  float leftMmPerTick = 0.0f;
  float rightMmPerTick = 0.0f;
  float trackMm = 0.0f;
  uint16_t straightSamples = 0;
  uint16_t turnSamples = 0;
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
  // fused Encoder + MPU6050 estimator.
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
  void resetWheelCounts(EncoderResetReason reason = EncoderResetReason::UNKNOWN);
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
  uint32_t resetGeneration() const { return resetGeneration_; }
  EncoderResetReason lastResetReason() const { return lastResetReason_; }
  static const char* resetReasonText(EncoderResetReason reason);

  // Guided calibration is intentionally separate from normal motion. The
  // caller starts/ends a measurement while the chassis is stopped at both
  // boundaries, supplies the measured floor distance/turn angle, and commits
  // only after both straight and turn samples pass validation.
  bool startCalibrationStraight();
  bool finishCalibrationStraight(float referenceMm);
  bool startCalibrationTurn();
  bool finishCalibrationTurn(float referenceDeg);
  bool commitCalibration();
  void abortCalibration();
  WheelCalibrationStatus calibrationStatus() const;
  const char* calibrationLastError() const;
  int64_t calibrationLastLeftDelta() const { return calibrationLastLeftDelta_; }
  int64_t calibrationLastRightDelta() const { return calibrationLastRightDelta_; }

 private:
  static int32_t counterDelta(uint32_t current, uint32_t previous);
  static float normalizeRad(float radians);
  void updateStallHealth(uint32_t nowMs, int32_t leftDelta,
                         int32_t rightDelta, int16_t leftCommand,
                         int16_t rightCommand);
  bool loadCalibration();
  bool saveCalibration() const;
  bool calibrationValuesValid(float leftMmPerTick, float rightMmPerTick,
                              float trackMm) const;

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
  uint32_t resetGeneration_ = 0;
  EncoderResetReason lastResetReason_ = EncoderResetReason::UNKNOWN;
  float leftMmPerTick_ = ENCODER_LEFT_MM_PER_TICK;
  float rightMmPerTick_ = ENCODER_RIGHT_MM_PER_TICK;
  float trackMm_ = WHEEL_TRACK_MM;
  bool calibrationPersisted_ = false;
  WheelCalibrationPhase calibrationPhase_ = WheelCalibrationPhase::NONE;
  int64_t calibrationStartLeftTicks_ = 0;
  int64_t calibrationStartRightTicks_ = 0;
  float calibrationCandidateLeftMmPerTick_ = ENCODER_LEFT_MM_PER_TICK;
  float calibrationCandidateRightMmPerTick_ = ENCODER_RIGHT_MM_PER_TICK;
  float calibrationCandidateTrackMm_ = WHEEL_TRACK_MM;
  uint16_t calibrationStraightSamples_ = 0;
  uint16_t calibrationTurnSamples_ = 0;
  uint16_t committedStraightSamples_ = 0;
  uint16_t committedTurnSamples_ = 0;
  const char* calibrationLastError_ = "NONE";
  int64_t calibrationLastLeftDelta_ = 0;
  int64_t calibrationLastRightDelta_ = 0;
};

#endif

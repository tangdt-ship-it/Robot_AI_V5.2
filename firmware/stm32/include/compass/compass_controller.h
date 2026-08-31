#ifndef COMPASS_CONTROLLER_H
#define COMPASS_CONTROLLER_H

#include <Arduino.h>
#include <Compass_Uart_V2_STM32.h>

enum class CompassMotionMode : uint8_t {
  STATIONARY,
  STRAIGHT,
  ROTATING
};

class CompassController {
 public:
  CompassController();
  void begin();
  void update();
  void resetZero();
  void setRobotMoving(bool moving);
  void setMotionMode(CompassMotionMode mode);

  float getAngle() const { return angleDeg_; }
  int16_t getRaw() const { return raw_; }
  float getRawAngle() const { return rawAngleDeg_; }
  bool isConnected() const;
  bool isCalibrating() const { return driftCalibrating_; }
  float driftRateDegS() const { return driftRateDegS_; }
  uint32_t lastSampleDtMs() const { return lastSampleDtMs_; }
  uint32_t sampleSequence() const { return sampleSequence_; }
  uint32_t zeroGeneration() const { return zeroGeneration_; }
  bool zeroPending() const { return zeroPending_; }
  uint32_t rejectedSamples() const { return rejectedSamples_; }
  uint8_t consecutiveRejectedSamples() const {
    return consecutiveRejectedSamples_;
  }
  CompassMotionMode motionMode() const { return motionMode_; }

  static float normalize(float angle);
  static float shortestDelta(float target, float current);

 private:
  void beginDriftCalibration(uint32_t nowMs);
  void addCalibrationSample(float relativeAngle, uint32_t nowMs);
  void finishDriftCalibration();
  void resetAdaptiveWindow();
  void addAdaptiveSample(float relativeAngle, uint32_t nowMs);
  void finishAdaptiveWindow();
  void updateDriftCompensation(uint32_t nowMs);
  float compensateDrift(float relativeAngle) const;
  void acceptSample(int16_t rawValue, uint32_t nowMs);
  bool validateSample(int16_t rawValue, uint32_t nowMs);
  void rejectSample(float relativeAngle);

  Compass_Uart_V2 device_;
  int16_t raw_ = 0;
  float rawAngleDeg_ = 0.0f;
  float zeroRaw_ = 0.0f;
  float angleDeg_ = 0.0f;
  float filteredAngleDeg_ = 0.0f;
  bool filterReady_ = false;
  bool zeroPending_ = true;
  uint32_t lastRequestMs_ = 0;
  uint32_t lastGoodMs_ = 0;
  uint32_t lastSampleDtMs_ = 0;
  uint32_t sampleSequence_ = 0;
  uint32_t zeroGeneration_ = 0;

  bool driftCalibrating_ = false;
  uint32_t driftCalStartMs_ = 0;
  uint32_t driftFirstMs_ = 0;
  uint32_t driftLastMs_ = 0;
  float driftFirstUnwrapped_ = 0.0f;
  float driftLastUnwrapped_ = 0.0f;
  float driftPreviousWrapped_ = 0.0f;
  uint16_t driftSamples_ = 0;
  float driftSumTime_ = 0.0f;
  float driftSumAngle_ = 0.0f;
  float driftSumTimeSquared_ = 0.0f;
  float driftSumTimeAngle_ = 0.0f;
  float driftRateDegS_ = 0.0f;
  uint32_t driftReferenceMs_ = 0;
  float driftCompensationDeg_ = 0.0f;
  bool calibrationHoldActive_ = false;
  float calibrationHoldAngleDeg_ = 0.0f;
  float lastRelativeAngleDeg_ = 0.0f;
  bool relativeAngleReady_ = false;
  bool stationaryHoldActive_ = false;
  float stationaryHoldAngleDeg_ = 0.0f;
  uint32_t stationaryManualMotionMs_ = 0;

  bool robotMoving_ = false;
  CompassMotionMode motionMode_ = CompassMotionMode::STATIONARY;
  uint32_t rejectedSamples_ = 0;
  uint8_t consecutiveRejectedSamples_ = 0;
  uint32_t adaptiveBlockedUntilMs_ = 0;
  uint32_t adaptiveFirstMs_ = 0;
  uint32_t adaptiveLastMs_ = 0;
  float adaptiveFirstUnwrapped_ = 0.0f;
  float adaptiveLastUnwrapped_ = 0.0f;
  float adaptivePreviousWrapped_ = 0.0f;
  float adaptiveTravelAbs_ = 0.0f;
  uint16_t adaptiveSamples_ = 0;
  float adaptiveSumTime_ = 0.0f;
  float adaptiveSumAngle_ = 0.0f;
  float adaptiveSumTimeSquared_ = 0.0f;
  float adaptiveSumTimeAngle_ = 0.0f;
};

#endif

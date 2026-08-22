#ifndef HEADING_FUSION_H
#define HEADING_FUSION_H

#include <Arduino.h>

enum class FusionHealth : uint8_t {
  NO_SOURCE = 0,
  DEGRADED = 1,
  FUSED = 2,
};

class HeadingFusion {
 public:
  void begin();
  void reset(float referenceHeadingDeg = 0.0f,
             float encoderHeadingRad = 0.0f);
  // Sensor sequence numbers ensure one physical sample is never integrated
  // multiple times when the main loop runs faster than the IMU/Compass.
  void update(float compassHeadingDeg, bool compassOk,
              uint32_t compassSequence,
              float gyroZDegS, bool imuOk, uint32_t imuSequence,
              float encoderHeadingRad, bool encoderOk,
              bool robotMoving);

  bool ready() const { return initialized_ && health_ != FusionHealth::NO_SOURCE; }
  float headingDeg() const { return headingDeg_; }
  float headingRad() const;
  float yawRateDegS() const { return yawRateDegS_; }
  float confidencePct() const { return confidencePct_; }
  uint32_t sampleSequence() const { return sampleSequence_; }
  FusionHealth health() const { return health_; }
  const char* healthText() const;
  const char* sourceText() const { return sourceText_; }
  float encoderDisagreementDeg() const { return encoderDisagreementDeg_; }
  float effectiveEncoderWeight() const { return effectiveEncoderWeight_; }
  bool usingCompass() const { return usingCompass_; }
  bool usingImu() const { return usingImu_; }
  bool usingEncoder() const { return usingEncoder_; }

  static float normalize(float degrees);
  static float shortestDelta(float target, float current);

 private:
  bool initialized_ = false;
  bool encoderReferenceReady_ = false;
  float headingDeg_ = 0.0f;
  float yawRateDegS_ = 0.0f;
  float confidencePct_ = 0.0f;
  float lastEncoderHeadingDeg_ = 0.0f;
  float pendingEncoderDeltaDeg_ = 0.0f;
  float encoderDisagreementDeg_ = 0.0f;
  float effectiveEncoderWeight_ = 0.0f;
  uint32_t lastImuSequence_ = 0;
  uint32_t lastCompassSequence_ = 0;
  uint32_t lastImuConsumeMs_ = 0;
  uint32_t sampleSequence_ = 0;
  FusionHealth health_ = FusionHealth::NO_SOURCE;
  bool usingCompass_ = false;
  bool usingImu_ = false;
  bool usingEncoder_ = false;
  const char* sourceText_ = "NONE";
};

#endif

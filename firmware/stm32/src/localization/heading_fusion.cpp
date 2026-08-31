#include <localization/heading_fusion.h>
#include <robot_config.h>

#include <math.h>

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
}

float HeadingFusion::normalize(float degrees) {
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees <= -180.0f) degrees += 360.0f;
  return degrees;
}

float HeadingFusion::shortestDelta(float target, float current) {
  return normalize(target - current);
}

void HeadingFusion::begin() {
  initialized_ = false;
  encoderReferenceReady_ = false;
  headingDeg_ = 0.0f;
  yawRateDegS_ = 0.0f;
  confidencePct_ = 0.0f;
  lastEncoderHeadingDeg_ = 0.0f;
  pendingEncoderDeltaDeg_ = 0.0f;
  encoderDisagreementDeg_ = 0.0f;
  effectiveEncoderWeight_ = 0.0f;
  lastImuSequence_ = 0;
  lastCompassSequence_ = 0;
  lastImuConsumeMs_ = millis();
  sampleSequence_ = 0;
  health_ = FusionHealth::NO_SOURCE;
  usingCompass_ = usingImu_ = usingEncoder_ = false;
  sourceText_ = "NONE";
}

void HeadingFusion::reset(float referenceHeadingDeg, float encoderHeadingRad) {
  headingDeg_ = normalize(referenceHeadingDeg);
  lastEncoderHeadingDeg_ = normalize(encoderHeadingRad * kRadToDeg);
  encoderReferenceReady_ = true;
  pendingEncoderDeltaDeg_ = 0.0f;
  encoderDisagreementDeg_ = 0.0f;
  effectiveEncoderWeight_ = 0.0f;
  initialized_ = true;
  yawRateDegS_ = 0.0f;
  lastImuConsumeMs_ = millis();
  ++sampleSequence_;
}

void HeadingFusion::update(float compassHeadingDeg, bool compassOk,
                           uint32_t compassSequence,
                           float gyroZDegS, bool imuOk,
                           uint32_t imuSequence,
                           float encoderHeadingRad, bool encoderOk,
                           bool robotMoving) {
  const uint32_t now = millis();
  bool changed = false;

  const float encoderHeadingDeg = normalize(encoderHeadingRad * kRadToDeg);
  float encoderDeltaDeg = 0.0f;
  if (encoderOk) {
    if (encoderReferenceReady_) {
      encoderDeltaDeg = shortestDelta(encoderHeadingDeg,
                                      lastEncoderHeadingDeg_);
      if (fabsf(encoderDeltaDeg) > 0.00001f) changed = true;
    }
    lastEncoderHeadingDeg_ = encoderHeadingDeg;
    encoderReferenceReady_ = true;
    pendingEncoderDeltaDeg_ += encoderDeltaDeg;
  } else {
    encoderReferenceReady_ = false;
    pendingEncoderDeltaDeg_ = 0.0f;
  }

  const bool newImuSample = imuOk && imuSequence != 0U &&
                            imuSequence != lastImuSequence_;
  const bool newCompassSample = compassOk && compassSequence != 0U &&
                                compassSequence != lastCompassSequence_;

  if (!initialized_) {
    if (compassOk) {
      headingDeg_ = normalize(compassHeadingDeg);
    } else if (encoderOk) {
      headingDeg_ = encoderHeadingDeg;
    } else {
      headingDeg_ = 0.0f;
    }
    initialized_ = compassOk || imuOk || encoderOk;
    changed = initialized_;
  }

  float motionDeltaDeg = 0.0f;
  float rateSampleDegS = 0.0f;
  if (newImuSample) {
    float dt = static_cast<float>(now - lastImuConsumeMs_) / 1000.0f;
    dt = constrain(dt, FUSION_MIN_DT_S, FUSION_MAX_DT_S);
    lastImuConsumeMs_ = now;
    lastImuSequence_ = imuSequence;
    const float gyroDeltaDeg = gyroZDegS * dt;
    if (encoderOk) {
      encoderDisagreementDeg_ = fabsf(pendingEncoderDeltaDeg_ - gyroDeltaDeg);
      // With the Compass intentionally unavailable, integrating a small
      // gyro-Z bias while both motors are stopped makes a saved HOME heading
      // drift away from the physical chassis.  Encoder yaw is authoritative
      // for the stationary condition: retain the heading until either a
      // commanded/coasting turn produces encoder yaw or the chassis moves.
      if (!robotMoving && fabsf(pendingEncoderDeltaDeg_) <= 0.00001f) {
        effectiveEncoderWeight_ = FUSION_ENCODER_DELTA_WEIGHT;
        motionDeltaDeg = 0.0f;
        pendingEncoderDeltaDeg_ = 0.0f;
        rateSampleDegS = 0.0f;
        changed = true;
      } else {
      float agreement = 1.0f;
      if (encoderDisagreementDeg_ > FUSION_ENCODER_AGREEMENT_DEG) {
        agreement = 1.0f - (encoderDisagreementDeg_ - FUSION_ENCODER_AGREEMENT_DEG) /
                              (FUSION_ENCODER_REJECT_DEG - FUSION_ENCODER_AGREEMENT_DEG);
        agreement = constrain(agreement, 0.0f, 1.0f);
      }
      effectiveEncoderWeight_ = FUSION_ENCODER_DELTA_WEIGHT * agreement;
      motionDeltaDeg = (1.0f - effectiveEncoderWeight_) * gyroDeltaDeg +
                       effectiveEncoderWeight_ * pendingEncoderDeltaDeg_;
      }
    } else {
      encoderDisagreementDeg_ = 0.0f;
      effectiveEncoderWeight_ = 0.0f;
      motionDeltaDeg = gyroDeltaDeg;
    }
    pendingEncoderDeltaDeg_ = 0.0f;
    rateSampleDegS = dt > 0.0f ? motionDeltaDeg / dt : 0.0f;
    changed = true;
  } else if (!imuOk && encoderOk && fabsf(pendingEncoderDeltaDeg_) > 0.00001f) {
    // Degraded mode without IMU: consume encoder yaw directly.
    motionDeltaDeg = pendingEncoderDeltaDeg_;
    pendingEncoderDeltaDeg_ = 0.0f;
    rateSampleDegS = 0.0f;
    changed = true;
  }

  if (initialized_ && motionDeltaDeg != 0.0f) {
    headingDeg_ = normalize(headingDeg_ + motionDeltaDeg);
  }

  // Apply a Compass correction once per new Compass sample, never once per
  // main-loop iteration. This prevents repeated correction of stale data.
  if (initialized_ && newCompassSample) {
    lastCompassSequence_ = compassSequence;
    const float compassError = shortestDelta(compassHeadingDeg, headingDeg_);
    const bool acceptCorrection = !robotMoving ||
        fabsf(compassError) <= FUSION_COMPASS_MOVING_GATE_DEG;
    if (acceptCorrection) {
      const float gain = robotMoving ? FUSION_COMPASS_GAIN_MOVING
                                     : FUSION_COMPASS_GAIN_STATIONARY;
      const float correction = constrain(compassError * gain,
                                         -FUSION_COMPASS_MAX_STEP_DEG,
                                         FUSION_COMPASS_MAX_STEP_DEG);
      headingDeg_ = normalize(headingDeg_ + correction);
    }
    changed = true;
  }

  if (newImuSample) {
    yawRateDegS_ += FUSION_YAW_RATE_FILTER *
                    (rateSampleDegS - yawRateDegS_);
  } else if (!robotMoving) {
    yawRateDegS_ *= 0.95f;
  }

  usingCompass_ = compassOk;
  usingImu_ = imuOk;
  usingEncoder_ = encoderOk;
  const uint8_t sourceCount = static_cast<uint8_t>(compassOk) +
                              static_cast<uint8_t>(imuOk) +
                              static_cast<uint8_t>(encoderOk);
  if (!initialized_ || sourceCount == 0U) {
    health_ = FusionHealth::NO_SOURCE;
    confidencePct_ = 0.0f;
    sourceText_ = "NONE";
  } else if (sourceCount >= 2U) {
    health_ = FusionHealth::FUSED;
    confidencePct_ = sourceCount == 3U ? 96.0f : 84.0f;
    if (imuOk && encoderOk && effectiveEncoderWeight_ < FUSION_ENCODER_DELTA_WEIGHT) {
      confidencePct_ = 84.0f + 12.0f * effectiveEncoderWeight_ /
                                     FUSION_ENCODER_DELTA_WEIGHT;
    }
    if (imuOk && encoderOk && compassOk) sourceText_ = "I+E+C";
    else if (imuOk && encoderOk) sourceText_ = "I+E";
    else if (imuOk && compassOk) sourceText_ = "I+C";
    else sourceText_ = "E+C";
  } else {
    health_ = FusionHealth::DEGRADED;
    confidencePct_ = 62.0f;
    sourceText_ = imuOk ? "IMU" : (encoderOk ? "ENC" : "COMP");
  }
  if (changed) ++sampleSequence_;
}

float HeadingFusion::headingRad() const {
  return headingDeg_ * kDegToRad;
}

const char* HeadingFusion::healthText() const {
  switch (health_) {
    case FusionHealth::NO_SOURCE: return "LOST";
    case FusionHealth::DEGRADED: return "DEG";
    case FusionHealth::FUSED: return "OK";
  }
  return "UNKNOWN";
}

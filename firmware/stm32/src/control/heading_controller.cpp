#include <control/heading_controller.h>
#include <robot_config.h>

void HeadingController::begin() {
  // kp: heading error -> desired yaw rate; ki: slow wheel-trim learner;
  // kd: yaw-rate loop gain. Kept behind setTunings for future calibration.
  setTunings(HEADING_ANGLE_TO_RATE_GAIN, HEADING_INTEGRAL_GAIN,
             HEADING_YAW_RATE_GAIN);
  reset();
}

void HeadingController::setTunings(float kp, float ki, float kd) {
  kp_ = constrain(kp, 0.0f, 10.0f);
  ki_ = constrain(ki, 0.0f, 1.0f);
  kd_ = constrain(kd, 0.0f, 10.0f);
}

float HeadingController::shortestError(float target, float current) {
  float error = target - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

void HeadingController::capture(float currentHeading) {
  // A disturbed session stays latched until RobotController explicitly ends
  // it with reset(). This prevents the normal "capture if inactive" path
  // from immediately re-enabling a poisoned Compass signal.
  if (disturbed_) return;
  targetHeading_ = currentHeading;
  active_ = true;
  error_ = 0.0f;
  integral_ = 0.0f;
  lastHeading_ = currentHeading;
  yawRateFiltered_ = 0.0f;
  desiredYawRate_ = 0.0f;
  output_ = 0.0f;
  lastUpdateMs_ = millis();
  headingReady_ = true;
  headingLocked_ = true;
  wrongResponseTimeS_ = 0.0f;
}

void HeadingController::reset() {
  active_ = false;
  error_ = 0.0f;
  integral_ = 0.0f;
  lastHeading_ = 0.0f;
  yawRateFiltered_ = 0.0f;
  desiredYawRate_ = 0.0f;
  output_ = 0.0f;
  lastUpdateMs_ = 0;
  headingReady_ = false;
  headingLocked_ = true;
  disturbed_ = false;
  wrongResponseTimeS_ = 0.0f;
}

int16_t HeadingController::update(float currentHeading,
                                  int16_t commandMagnitude) {
  if (!active_) return 0;
  const uint32_t nowMs = millis();
  float dt = lastUpdateMs_ == 0U
      ? 0.02f
      : static_cast<float>(nowMs - lastUpdateMs_) / 1000.0f;
  lastUpdateMs_ = nowMs;
  dt = constrain(dt, HEADING_MIN_DT_S, HEADING_MAX_DT_S);
  const float yawRateRaw = headingReady_
      ? shortestError(currentHeading, lastHeading_) / dt
      : 0.0f;
  headingReady_ = true;
  lastHeading_ = currentHeading;
  yawRateFiltered_ += HEADING_YAW_RATE_FILTER *
                      (yawRateRaw - yawRateFiltered_);

  error_ = shortestError(targetHeading_, currentHeading);
  const float absError = fabsf(error_);
  if (headingLocked_) {
    if (absError > HEADING_LOCK_EXIT_DEG) headingLocked_ = false;
  } else if (absError < HEADING_LOCK_ENTER_DEG) {
    headingLocked_ = true;
  }

  // Closed-loop plausibility test. On this chassis a positive correction
  // (left faster, right slower) must create negative yaw. A sustained
  // correction with missing/opposite response means that magnetic/UART motor
  // interference is steering the estimator rather than the chassis.
  if (!headingLocked_ && fabsf(output_) >= HEADING_RESPONSE_TEST_OUTPUT) {
    const float responseProduct = output_ * yawRateFiltered_;
    const bool respondingCorrectly =
        responseProduct <= -(HEADING_RESPONSE_TEST_OUTPUT *
                             HEADING_RESPONSE_TEST_RATE_DEG_S);
    if (respondingCorrectly) {
      wrongResponseTimeS_ = max(
          0.0f, wrongResponseTimeS_ -
                    HEADING_RESPONSE_RECOVERY_MULTIPLIER * dt);
    } else {
      wrongResponseTimeS_ += dt;
    }
  } else {
    wrongResponseTimeS_ = max(
        0.0f, wrongResponseTimeS_ -
                  HEADING_RESPONSE_RECOVERY_MULTIPLIER * dt);
  }
  if (wrongResponseTimeS_ >= HEADING_RESPONSE_FAULT_TIME_S) {
    disturbed_ = true;
    ++disturbanceCount_;
    active_ = false;
    integral_ = 0.0f;
    output_ = 0.0f;
    desiredYawRate_ = 0.0f;
    return 0;
  }
  const float controlError = headingLocked_ ? 0.0f : error_;
  desiredYawRate_ = constrain(
      kp_ * controlError,
      -HEADING_MAX_DESIRED_YAW_RATE_DEG_S,
      HEADING_MAX_DESIRED_YAW_RATE_DEG_S);

  if (headingLocked_) {
    // Remove old path trim smoothly after the heading has settled.
    integral_ *= constrain(1.0f - 0.8f * dt, 0.0f, 1.0f);
  }
  const float integralCandidate = constrain(
      integral_ + controlError * dt,
      -HEADING_INTEGRAL_LIMIT, HEADING_INTEGRAL_LIMIT);
  float limit = static_cast<float>(commandMagnitude) *
                HEADING_MAX_CORRECTION_RATIO;
  if (limit < HEADING_MIN_CORRECTION_LIMIT) {
    limit = HEADING_MIN_CORRECTION_LIMIT;
  }
  // The measured chassis polarity is negative: a positive wheel correction
  // produces a negative Compass yaw. Outer rate command plus inner rate
  // feedback therefore have this sign. The integral learns slow L/R mismatch.
  float raw = kd_ * (yawRateFiltered_ - desiredYawRate_) -
              ki_ * integralCandidate;
  // Conditional integration: accept normal integration or a step which moves
  // a saturated output back toward its usable range.
  if (fabsf(raw) < limit ||
      (raw > limit && controlError > 0.0f) ||
      (raw < -limit && controlError < 0.0f)) {
    integral_ = integralCandidate;
  }
  raw = kd_ * (yawRateFiltered_ - desiredYawRate_) - ki_ * integral_;
  raw = constrain(raw, -limit, limit);

  // Far from the target, never steer in the wrong direction. Inside the small
  // brake window the rate loop may counter-steer gently to remove inertia;
  // this is what prevents repeated left/right overshoot.
  const float requiredCorrection = -controlError;
  if (absError > HEADING_BRAKE_WINDOW_DEG &&
      requiredCorrection != 0.0f && raw * requiredCorrection < 0.0f) {
    raw = 0.0f;
  }
  if (absError > HEADING_BRAKE_WINDOW_DEG &&
      requiredCorrection != 0.0f &&
      output_ * requiredCorrection < 0.0f) {
    output_ = 0.0f;
  }

  const float slew = HEADING_CORRECTION_SLEW_PER_SECOND * dt;
  const float outputDelta = constrain(raw - output_, -slew, slew);
  output_ += outputDelta;
  if (headingLocked_ && fabsf(yawRateFiltered_) <
      HEADING_YAW_RATE_STOP_DEG_S && fabsf(output_) < 0.6f) {
    output_ = 0.0f;
  }
  return static_cast<int16_t>(roundf(output_));
}

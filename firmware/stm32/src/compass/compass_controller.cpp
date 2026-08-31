#include <compass/compass_controller.h>
#include <robot_config.h>

CompassController::CompassController()
    : device_(COMPASS_RX_STM32, COMPASS_TX_STM32) {}

float CompassController::normalize(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float CompassController::shortestDelta(float target, float current) {
  return normalize(target - current);
}

void CompassController::begin() {
  device_.begin(COMPASS_BAUD);
  device_.reset();
  zeroPending_ = true;
  filterReady_ = false;
  lastRequestMs_ = millis();
  driftCalibrating_ = false;
  driftCompensationDeg_ = 0.0f;
  relativeAngleReady_ = false;
  stationaryHoldActive_ = true;
  stationaryHoldAngleDeg_ = 0.0f;
  resetAdaptiveWindow();
}

bool CompassController::isConnected() const {
  return lastGoodMs_ != 0U && (millis() - lastGoodMs_) < COMPASS_LOST_MS &&
         consecutiveRejectedSamples_ < COMPASS_REJECT_DISCONNECT_COUNT;
}

void CompassController::beginDriftCalibration(uint32_t nowMs) {
  driftCalibrating_ = true;
  driftCalStartMs_ = nowMs;
  driftFirstMs_ = 0;
  driftLastMs_ = 0;
  driftSamples_ = 0;
  driftSumTime_ = 0.0f;
  driftSumAngle_ = 0.0f;
  driftSumTimeSquared_ = 0.0f;
  driftSumTimeAngle_ = 0.0f;
  driftRateDegS_ = 0.0f;
  driftReferenceMs_ = 0;
  driftCompensationDeg_ = 0.0f;
  calibrationHoldActive_ = !robotMoving_;
  calibrationHoldAngleDeg_ = filteredAngleDeg_;
  resetAdaptiveWindow();
}

void CompassController::addCalibrationSample(float relativeAngle,
                                             uint32_t nowMs) {
  // Ignore the Compass module's reset/warm-up transient. The timed drift
  // window begins only after this settling interval.
  if (driftFirstMs_ == 0U &&
      (nowMs - driftCalStartMs_) < COMPASS_DRIFT_SETTLE_MS) {
    return;
  }
  if (driftSamples_ == 0U) {
    driftFirstMs_ = nowMs;
    driftLastMs_ = nowMs;
    driftFirstUnwrapped_ = relativeAngle;
    driftLastUnwrapped_ = relativeAngle;
    driftPreviousWrapped_ = relativeAngle;
  } else {
    driftLastUnwrapped_ += shortestDelta(relativeAngle, driftPreviousWrapped_);
    driftPreviousWrapped_ = relativeAngle;
    driftLastMs_ = nowMs;
  }
  const float sampleTime =
      static_cast<float>(nowMs - driftFirstMs_) / 1000.0f;
  driftSumTime_ += sampleTime;
  driftSumAngle_ += driftLastUnwrapped_;
  driftSumTimeSquared_ += sampleTime * sampleTime;
  driftSumTimeAngle_ += sampleTime * driftLastUnwrapped_;
  if (driftSamples_ < 65535U) ++driftSamples_;
  if (driftFirstMs_ != 0U &&
      (nowMs - driftFirstMs_) >= COMPASS_DRIFT_CAL_MS)
    finishDriftCalibration();
}

void CompassController::finishDriftCalibration() {
  if (driftSamples_ >= COMPASS_DRIFT_MIN_SAMPLES && driftLastMs_ > driftFirstMs_) {
    const float seconds = static_cast<float>(driftLastMs_ - driftFirstMs_) / 1000.0f;
    if (seconds > 0.2f) {
      const float travel = driftLastUnwrapped_ - driftFirstUnwrapped_;
      const float count = static_cast<float>(driftSamples_);
      const float denominator =
          count * driftSumTimeSquared_ - driftSumTime_ * driftSumTime_;
      const float measured = fabsf(denominator) > 0.0001f
          ? (count * driftSumTimeAngle_ -
             driftSumTime_ * driftSumAngle_) / denominator
          : travel / seconds;
      // Never convert a real rotation into a capped drift rate. That old
      // behavior creates an artificial angle ramp for the rest of the run.
      if (fabsf(measured) <= COMPASS_MAX_ACCEPTED_DRIFT_DEG_S &&
          fabsf(travel) <= COMPASS_MAX_CALIBRATION_TRAVEL_DEG) {
        driftRateDegS_ = measured;
      } else {
        driftRateDegS_ = 0.0f;
      }
    }
  }
  driftCalibrating_ = false;
  driftReferenceMs_ = driftLastMs_;
  // The output is deliberately held while the robot is stationary during
  // initial calibration. Rebase accumulated bias here so calibration ends
  // without a visible 0.1-degree staircase or a heading-target jump.
  driftCompensationDeg_ =
      normalize(driftPreviousWrapped_ - filteredAngleDeg_);
  calibrationHoldActive_ = false;
  stationaryHoldActive_ = !robotMoving_;
  stationaryHoldAngleDeg_ = filteredAngleDeg_;
  resetAdaptiveWindow();
}

void CompassController::resetAdaptiveWindow() {
  adaptiveFirstMs_ = 0;
  adaptiveLastMs_ = 0;
  adaptiveSamples_ = 0;
  adaptiveTravelAbs_ = 0.0f;
  adaptiveSumTime_ = 0.0f;
  adaptiveSumAngle_ = 0.0f;
  adaptiveSumTimeSquared_ = 0.0f;
  adaptiveSumTimeAngle_ = 0.0f;
}

void CompassController::setRobotMoving(bool moving) {
  setMotionMode(moving ? CompassMotionMode::STRAIGHT
                       : CompassMotionMode::STATIONARY);
}

void CompassController::setMotionMode(CompassMotionMode mode) {
  const bool moving = mode != CompassMotionMode::STATIONARY;
  if (mode != motionMode_) {
    motionMode_ = mode;
    // A motor-state transition changes the EMI environment. Do not carry a
    // rejected UART burst into the new state, but retain the last accepted
    // angle as the rate-gate reference.
    consecutiveRejectedSamples_ = 0;
  }
  if (moving == robotMoving_) return;
  robotMoving_ = moving;
  resetAdaptiveWindow();
  if (driftCalibrating_ && moving) {
    // Commit the quiet samples collected so far before the first drive. The
    // previous implementation discarded them, exposed the hidden raw drift
    // as soon as stationary hold released, and made heading control steer a
    // stationary raised-wheel chassis. If motion starts extremely early there
    // are not enough samples, so retain the safe zero-rate fallback.
    if (driftSamples_ >= COMPASS_DRIFT_MIN_SAMPLES) {
      finishDriftCalibration();
    } else {
      driftCalibrating_ = false;
      calibrationHoldActive_ = false;
      driftReferenceMs_ = millis();
    }
  }
  if (moving) {
    // Start every drive session with the current held heading and no jump.
    // Subsequent raw changes are therefore real yaw for heading control.
    if (relativeAngleReady_) {
      driftCompensationDeg_ =
          normalize(lastRelativeAngleDeg_ - filteredAngleDeg_);
      driftReferenceMs_ = millis();
    }
    stationaryHoldActive_ = false;
    adaptiveBlockedUntilMs_ = millis() + COMPASS_ADAPTIVE_ROTATION_GUARD_MS;
  } else {
    stationaryHoldActive_ = true;
    stationaryHoldAngleDeg_ = filteredAngleDeg_;
  }
}

void CompassController::rejectSample(float relativeAngle) {
  (void)relativeAngle;
  ++rejectedSamples_;
  if (consecutiveRejectedSamples_ < 255U) {
    ++consecutiveRejectedSamples_;
  }
}

bool CompassController::validateSample(int16_t rawValue, uint32_t nowMs) {
  // The first response establishes the local zero and has no predecessor.
  if (zeroPending_ || !relativeAngleReady_ || lastGoodMs_ == 0U) return true;

  const float relative = normalize(
      (static_cast<float>(rawValue) - zeroRaw_) / COMPASS_SCALE);
  uint32_t dtMs = nowMs - lastGoodMs_;
  // Never make the gate wider just because bad samples have been rejected.
  // Otherwise a persistent EMI-induced offset eventually validates itself.
  dtMs = constrain(dtMs, 1U, COMPASS_RATE_GATE_MAX_DT_MS);
  const float maxRate = motionMode_ == CompassMotionMode::STRAIGHT
                            ? COMPASS_STRAIGHT_MAX_RATE_DEG_S
                            : COMPASS_TURN_MAX_RATE_DEG_S;
  const float allowedStep = COMPASS_RATE_GATE_MARGIN_DEG +
      maxRate * static_cast<float>(dtMs) / 1000.0f;
  if (fabsf(shortestDelta(relative, lastRelativeAngleDeg_)) > allowedStep) {
    rejectSample(relative);
    return false;
  }
  consecutiveRejectedSamples_ = 0;
  return true;
}

void CompassController::addAdaptiveSample(float relativeAngle,
                                          uint32_t nowMs) {
  if (robotMoving_ || static_cast<int32_t>(nowMs - adaptiveBlockedUntilMs_) < 0) {
    resetAdaptiveWindow();
    return;
  }

  if (adaptiveSamples_ == 0U) {
    adaptiveFirstMs_ = nowMs;
    adaptiveLastMs_ = nowMs;
    adaptiveFirstUnwrapped_ = relativeAngle;
    adaptiveLastUnwrapped_ = relativeAngle;
    adaptivePreviousWrapped_ = relativeAngle;
  } else {
    const float step = shortestDelta(relativeAngle, adaptivePreviousWrapped_);
    if (fabsf(step) > COMPASS_ADAPTIVE_MAX_STEP_DEG) {
      resetAdaptiveWindow();
      adaptiveBlockedUntilMs_ = nowMs + COMPASS_ADAPTIVE_ROTATION_GUARD_MS;
      return;
    }
    adaptiveLastUnwrapped_ += step;
    adaptiveTravelAbs_ += fabsf(step);
    adaptivePreviousWrapped_ = relativeAngle;
    adaptiveLastMs_ = nowMs;
    if (adaptiveTravelAbs_ > COMPASS_ADAPTIVE_MAX_TRAVEL_DEG) {
      resetAdaptiveWindow();
      adaptiveBlockedUntilMs_ = nowMs + COMPASS_ADAPTIVE_ROTATION_GUARD_MS;
      return;
    }
  }

  const float sampleTime =
      static_cast<float>(nowMs - adaptiveFirstMs_) / 1000.0f;
  adaptiveSumTime_ += sampleTime;
  adaptiveSumAngle_ += adaptiveLastUnwrapped_;
  adaptiveSumTimeSquared_ += sampleTime * sampleTime;
  adaptiveSumTimeAngle_ += sampleTime * adaptiveLastUnwrapped_;
  if (adaptiveSamples_ < 65535U) ++adaptiveSamples_;
  if ((nowMs - adaptiveFirstMs_) >= COMPASS_ADAPTIVE_WINDOW_MS) {
    finishAdaptiveWindow();
  }
}

void CompassController::finishAdaptiveWindow() {
  if (adaptiveSamples_ >= COMPASS_ADAPTIVE_MIN_SAMPLES &&
      adaptiveLastMs_ > adaptiveFirstMs_) {
    const float count = static_cast<float>(adaptiveSamples_);
    const float denominator = count * adaptiveSumTimeSquared_ -
                              adaptiveSumTime_ * adaptiveSumTime_;
    const float seconds =
        static_cast<float>(adaptiveLastMs_ - adaptiveFirstMs_) / 1000.0f;
    const float travel = adaptiveLastUnwrapped_ - adaptiveFirstUnwrapped_;
    const float measured = fabsf(denominator) > 0.0001f
        ? (count * adaptiveSumTimeAngle_ -
           adaptiveSumTime_ * adaptiveSumAngle_) / denominator
        : travel / seconds;
    if (fabsf(measured) <= COMPASS_MAX_ACCEPTED_DRIFT_DEG_S &&
        adaptiveTravelAbs_ <= COMPASS_ADAPTIVE_MAX_TRAVEL_DEG) {
      driftRateDegS_ =
          COMPASS_ADAPTIVE_RATE_BLEND * measured +
          (1.0f - COMPASS_ADAPTIVE_RATE_BLEND) * driftRateDegS_;
    }
  }
  resetAdaptiveWindow();
}

void CompassController::updateDriftCompensation(uint32_t nowMs) {
  if (driftCalibrating_ || driftReferenceMs_ == 0U) return;
  const float seconds =
      static_cast<float>(nowMs - driftReferenceMs_) / 1000.0f;
  driftCompensationDeg_ =
      normalize(driftCompensationDeg_ + driftRateDegS_ * seconds);
  driftReferenceMs_ = nowMs;
}

float CompassController::compensateDrift(float relativeAngle) const {
  if (driftCalibrating_ || driftReferenceMs_ == 0U) return relativeAngle;
  return normalize(relativeAngle - driftCompensationDeg_);
}

void CompassController::acceptSample(int16_t rawValue, uint32_t nowMs) {
  raw_ = rawValue;
  rawAngleDeg_ = normalize(static_cast<float>(rawValue) / COMPASS_SCALE);
  lastSampleDtMs_ = lastGoodMs_ == 0U ? 0U : nowMs - lastGoodMs_;
  lastGoodMs_ = nowMs;
  ++sampleSequence_;

  if (zeroPending_) {
    zeroRaw_ = static_cast<float>(rawValue);
    filteredAngleDeg_ = 0.0f;
    angleDeg_ = 0.0f;
    filterReady_ = true;
    zeroPending_ = false;
    ++zeroGeneration_;
    beginDriftCalibration(nowMs);
  }

  const float relative = normalize(
      (static_cast<float>(rawValue) - zeroRaw_) / COMPASS_SCALE);
  const float rawStep = relativeAngleReady_
      ? shortestDelta(relative, lastRelativeAngleDeg_)
      : 0.0f;
  lastRelativeAngleDeg_ = relative;
  relativeAngleReady_ = true;
  if (driftCalibrating_) {
    if (!robotMoving_) addCalibrationSample(relative, nowMs);
  } else {
    updateDriftCompensation(nowMs);
    addAdaptiveSample(relative, nowMs);
  }
  float candidate = compensateDrift(relative);
  if (driftCalibrating_ && calibrationHoldActive_ && !robotMoving_) {
    candidate = calibrationHoldAngleDeg_;
  } else if (!robotMoving_) {
    // A single sensor count (0.102 degree) is bias/quantization. A larger
    // per-sample change is deliberate manual rotation and temporarily
    // releases the hold. Motor-driven turns always release it immediately.
    if (fabsf(rawStep) > COMPASS_MANUAL_ROTATION_STEP_DEG) {
      stationaryHoldActive_ = false;
      stationaryManualMotionMs_ = nowMs;
    } else if (!stationaryHoldActive_ &&
               (nowMs - stationaryManualMotionMs_) >=
                   COMPASS_MANUAL_ROTATION_SETTLE_MS) {
      stationaryHoldActive_ = true;
      stationaryHoldAngleDeg_ = filteredAngleDeg_;
      driftCompensationDeg_ = normalize(relative - filteredAngleDeg_);
      driftReferenceMs_ = nowMs;
    }
    if (stationaryHoldActive_) candidate = stationaryHoldAngleDeg_;
  }

  if (!filterReady_) {
    filteredAngleDeg_ = candidate;
    filterReady_ = true;
  } else {
    float delta = shortestDelta(candidate, filteredAngleDeg_);
    // validateSample() has already applied a time- and motion-aware rate
    // gate. Keeping a second fixed 45-degree gate here would hide rejection
    // state from the health logic and allow invalid data to look connected.
    if (fabsf(delta) <= COMPASS_NOISE_DEADBAND_DEG) delta = 0.0f;
    const float alpha = fabsf(delta) >= COMPASS_FAST_DELTA_DEG
                            ? COMPASS_ALPHA_FAST
                            : COMPASS_ALPHA_SLOW;
    filteredAngleDeg_ = normalize(filteredAngleDeg_ + alpha * delta);
  }
  angleDeg_ = filteredAngleDeg_;
}

void CompassController::update() {
  const uint32_t now = millis();
  if (!device_.readPending() && (now - lastRequestMs_) >= COMPASS_POLL_MS) {
    if (device_.requestRead()) lastRequestMs_ = now;
  }
  int16_t value = 0;
  if (device_.pollRead(value) && validateSample(value, now)) {
    acceptSample(value, now);
  }
}

void CompassController::resetZero() {
  device_.reset();
  zeroPending_ = true;
  filterReady_ = false;
  angleDeg_ = 0.0f;
  driftCalibrating_ = false;
  driftRateDegS_ = 0.0f;
  driftReferenceMs_ = 0;
  driftCompensationDeg_ = 0.0f;
  calibrationHoldActive_ = true;
  calibrationHoldAngleDeg_ = 0.0f;
  relativeAngleReady_ = false;
  stationaryHoldActive_ = true;
  stationaryHoldAngleDeg_ = 0.0f;
  stationaryManualMotionMs_ = 0;
  motionMode_ = CompassMotionMode::STATIONARY;
  robotMoving_ = false;
  consecutiveRejectedSamples_ = 0;
  resetAdaptiveWindow();
  lastRequestMs_ = millis();
}

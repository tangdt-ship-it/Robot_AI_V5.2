#include <control/robot_controller.h>
#include <robot_config.h>

RobotController::RobotController(MotorController& motors, Ps2Controller& ps2,
                                 CompassController& compass, LcdDisplay& display,
                                 HeadingController& heading,
                                 UltrasonicSensor& ultrasonic,
                                 WheelOdometry& odometry, Mpu6050& imu,
                                 HeadingFusion& fusion, Print& debugStream)
    : motors_(motors), ps2_(ps2), compass_(compass), display_(display),
      heading_(heading), ultrasonic_(ultrasonic), odometry_(odometry),
      imu_(imu), fusion_(fusion), debug_(debugStream) {}

const char* RobotController::motionOwnerText(MotionOwner owner) {
  switch (owner) {
    case MotionOwner::MCP: return "MCP";
    case MotionOwner::MISSION: return "MISSION";
    case MotionOwner::REPLAY: return "REPLAY";
    case MotionOwner::DIAGNOSTIC: return "DIAGNOSTIC";
    case MotionOwner::PS2: return "PS2";
    case MotionOwner::NONE: return "NONE";
  }
  return "NONE";
}

void RobotController::begin() {
  speedSetting_ = SPEED_DEFAULT;
  heading_.begin();
  state_ = RobotState::STOP;
  targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
  stopPwmLatched_ = true;
  motionOwner_ = MotionOwner::NONE;
  lastControlMs_ = millis();
  motors_.stop();
  compass_.setRobotMoving(false);
}

bool RobotController::headingAvailable() const {
  return fusion_.ready() || compass_.isConnected() || odometry_.healthy();
}

float RobotController::currentHeadingDeg() const {
  if (fusion_.ready()) return fusion_.headingDeg();
  if (compass_.isConnected()) return compass_.getAngle();
  return odometry_.data().encoderHeadingRad * 57.29577951308232f;
}

uint32_t RobotController::currentHeadingSequence() const {
  if (fusion_.ready()) return fusion_.sampleSequence();
  return compass_.sampleSequence();
}

void RobotController::changeSpeed(int8_t direction) {
  speedSetting_ = constrain(speedSetting_ + direction * SPEED_ADJUST_STEP,
                            SPEED_MIN, SPEED_MAX);
}

bool RobotController::setSpeedSetting(int16_t speed) {
  if (speed < SPEED_MIN || speed > SPEED_MAX) return false;
  speedSetting_ = speed;
  return true;
}

void RobotController::setBrakeEnabled(bool enabled) {
  obstacleLimited_ = false;
  if (brakeEnabled_ == enabled) {
    if (enabled) motors_.brake();
    return;
  }
  brakeEnabled_ = enabled;
  targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
  aiMotionMode_ = AiMotionMode::NONE;
  aiMotionDeadlineMs_ = 0;
  straightCommand_ = false;
  reversing_ = false;
  heading_.reset();
  lastHeadingCorrection_ = 0;
  state_ = RobotState::STOP;
  compass_.setRobotMoving(false);
  if (enabled) {
    stopPwmLatched_ = false;
    motors_.brake();
  } else if (obstacleBrakeActive_) {
    motors_.brake();
  } else {
    stopPwmLatched_ = true;
    motors_.stop();
  }
#if ROBOT_DEBUG
  debug_.print("BRAKE,LOCK="); debug_.print(enabled ? 1 : 0);
  debug_.print(",PWM="); debug_.println(enabled ? BRAKE_PWM_LOCK : PWM_STOP);
#endif
}

void RobotController::updateSpeedRepeat(uint32_t nowMs) {
  if (!speedHolding_) return;
  const Ps2State& state = ps2_.state();
  const bool held = (speedDirection_ > 0 && state.r1) ||
                    (speedDirection_ < 0 && state.l1);
  if (!held) { speedHolding_ = false; speedDirection_ = 0; return; }
  if (static_cast<int32_t>(nowMs - speedNextRepeatMs_) < 0) return;
  const uint32_t heldMs = nowMs - speedHoldStartMs_;
  const uint32_t repeatMs = heldMs >= 1800U ? 45U : (heldMs >= 900U ? 80U : 120U);
  changeSpeed(speedDirection_);
  speedNextRepeatMs_ = nowMs + repeatMs;
}

void RobotController::handleFunctionButtons() {
  const uint32_t now = millis();
  if (ps2_.buttonPressed(Ps2Button::R1)) {
    changeSpeed(1); speedHolding_ = true; speedDirection_ = 1;
    speedHoldStartMs_ = now; speedNextRepeatMs_ = now + 350U;
  } else if (ps2_.buttonPressed(Ps2Button::L1)) {
    changeSpeed(-1); speedHolding_ = true; speedDirection_ = -1;
    speedHoldStartMs_ = now; speedNextRepeatMs_ = now + 350U;
  }
  if (ps2_.buttonReleased(Ps2Button::R1) ||
      ps2_.buttonReleased(Ps2Button::L1)) {
    speedHolding_ = false; speedDirection_ = 0;
  }
  // Keep the legacy L2 compass-zero command, but detect the edge locally.
  // Some PS2 receiver firmwares do not preserve ButtonPressed() edge state
  // across a frame while another task is servicing the bus.
  // PS2 protocol buttons are active-low. Keep a raw-frame fallback because
  // some receiver/library combinations update rawButtons() before the
  // convenience Button() state; 0xFEFF is the canonical L2-down frame.
  const bool l2RawDown = (ps2_.rawButtons() & PSB_L2) == 0U;
  const bool l2Down = ps2_.state().l2 || l2RawDown;
  if (l2Down && !compassResetHeld_) {
    resetHeadingReference();
#if ROBOT_DEBUG
    debug_.println("COMPASS,RESET=PS2_L2");
#endif
  }
  compassResetHeld_ = l2Down;
  if (ps2_.buttonPressed(Ps2Button::R2)) {
    // Counter zeroing is a generation boundary.  Replay must never combine
    // samples from before and after this operation into one segment.
    odometry_.resetWheelCounts(EncoderResetReason::PS2_R2);
    display_.forceRefresh();
#if ROBOT_DEBUG
    debug_.print("ENCODER,RESET=PS2_R2,GEN=");
    debug_.println(odometry_.resetGeneration());
#endif
  }
  if (ps2_.buttonPressed(Ps2Button::START))
    setBrakeEnabled(!brakeEnabled_);
  if (ps2_.buttonPressed(Ps2Button::CROSS)) rampEnabled_ = !rampEnabled_;
  freeStop_ = ps2_.state().r3;
}

void RobotController::resetHeadingReference() {
  const float before = currentHeadingDeg();
  compass_.resetZero();
  heading_.reset();
  // Keep this identical to the proven L2 behavior, including the fusion
  // reference and immediate LCD refresh while the compass may be LOST.
  fusion_.reset(0.0f, odometry_.data().encoderHeadingRad);
  display_.forceRefresh();
#if ROBOT_DEBUG
  debug_.print("HDG_BEFORE=");
  debug_.print(before, 1);
  debug_.print(",HDG_AFTER=0.0,STM32_RESET_HANDLER_CALLED=1");
  debug_.println();
#endif
}

void RobotController::setMotionCommand(int16_t left, int16_t right,
                                       RobotState state,
                                       bool headingCandidate,
                                       bool reversing) {
  targetLeft_ = constrain(left, -255, 255);
  targetRight_ = constrain(right, -255, 255);
  if (state == RobotState::JOYSTICK_DRIVE || state == RobotState::FORWARD ||
      state == RobotState::BACKWARD || state == RobotState::TURN_LEFT ||
      state == RobotState::TURN_RIGHT) {
    motionOwner_ = MotionOwner::PS2;
  }
  if (targetLeft_ != 0 || targetRight_ != 0) stopPwmLatched_ = false;
  const bool newHeadingSession =
      headingCandidate && (!straightCommand_ || reversing_ != reversing ||
                           state_ != state);
  state_ = state;
  straightCommand_ = headingCandidate;
  reversing_ = reversing;
  if (straightCommand_ && headingAvailable()) {
    if (newHeadingSession) {
      heading_.reset();
      headingSuppressed_ = false;
      heading_.capture(currentHeadingDeg());
      lastHeadingSequence_ = 0;
      lastHeadingCorrection_ = 0;
#if ROBOT_DEBUG
      debug_.print("HEADING,CAPTURE=");
      debug_.print(heading_.targetHeading(), 2);
      debug_.print(",DIR=");
      debug_.println(reversing ? "REV" : "FWD");
#endif
    }
  } else {
    heading_.reset();
    lastHeadingCorrection_ = 0;
    if (!straightCommand_) headingSuppressed_ = false;
  }
}

void RobotController::calculateMotionCommand() {
  const Ps2State& state = ps2_.state();
  if (state.up) {
    setMotionCommand(speedSetting_, speedSetting_, RobotState::FORWARD, true, false);
    return;
  }
  if (state.down) {
    const int16_t speed = min(speedSetting_, REVERSE_SPEED_MAX);
    setMotionCommand(-speed, -speed, RobotState::BACKWARD, true, true);
    return;
  }
  if (state.left) {
    setMotionCommand(-speedSetting_, speedSetting_, RobotState::TURN_LEFT, false, false);
    return;
  }
  if (state.right) {
    setMotionCommand(speedSetting_, -speedSetting_, RobotState::TURN_RIGHT, false, false);
    return;
  }

  const int16_t drive = static_cast<int16_t>(
      static_cast<int32_t>(state.ly) * speedSetting_ / 1000L);
  int16_t rotate = static_cast<int16_t>(
      static_cast<int32_t>(state.lx) * speedSetting_ / 1000L);
  const int16_t rotateLimit = static_cast<int16_t>(speedSetting_ * 60L / 100L);
  rotate = constrain(rotate, -rotateLimit, rotateLimit);
  if (drive < 0) rotate = -rotate;
  const int16_t left = constrain(drive + rotate, -255, 255);
  const int16_t right = constrain(drive - rotate, -255, 255);
  if (left == 0 && right == 0) {
    targetLeft_ = targetRight_ = 0;
    state_ = RobotState::STOP;
    straightCommand_ = false;
    reversing_ = false;
    heading_.reset();
    return;
  }
  const bool straight = state.lx == 0 && drive != 0;
  setMotionCommand(left, right, RobotState::JOYSTICK_DRIVE, straight, drive < 0);
}

void RobotController::handleManualControl() { calculateMotionCommand(); }

int16_t RobotController::rampToward(int16_t current, int16_t target) {
  if (current < target) {
    current += RAMP_STEP;
    if (current < RAMP_START && target > 0) current = RAMP_START;
    if (current > target) current = target;
  } else if (current > target) {
    current -= RAMP_STEP;
    if (current > -RAMP_START && target < 0) current = -RAMP_START;
    if (current < target) current = target;
  }
  return current;
}

void RobotController::updateRamp() {
  if (targetLeft_ == 0 && targetRight_ == 0) {
    currentLeft_ = currentRight_ = 0;
    return;
  }
  if (!rampEnabled_) {
    currentLeft_ = targetLeft_;
    currentRight_ = targetRight_;
    return;
  }
  currentLeft_ = rampToward(currentLeft_, targetLeft_);
  currentRight_ = rampToward(currentRight_, targetRight_);
}

void RobotController::applyMotorCommand() {
  updateRamp();
  int16_t left = currentLeft_;
  int16_t right = currentRight_;
  if (brakeEnabled_) {
    targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
    state_ = RobotState::STOP;
    straightCommand_ = false;
    heading_.reset();
    lastHeadingCorrection_ = 0;
    compass_.setRobotMoving(false);
    motors_.brake();
    return;
  }

  // Limit only the forward translation component. The turn component and
  // every reverse command remain available so the operator/AI can escape.
  obstacleLimited_ = false;
  const int16_t forward = static_cast<int16_t>((left + right) / 2);
  const int16_t turn = static_cast<int16_t>((left - right) / 2);
  if (forward > 0) {
    const int16_t limitedForward =
        ultrasonic_.limitForwardCommand(forward);
    if (limitedForward < forward) {
      obstacleLimited_ = true;
      left = constrain(limitedForward + turn, -255, 255);
      right = constrain(limitedForward - turn, -255, 255);
      if (limitedForward == 0 && abs(turn) < OBSTACLE_MIN_FORWARD_COMMAND) {
        const bool aiWasDriving = aiMotionMode_ == AiMotionMode::PULSE ||
                                  aiMotionMode_ == AiMotionMode::CONTINUOUS ||
                                  aiMotionMode_ == AiMotionMode::DISTANCE;
        if (aiWasDriving) {
          aiMotionMode_ = AiMotionMode::NONE;
          aiMotionDeadlineMs_ = 0;
        }
        targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
        state_ = RobotState::STOP;
        straightCommand_ = false;
        heading_.reset();
        lastHeadingCorrection_ = 0;
        obstacleBrakeActive_ = true;
        obstacleReleaseDistanceCm_ = min(
            static_cast<float>(OBSTACLE_CAUTION_CM + OBSTACLE_HYSTERESIS_CM),
            ultrasonic_.stoppingDistanceCm(forward) +
                OBSTACLE_HYSTERESIS_CM);
        compass_.setRobotMoving(false);
        motors_.brake();
#if ROBOT_DEBUG
        debug_.print("OBSTACLE,ACTION=BRAKE,DIST=");
        debug_.print(ultrasonic_.distanceCm(), 1);
        debug_.print(",AI_CANCEL="); debug_.println(aiWasDriving ? 1 : 0);
#endif
        return;
      }
    }
  } else {
    obstacleBrakeActive_ = false;
    obstacleReleaseDistanceCm_ = 0.0f;
  }
  if (straightCommand_ && !headingSuppressed_ && headingAvailable()) {
    if (!heading_.isActive()) {
      heading_.capture(currentHeadingDeg());
      lastHeadingSequence_ = 0;
      lastHeadingCorrection_ = 0;
    }
    // V4.2 updates steering once per fused-heading sample. Motor/PS2 loops
    // may run faster and reuse the last correction between estimator samples.
    if (currentHeadingSequence() != lastHeadingSequence_) {
      lastHeadingSequence_ = currentHeadingSequence();
      const int16_t magnitude =
          static_cast<int16_t>((abs(left) + abs(right)) / 2);
      lastHeadingCorrection_ = heading_.update(
          currentHeadingDeg(), magnitude);
      if (heading_.isDisturbed()) {
        headingSuppressed_ = true;
        lastHeadingCorrection_ = 0;
#if ROBOT_DEBUG
        debug_.println("HEADING,FAULT=MOTOR_INTERFERENCE,ACTION=OPEN_LOOP");
#endif
      }
    }
    left = constrain(left + lastHeadingCorrection_, -255, 255);
    right = constrain(right - lastHeadingCorrection_, -255, 255);
  } else if (straightCommand_) {
    lastHeadingCorrection_ = 0;
    // Continue open-loop only when every heading source is unavailable.
    // A missing optional Compass no longer disables steering when the fused
    // Heading source is healthy.
    headingSuppressed_ = !headingAvailable();
  }
  if (aiMotionMode_ == AiMotionMode::DISTANCE) {
    const int16_t balance = distanceWheelBalance();
    const int16_t direction = reversing_ ? -1 : 1;
    left = constrain(left - direction * balance, -255, 255);
    right = constrain(right + direction * balance, -255, 255);
  }
  if (left == 0 && right == 0) {
    compass_.setRobotMoving(false);
    if (stopPwmLatched_) {
      motors_.stop();
    } else if (brakeEnabled_ && !freeStop_) {
      motors_.brake();
    } else {
      motors_.stop();
    }
  } else {
    obstacleBrakeActive_ = false;
    compass_.setMotionMode(straightCommand_ ? CompassMotionMode::STRAIGHT
                                           : CompassMotionMode::ROTATING);
    motors_.setSpeeds(left, right);
  }
}

void RobotController::stopImmediately() {
  targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
  obstacleLimited_ = false;
  state_ = RobotState::STOP;
  straightCommand_ = false;
  reversing_ = false;
  heading_.reset();
  lastHeadingCorrection_ = 0;
  headingSuppressed_ = false;
  aiMotionMode_ = AiMotionMode::NONE;
  aiMotionDeadlineMs_ = 0;
  aiTurnSettleStartMs_ = 0;
  aiTurnCommandSpeed_ = 0;
  aiTurnPulseDriving_ = false;
  aiTurnPulseUntilMs_ = 0;
  aiTurnCoastUntilMs_ = 0;
  motionOwner_ = MotionOwner::NONE;
  aiTurnYawRateDegS_ = 0.0f;
  const bool lockBrake = !freeStop_ &&
                         (brakeEnabled_ || obstacleBrakeActive_);
  stopPwmLatched_ = !lockBrake;
  if (lockBrake) {
    motors_.brake();
  } else {
    motors_.stop();
  }
  compass_.setRobotMoving(false);
}

bool RobotController::canStartAiMotion(uint32_t nowMs) const {
  return !brakeEnabled_ && aiMotionMode_ == AiMotionMode::NONE &&
         (motionOwner_ == MotionOwner::NONE || motionOwner_ == MotionOwner::MCP) &&
         (ps2_.frameTimedOut(nowMs) || !ps2_.motionCommandActive());
}

bool RobotController::startAiMotion(int16_t left, int16_t right,
                                    bool headingHold, bool reversing,
                                    uint32_t timeoutMs) {
  const uint32_t now = millis();
  if (!canStartAiMotion(now)) return false;
  motionOwner_ = MotionOwner::MCP;
  timeoutMs = constrain(timeoutMs, 50U, ROBOT_AI_MOTION_PULSE_MS);
  aiMotionMode_ = AiMotionMode::PULSE;
  aiMotionDeadlineMs_ = now + timeoutMs;
  setMotionCommand(left, right, RobotState::AI_MODE, headingHold, reversing);
  applyMotorCommand();
  return true;
}

bool RobotController::startAiContinuous(int16_t left, int16_t right,
                                        bool headingHold, bool reversing) {
  const uint32_t now = millis();
  if (!canStartAiMotion(now)) return false;
  motionOwner_ = MotionOwner::MCP;
  aiMotionMode_ = AiMotionMode::CONTINUOUS;
  aiMotionDeadlineMs_ = 0;
  setMotionCommand(left, right, RobotState::AI_MODE, headingHold, reversing);
  applyMotorCommand();
  return true;
}

bool RobotController::startAiDistance(bool forward, uint32_t distanceMm,
                                      int16_t speed) {
  const uint32_t now = millis();
  if (!canStartAiMotion(now) || !odometry_.ready() || !odometry_.healthy() ||
      distanceMm == 0U || distanceMm > ROBOT_AI_DISTANCE_MAX_MM) {
    return false;
  }
  motionOwner_ = MotionOwner::MCP;
  aiMotionMode_ = AiMotionMode::DISTANCE;
  aiMotionDeadlineMs_ = now + ROBOT_AI_DISTANCE_TIMEOUT_MS;
  aiDistanceStartMm_ = odometry_.data().distanceMm;
  aiDistanceStartLeftMm_ = odometry_.data().leftDistanceMm;
  aiDistanceStartRightMm_ = odometry_.data().rightDistanceMm;
  aiDistanceTargetMm_ = static_cast<float>(distanceMm);
  aiDistanceForward_ = forward;
  aiDistanceResultPending_ = false;
  const int16_t command = forward ? speed : -speed;
  setMotionCommand(command, command, RobotState::AI_MODE, true, !forward);
  applyMotorCommand();
  return true;
}

float RobotController::aiDistanceTravelledMm() const {
  const float delta = odometry_.data().distanceMm - aiDistanceStartMm_;
  return aiDistanceForward_ ? delta : -delta;
}

int16_t RobotController::distanceWheelBalance() const {
  if (aiMotionMode_ != AiMotionMode::DISTANCE) return 0;
  const float leftTravelMm =
      fabsf(odometry_.data().leftDistanceMm - aiDistanceStartLeftMm_);
  const float rightTravelMm =
      fabsf(odometry_.data().rightDistanceMm - aiDistanceStartRightMm_);
  const float errorMm = leftTravelMm - rightTravelMm;
  const int16_t correction = static_cast<int16_t>(lroundf(
      errorMm * DISTANCE_WHEEL_BALANCE_GAIN));
  return constrain(correction, -DISTANCE_WHEEL_BALANCE_MAX,
                   DISTANCE_WHEEL_BALANCE_MAX);
}

void RobotController::finishAiDistance(AiDistanceResultCode code) {
  aiDistanceResult_.code = code;
  aiDistanceResult_.targetMm = aiDistanceTargetMm_;
  aiDistanceResult_.travelledMm = aiDistanceTravelledMm();
  aiDistanceResultPending_ = true;
  stopImmediately();
}

bool RobotController::takeAiDistanceResult(AiDistanceResult& result) {
  if (!aiDistanceResultPending_) return false;
  result = aiDistanceResult_;
  aiDistanceResultPending_ = false;
  aiDistanceResult_ = {};
  return true;
}

bool RobotController::startAiTurnRelative(bool left, float degrees,
                                          int16_t maxSpeed) {
  if (!headingAvailable() || degrees <= 0.0f ||
      degrees > static_cast<float>(TURN_MAX_RELATIVE_DEG)) {
    return false;
  }
  // Capture the current heading exactly once. Physical measurement on this
  // Physical measurement shows LEFT increases Heading and RIGHT decreases it.
  const float startHeading = currentHeadingDeg();
  const float delta = left ? degrees : -degrees;
  const float target = CompassController::normalize(startHeading + delta);
#if ROBOT_DEBUG
  debug_.print("TURN,MODE=REL,DIR=");
  debug_.print(left ? "LEFT" : "RIGHT");
  debug_.print(",FROM="); debug_.print(startHeading, 2);
  debug_.print(",DELTA="); debug_.print(delta, 2);
  debug_.print(",TARGET="); debug_.println(target, 2);
#endif
  return startAiTurnAbsolute(target, maxSpeed);
}

bool RobotController::startAiTurnAbsolute(float targetHeading,
                                          int16_t maxSpeed) {
  const uint32_t now = millis();
  if (!canStartAiMotion(now) || !headingAvailable()) return false;
  motionOwner_ = MotionOwner::MCP;
  aiMotionMode_ = AiMotionMode::TURN;
  aiMotionDeadlineMs_ = now + TURN_TIMEOUT_MS;
  aiTurnTargetDeg_ = CompassController::normalize(targetHeading);
  aiTurnErrorDeg_ = CompassController::shortestDelta(
      aiTurnTargetDeg_, currentHeadingDeg());
  aiTurnMaxSpeed_ = constrain(maxSpeed, TURN_MIN_SPEED, TURN_MAX_SPEED);
  aiTurnCommandSpeed_ = 0;
  aiTurnSettleStartMs_ = 0;
  aiTurnLastSampleSequence_ = currentHeadingSequence();
  aiTurnYawRateDegS_ = 0.0f;
  aiTurnPreviousHeadingDeg_ = currentHeadingDeg();
  aiTurnPreviousSampleMs_ = now;
  aiTurnLastErrorSign_ = aiTurnErrorDeg_ < 0.0f ? -1 : 1;
  aiTurnPulseDriving_ = false;
  aiTurnPulseUntilMs_ = 0;
  aiTurnCoastUntilMs_ = 0;
  aiTurnResultPending_ = false;
  stopPwmLatched_ = false;
  straightCommand_ = false;
  reversing_ = false;
  heading_.reset();
  state_ = RobotState::AI_MODE;
  updateAiTurn(now);
  return true;
}

void RobotController::finishAiTurn(AiTurnResultCode code) {
  const float headingNow = currentHeadingDeg();
  const float target = aiTurnTargetDeg_;
  const float error = CompassController::shortestDelta(target, headingNow);
  stopImmediately();
  aiTurnResult_.code = code;
  aiTurnResult_.headingDeg = headingNow;
  aiTurnResult_.targetDeg = target;
  aiTurnResult_.errorDeg = error;
  aiTurnResultPending_ = true;
}

bool RobotController::takeAiTurnResult(AiTurnResult& result) {
  if (!aiTurnResultPending_) return false;
  result = aiTurnResult_;
  aiTurnResultPending_ = false;
  aiTurnResult_ = {};
  return true;
}

void RobotController::updateAiDistance(uint32_t nowMs) {
  if (static_cast<int32_t>(nowMs - aiMotionDeadlineMs_) >= 0) {
    finishAiDistance(AiDistanceResultCode::TIMEOUT);
    return;
  }
  if (aiDistanceTravelledMm() >= aiDistanceTargetMm_) {
    finishAiDistance(AiDistanceResultCode::DONE);
    return;
  }
  applyMotorCommand();
  if (aiMotionMode_ != AiMotionMode::DISTANCE) {
    finishAiDistance(ultrasonic_.isFresh()
                         ? AiDistanceResultCode::OBSTACLE
                         : AiDistanceResultCode::TIMEOUT);
  }
}

void RobotController::updateAiTurn(uint32_t nowMs) {
  if (aiMotionMode_ != AiMotionMode::TURN) return;
  // A turn-in-place does not add forward travel. Permit it when at least one
  // fresh sector is explicitly clear, even if the other ultrasonic channel is
  // temporarily unknown/timeout. Keep every caution/blocked/emergency result
  // as a hard stop, and do not turn when no sector is trustworthy.
  const ObstacleZone turnZone = ultrasonic_.overallZone();
  const bool oneSectorClear =
      turnZone == ObstacleZone::CLEAR ||
      (turnZone == ObstacleZone::UNKNOWN &&
       (ultrasonic_.frontLeftZone() == ObstacleZone::CLEAR ||
        ultrasonic_.frontRightZone() == ObstacleZone::CLEAR));
  if (!ultrasonic_.isFresh() || !oneSectorClear) {
#if ROBOT_DEBUG
    debug_.print("TURN,STOP=OBSTACLE,ZONE=");
    debug_.print(UltrasonicSensor::zoneText(turnZone));
    debug_.print(",FRESH=");
    debug_.println(ultrasonic_.isFresh() ? 1 : 0);
#endif
    finishAiTurn(AiTurnResultCode::OBSTACLE);
    return;
  }
  if (!headingAvailable()) {
    finishAiTurn(AiTurnResultCode::HEADING_LOST);
    return;
  }
  if (static_cast<int32_t>(nowMs - aiMotionDeadlineMs_) >= 0) {
    finishAiTurn(AiTurnResultCode::TIMEOUT);
    return;
  }

  bool overshot = false;
  if (currentHeadingSequence() != aiTurnLastSampleSequence_) {
    aiTurnLastSampleSequence_ = currentHeadingSequence();
    const float headingNow = currentHeadingDeg();
    const uint32_t dtMs = nowMs - aiTurnPreviousSampleMs_;
    if (dtMs >= 5U && dtMs <= 200U) {
      const float rawRate = CompassController::shortestDelta(
          headingNow, aiTurnPreviousHeadingDeg_) * 1000.0f /
          static_cast<float>(dtMs);
      aiTurnYawRateDegS_ +=
          TURN_RATE_FILTER * (rawRate - aiTurnYawRateDegS_);
    }
    aiTurnPreviousHeadingDeg_ = headingNow;
    aiTurnPreviousSampleMs_ = nowMs;
    aiTurnErrorDeg_ = CompassController::shortestDelta(
        aiTurnTargetDeg_, headingNow);
    const int8_t errorSign = aiTurnErrorDeg_ < 0.0f ? -1 : 1;
    if (fabsf(aiTurnErrorDeg_) > TURN_TOLERANCE_DEG &&
        aiTurnLastErrorSign_ != 0 && errorSign != aiTurnLastErrorSign_) {
      overshot = true;
      aiTurnCoastUntilMs_ = nowMs + TURN_OVERSHOOT_COAST_MS;
      aiTurnPulseDriving_ = false;
    }
    aiTurnLastErrorSign_ = errorSign;
  }
  const float absError = fabsf(aiTurnErrorDeg_);
  const float absYawRate = fabsf(aiTurnYawRateDegS_);

  // DONE requires both position and angular velocity to be settled. Merely
  // crossing the target is not success because the chassis may still coast.
  if (absError <= TURN_TOLERANCE_DEG &&
      absYawRate <= TURN_SETTLE_RATE_DEG_S) {
    targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
    aiTurnCommandSpeed_ = 0;
    aiTurnPulseDriving_ = false;
    motors_.stop();
    // Keep raw Compass tracking active during settle even though PWM is STOP.
    // This exposes mechanical coast/overshoot; leaving stationary hold active
    // here could hide a slow overshoot and incorrectly declare DONE.
    compass_.setMotionMode(CompassMotionMode::ROTATING);
    if (aiTurnSettleStartMs_ == 0U) aiTurnSettleStartMs_ = nowMs;
    if ((nowMs - aiTurnSettleStartMs_) >= TURN_SETTLE_MS) {
      finishAiTurn(AiTurnResultCode::DONE);
    }
    return;
  }

  aiTurnSettleStartMs_ = 0;
  compass_.setMotionMode(CompassMotionMode::ROTATING);

  auto stopTurnDrive = [&]() {
    targetLeft_ = targetRight_ = currentLeft_ = currentRight_ = 0;
    aiTurnCommandSpeed_ = 0;
    motors_.stop();
  };

  // Predict where the heading will be after motor response and mechanical
  // coast. If that projection reaches/crosses the target, release PWM now.
  const float projectedError =
      aiTurnErrorDeg_ - aiTurnYawRateDegS_ * TURN_PREDICT_TIME_S;
  const bool movingTowardTarget =
      aiTurnErrorDeg_ * aiTurnYawRateDegS_ > 0.0f;
  const bool projectedToTarget =
      movingTowardTarget &&
      (fabsf(projectedError) <= TURN_TOLERANCE_DEG ||
       projectedError * aiTurnErrorDeg_ <= 0.0f);

  if (overshot || projectedToTarget ||
      (absError <= TURN_TOLERANCE_DEG &&
       absYawRate > TURN_SETTLE_RATE_DEG_S)) {
    aiTurnPulseDriving_ = false;
    if (static_cast<int32_t>(nowMs - aiTurnCoastUntilMs_) >= 0) {
      aiTurnCoastUntilMs_ = nowMs + TURN_CORRECTION_COAST_MS;
    }
    stopTurnDrive();
    return;
  }

  if (static_cast<int32_t>(nowMs - aiTurnCoastUntilMs_) < 0) {
    stopTurnDrive();
    return;
  }

  // Inside the final zone, never hold continuous torque. A short drive pulse
  // followed by a longer observation interval lets the Compass measure the
  // actual response before choosing the next direction, including reversal
  // after overshoot.
  if (absError <= TURN_PULSE_ZONE_DEG) {
    if (aiTurnPulseDriving_) {
      if (static_cast<int32_t>(nowMs - aiTurnPulseUntilMs_) < 0) return;
      aiTurnPulseDriving_ = false;
      aiTurnCoastUntilMs_ = nowMs + TURN_CORRECTION_COAST_MS;
      stopTurnDrive();
      return;
    }
    if (absYawRate > TURN_CORRECTION_START_RATE_DEG_S) {
      aiTurnCoastUntilMs_ = nowMs + TURN_CORRECTION_COAST_MS;
      stopTurnDrive();
      return;
    }
    aiTurnCommandSpeed_ = constrain(TURN_CORRECTION_SPEED, TURN_MIN_SPEED,
                                    aiTurnMaxSpeed_);
    aiTurnPulseDriving_ = true;
    // Short 45 ms pulses are precise near target but do not consistently
    // overcome static friction on this chassis when 5..8 degrees remain.
    // Scale only pulse duration (never requested speed) with position error.
    const uint32_t pulseMs =
        absError > 5.0f ? TURN_CORRECTION_PULSE_FAR_MS
        : absError > 3.0f ? TURN_CORRECTION_PULSE_MID_MS
                          : TURN_CORRECTION_PULSE_NEAR_MS;
    aiTurnPulseUntilMs_ = nowMs + pulseMs;
  } else {
    aiTurnPulseDriving_ = false;
    float ratio = absError >= TURN_SLOW_ZONE_DEG
                      ? 1.0f
                      : (absError - TURN_PULSE_ZONE_DEG) /
                            (TURN_SLOW_ZONE_DEG - TURN_PULSE_ZONE_DEG);
    ratio = constrain(ratio, 0.0f, 1.0f);
    const int16_t speed = static_cast<int16_t>(lroundf(
        TURN_MIN_SPEED + ratio * (aiTurnMaxSpeed_ - TURN_MIN_SPEED)));
    aiTurnCommandSpeed_ = constrain(speed, TURN_MIN_SPEED, aiTurnMaxSpeed_);
  }

  // Measured turn polarity: L=-,R=+ increases Heading (left turn),
  // while L=+,R=- decreases it (right turn).
  if (aiTurnErrorDeg_ > 0.0f) {
    targetLeft_ = -aiTurnCommandSpeed_;
    targetRight_ = aiTurnCommandSpeed_;
  } else {
    targetLeft_ = aiTurnCommandSpeed_;
    targetRight_ = -aiTurnCommandSpeed_;
  }
  currentLeft_ = targetLeft_;
  currentRight_ = targetRight_;
  state_ = RobotState::AI_MODE;
  straightCommand_ = false;
  reversing_ = false;
  heading_.reset();
  applyMotorCommand();
}

void RobotController::updateFast() {
  motors_.update();
  const uint32_t now = millis();
  const bool ps2Usable = !ps2_.frameTimedOut(now);
  if (!ps2_.state().frameFresh) {
    if (aiMotionMode_ == AiMotionMode::PULSE &&
        static_cast<int32_t>(now - aiMotionDeadlineMs_) >= 0) {
      stopImmediately();
    } else if (!ps2Usable && aiMotionMode_ == AiMotionMode::NONE) {
      stopImmediately();
    }
    return;
  }

#if ROBOT_DEBUG
  struct ButtonEvent { Ps2Button button; const char* name; };
  static const ButtonEvent events[] = {
      {Ps2Button::UP, "UP"}, {Ps2Button::DOWN, "DOWN"},
      {Ps2Button::LEFT, "LEFT"}, {Ps2Button::RIGHT, "RIGHT"},
      {Ps2Button::L1, "L1"}, {Ps2Button::L2, "L2"},
      {Ps2Button::R1, "R1"}, {Ps2Button::R2, "R2"},
      {Ps2Button::START, "START"}, {Ps2Button::CROSS, "CROSS"},
      {Ps2Button::R3, "R3"}};
  for (const ButtonEvent& event : events) {
    if (ps2_.buttonPressed(event.button)) {
      debug_.print("BUTTON,EVENT="); debug_.print(event.name); debug_.println("_DOWN");
    }
    if (ps2_.buttonReleased(event.button)) {
      debug_.print("BUTTON,EVENT="); debug_.print(event.name); debug_.println("_UP");
    }
  }
#endif

  if (ps2Usable) handleFunctionButtons();
  if (ps2Usable && ps2_.motionCommandActive()) {
    // A live PS2 command always preempts and cancels AI motion.
    aiMotionMode_ = AiMotionMode::NONE;
    aiMotionDeadlineMs_ = 0;
    handleManualControl();
    applyMotorCommand();
    return;
  }
  if (aiMotionMode_ == AiMotionMode::PULSE &&
      static_cast<int32_t>(now - aiMotionDeadlineMs_) < 0) {
    return;
  }
  if (aiMotionMode_ == AiMotionMode::CONTINUOUS ||
      aiMotionMode_ == AiMotionMode::TURN ||
      aiMotionMode_ == AiMotionMode::DISTANCE) {
    return;
  }
  // Release/timeout bypasses ramp, heading sensor, LCD and the normal control period.
  stopImmediately();
}

void RobotController::emitDiagnostics(uint32_t nowMs) {
#if ROBOT_DEBUG
  if ((nowMs - lastUltrasonicDebugMs_) >= 200U) {
    lastUltrasonicDebugMs_ = nowMs;
    debug_.print("ULTRA,DIST="); debug_.print(ultrasonic_.distanceCm(), 1);
    debug_.print(",RAW="); debug_.print(ultrasonic_.rawDistanceCm(), 1);
    debug_.print(",ZONE=");
    debug_.print(UltrasonicSensor::zoneText(ultrasonic_.zone()));
    debug_.print(",VALID="); debug_.print(ultrasonic_.echoValid() ? 1 : 0);
    debug_.print(",FRESH="); debug_.print(ultrasonic_.isFresh() ? 1 : 0);
    debug_.print(",HEALTH="); debug_.print(UltrasonicSensor::healthText(ultrasonic_.health()));
    debug_.print(",RATE="); debug_.print(ultrasonic_.approachRateCmS(), 1);
    debug_.print(",LIMITED="); debug_.println(obstacleLimited_ ? 1 : 0);
    debug_.print("ULTRA2,L="); debug_.print(ultrasonic_.frontLeftDistanceCm(), 1);
    debug_.print(",R="); debug_.print(ultrasonic_.frontRightDistanceCm(), 1);
    debug_.print(",LZ="); debug_.print(UltrasonicSensor::zoneText(ultrasonic_.frontLeftZone()));
    debug_.print(",RZ="); debug_.print(UltrasonicSensor::zoneText(ultrasonic_.frontRightZone()));
    debug_.print(",LH="); debug_.print(UltrasonicSensor::healthText(ultrasonic_.frontLeft().health));
    debug_.print(",RH="); debug_.print(UltrasonicSensor::healthText(ultrasonic_.frontRight().health));
    debug_.print(",ZONE="); debug_.print(UltrasonicSensor::zoneText(ultrasonic_.overallZone()));
    debug_.print(",SUG="); debug_.println(UltrasonicSensor::avoidanceText(ultrasonic_.suggestedAvoidance()));
    return;
  }
  static uint32_t lastFusionDebugMs = 0;
  if ((nowMs - lastFusionDebugMs) >= 250U) {
    lastFusionDebugMs = nowMs;
    debug_.print("IMU,READY="); debug_.print(imu_.connected() ? 1 : 0);
    debug_.print(",CAL="); debug_.print(imu_.calibrated() ? 1 : 0);
    debug_.print(",HEALTH="); debug_.print(imu_.healthText());
    debug_.print(",GZ="); debug_.print(imu_.data().gyroZDps, 2);
    debug_.print(",AX="); debug_.print(imu_.data().accelXg, 2);
    debug_.print(",AY="); debug_.print(imu_.data().accelYg, 2);
    debug_.print(",AZ="); debug_.println(imu_.data().accelZg, 2);
    debug_.print("FUSION,READY="); debug_.print(fusion_.ready() ? 1 : 0);
    debug_.print(",HEALTH="); debug_.print(fusion_.healthText());
    debug_.print(",SRC="); debug_.print(fusion_.sourceText());
    debug_.print(",H="); debug_.print(fusion_.headingDeg(), 2);
    debug_.print(",RATE="); debug_.print(fusion_.yawRateDegS(), 2);
    debug_.print(",CONF="); debug_.println(fusion_.confidencePct(), 0);
    debug_.print("ODOM,X="); debug_.print(odometry_.data().xMm, 1);
    debug_.print(",Y="); debug_.print(odometry_.data().yMm, 1);
    debug_.print(",H="); debug_.print(odometry_.data().headingRad, 3);
    debug_.print(",EH="); debug_.print(odometry_.data().encoderHeadingRad, 3);
    debug_.print(",V="); debug_.print(odometry_.data().linearVelocityMmS, 1);
    debug_.print(",W="); debug_.println(odometry_.data().angularVelocityRadS, 3);
    return;
  }
  if (compass_.sampleSequence() != lastCompassSequence_ &&
      (nowMs - lastCompassDebugMs_) >= 100U) {
    lastCompassSequence_ = compass_.sampleSequence();
    lastCompassDebugMs_ = nowMs;
    debug_.print("COMPASS,RAW="); debug_.print(compass_.getRaw());
    debug_.print(",ANGLE="); debug_.print(compass_.getAngle(), 2);
    debug_.print(",DRIFT="); debug_.print(compass_.driftRateDegS(), 4);
    debug_.print(",DT="); debug_.print(compass_.lastSampleDtMs());
    debug_.print(",REJ="); debug_.print(compass_.rejectedSamples());
    debug_.print(",REJ_SEQ=");
    debug_.println(compass_.consecutiveRejectedSamples());
    return;
  }
  static uint32_t lastEncoderLcdDebugMs = 0;
  if ((nowMs - lastEncoderLcdDebugMs) >= 250U) {
    lastEncoderLcdDebugMs = nowMs;
    debug_.print("ENCODER,READY="); debug_.print(odometry_.ready() ? 1 : 0);
    debug_.print(",HEALTH="); debug_.print(odometry_.healthText());
    debug_.print(",L="); debug_.print(static_cast<long>(odometry_.data().leftTicks));
    debug_.print(",R="); debug_.print(static_cast<long>(odometry_.data().rightTicks));
    debug_.print(",LV="); debug_.print(odometry_.data().leftVelocityMmS, 1);
    debug_.print(",RV="); debug_.println(odometry_.data().rightVelocityMmS, 1);
    return;
  }
  if ((nowMs - lastPs2DebugMs_) >= 100U) {
    lastPs2DebugMs_ = nowMs;
    const Ps2State& state = ps2_.state();
    debug_.print("PS2,STATE="); debug_.print(ps2_.receiverStatusText(nowMs));
    debug_.print(",LX="); debug_.print(state.lx);
    debug_.print(",LY="); debug_.print(state.ly);
    debug_.print(",FRAME_DT="); debug_.println(ps2_.lastFrameIntervalMs());
    debug_.print("PS2LINK,CFG="); debug_.print(ps2_.configResult());
    debug_.print(",MODE=0x"); debug_.print(ps2_.mode(), HEX);
    debug_.print(",BUTTONS=0x"); debug_.print(ps2_.rawButtons(), HEX);
    debug_.print(",ERR="); debug_.print(ps2_.consecutiveErrors());
    debug_.print(",GOOD="); debug_.println(ps2_.goodFrameCount());
    debug_.print("PS2RAW");
    for (uint8_t i = 0; i < 9; ++i) {
      debug_.print(i == 0 ? ",B0=0x" : ",B");
      if (i != 0) { debug_.print(i); debug_.print("=0x"); }
      const uint8_t value = ps2_.rawByte(i);
      if (value < 0x10) debug_.print('0');
      debug_.print(value, HEX);
    }
    debug_.println();
    return;
  }
  if ((nowMs - lastMotorDebugMs_) >= 100U) {
    lastMotorDebugMs_ = nowMs;
    const bool turnActive = aiMotionMode_ == AiMotionMode::TURN;
    debug_.print("MOTOR,L="); debug_.print(motors_.leftSpeed());
    debug_.print(",R="); debug_.print(motors_.rightSpeed());
    debug_.print(",PWM_L="); debug_.print(motors_.leftPwm());
    debug_.print(",PWM_R="); debug_.print(motors_.rightPwm());
    debug_.print(",PID_EN="); debug_.print(WHEEL_SPEED_PID_ENABLED ? 1 : 0);
    debug_.print(",APPLIED_L="); debug_.print(motors_.leftAppliedCommand());
    debug_.print(",APPLIED_R="); debug_.print(motors_.rightAppliedCommand());
    debug_.print(",TGT=");
    debug_.print(turnActive ? aiTurnTargetDeg_ : heading_.targetHeading(), 2);
    debug_.print(",CUR="); debug_.print(currentHeadingDeg(), 2);
    debug_.print(",ERR=");
    debug_.print(turnActive ? aiTurnErrorDeg_ : heading_.error(), 2);
    debug_.print(",RATE=");
    debug_.print(turnActive ? aiTurnYawRateDegS_ : heading_.yawRate(), 2);
    debug_.print(",RATE_TGT="); debug_.print(heading_.desiredYawRate(), 2);
    debug_.print(",OUT=");
    debug_.print(turnActive ? aiTurnCommandSpeed_ : lastHeadingCorrection_);
    debug_.print(",HDG_FAULT=");
    debug_.print(headingSuppressed_ ? 1 : 0);
    debug_.print(",BRAKING="); debug_.println(motors_.isBraking() ? 1 : 0);
  }
#else
  (void)nowMs;
#endif
}

void RobotController::updateControl() {
  const uint32_t now = millis();
  emitDiagnostics(now);
  if ((now - lastControlMs_) < CONTROL_PERIOD_MS) return;
  lastControlMs_ += CONTROL_PERIOD_MS;
  if ((now - lastControlMs_) > CONTROL_PERIOD_MS * 4U) lastControlMs_ = now;
  const bool ps2Usable = !ps2_.frameTimedOut(now);
  if (ps2Usable) updateSpeedRepeat(now);

  // Encoder stall detection is enforced for AI-owned motion. Manual PS2 keeps
  // operator authority even if an encoder is unplugged, but autonomous motion
  // must never continue blindly when a commanded wheel produces no ticks.
  if (aiMotionMode_ != AiMotionMode::NONE && odometry_.stallFault()) {
#if ROBOT_DEBUG
    debug_.print("ENCODER,FAULT=");
    debug_.println(odometry_.healthText());
#endif
    if (aiMotionMode_ == AiMotionMode::DISTANCE) {
      finishAiDistance(AiDistanceResultCode::ENCODER_FAULT);
    } else if (aiMotionMode_ == AiMotionMode::TURN) {
      finishAiTurn(AiTurnResultCode::MOTION_FAULT);
    } else {
      stopImmediately();
    }
    return;
  }
  if (ps2Usable && ps2_.motionCommandActive()) {
    aiMotionMode_ = AiMotionMode::NONE;
    aiMotionDeadlineMs_ = 0;
    calculateMotionCommand();
    applyMotorCommand();
    return;
  }
  if (aiMotionMode_ == AiMotionMode::TURN) {
    updateAiTurn(now);
    return;
  }
  if (aiMotionMode_ == AiMotionMode::DISTANCE) {
    updateAiDistance(now);
    return;
  }
  if (aiMotionMode_ == AiMotionMode::PULSE) {
    if (!headingAvailable()) {
      stopImmediately();
      return;
    }
    if (static_cast<int32_t>(now - aiMotionDeadlineMs_) >= 0) {
      stopImmediately();
    } else {
      applyMotorCommand();
    }
    return;
  }
  if (aiMotionMode_ == AiMotionMode::CONTINUOUS) {
    if (!headingAvailable()) {
      stopImmediately();
      return;
    }
    applyMotorCommand();
    return;
  }
  {
    obstacleLimited_ = false;
    if (obstacleBrakeActive_ && ultrasonic_.isFresh() &&
        ultrasonic_.distanceCm() >= obstacleReleaseDistanceCm_) {
      obstacleBrakeActive_ = false;
      obstacleReleaseDistanceCm_ = 0.0f;
      stopPwmLatched_ = true;
    }
    if ((brakeEnabled_ || obstacleBrakeActive_) && !freeStop_) {
      motors_.brake();
    } else {
      motors_.stop();
    }
    compass_.setRobotMoving(false);
    currentLeft_ = currentRight_ = targetLeft_ = targetRight_ = 0;
    straightCommand_ = false;
    reversing_ = false;
    headingSuppressed_ = false;
    heading_.reset();
    lastHeadingCorrection_ = 0;
    state_ = RobotState::STOP;
    return;
  }
}

void RobotController::updateDisplay() {
  const uint32_t now = millis();
  LcdDisplayData data;
  data.headingDeg = currentHeadingDeg();
  data.ps2Status = ps2_.receiverStatusText(now);
  data.speed = speedSetting_;
  data.left = motors_.leftSpeed();
  data.right = motors_.rightSpeed();
  data.leftEncoderTicks = static_cast<int32_t>(odometry_.data().leftTicks);
  data.rightEncoderTicks = static_cast<int32_t>(odometry_.data().rightTicks);
  data.encoderReady = odometry_.ready();
  data.encoderHealth = odometry_.healthText();
  data.headingEnabled = heading_.isActive();
  data.headingError = heading_.error();
  data.headingTarget = heading_.isActive() ? heading_.targetHeading() : 0.0f;
  data.rampEnabled = rampEnabled_;
  data.ultrasonicValid = ultrasonic_.isFresh();
  data.ultrasonicDistanceCm = ultrasonic_.distanceCm();
  data.obstacleZone = UltrasonicSensor::zoneText(ultrasonic_.zone());
  data.brakeLocked = brakeEnabled_ || obstacleBrakeActive_;
  data.imuStatus = imu_.healthText();
  data.fusionStatus = fusion_.healthText();
  display_.setData(data);
  display_.update();
}

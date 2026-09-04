#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include <control/heading_controller.h>
#include <display/lcd_display.h>
#include <encoders/wheel_odometry.h>
#include <imu/mpu6050.h>
#include <localization/heading_fusion.h>
#include <motor/motor_controller.h>
#include <ps2/ps2_controller.h>
#include <robot_config.h>
#include <sensors/ultrasonic_sensor.h>

enum class RobotState : uint8_t {
  STOP,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  JOYSTICK_DRIVE,
  AI_MODE,
  AUTONOMOUS_MODE
};

enum class AiMotionMode : uint8_t {
  NONE, PULSE, CONTINUOUS, TURN, DISTANCE, GUIDED_WAYPOINT
};
enum class MotionOwner : uint8_t { NONE, MCP, MISSION, REPLAY, DIAGNOSTIC, PS2 };

enum class AiTurnProfile : uint8_t { PRECISE, MAP_COARSE };

enum class AiTurnResultCode : uint8_t {
  NONE, DONE, TIMEOUT, HEADING_LOST, MOTION_FAULT, OBSTACLE, CANCELLED
};

struct AiTurnResult {
  MotionOwner owner = MotionOwner::NONE;
  AiTurnResultCode code = AiTurnResultCode::NONE;
  // Non-zero for STM32-local MAP replay. MCP results keep the default zero
  // value, so the replay fence never changes RobotLink correlation semantics.
  uint32_t motionGeneration = 0U;
  float headingDeg = 0.0f;
  float targetDeg = 0.0f;
  float errorDeg = 0.0f;
};

enum class AiDistanceResultCode : uint8_t {
  NONE, DONE, TIMEOUT, OBSTACLE, ENCODER_FAULT, CANCELLED,
  HEADING_LOST, REALIGN_REQUIRED
};

struct AiDistanceResult {
  MotionOwner owner = MotionOwner::NONE;
  AiDistanceResultCode code = AiDistanceResultCode::NONE;
  // Non-zero for STM32-local MAP replay; see AiTurnResult above.
  uint32_t motionGeneration = 0U;
  float targetMm = 0.0f;
  float travelledMm = 0.0f;
};

class RobotController {
 public:
  RobotController(MotorController& motors, Ps2Controller& ps2,
                  LcdDisplay& display,
                  HeadingController& heading, UltrasonicSensor& ultrasonic,
                  WheelOdometry& odometry, Mpu6050& imu,
                  HeadingFusion& fusion, Print& debugStream);
  void begin();
  void updateFast();
  void updateControl();
  void updateDisplay();

  void handleManualControl();
  void handleFunctionButtons();
  // Shared reset path for the physical L2 button and RobotLink voice command.
  void resetHeadingReference();
  void calculateMotionCommand();
  void applyMotorCommand();
  // Stop the chassis now. An externally requested stop can also require a
  // fresh PS2 motion frame before Manual is allowed to energize the motors
  // again; this prevents a stale frame from restarting the last command.
  void stopImmediately(bool requireFreshManualCommand = false);
  bool startAiMotion(int16_t left, int16_t right, bool headingHold,
                     bool reversing, uint32_t timeoutMs);
  bool startAiContinuous(int16_t left, int16_t right, bool headingHold,
                         bool reversing);
  bool startAiDistance(bool forward, uint32_t distanceMm, int16_t speed);
  bool startAiTurnRelative(bool left, float degrees, int16_t maxSpeed);
  bool startAiTurnAbsolute(float targetHeading, int16_t maxSpeed);
  // MAP replay uses the same closed-loop primitives but a distinct owner and
  // result path. This prevents replay from borrowing MCP SID/OP correlation.
  bool canStartReplayMotion(uint32_t nowMs) const;
  bool startReplayDistance(bool forward, uint32_t distanceMm, int16_t speed,
                           uint32_t motionGeneration = 0U);
  bool startReplayTurnRelative(bool left, float degrees, int16_t maxSpeed,
                               uint32_t motionGeneration = 0U,
                               AiTurnProfile profile = AiTurnProfile::PRECISE);
  bool startReplayGuidedWaypoint(float targetXMm, float targetYMm,
                                 float segmentStartXMm,
                                 float segmentStartYMm, int16_t speed,
                                 float arrivalBearingDeg,
                                 uint32_t motionGeneration = 0U);
  bool takeAiTurnResult(AiTurnResult& result);
  bool takeAiDistanceResult(AiDistanceResult& result);

  RobotState state() const { return state_; }
  int16_t speedSetting() const { return speedSetting_; }
  bool rampEnabled() const { return rampEnabled_; }
  bool brakeEnabled() const { return brakeEnabled_; }
  bool setSpeedSetting(int16_t speed);
  void setRampEnabled(bool enabled) { rampEnabled_ = enabled; }
  void setBrakeEnabled(bool enabled);
  bool aiMotionActive() const { return aiMotionMode_ != AiMotionMode::NONE; }
  bool aiTurnActive() const { return aiMotionMode_ == AiMotionMode::TURN; }
  bool aiDistanceActive() const {
    return aiMotionMode_ == AiMotionMode::DISTANCE;
  }
  bool guidedWaypointActive() const {
    return aiMotionMode_ == AiMotionMode::GUIDED_WAYPOINT;
  }
  bool motorsStopped() const { return motors_.leftSpeed() == 0 && motors_.rightSpeed() == 0; }
  uint32_t headingResetGeneration() const { return headingResetGeneration_; }
  float aiDistanceTargetMm() const { return aiDistanceTargetMm_; }
  float aiDistanceTravelledMm() const;
  float aiTurnTarget() const { return aiTurnTargetDeg_; }
  float aiTurnError() const { return aiTurnErrorDeg_; }
  int16_t aiTurnSpeed() const { return aiTurnCommandSpeed_; }
  float guidedRemainingMm() const { return guidedRemainingMm_; }
  float guidedBearingDeg() const { return guidedBearingDeg_; }
  float guidedHeadingErrorDeg() const { return guidedHeadingErrorDeg_; }
  float guidedCrossTrackMm() const { return guidedCrossTrackMm_; }
  int16_t guidedBaseSpeed() const { return guidedBaseSpeed_; }
  int16_t guidedSteering() const { return guidedSteering_; }
  int16_t targetLeftCommand() const { return targetLeft_; }
  int16_t targetRightCommand() const { return targetRight_; }
  int16_t currentLeftCommand() const { return currentLeft_; }
  int16_t currentRightCommand() const { return currentRight_; }
  uint8_t aiMotionModeValue() const {
    return static_cast<uint8_t>(aiMotionMode_);
  }
  bool obstacleLimited() const { return obstacleLimited_; }
  bool encoderFault() const { return odometry_.stallFault(); }
  MotionOwner motionOwner() const { return motionOwner_; }
  static const char* motionOwnerText(MotionOwner owner);

 private:
  void setMotionCommand(int16_t left, int16_t right, RobotState state,
                        bool headingCandidate, bool reversing);
  void updateRamp();
  void updateSpeedRepeat(uint32_t nowMs);
  void changeSpeed(int8_t direction);
  void emitDiagnostics(uint32_t nowMs);
  void updateAiTurn(uint32_t nowMs);
  void updateAiDistance(uint32_t nowMs);
  void cancelAiMotionForManual();
  int16_t distanceWheelBalance() const;
  bool canStartAiMotion(uint32_t nowMs) const;
  bool canStartMotion(uint32_t nowMs, MotionOwner owner) const;
  bool headingAvailable() const;
  float currentHeadingDeg() const;
  uint32_t currentHeadingSequence() const;
  bool startDistance(MotionOwner owner, bool forward, uint32_t distanceMm,
                     int16_t speed, uint32_t motionGeneration = 0U);
  bool startTurnSession(MotionOwner owner, float targetHeading,
                        float targetUnwrappedHeading, bool multiTurn,
                        int16_t maxSpeed, uint32_t motionGeneration = 0U,
                        AiTurnProfile profile = AiTurnProfile::PRECISE);
  void finishAiTurn(AiTurnResultCode code);
  void finishAiDistance(AiDistanceResultCode code);
  void updateAiGuidedWaypoint(uint32_t nowMs);
  int16_t guidedSteeringCommand(float headingErrorDeg,
                                float crossTrackErrorMm) const;
  static int16_t rampToward(int16_t current, int16_t target);

  MotorController& motors_;
  Ps2Controller& ps2_;
  LcdDisplay& display_;
  HeadingController& heading_;
  UltrasonicSensor& ultrasonic_;
  WheelOdometry& odometry_;
  Mpu6050& imu_;
  HeadingFusion& fusion_;
  Print& debug_;

  RobotState state_ = RobotState::STOP;
  int16_t speedSetting_ = 30;
  int16_t targetLeft_ = 0;
  int16_t targetRight_ = 0;
  int16_t currentLeft_ = 0;
  int16_t currentRight_ = 0;
  bool straightCommand_ = false;
  bool reversing_ = false;
  // Normal STOP uses PWM 255. An explicit brake lock or obstacle safety hold
  // overrides it with the configured 251..254 locked PWM.
  bool stopPwmLatched_ = true;
  bool rampEnabled_ = false;
  bool brakeEnabled_ = false;
  bool obstacleBrakeActive_ = false;
  bool obstacleLimited_ = false;
  float obstacleReleaseDistanceCm_ = 0.0f;
  bool freeStop_ = false;
  bool headingResetHeld_ = false;
  bool speedHolding_ = false;
  int8_t speedDirection_ = 0;
  uint32_t speedHoldStartMs_ = 0;
  uint32_t speedNextRepeatMs_ = 0;
  uint32_t lastControlMs_ = 0;
  uint32_t lastPs2DebugMs_ = 0;
  uint32_t lastMotorDebugMs_ = 0;
  uint32_t lastUltrasonicDebugMs_ = 0;
  uint32_t lastHeadingSequence_ = 0;
  int16_t lastHeadingCorrection_ = 0;
  bool headingSuppressed_ = false;
  AiMotionMode aiMotionMode_ = AiMotionMode::NONE;
  uint32_t aiMotionDeadlineMs_ = 0;
  float aiTurnTargetDeg_ = 0.0f;
  float aiTurnTargetUnwrappedDeg_ = 0.0f;
  float aiTurnUnwrappedHeadingDeg_ = 0.0f;
  bool aiTurnMultiTurn_ = false;
  float aiTurnErrorDeg_ = 0.0f;
  AiTurnProfile aiTurnProfile_ = AiTurnProfile::PRECISE;
  float aiTurnToleranceDeg_ = TURN_TOLERANCE_DEG;
  uint32_t aiTurnSettleMs_ = TURN_SETTLE_MS;
  int16_t aiTurnMaxSpeed_ = TURN_MIN_SPEED;
  int16_t aiTurnCommandSpeed_ = 0;
  uint32_t aiTurnSettleStartMs_ = 0;
  uint32_t aiTurnLastSampleSequence_ = 0;
  float aiTurnYawRateDegS_ = 0.0f;
  float aiTurnPreviousHeadingDeg_ = 0.0f;
  uint32_t aiTurnPreviousSampleMs_ = 0;
  int8_t aiTurnLastErrorSign_ = 0;
  uint32_t aiTurnStartBoostUntilMs_ = 0;
  bool aiTurnPulseDriving_ = false;
  uint32_t aiTurnPulseUntilMs_ = 0;
  uint32_t aiTurnCoastUntilMs_ = 0;
  bool aiTurnResultPending_ = false;
  AiTurnResult aiTurnResult_;
  uint32_t aiMotionGeneration_ = 0U;
  float aiDistanceStartMm_ = 0.0f;
  float aiDistanceStartLeftMm_ = 0.0f;
  float aiDistanceStartRightMm_ = 0.0f;
  float aiDistanceTargetMm_ = 0.0f;
  bool aiDistanceForward_ = true;
  bool aiDistanceResultPending_ = false;
  AiDistanceResult aiDistanceResult_;
  float guidedTargetXMm_ = 0.0f;
  float guidedTargetYMm_ = 0.0f;
  float guidedSegmentStartXMm_ = 0.0f;
  float guidedSegmentStartYMm_ = 0.0f;
  float guidedArrivalBearingDeg_ = 0.0f;
  float guidedRemainingMm_ = 0.0f;
  float guidedBearingDeg_ = 0.0f;
  float guidedHeadingErrorDeg_ = 0.0f;
  float guidedCrossTrackMm_ = 0.0f;
  int16_t guidedBaseSpeed_ = 0;
  int16_t guidedRequestedSpeed_ = 0;
  int16_t guidedSteering_ = 0;
  MotionOwner motionOwner_ = MotionOwner::NONE;
  bool manualResumeRequired_ = false;
  uint32_t headingResetGeneration_ = 0;
};

#endif

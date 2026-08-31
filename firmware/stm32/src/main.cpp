#include <Arduino.h>
#include <communication/robot_link_server.h>
#include <compass/compass_controller.h>
#include <control/heading_controller.h>
#include <control/robot_controller.h>
#include <debug/debug_output.h>
#include <display/lcd_display.h>
#include <encoders/wheel_odometry.h>
#include <imu/mpu6050.h>
#include <localization/heading_fusion.h>
#include <motor/motor_controller.h>
#include <ps2/ps2_controller.h>
#include <robot_config.h>
#include <sensors/ultrasonic_sensor.h>
#include <safety/safety_watchdog.h>

HardwareSerial robotDebugSerial(ROBOT_DEBUG_RX_PIN, ROBOT_DEBUG_TX_PIN);
HardwareSerial robotAiSerial(ROBOT_LINK_RX_STM32, ROBOT_LINK_TX_STM32);
RttStream robotRtt;
DebugOutput robotDebug(robotDebugSerial, robotRtt);
RobotLinkServer robotLink(robotAiSerial, robotDebug);
MotorController motors;
Ps2Controller ps2;
CompassController compass;
LcdDisplay display(LCD_SCL_PIN, LCD_SDA_PIN, LCD_ADDRESS);
HeadingController heading;
UltrasonicSensor ultrasonic;
WheelOdometry wheelOdometry;
Mpu6050 imu(MPU6050_SCL_PIN, MPU6050_SDA_PIN, MPU6050_ADDRESS);
HeadingFusion headingFusion;
SafetyWatchdog safetyWatchdog;
RobotController robot(motors, ps2, compass, display, heading, ultrasonic,
                      wheelOdometry, imu, headingFusion, robotDebug);

void setup() {
  motors.begin();
#if ROBOT_DEBUG
  robotDebugSerial.begin(ROBOT_DEBUG_BAUD);
  robotDebug.println("BOOT,ROBOT_AI=1,MOTOR_PWM=255");
#endif
#if ROBOT_PS2_PROBE
  ps2.begin();
  robotLink.begin(ROBOT_LINK_BAUD);
  return;
#endif
#if ROBOT_IMU_PROBE
  headingFusion.begin();
  const bool imuProbeOk = imu.begin();
  robotDebug.print("IMU_PROBE,INIT=");
  robotDebug.println(imuProbeOk ? "OK" : "FAILED");
  return;
#endif
  if (ENCODER_ENABLED) {
    const bool encoderOk = wheelOdometry.begin();
#if ROBOT_DEBUG
    robotDebug.print("ENCODER,INIT=");
    robotDebug.println(encoderOk ? "OK" : "FAILED");
#endif
  }
#if ROBOT_ENCODER_PROBE
  robotDebug.println("TEST,ENCODER_PROBE=START,MANUAL_WHEEL_ROTATION=1");
  return;
#endif
  (void)display.begin();
  compass.begin();
  headingFusion.begin();
  const bool imuOk = imu.begin();
#if ROBOT_DEBUG
  robotDebug.print("IMU,INIT=");
  robotDebug.print(imuOk ? "OK" : "FAILED");
  robotDebug.print(",SCL=PC8,SDA=PB5,ADDR=0x");
  robotDebug.println(MPU6050_ADDRESS, HEX);
#endif
  ultrasonic.begin();
#if ROBOT_COMPASS_MOTOR_TEST
  robotDebug.println("TEST,COMPASS_MOTOR=START,RAISED_WHEELS=1");
  return;
#endif
  ps2.begin();
  robot.begin();
  robotLink.begin(ROBOT_LINK_BAUD);
  safetyWatchdog.begin();
}

void loop() {
#if ROBOT_PS2_PROBE
  ps2.update();
  RobotTelemetry ps2ProbeTelemetry;
  const uint32_t now = millis();
  ps2ProbeTelemetry.ps2Status = static_cast<uint8_t>(ps2.receiverStatus(now));
  ps2ProbeTelemetry.ps2Connected = ps2.state().receiverConnected;
  ps2ProbeTelemetry.ps2Fresh = !ps2.frameTimedOut(now);
  ps2ProbeTelemetry.ps2Enabled = ps2.configured();
  ps2ProbeTelemetry.ps2FrameAgeMs = ps2.frameAgeMs(now);
  ps2ProbeTelemetry.ps2Mode = ps2.mode();
  ps2ProbeTelemetry.ps2Buttons = ps2.rawButtons();
  ps2ProbeTelemetry.ps2Errors = ps2.consecutiveErrors();
  for (uint8_t i = 0; i < 9U; ++i)
    ps2ProbeTelemetry.ps2Raw[i] = ps2.rawByte(i);
  robotLink.update(ps2ProbeTelemetry);
  if (robotLink.takeStopRequest()) {
    motors.stop();
    robotLink.completeStopRequest();
  }
  yield();
  return;
#endif
#if ROBOT_IMU_PROBE
  static uint32_t lastImuLogMs = 0;
  imu.update();
  const uint32_t now = millis();
  if ((now - lastImuLogMs) >= 100U) {
    lastImuLogMs = now;
    robotDebug.print("IMU_PROBE,READY=");
    robotDebug.print(imu.ready() ? 1 : 0);
    robotDebug.print(",CAL=");
    robotDebug.print(imu.calibrated() ? 1 : 0);
    robotDebug.print(",HEALTH=");
    robotDebug.print(imu.healthText());
    robotDebug.print(",GZ=");
    robotDebug.print(imu.data().gyroZDps, 2);
    robotDebug.print(",AX=");
    robotDebug.print(imu.data().accelXg, 3);
    robotDebug.print(",AY=");
    robotDebug.print(imu.data().accelYg, 3);
    robotDebug.print(",AZ=");
    robotDebug.print(imu.data().accelZg, 3);
    robotDebug.print(",BIAS_Z=");
    robotDebug.println(imu.gyroBiasZDegS(), 3);
  }
  yield();
  return;
#endif
#if ROBOT_ENCODER_PROBE
  static uint32_t lastLogMs = 0;
  wheelOdometry.update(0, 0);
  const uint32_t now = millis();
  if ((now - lastLogMs) >= 100U) {
    lastLogMs = now;
    robotDebug.print("ENCODER_PROBE,LT=");
    robotDebug.print(static_cast<long>(wheelOdometry.data().leftTicks));
    robotDebug.print(",RT=");
    robotDebug.print(static_cast<long>(wheelOdometry.data().rightTicks));
    robotDebug.print(",LV=");
    robotDebug.print(wheelOdometry.data().leftVelocityMmS, 1);
    robotDebug.print(",RV=");
    robotDebug.print(wheelOdometry.data().rightVelocityMmS, 1);
    robotDebug.print(",HEALTH=");
    robotDebug.println(wheelOdometry.healthText());
  }
  yield();
  return;
#endif
#if ROBOT_COMPASS_MOTOR_TEST
  static uint32_t startedMs = millis();
  static uint32_t lastLogMs = 0;
  static uint8_t lastPhase = 255;
  const uint32_t elapsed = millis() - startedMs;
  uint8_t phase = 0;
  int16_t command = 0;
  if (elapsed < 12000U) {
    phase = 0;  // warm-up / idle
  } else if (elapsed < 24000U) {
    phase = 1; command = 20;
  } else if (elapsed < 30000U) {
    phase = 2;
  } else if (elapsed < 42000U) {
    phase = 3; command = -20;
  } else if (elapsed < 48000U) {
    phase = 4;
  } else if (elapsed < 60000U) {
    phase = 5; command = 60;
  } else if (elapsed < 66000U) {
    phase = 6;
  } else if (elapsed < 78000U) {
    phase = 7; command = -60;
  } else if (elapsed < 84000U) {
    phase = 8;
  } else {
    phase = 9;
  }
  if (phase != lastPhase) {
    // Exercise the real non-blocking brake at every drive-to-stop boundary.
    if (command == 0 &&
        (motors.leftSpeed() != 0 || motors.rightSpeed() != 0)) {
      motors.brake();
    } else {
      motors.stop();
      if (command != 0) motors.setSpeeds(command, command);
    }
    compass.setMotionMode(command == 0 ? CompassMotionMode::STATIONARY
                                      : CompassMotionMode::STRAIGHT);
    robotDebug.print("TEST,PHASE="); robotDebug.print(phase);
    robotDebug.print(",CMD="); robotDebug.println(command);
    lastPhase = phase;
  }
  compass.update();
  if ((millis() - lastLogMs) >= 50U) {
    lastLogMs = millis();
    robotDebug.print("TEST,SAMPLE,T="); robotDebug.print(elapsed);
    robotDebug.print(",PHASE="); robotDebug.print(phase);
    robotDebug.print(",CMD="); robotDebug.print(command);
    robotDebug.print(",RAW="); robotDebug.print(compass.getRaw());
    robotDebug.print(",RAW_DEG="); robotDebug.print(compass.getRawAngle(), 3);
    robotDebug.print(",OUT="); robotDebug.print(compass.getAngle(), 3);
    robotDebug.print(",DRIFT="); robotDebug.print(compass.driftRateDegS(), 4);
    robotDebug.print(",REJ="); robotDebug.print(compass.rejectedSamples());
    robotDebug.print(",M_L="); robotDebug.print(motors.leftSpeed());
    robotDebug.print(",M_R="); robotDebug.print(motors.rightSpeed());
    robotDebug.print(",PWM_L="); robotDebug.print(motors.leftPwm());
    robotDebug.print(",PWM_R="); robotDebug.print(motors.rightPwm());
    robotDebug.print(",BRAKING="); robotDebug.println(motors.isBraking() ? 1 : 0);
  }
  motors.update();
  if (phase == 9) motors.stop();
  yield();
  return;
#endif
  if (ENCODER_ENABLED) wheelOdometry.update(motors.leftSpeed(), motors.rightSpeed());
  ultrasonic.update();
  ps2.update();
  // Sample all localization sensors before motion control. Encoder travel is
  // integrated only after the fused heading for this loop has been computed.
  compass.update();
  imu.update();
  static uint32_t lastCompassZeroGeneration = 0;
  if (compass.zeroGeneration() != lastCompassZeroGeneration) {
    lastCompassZeroGeneration = compass.zeroGeneration();
    headingFusion.reset(compass.getAngle(),
                        wheelOdometry.data().encoderHeadingRad);
  }
  const bool chassisMoving = motors.leftSpeed() != 0 || motors.rightSpeed() != 0;
  headingFusion.update(
      compass.getAngle(), compass.isConnected() && !compass.isCalibrating(),
      compass.sampleSequence(), imu.data().gyroZDps, imu.ready(),
      imu.data().sampleSequence, wheelOdometry.data().encoderHeadingRad,
      wheelOdometry.healthy(), chassisMoving);
  wheelOdometry.integratePose(headingFusion.headingDeg(), headingFusion.ready());
  robot.updateFast();
  robot.updateControl();
  RobotTelemetry telemetry;
  telemetry.headingDeg = headingFusion.ready() ? headingFusion.headingDeg() : compass.getAngle();
  telemetry.leftCommand = motors.leftSpeed();
  telemetry.rightCommand = motors.rightSpeed();
  telemetry.speedSetting = robot.speedSetting();
  telemetry.robotState = static_cast<uint8_t>(robot.state());
  telemetry.motionOwner = static_cast<uint8_t>(robot.motionOwner());
  telemetry.ps2Status = static_cast<uint8_t>(ps2.receiverStatus(millis()));
  telemetry.ps2Connected = ps2.state().receiverConnected;
  telemetry.ps2Fresh = !ps2.frameTimedOut(millis());
  telemetry.ps2Enabled = ps2.configured();
  telemetry.ps2Mode = ps2.mode();
  telemetry.ps2Buttons = ps2.rawButtons();
  telemetry.ps2Errors = ps2.consecutiveErrors();
  for (uint8_t i = 0; i < 9U; ++i) telemetry.ps2Raw[i] = ps2.rawByte(i);
  telemetry.compassConnected = compass.isConnected();
  telemetry.compassCalibrating = compass.isCalibrating();
  telemetry.compassHeadingDeg = compass.getAngle();
  telemetry.headingReady = headingFusion.ready() || compass.isConnected() ||
                           wheelOdometry.healthy();
  telemetry.imuConnected = imu.connected();
  telemetry.imuCalibrated = imu.calibrated();
  telemetry.imuHealth = static_cast<uint8_t>(imu.health());
  telemetry.imuAccelXG = imu.data().accelXg;
  telemetry.imuAccelYG = imu.data().accelYg;
  telemetry.imuAccelZG = imu.data().accelZg;
  telemetry.imuGyroZDps = imu.data().gyroZDps;
  telemetry.fusionReady = headingFusion.ready();
  telemetry.fusionHealth = static_cast<uint8_t>(headingFusion.health());
  telemetry.fusionConfidencePct = headingFusion.confidencePct();
  telemetry.fusionYawRateDegS = headingFusion.yawRateDegS();
  telemetry.fusionSource = headingFusion.sourceText();
  telemetry.fusionEncoderDisagreementDeg = headingFusion.encoderDisagreementDeg();
  telemetry.fusionEffectiveEncoderWeight = headingFusion.effectiveEncoderWeight();
  telemetry.ps2CommandActive = ps2.motionCommandActive();
  telemetry.rampEnabled = robot.rampEnabled();
  telemetry.brakeEnabled = robot.brakeEnabled();
  telemetry.ps2FrameAgeMs = ps2.frameAgeMs(millis());
  telemetry.compassZeroGeneration = compass.zeroGeneration();
  telemetry.aiTurnActive = robot.aiTurnActive();
  telemetry.aiTurnTargetDeg = robot.aiTurnTarget();
  telemetry.aiTurnErrorDeg = robot.aiTurnError();
  telemetry.aiTurnSpeed = robot.aiTurnSpeed();
  telemetry.leftEncoderTicks = wheelOdometry.data().leftTicks;
  telemetry.rightEncoderTicks = wheelOdometry.data().rightTicks;
  telemetry.odometryDistanceMm = wheelOdometry.data().distanceMm;
  telemetry.odometryXmm = wheelOdometry.data().xMm;
  telemetry.odometryYmm = wheelOdometry.data().yMm;
  telemetry.odometryHeadingRad = wheelOdometry.data().headingRad;
  telemetry.odometryEncoderHeadingRad = wheelOdometry.data().encoderHeadingRad;
  telemetry.odometryLinearVelocityMmS = wheelOdometry.data().linearVelocityMmS;
  telemetry.odometryAngularVelocityRadS = wheelOdometry.data().angularVelocityRadS;
  telemetry.leftVelocityMmS = wheelOdometry.data().leftVelocityMmS;
  telemetry.rightVelocityMmS = wheelOdometry.data().rightVelocityMmS;
  telemetry.encoderReady = wheelOdometry.ready();
  telemetry.encoderHealth = static_cast<uint8_t>(wheelOdometry.health());
  telemetry.aiDistanceActive = robot.aiDistanceActive();
  telemetry.aiDistanceTargetMm = robot.aiDistanceTargetMm();
  telemetry.aiDistanceTravelledMm = robot.aiDistanceTravelledMm();
  telemetry.ultrasonicFresh = ultrasonic.isFresh();
  telemetry.ultrasonicEchoValid = ultrasonic.echoValid();
  telemetry.obstacleDistanceCm = ultrasonic.distanceCm();
  telemetry.obstacleApproachRateCmS = ultrasonic.approachRateCmS();
  telemetry.obstacleZone = static_cast<uint8_t>(ultrasonic.zone());
  telemetry.obstacleHealth = static_cast<uint8_t>(ultrasonic.health());
  telemetry.obstacleZoneSequence = ultrasonic.zoneSequence();
  telemetry.obstacleBlocked = ultrasonic.hardBlocked();
  telemetry.obstacleLimited = robot.obstacleLimited();
  telemetry.frontLeftDistanceCm = ultrasonic.frontLeftDistanceCm();
  telemetry.frontRightDistanceCm = ultrasonic.frontRightDistanceCm();
  telemetry.frontLeftRateCmS = ultrasonic.frontLeft().rateCmS;
  telemetry.frontRightRateCmS = ultrasonic.frontRight().rateCmS;
  telemetry.frontLeftZone = static_cast<uint8_t>(ultrasonic.frontLeftZone());
  telemetry.frontRightZone = static_cast<uint8_t>(ultrasonic.frontRightZone());
  telemetry.frontLeftHealth = static_cast<uint8_t>(ultrasonic.frontLeft().health);
  telemetry.frontRightHealth = static_cast<uint8_t>(ultrasonic.frontRight().health);
  telemetry.frontLeftAgeMs = ultrasonic.frontLeft().ageMs;
  telemetry.frontRightAgeMs = ultrasonic.frontRight().ageMs;
  telemetry.obstacleSuggestion = static_cast<uint8_t>(ultrasonic.suggestedAvoidance());
  telemetry.frontLeftFailureCount = ultrasonic.frontLeft().failureCount;
  telemetry.frontRightFailureCount = ultrasonic.frontRight().failureCount;
  telemetry.encoderResetGeneration = wheelOdometry.resetGeneration();
  robotLink.update(telemetry);
  // DONE,STOP means the requested electrical stop/brake is already applied.
  static bool stopCompletionPending = false;
  if (robotLink.takeStopRequest()) {
    robot.stopImmediately();
    stopCompletionPending = true;
  }
  if (stopCompletionPending && motors.leftSpeed() == 0 &&
      motors.rightSpeed() == 0) {
    robotLink.completeStopRequest();
    stopCompletionPending = false;
  } else if (!stopCompletionPending) {
    RobotLinkMotionRequest request;
    if (robotLink.takeMotionRequest(request)) {
      const int16_t speed = request.speed;
      bool started = false;
      switch (request.motion) {
        case RobotLinkMotion::FORWARD:
          started = request.distanceMm != 0U
                        ? robot.startAiDistance(true, request.distanceMm, speed)
                        : request.continuous
                        ? robot.startAiContinuous(speed, speed, true, false)
                        : robot.startAiMotion(speed, speed, true, false,
                                              ROBOT_AI_MOTION_PULSE_MS);
          break;
        case RobotLinkMotion::BACKWARD:
          started = request.distanceMm != 0U
                        ? robot.startAiDistance(false, request.distanceMm, speed)
                        : request.continuous
                        ? robot.startAiContinuous(-speed, -speed, true, true)
                        : robot.startAiMotion(-speed, -speed, true, true,
                                              ROBOT_AI_MOTION_PULSE_MS);
          break;
        case RobotLinkMotion::LEFT:
          started = request.continuous
                        ? robot.startAiContinuous(-speed, speed, false, false)
                        : robot.startAiMotion(-speed, speed, false, false,
                                              ROBOT_AI_MOTION_PULSE_MS);
          break;
        case RobotLinkMotion::RIGHT:
          started = request.continuous
                        ? robot.startAiContinuous(speed, -speed, false, false)
                        : robot.startAiMotion(speed, -speed, false, false,
                                              ROBOT_AI_MOTION_PULSE_MS);
          break;
        case RobotLinkMotion::TURN_REL_LEFT:
          started = robot.startAiTurnRelative(true, request.angleDeg, speed);
          break;
        case RobotLinkMotion::TURN_REL_RIGHT:
          started = robot.startAiTurnRelative(false, request.angleDeg, speed);
          break;
        case RobotLinkMotion::TURN_ABSOLUTE:
          started = robot.startAiTurnAbsolute(request.angleDeg, speed);
          break;
        case RobotLinkMotion::NONE:
          break;
      }
      robotLink.completeMotionRequest(request, started);
    }
  }
  RobotLinkConfigRequest configRequest;
  if (robotLink.takeConfigRequest(configRequest)) {
    bool applied = false;
    uint32_t compassGeneration = 0;
    switch (configRequest.type) {
      case RobotLinkConfigType::SET_SPEED:
        applied = robot.setSpeedSetting(configRequest.value);
        break;
      case RobotLinkConfigType::SET_BRAKE:
        robot.setBrakeEnabled(configRequest.value != 0);
        applied = true;
        break;
      case RobotLinkConfigType::SET_RAMP:
        robot.setRampEnabled(configRequest.value != 0);
        applied = true;
        break;
      case RobotLinkConfigType::RESET_COMPASS:
        compassGeneration = compass.zeroGeneration();
        robot.resetHeadingReference();
        applied = true;
        break;
      case RobotLinkConfigType::RESET_ENCODERS:
        // A counter reset during an operation is an unresolved odometry
        // boundary.  Stop first and require the caller to establish a new
        // segment; never silently continue with mixed generations.
        if (robot.aiMotionActive() || motors.leftSpeed() != 0 ||
            motors.rightSpeed() != 0) {
          robot.stopImmediately();
          robotDebug.print("ENCODER,RESET_REJECTED=IN_MOTION,GEN=");
          robotDebug.println(wheelOdometry.resetGeneration());
          applied = false;
        } else {
          wheelOdometry.resetWheelCounts(EncoderResetReason::ROBOTLINK);
          robotDebug.print("ENCODER,RESET=ROBOTLINK,GEN=");
          robotDebug.println(wheelOdometry.resetGeneration());
          applied = true;
        }
        break;
      case RobotLinkConfigType::NONE:
        break;
    }
    robotLink.completeConfigRequest(configRequest, applied,
                                    compassGeneration);
  }
  AiTurnResult turnResult;
  if (robot.takeAiTurnResult(turnResult)) {
    robotLink.reportTurnResult(static_cast<uint8_t>(turnResult.code),
                               turnResult.headingDeg, turnResult.targetDeg,
                               turnResult.errorDeg);
  }
  AiDistanceResult distanceResult;
  if (robot.takeAiDistanceResult(distanceResult)) {
    robotLink.reportDistanceResult(static_cast<uint8_t>(distanceResult.code),
                                   distanceResult.targetMm,
                                   distanceResult.travelledMm);
  }
  // Apply the optional STM32-local wheel-speed loop after all command/safety
  // owners for this iteration have had a chance to update the requested
  // command. Encoder velocity was sampled at the beginning of this loop.
  motors.updateSpeedPid(wheelOdometry.data().leftVelocityMmS,
                        wheelOdometry.data().rightVelocityMmS);
  robot.updateDisplay();
  safetyWatchdog.kick();
  yield();
}

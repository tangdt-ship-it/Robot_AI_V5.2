#ifndef ROBOT_LINK_SERVER_H
#define ROBOT_LINK_SERVER_H

#include <Arduino.h>
#include <robot_link_protocol.h>

struct RobotTelemetry {
  float headingDeg = 0.0f;
  int16_t leftCommand = 0;
  int16_t rightCommand = 0;
  int16_t speedSetting = 0;
  uint8_t robotState = 0;
  uint8_t motionOwner = 0;
  uint8_t ps2Status = 0;
  bool ps2Connected = false;
  bool ps2Fresh = false;
  bool ps2Enabled = false;
  uint8_t ps2Mode = 0;
  uint16_t ps2Buttons = 0xFFFF;
  uint8_t ps2Errors = 0;
  uint8_t ps2Raw[9] = {};
  bool compassConnected = false;
  bool compassCalibrating = false;
  float compassHeadingDeg = 0.0f;
  bool headingReady = false;
  bool imuConnected = false;
  bool imuCalibrated = false;
  uint8_t imuHealth = 0;
  float imuAccelXG = 0.0f;
  float imuAccelYG = 0.0f;
  float imuAccelZG = 0.0f;
  float imuGyroZDps = 0.0f;
  bool fusionReady = false;
  uint8_t fusionHealth = 0;
  float fusionConfidencePct = 0.0f;
  float fusionYawRateDegS = 0.0f;
  const char* fusionSource = "NONE";
  float fusionEncoderDisagreementDeg = 0.0f;
  float fusionEffectiveEncoderWeight = 0.0f;
  bool ps2CommandActive = false;
  bool rampEnabled = false;
  bool brakeEnabled = false;
  uint32_t ps2FrameAgeMs = 0;
  uint32_t compassZeroGeneration = 0;
  bool aiTurnActive = false;
  float aiTurnTargetDeg = 0.0f;
  float aiTurnErrorDeg = 0.0f;
  int16_t aiTurnSpeed = 0;
  int64_t leftEncoderTicks = 0;
  int64_t rightEncoderTicks = 0;
  float odometryDistanceMm = 0.0f;
  float odometryXmm = 0.0f;
  float odometryYmm = 0.0f;
  float odometryHeadingRad = 0.0f;
  float odometryEncoderHeadingRad = 0.0f;
  float odometryLinearVelocityMmS = 0.0f;
  float odometryAngularVelocityRadS = 0.0f;
  float leftVelocityMmS = 0.0f;
  float rightVelocityMmS = 0.0f;
  bool encoderReady = false;
  uint8_t encoderHealth = 0;
  bool aiDistanceActive = false;
  float aiDistanceTargetMm = 0.0f;
  float aiDistanceTravelledMm = 0.0f;
  bool ultrasonicFresh = false;
  bool ultrasonicEchoValid = false;
  float obstacleDistanceCm = 0.0f;
  float obstacleApproachRateCmS = 0.0f;
  uint8_t obstacleZone = 0;
  uint8_t obstacleHealth = 0;
  uint32_t obstacleZoneSequence = 0;
  bool obstacleBlocked = false;
  bool obstacleLimited = false;
  float frontLeftDistanceCm = 0.0f;
  float frontRightDistanceCm = 0.0f;
  float frontLeftRateCmS = 0.0f;
  float frontRightRateCmS = 0.0f;
  uint8_t frontLeftZone = 0;
  uint8_t frontRightZone = 0;
  uint8_t frontLeftHealth = 0;
  uint8_t frontRightHealth = 0;
  uint32_t frontLeftAgeMs = 0;
  uint32_t frontRightAgeMs = 0;
  uint8_t obstacleSuggestion = 3;
  uint32_t frontLeftFailureCount = 0;
  uint32_t frontRightFailureCount = 0;
  uint32_t encoderResetGeneration = 0;
};

enum class RobotLinkMotion : uint8_t {
  NONE,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  TURN_REL_LEFT,
  TURN_REL_RIGHT,
  TURN_ABSOLUTE
};

struct RobotLinkMotionRequest {
  RobotLinkMotion motion = RobotLinkMotion::NONE;
  int16_t speed = 0;
  int16_t angleDeg = 0;
  uint32_t distanceMm = 0;
  bool continuous = false;
  bool deferredAck = false;
  uint32_t sessionId = 0;
  uint32_t operationId = 0;
};

enum class RobotLinkConfigType : uint8_t {
  NONE,
  SET_SPEED,
  SET_BRAKE,
  SET_RAMP,
  RESET_COMPASS,
  RESET_ENCODERS
};

struct RobotLinkConfigRequest {
  RobotLinkConfigType type = RobotLinkConfigType::NONE;
  int16_t value = 0;
};

class RobotLinkServer {
 public:
  RobotLinkServer(HardwareSerial& serial, Print& debugStream)
      : serial_(serial), debug_(debugStream) {}

  void begin(uint32_t baud);
  void update(const RobotTelemetry& telemetry);
  bool connected() const;
  bool takeStopRequest();
  bool takeMotionRequest(RobotLinkMotionRequest& request);
  void completeMotionRequest(const RobotLinkMotionRequest& request,
                             bool success);
  void completeStopRequest();
  void reportTurnResult(uint8_t code, float headingDeg, float targetDeg,
                        float errorDeg);
  void reportDistanceResult(uint8_t code, float targetMm, float travelledMm);
  bool takeConfigRequest(RobotLinkConfigRequest& request);
  void completeConfigRequest(const RobotLinkConfigRequest& request,
                             bool success,
                             uint32_t compassZeroGeneration = 0);
  uint32_t validFrames() const { return validFrames_; }
  uint32_t receivedBytes() const { return receivedBytes_; }

 private:
  void handleFrame(const RobotLink::Frame& frame,
                   const RobotTelemetry& telemetry);
  void handleAsciiByte(char value, const RobotTelemetry& telemetry);
  void handleAsciiFrame(const char* frame, const RobotTelemetry& telemetry,
                        bool integrityProtected = false);
  bool acceptSequence(uint16_t sequence);
  void sendAsciiState(const RobotTelemetry& telemetry);
  void sendAsciiStateDetailed(const RobotTelemetry& telemetry);

  HardwareSerial& serial_;
  Print& debug_;
  RobotLink::Parser parser_;
  uint32_t lastConfirmedRxMs_ = 0;
  uint32_t lastSessionActivityMs_ = 0;
  uint32_t lastMotionHeartbeatMs_ = 0;
  uint32_t validFrames_ = 0;
  uint32_t receivedBytes_ = 0;
  uint32_t lastDiagnosticMs_ = 0;
  bool linkReported_ = false;
  bool stopRequested_ = false;
  bool motionRequested_ = false;
  RobotLinkMotionRequest motionRequest_;
  bool motionRequestInFlight_ = false;
  bool stopAckPending_ = false;
  bool motionLeaseActive_ = false;
  bool turnLeaseActive_ = false;
  uint32_t activeMotionSessionId_ = 0;
  uint32_t activeMotionOperationId_ = 0;
  uint32_t lastTurnProgressMs_ = 0;
  bool configRequestReady_ = false;
  bool configRequestInFlight_ = false;
  RobotLinkConfigRequest configRequest_;
  bool aiMode_ = false;
  bool rxSequenceInitialized_ = false;
  uint16_t lastRxSequence_ = 0;
  bool helloReceived_ = false;
  bool motionSessionReady_ = false;
  char asciiBuffer_[96] = {};
  uint8_t asciiLength_ = 0;
  bool asciiReceiving_ = false;
  bool compassResetAwaitingSample_ = false;
  uint32_t compassResetGeneration_ = 0;
  uint32_t compassResetStartedMs_ = 0;
  uint32_t lastObstacleZoneSequence_ = 0;
  float lastObstacleDistanceCm_ = 0.0f;
  uint8_t lastObstacleZone_ = 0;
  bool lastObstacleFresh_ = false;
  bool lastObstacleMotorsStopped_ = false;
  bool obstacleEventInitialized_ = false;
  bool encoderFaultReported_ = false;
};

#endif

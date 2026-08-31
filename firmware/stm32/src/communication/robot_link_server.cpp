#include <communication/robot_link_server.h>
#include <display/lcd_display.h>
#include <robot_config.h>

#include <stdlib.h>

extern LcdDisplay display;

namespace {
const char* ObstacleZoneName(uint8_t zone) {
  static const char* const names[] = {
      "UNKNOWN", "CLEAR", "CAUTION", "BLOCKED", "EMERGENCY"};
  return zone < 5U ? names[zone] : names[0];
}

const char* SensorHealthName(uint8_t health) {
  static const char* const names[] = {"UNKNOWN", "HEALTHY", "STALE",
                                      "TIMEOUT", "INVALID",
                                      "DISCONNECTED_OR_FAULT", "DEGRADED"};
  return health < 7U ? names[health] : names[0];
}

const char* EncoderHealthName(uint8_t health) {
  static const char* const names[] = {
      "DISABLED", "OK", "INIT_FAILED", "LEFT_STALL", "RIGHT_STALL",
      "BOTH_STALL"};
  return health < 6U ? names[health] : "UNKNOWN";
}

const char* ImuHealthName(uint8_t health) {
  static const char* const names[] = {
      "DISCONNECTED", "CALIBRATING", "OK", "READ_FAULT"};
  return health < 4U ? names[health] : "UNKNOWN";
}

const char* FusionHealthName(uint8_t health) {
  static const char* const names[] = {"NO_SOURCE", "DEGRADED", "FUSED"};
  return health < 3U ? names[health] : "UNKNOWN";
}

const char* MotionOwnerName(uint8_t owner) {
  static const char* const names[] = {
      "NONE", "MCP", "MISSION", "REPLAY", "DIAGNOSTIC", "PS2"};
  return owner < 6U ? names[owner] : "UNKNOWN";
}

bool RequiresIntegrity(const char* frame) {
  if (frame == nullptr) return false;
  return strncmp(frame, "MODE,", 5) == 0 ||
         strncmp(frame, "MOVE,", 5) == 0 ||
         strncmp(frame, "CMD,", 4) == 0 ||
         strncmp(frame, "TURN,", 5) == 0 ||
         strncmp(frame, "SET,", 4) == 0 ||
         strncmp(frame, "CAL,", 4) == 0 ||
         strncmp(frame, "MAP,UI,", 7) == 0 ||
         strcmp(frame, "COMPASS,RESET") == 0 ||
         strcmp(frame, "ENCODER,RESET") == 0 ||
         strcmp(frame, "HB") == 0 || strcmp(frame, "KEEPALIVE") == 0;
}

bool ParseCalibrationReference(const char* frame, const char* prefix,
                               float& reference) {
  const size_t prefixLength = strlen(prefix);
  if (strncmp(frame, prefix, prefixLength) != 0) return false;
  const char* valueStart = frame + prefixLength;
  if (*valueStart == '\0') return false;
  char* valueEnd = nullptr;
  const float parsed = strtof(valueStart, &valueEnd);
  if (valueEnd == valueStart || *valueEnd != '\0') return false;
  reference = parsed;
  return true;
}

void PrintCorrelation(HardwareSerial& serial, uint32_t sessionId,
                      uint32_t operationId) {
  if (sessionId == 0U || operationId == 0U) return;
  serial.print(",SID,");
  serial.print(sessionId);
  serial.print(",OP,");
  serial.print(operationId);
}
}  // namespace

void RobotLinkServer::begin(uint32_t baud) {
  serial_.begin(baud);
  lastConfirmedRxMs_ = 0;
  // V4 uses one unambiguous direction profile: CRC-protected RobotLink V3
  // commands inbound, human-readable angle-bracket responses/events outbound.
  serial_.print("<BOOT,STM32,ROBOT_AI,PROTO,3,CRC,REQUIRED>\r\n");
}

bool RobotLinkServer::connected() const {
  return lastConfirmedRxMs_ != 0U &&
         (millis() - lastConfirmedRxMs_) <= ROBOT_LINK_TIMEOUT_MS;
}

bool RobotLinkServer::takeStopRequest() {
  if (!stopRequested_) return false;
  stopRequested_ = false;
  return true;
}

bool RobotLinkServer::takeMotionRequest(RobotLinkMotionRequest& request) {
  if (!motionRequested_) return false;
  request = motionRequest_;
  motionRequested_ = false;
  return true;
}

void RobotLinkServer::completeMotionRequest(
    const RobotLinkMotionRequest& request, bool success) {
  if (!request.deferredAck || !motionRequestInFlight_) return;
  motionRequestInFlight_ = false;
  if (!success) {
    // All known static reject causes are checked before the request is queued.
    // A failure here is therefore a race/runtime reject (for example PS2
    // becoming active between validation and execution). Do not mislabel it
    // as HEADING_LOST or PS2_OVERRIDE without evidence.
    serial_.print("<ERR,MOTION_REJECTED");
    PrintCorrelation(serial_, request.sessionId, request.operationId);
    serial_.print(">\r\n");
    return;
  }
  activeMotionSessionId_ = request.sessionId;
  activeMotionOperationId_ = request.operationId;
  motionLeaseActive_ = true;
  lastMotionHeartbeatMs_ = millis();
  lastSessionActivityMs_ = lastMotionHeartbeatMs_;
  if (request.continuous) {
    serial_.print("<ACK,MOVE,");
    switch (request.motion) {
      case RobotLinkMotion::FORWARD: serial_.print("FWD"); break;
      case RobotLinkMotion::BACKWARD: serial_.print("BACK"); break;
      case RobotLinkMotion::LEFT: serial_.print("LEFT"); break;
      case RobotLinkMotion::RIGHT: serial_.print("RIGHT"); break;
      default: serial_.print("UNKNOWN"); break;
    }
    serial_.print(",");
    serial_.print(request.speed);
    serial_.print(",CONT");
    PrintCorrelation(serial_, request.sessionId, request.operationId);
    serial_.print(">\r\n");
    return;
  }
  if (request.distanceMm != 0U) {
    serial_.print("<ACK,MOVE,");
    serial_.print(request.motion == RobotLinkMotion::FORWARD ? "FWD," : "BACK,");
    serial_.print(request.distanceMm);
    serial_.print(",MM,");
    serial_.print(request.speed);
    PrintCorrelation(serial_, request.sessionId, request.operationId);
    serial_.print(">\r\n");
    return;
  }
  turnLeaseActive_ = true;
  serial_.print("<ACK,TURN,");
  if (request.motion == RobotLinkMotion::TURN_ABSOLUTE) {
    serial_.print("ABS,");
    serial_.print(request.angleDeg);
  } else {
    serial_.print("REL,");
    serial_.print(request.motion == RobotLinkMotion::TURN_REL_LEFT ? "LEFT"
                                                                  : "RIGHT");
    serial_.print(",");
    serial_.print(request.angleDeg);
  }
  PrintCorrelation(serial_, request.sessionId, request.operationId);
  serial_.print(">\r\n");
}

void RobotLinkServer::completeStopRequest() {
  motionLeaseActive_ = false;
  if (turnLeaseActive_) {
    serial_.print("<ERR,TURN,CANCELLED");
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  }
  turnLeaseActive_ = false;
  activeMotionSessionId_ = 0;
  activeMotionOperationId_ = 0;
  if (stopAckPending_) {
    stopAckPending_ = false;
    serial_.print("<ACK,STOP>\r\n");
    serial_.print("<DONE,STOP>\r\n");
  }
}

void RobotLinkServer::reportTurnResult(uint8_t code, float headingDeg,
                                       float targetDeg, float errorDeg) {
  motionLeaseActive_ = false;
  turnLeaseActive_ = false;
  if (code == 1U) {
    serial_.print("<DONE,TURN,H,");
    serial_.print(headingDeg, 1);
    serial_.print(",TGT,");
    serial_.print(targetDeg, 1);
    serial_.print(",ERR,");
    serial_.print(errorDeg, 1);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 2U) {
    serial_.print("<ERR,TURN,TIMEOUT");
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 3U) {
    serial_.print("<ERR,HEADING,LOST");
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 4U) {
    serial_.print("<ERR,TURN,MOTION_FAULT");
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 5U) {
    serial_.print("<ERR,TURN,OBSTACLE");
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  }
  activeMotionSessionId_ = 0;
  activeMotionOperationId_ = 0;
}

void RobotLinkServer::reportDistanceResult(uint8_t code, float targetMm,
                                           float travelledMm) {
  // stopImmediately() clears LIMITED before update() observes the stop.
  // Emit its asynchronous event once, immediately before the terminal result.
  if (code == 3U && motionLeaseActive_ && lastObstacleFresh_ &&
      lastObstacleMotorsStopped_) {
    serial_.print("<EVENT,OBSTACLE,STOPPED,ZONE,");
    serial_.print(ObstacleZoneName(lastObstacleZone_));
    serial_.print(",DIST,");
    serial_.print(lastObstacleDistanceCm_, 1);
    serial_.print(">\r\n");
  }
  motionLeaseActive_ = false;
  if (code == 1U) {
    serial_.print("<DONE,MOVE,TARGET,");
    serial_.print(targetMm, 1);
    serial_.print(",TRAVEL,");
    serial_.print(travelledMm, 1);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 2U) {
    serial_.print("<ERR,MOVE,TIMEOUT,TRAVEL,");
    serial_.print(travelledMm, 1);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 3U) {
    serial_.print("<ERR,MOVE,OBSTACLE,TRAVEL,");
    serial_.print(travelledMm, 1);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  } else if (code == 4U) {
    serial_.print("<ERR,MOVE,ENCODER_FAULT,TRAVEL,");
    serial_.print(travelledMm, 1);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  }
  activeMotionSessionId_ = 0;
  activeMotionOperationId_ = 0;
}

bool RobotLinkServer::takeConfigRequest(RobotLinkConfigRequest& request) {
  if (!configRequestReady_) return false;
  request = configRequest_;
  configRequestReady_ = false;
  return true;
}

void RobotLinkServer::completeConfigRequest(
    const RobotLinkConfigRequest& request, bool success,
    uint32_t compassZeroGeneration) {
  if (!configRequestInFlight_ || request.type != configRequest_.type) return;
  configRequestInFlight_ = false;
  configRequest_ = {};
  if (!success) {
    serial_.print("<ERR,APPLY_FAILED>\r\n");
    return;
  }
  switch (request.type) {
    case RobotLinkConfigType::SET_SPEED:
      serial_.print("<ACK,SET,SPEED,");
      serial_.print(request.value);
      serial_.print(">\r\n");
      break;
    case RobotLinkConfigType::SET_BRAKE:
      serial_.print(request.value ? "<ACK,SET,BRAKE,ON>\r\n"
                                  : "<ACK,SET,BRAKE,OFF>\r\n");
      break;
    case RobotLinkConfigType::SET_RAMP:
      serial_.print(request.value ? "<ACK,SET,RAMP,ON>\r\n"
                                  : "<ACK,SET,RAMP,OFF>\r\n");
      break;
    case RobotLinkConfigType::RESET_COMPASS:
      serial_.print("<ACK,COMPASS,RESET>\r\n");
      compassResetAwaitingSample_ = true;
      compassResetGeneration_ = compassZeroGeneration;
      compassResetStartedMs_ = millis();
      break;
    case RobotLinkConfigType::RESET_ENCODERS:
      serial_.print("<ACK,ENCODER,RESET>\r\n");
      break;
    case RobotLinkConfigType::NONE:
      break;
  }
}

bool RobotLinkServer::takeCalibrationRequest(
    RobotLinkCalibrationRequest& request) {
  if (!calibrationRequestReady_) return false;
  request = calibrationRequest_;
  calibrationRequestReady_ = false;
  return true;
}

void RobotLinkServer::completeCalibrationRequest(
    const RobotLinkCalibrationRequest& request, bool success,
    const RobotLinkCalibrationStatus& status) {
  if (!calibrationRequestInFlight_ ||
      request.type != calibrationRequest_.type) {
    return;
  }
  calibrationRequestInFlight_ = false;
  calibrationRequest_ = {};
  if (!success) {
    // Signal failure first so SendAndWait() cannot return successfully on the
    // diagnostic snapshot before the NACK/ERR is observed. The snapshot then
    // refreshes the ESP32 cache with the actual current phase/candidate.
    serial_.print("<ERR,CALIBRATION,REJECTED>\r\n");
    // Preserve the current phase/candidate snapshot even on rejection so the
    // ESP32 console cannot display a stale ACTIVE state after a bad sample.
    const char* phase = status.phase == 1U ? "STRAIGHT"
                        : status.phase == 2U ? "TURN" : "NONE";
    serial_.print("<VALUE,CALIBRATION,PHASE,");
    serial_.print(phase);
    serial_.print(",VALID,");
    serial_.print(status.valid ? 1 : 0);
    serial_.print(",PERSISTED,");
    serial_.print(status.persisted ? 1 : 0);
    serial_.print(",ACTIVE,");
    serial_.print(status.phase != 0U ? 1 : 0);
    serial_.print(",LEFT_MPT,");
    serial_.print(status.leftMmPerTick, 6);
    serial_.print(",RIGHT_MPT,");
    serial_.print(status.rightMmPerTick, 6);
    serial_.print(",TRACK_MM,");
    serial_.print(status.trackMm, 2);
    serial_.print(",STRAIGHT_SAMPLES,");
    serial_.print(status.straightSamples);
    serial_.print(",TURN_SAMPLES,");
    serial_.print(status.turnSamples);
    serial_.print(">\r\n");
    return;
  }
  const char* phase = status.phase == 1U ? "STRAIGHT"
                      : status.phase == 2U ? "TURN" : "NONE";
  serial_.print("<VALUE,CALIBRATION,PHASE,");
  serial_.print(phase);
  serial_.print(",VALID,");
  serial_.print(status.valid ? 1 : 0);
  serial_.print(",PERSISTED,");
  serial_.print(status.persisted ? 1 : 0);
  serial_.print(",ACTIVE,");
  serial_.print(status.phase != 0U ? 1 : 0);
  serial_.print(",LEFT_MPT,");
  serial_.print(status.leftMmPerTick, 6);
  serial_.print(",RIGHT_MPT,");
  serial_.print(status.rightMmPerTick, 6);
  serial_.print(",TRACK_MM,");
  serial_.print(status.trackMm, 2);
  serial_.print(",STRAIGHT_SAMPLES,");
  serial_.print(status.straightSamples);
  serial_.print(",TURN_SAMPLES,");
  serial_.print(status.turnSamples);
  serial_.print(">\r\n");
}

void RobotLinkServer::sendAsciiState(const RobotTelemetry& telemetry) {
  const int heading = static_cast<int>(lroundf(telemetry.headingDeg));
  const int speed = (abs(telemetry.leftCommand) +
                     abs(telemetry.rightCommand)) /
                    2;
  serial_.print(aiMode_ ? "<STATE,AI,H," : "<STATE,MANUAL,H,");
  serial_.print(heading);
  serial_.print(",S,");
  serial_.print(speed);
  serial_.print(",L,");
  serial_.print(telemetry.leftCommand);
  serial_.print(",R,");
  serial_.print(telemetry.rightCommand);
  serial_.print(",MOVE,");
  serial_.print((telemetry.leftCommand != 0 || telemetry.rightCommand != 0)
                    ? 1
                    : 0);
  serial_.print(">\r\n");
}

void RobotLinkServer::sendAsciiStateDetailed(const RobotTelemetry& telemetry) {
  serial_.print("<STATE,MODE,");
  serial_.print(aiMode_ ? "AI" : "MANUAL");
  serial_.print(",SPEED,");
  serial_.print(telemetry.speedSetting);
  serial_.print(",BRAKE,");
  serial_.print(telemetry.brakeEnabled ? "ON" : "OFF");
  serial_.print(",RAMP,");
  serial_.print(telemetry.rampEnabled ? "ON" : "OFF");
  serial_.print(",H,");
  serial_.print(telemetry.headingDeg, 1);
  serial_.print(",L,");
  serial_.print(telemetry.leftCommand);
  serial_.print(",R,");
  serial_.print(telemetry.rightCommand);
  serial_.print(",MOVE,");
  serial_.print((telemetry.leftCommand != 0 || telemetry.rightCommand != 0)
                    ? 1
                    : 0);
  serial_.print(",PS2,");
  serial_.print(telemetry.ps2Connected ? "RX" : "LOST");
  serial_.print(",COMPASS,");
  serial_.print(telemetry.compassConnected ? "OK" : "LOST");
  serial_.print(",AI_LINK,");
  serial_.print(connected() ? "OK" : "WAIT");
  serial_.print(",OWNER,");
  serial_.print(MotionOwnerName(telemetry.motionOwner));
  serial_.print(">\r\n");
}

void RobotLinkServer::handleAsciiFrame(const char* frame,
                                       const RobotTelemetry& telemetry,
                                       bool integrityProtected) {
  const uint32_t now = millis();
  lastConfirmedRxMs_ = now;
  if (integrityProtected) lastSessionActivityMs_ = now;
  if (strcmp(frame, "STOP") != 0 && strcmp(frame, "CMD,STOP") != 0 &&
      RequiresIntegrity(frame) && !integrityProtected) {
    serial_.print("<ERR,CRC_REQUIRED>\r\n");
    return;
  }
  if (strcmp(frame, "PING") == 0) {
    if (!helloReceived_) {
      serial_.print("<ERR,HELLO_REQUIRED>\r\n");
      return;
    }
    motionSessionReady_ = true;
    serial_.print("<PONG>\r\n");
    return;
  }
  if (strcmp(frame, "HELLO,PROTO,3") == 0) {
    serial_.print("<HELLO,STM32,ROBOT_AI,PROTO,3,CRC,CCITT>\r\n");
    return;
  }
  if (strncmp(frame, "HELLO,PROTO,", 12) == 0) {
    serial_.print("<ERR,PROTOCOL_VERSION,EXPECTED,3>\r\n");
    return;
  }
  if (strcmp(frame, "HB") == 0 || strcmp(frame, "KEEPALIVE") == 0) {
    if (aiMode_ && motionLeaseActive_) lastMotionHeartbeatMs_ = now;
    return;
  }
  if (strcmp(frame, "GET") == 0) {
    sendAsciiState(telemetry);
    return;
  }
  if (strcmp(frame, "GET,STATE") == 0) {
    sendAsciiStateDetailed(telemetry);
    return;
  }
  if (strcmp(frame, "GET,SPEED") == 0) {
    serial_.print("<VALUE,SPEED,");
    serial_.print(telemetry.speedSetting);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,BRAKE") == 0) {
    serial_.print(telemetry.brakeEnabled ? "<VALUE,BRAKE,ON>\r\n"
                                        : "<VALUE,BRAKE,OFF>\r\n");
    return;
  }
  if (strcmp(frame, "GET,RAMP") == 0) {
    serial_.print(telemetry.rampEnabled ? "<VALUE,RAMP,ON>\r\n"
                                       : "<VALUE,RAMP,OFF>\r\n");
    return;
  }
  if (strcmp(frame, "GET,HEADING") == 0) {
    if (!telemetry.headingReady) {
      serial_.print("<ERR,HEADING,LOST>\r\n");
    } else {
      serial_.print("<VALUE,HEADING,");
      serial_.print(telemetry.headingDeg, 1);
      serial_.print(">\r\n");
    }
    return;
  }
  if (strcmp(frame, "GET,ODOMETRY") == 0) {
    serial_.print("<VALUE,ODOMETRY,DIST,");
    serial_.print(telemetry.odometryDistanceMm, 1);
    serial_.print(",X,");
    serial_.print(telemetry.odometryXmm, 1);
    serial_.print(",Y,");
    serial_.print(telemetry.odometryYmm, 1);
    serial_.print(",H,");
    serial_.print(telemetry.odometryHeadingRad, 3);
    serial_.print(",LT,");
    serial_.print(static_cast<long>(telemetry.leftEncoderTicks));
    serial_.print(",RT,");
    serial_.print(static_cast<long>(telemetry.rightEncoderTicks));
    serial_.print(",RESET_GEN,");
    serial_.print(telemetry.encoderResetGeneration);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,ENCODER") == 0) {
    serial_.print("<VALUE,ENCODER,READY,");
    serial_.print(telemetry.encoderReady ? 1 : 0);
    serial_.print(",HEALTH,");
    serial_.print(EncoderHealthName(telemetry.encoderHealth));
    serial_.print(",LV,");
    serial_.print(telemetry.leftVelocityMmS, 1);
    serial_.print(",RV,");
    serial_.print(telemetry.rightVelocityMmS, 1);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,IMU") == 0) {
    serial_.print("<VALUE,IMU,READY,");
    serial_.print(telemetry.imuConnected ? 1 : 0);
    serial_.print(",CAL,");
    serial_.print(telemetry.imuCalibrated ? 1 : 0);
    serial_.print(",HEALTH,");
    serial_.print(ImuHealthName(telemetry.imuHealth));
    serial_.print(",GZ,");
    serial_.print(telemetry.imuGyroZDps, 2);
    serial_.print(",AX,");
    serial_.print(telemetry.imuAccelXG, 3);
    serial_.print(",AY,");
    serial_.print(telemetry.imuAccelYG, 3);
    serial_.print(",AZ,");
    serial_.print(telemetry.imuAccelZG, 3);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,FUSION") == 0) {
    serial_.print("<VALUE,FUSION,READY,");
    serial_.print(telemetry.fusionReady ? 1 : 0);
    serial_.print(",HEALTH,");
    serial_.print(FusionHealthName(telemetry.fusionHealth));
    serial_.print(",H,");
    serial_.print(telemetry.headingDeg, 2);
    serial_.print(",RATE,");
    serial_.print(telemetry.fusionYawRateDegS, 2);
    serial_.print(",CONF,");
    serial_.print(telemetry.fusionConfidencePct, 1);
    serial_.print(",SRC,");
    serial_.print(telemetry.fusionSource != nullptr ? telemetry.fusionSource : "NONE");
    serial_.print(",EDIS,");
    serial_.print(telemetry.fusionEncoderDisagreementDeg, 2);
    serial_.print(",EW,");
    serial_.print(telemetry.fusionEffectiveEncoderWeight, 3);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,COMPASS_STATUS") == 0) {
    serial_.print("<VALUE,COMPASS,");
    serial_.print(telemetry.compassConnected ? "OK" : "LOST");
    serial_.print(",CAL,");
    serial_.print(telemetry.compassCalibrating ? 1 : 0);
    serial_.print(",H,");
    serial_.print(telemetry.compassHeadingDeg, 1);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "GET,OBSTACLE") == 0) {
    serial_.print("<VALUE,OBSTACLE,FRESH,");
    serial_.print(telemetry.ultrasonicFresh ? 1 : 0);
    serial_.print(",ECHO,");
    serial_.print(telemetry.ultrasonicEchoValid ? 1 : 0);
    serial_.print(",HEALTH,");
    serial_.print(SensorHealthName(telemetry.obstacleHealth));
    serial_.print(",DIST,");
    serial_.print(telemetry.obstacleDistanceCm, 1);
    serial_.print(",RATE,");
    serial_.print(telemetry.obstacleApproachRateCmS, 1);
    serial_.print(",ZONE,");
    serial_.print(ObstacleZoneName(telemetry.obstacleZone));
    serial_.print(",LIMIT,");
    serial_.print(telemetry.obstacleLimited ? 1 : 0);
    serial_.print(",LEFT,");
    serial_.print(telemetry.frontLeftDistanceCm, 1);
    serial_.print(",RIGHT,");
    serial_.print(telemetry.frontRightDistanceCm, 1);
    serial_.print(",LZ,");
    serial_.print(ObstacleZoneName(telemetry.frontLeftZone));
    serial_.print(",RZ,");
    serial_.print(ObstacleZoneName(telemetry.frontRightZone));
    serial_.print(",LH,");
    serial_.print(SensorHealthName(telemetry.frontLeftHealth));
    serial_.print(",RH,");
    serial_.print(SensorHealthName(telemetry.frontRightHealth));
    serial_.print(",LAGE,");
    serial_.print(telemetry.frontLeftAgeMs);
    serial_.print(",RAGE,");
    serial_.print(telemetry.frontRightAgeMs);
    serial_.print(",NEAREST,");
    serial_.print(telemetry.obstacleDistanceCm, 1);
    serial_.print(",SUG,");
    static const char* const suggestions[] = {"NONE", "LEFT", "RIGHT", "STOP"};
    serial_.print(suggestions[telemetry.obstacleSuggestion < 4U ? telemetry.obstacleSuggestion : 3U]);
    serial_.print(",LF,");
    serial_.print(telemetry.frontLeftFailureCount);
    serial_.print(",RF,");
    serial_.print(telemetry.frontRightFailureCount);
    serial_.print(",RESET_GEN,");
    serial_.print(telemetry.encoderResetGeneration);
    serial_.print(">\r\n");
    return;
  }
  if (strcmp(frame, "PS2,STATUS") == 0) {
    static const char* const statusNames[] = {"WAIT", "ACT", "RX", "LOST"};
    const uint8_t index = telemetry.ps2Status < 4U ? telemetry.ps2Status : 3U;
    serial_.print("<PS2,STATE,");
    serial_.print(statusNames[index]);
    serial_.print(",ENABLED,");
    serial_.print(telemetry.ps2Enabled ? 1 : 0);
    serial_.print(",FRESH,");
    serial_.print(telemetry.ps2Fresh ? 1 : 0);
    serial_.print(",AGE,");
    serial_.print(telemetry.ps2FrameAgeMs);
    serial_.print(",MODE,");
    serial_.print(telemetry.ps2Mode, HEX);
    serial_.print(",BTN,");
    serial_.print(telemetry.ps2Buttons, HEX);
    serial_.print(",ERR,");
    serial_.print(telemetry.ps2Errors);
    serial_.print(",RAW,");
    for (uint8_t i = 0; i < 9U; ++i) {
      if (i != 0U) serial_.print(':');
      if (telemetry.ps2Raw[i] < 0x10U) serial_.print('0');
      serial_.print(telemetry.ps2Raw[i], HEX);
    }
    serial_.print(">\r\n");
    return;
  }

  if (strncmp(frame, "MAP,UI,", 7) == 0) {
    unsigned int slot = 0;
    unsigned int storeState = 0;
    unsigned int mode = 0;
    unsigned int points = 0;
    unsigned int maxPoints = 0;
    unsigned long lengthMm = 0;
    unsigned int replayWp = 0;
    unsigned int replayTotal = 0;
    unsigned long replayTargetMm = 0;
    unsigned long replayTravelMm = 0;
    unsigned long replayErrorMm = 0;
    unsigned int replayOperation = 0;
    char trailing = '\0';
    const int fields = sscanf(
        frame, "MAP,UI,%u,%u,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu,%u%c", &slot,
        &storeState, &mode, &points, &maxPoints, &lengthMm, &replayWp,
        &replayTotal, &replayTargetMm, &replayTravelMm, &replayErrorMm,
        &replayOperation, &trailing);
    const bool v3 = fields == 12;
    if (!v3) {
      replayWp = replayTotal = 0;
      replayTargetMm = replayTravelMm = replayErrorMm = 0;
      replayOperation = 0;
      const int v2Fields = sscanf(
          frame, "MAP,UI,%u,%u,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu%c", &slot,
          &storeState, &mode, &points, &maxPoints, &lengthMm, &replayWp,
          &replayTotal, &replayTargetMm, &replayTravelMm, &replayErrorMm,
          &trailing);
      if (v2Fields == 11) {
        // V2 has no operation field; preserve compatibility.
      } else {
      const int v1Fields = sscanf(frame, "MAP,UI,%u,%u,%u,%u,%u,%lu%c",
                                  &slot, &storeState, &mode, &points,
                                  &maxPoints, &lengthMm, &trailing);
      if (v1Fields != 6) {
        serial_.print("<ERR,MAP_UI>\r\n");
        return;
      }
      }
    }
    if (slot < 1U || slot > 2U || storeState > 3U || mode > 8U ||
        replayOperation > 3U ||
        maxPoints == 0U || maxPoints > 128U || points > maxPoints ||
        (replayTotal > 0U && (replayTotal > maxPoints || replayWp > replayTotal))) {
      serial_.print("<ERR,MAP_UI>\r\n");
      return;
    }
    display.setMapStatus(static_cast<uint8_t>(slot),
                         static_cast<uint8_t>(storeState),
                         static_cast<uint8_t>(mode),
                         static_cast<uint16_t>(points),
                         static_cast<uint16_t>(maxPoints),
                         static_cast<uint32_t>(lengthMm),
                         static_cast<uint16_t>(replayWp),
                         static_cast<uint16_t>(replayTotal),
                         static_cast<uint32_t>(replayTargetMm),
                         static_cast<uint32_t>(replayTravelMm),
                         static_cast<uint32_t>(replayErrorMm),
                         static_cast<uint8_t>(replayOperation));
    return;
  }

  RobotLinkCalibrationRequest calibration;
  if (strcmp(frame, "CAL,STATUS") == 0) {
    calibration.type = RobotLinkCalibrationType::STATUS;
  } else if (strcmp(frame, "CAL,BEGIN,STRAIGHT") == 0) {
    calibration.type = RobotLinkCalibrationType::BEGIN_STRAIGHT;
  } else if (ParseCalibrationReference(frame, "CAL,END,STRAIGHT,",
                                       calibration.reference)) {
    calibration.type = RobotLinkCalibrationType::END_STRAIGHT;
  } else if (strcmp(frame, "CAL,BEGIN,TURN") == 0) {
    calibration.type = RobotLinkCalibrationType::BEGIN_TURN;
  } else if (ParseCalibrationReference(frame, "CAL,END,TURN,",
                                       calibration.reference)) {
    calibration.type = RobotLinkCalibrationType::END_TURN;
  } else if (strcmp(frame, "CAL,COMMIT") == 0) {
    calibration.type = RobotLinkCalibrationType::COMMIT;
  } else if (strcmp(frame, "CAL,ABORT") == 0) {
    calibration.type = RobotLinkCalibrationType::ABORT;
  }
  if (calibration.type != RobotLinkCalibrationType::NONE) {
    if (calibrationRequestInFlight_ || configRequestInFlight_ ||
        compassResetAwaitingSample_) {
      serial_.print("<ERR,BUSY>\r\n");
      return;
    }
    calibrationRequest_ = calibration;
    calibrationRequestReady_ = true;
    calibrationRequestInFlight_ = true;
    return;
  }

  RobotLinkConfigRequest config;
  int configValue = 0;
  char trailing = '\0';
  if (sscanf(frame, "SET,SPEED,%d%c", &configValue, &trailing) == 1) {
    if (configValue < SPEED_MIN || configValue > SPEED_MAX) {
      serial_.print("<ERR,SPEED_RANGE,");
      serial_.print(SPEED_MIN);
      serial_.print(",");
      serial_.print(SPEED_MAX);
      serial_.print(">\r\n");
      return;
    }
    config.type = RobotLinkConfigType::SET_SPEED;
    config.value = static_cast<int16_t>(configValue);
  } else if (strcmp(frame, "SET,BRAKE,ON") == 0) {
    config.type = RobotLinkConfigType::SET_BRAKE;
    config.value = 1;
  } else if (strcmp(frame, "SET,BRAKE,OFF") == 0) {
    config.type = RobotLinkConfigType::SET_BRAKE;
  } else if (strcmp(frame, "SET,RAMP,ON") == 0) {
    config.type = RobotLinkConfigType::SET_RAMP;
    config.value = 1;
  } else if (strcmp(frame, "SET,RAMP,OFF") == 0) {
    config.type = RobotLinkConfigType::SET_RAMP;
  } else if (strcmp(frame, "COMPASS,RESET") == 0) {
    debug_.println("STM32_COMMAND_RECEIVED=COMPASS,RESET");
    config.type = RobotLinkConfigType::RESET_COMPASS;
  } else if (strcmp(frame, "ENCODER,RESET") == 0) {
    config.type = RobotLinkConfigType::RESET_ENCODERS;
  }
  if (config.type != RobotLinkConfigType::NONE) {
    if (configRequestInFlight_ || compassResetAwaitingSample_) {
      serial_.print("<ERR,BUSY>\r\n");
      return;
    }
    configRequest_ = config;
    configRequestReady_ = true;
    configRequestInFlight_ = true;
    return;
  }
  if (strcmp(frame, "CMD,STOP") == 0 || strcmp(frame, "STOP") == 0) {
    stopRequested_ = true;
    motionRequested_ = false;
    motionRequest_ = {};
    motionRequestInFlight_ = false;
    stopAckPending_ = true;
    if (aiMode_) lastSessionActivityMs_ = now;
    return;
  }
  if (strcmp(frame, "MODE,AI") == 0) {
    stopRequested_ = true;
    aiMode_ = true;
    lastSessionActivityMs_ = now;
    serial_.print("<ACK,MODE,AI>\r\n");
    return;
  }
  if (strcmp(frame, "MODE,MANUAL") == 0) {
    stopRequested_ = true;
    motionRequested_ = false;
    motionRequest_ = {};
    motionRequestInFlight_ = false;
    motionLeaseActive_ = false;
    if (turnLeaseActive_) serial_.print("<ERR,TURN,CANCELLED>\r\n");
    turnLeaseActive_ = false;
    aiMode_ = false;
    serial_.print("<ACK,MODE,MANUAL>\r\n");
    return;
  }

  RobotLinkMotion motion = RobotLinkMotion::NONE;
  const char* ack = nullptr;
  int speed = 0;
  int angle = 0;
  char trailingMotion = '\0';
  bool continuous = false;
  bool deferredAck = false;
  unsigned long sessionId = 0;
  unsigned long operationId = 0;
  bool correlatedMotion = false;
  const auto rejectMotion = [&](const char* reason, bool nack = false) {
    serial_.print(nack ? "<NACK," : "<ERR,");
    serial_.print(reason);
    if (correlatedMotion) {
      PrintCorrelation(serial_, static_cast<uint32_t>(sessionId),
                       static_cast<uint32_t>(operationId));
    }
    serial_.print(">\r\n");
  };
  if (sscanf(frame, "MOVE,FWD,%d,%d,SID,%lu,OP,%lu%c", &angle, &speed,
                    &sessionId, &operationId, &trailingMotion) == 4) {
    motion = RobotLinkMotion::FORWARD;
    deferredAck = true;
    correlatedMotion = true;
  } else if (sscanf(frame, "MOVE,BACK,%d,%d,SID,%lu,OP,%lu%c", &angle,
                    &speed, &sessionId, &operationId, &trailingMotion) == 4) {
    motion = RobotLinkMotion::BACKWARD;
    deferredAck = true;
    correlatedMotion = true;
  } else if (sscanf(frame, "TURN,REL,LEFT,%d,%d,SID,%lu,OP,%lu%c", &angle,
                    &speed, &sessionId, &operationId, &trailingMotion) == 4) {
    motion = RobotLinkMotion::TURN_REL_LEFT;
    deferredAck = true;
    correlatedMotion = true;
  } else if (sscanf(frame, "TURN,REL,RIGHT,%d,%d,SID,%lu,OP,%lu%c", &angle,
                    &speed, &sessionId, &operationId, &trailingMotion) == 4) {
    motion = RobotLinkMotion::TURN_REL_RIGHT;
    deferredAck = true;
    correlatedMotion = true;
  } else if (sscanf(frame, "TURN,ABS,%d,%d,SID,%lu,OP,%lu%c", &angle,
                    &speed, &sessionId, &operationId, &trailingMotion) == 4) {
    motion = RobotLinkMotion::TURN_ABSOLUTE;
    deferredAck = true;
    correlatedMotion = true;
  } else if (sscanf(frame, "MOVE,FWD,%d,CONT%c", &speed,
                    &trailingMotion) == 1) {
    motion = RobotLinkMotion::FORWARD;
    continuous = true;
    deferredAck = true;
  } else if (sscanf(frame, "MOVE,BACK,%d,CONT%c", &speed,
                    &trailingMotion) == 1) {
    motion = RobotLinkMotion::BACKWARD;
    continuous = true;
    deferredAck = true;
  } else if (sscanf(frame, "MOVE,LEFT,%d,CONT%c", &speed,
                    &trailingMotion) == 1) {
    motion = RobotLinkMotion::LEFT;
    continuous = true;
    deferredAck = true;
  } else if (sscanf(frame, "MOVE,RIGHT,%d,CONT%c", &speed,
                    &trailingMotion) == 1) {
    motion = RobotLinkMotion::RIGHT;
    continuous = true;
    deferredAck = true;
  } else if (sscanf(frame, "CMD,FWD,%d", &speed) == 1) {
    motion = RobotLinkMotion::FORWARD;
    ack = "<ACK,FWD>\r\n";
  } else if (sscanf(frame, "CMD,BACK,%d", &speed) == 1) {
    motion = RobotLinkMotion::BACKWARD;
    ack = "<ACK,BACK>\r\n";
  } else if (sscanf(frame, "CMD,LEFT,%d", &speed) == 1) {
    motion = RobotLinkMotion::LEFT;
    ack = "<ACK,LEFT>\r\n";
  } else if (sscanf(frame, "CMD,RIGHT,%d", &speed) == 1) {
    motion = RobotLinkMotion::RIGHT;
    ack = "<ACK,RIGHT>\r\n";
  }

  // V5 finite MOVE/TURN operations are transactional. A finite request that
  // fails to match one of the correlated forms above must never fall through
  // to a legacy parser and acquire a motion lease. Continuous/pulse commands
  // intentionally keep their existing compatibility path for this H0 fix.
  if (motion == RobotLinkMotion::NONE &&
      (strncmp(frame, "MOVE,", 5) == 0 || strncmp(frame, "TURN,", 5) == 0)) {
    serial_.print("<ERR,CORRELATION_REQUIRED>\r\n");
    return;
  }

  if (motion != RobotLinkMotion::NONE) {
    if (correlatedMotion && (sessionId == 0UL || operationId == 0UL)) {
      rejectMotion("OP_INVALID");
      return;
    }
    if (!motionSessionReady_) {
      rejectMotion("SESSION_NOT_READY");
      return;
    }
    if (!aiMode_) {
      rejectMotion("MODE", true);
      return;
    }
    if (telemetry.ps2CommandActive) {
      rejectMotion("PS2_OVERRIDE");
      return;
    }
    if (telemetry.brakeEnabled) {
      rejectMotion("BRAKE,LOCKED");
      return;
    }
    if (motion == RobotLinkMotion::FORWARD && telemetry.obstacleBlocked) {
      serial_.print("<ERR,OBSTACLE,DIST,");
      serial_.print(telemetry.obstacleDistanceCm, 1);
      if (correlatedMotion) {
        PrintCorrelation(serial_, static_cast<uint32_t>(sessionId),
                         static_cast<uint32_t>(operationId));
      }
      serial_.print(">\r\n");
      return;
    }
    if (deferredAck && (motionRequestInFlight_ || motionLeaseActive_)) {
      rejectMotion("BUSY");
      return;
    }
    if (deferredAck &&
        (speed < ROBOT_AI_SPEED_MIN || speed > ROBOT_AI_SPEED_MAX)) {
      rejectMotion("INVALID_SPEED");
      return;
    }
    const bool turn = motion == RobotLinkMotion::TURN_REL_LEFT ||
                      motion == RobotLinkMotion::TURN_REL_RIGHT ||
                      motion == RobotLinkMotion::TURN_ABSOLUTE;
    if (turn && !telemetry.headingReady) {
      rejectMotion("HEADING,LOST");
      return;
    }
    const bool distanceMove = deferredAck && !continuous &&
                              (motion == RobotLinkMotion::FORWARD ||
                               motion == RobotLinkMotion::BACKWARD);
    if (distanceMove && (angle < 1 ||
                         angle > static_cast<int>(ROBOT_AI_DISTANCE_MAX_MM))) {
      rejectMotion("INVALID_DISTANCE");
      return;
    }
    if (distanceMove && !telemetry.encoderReady) {
      rejectMotion("ENCODER,NOT_READY");
      return;
    }
    if (distanceMove && telemetry.encoderHealth != 1U) {
      rejectMotion("ENCODER,FAULT");
      return;
    }
    if ((motion == RobotLinkMotion::TURN_REL_LEFT ||
         motion == RobotLinkMotion::TURN_REL_RIGHT) &&
        (angle < 1 || angle > TURN_MAX_RELATIVE_DEG)) {
      rejectMotion("INVALID_ANGLE");
      return;
    }
    if (motion == RobotLinkMotion::TURN_ABSOLUTE &&
        (angle < -180 || angle > 180)) {
      rejectMotion("INVALID_ANGLE");
      return;
    }
    if (!deferredAck) {
      speed = constrain(speed, ROBOT_AI_SPEED_MIN, ROBOT_AI_SPEED_MAX);
    }
    motionRequest_.motion = motion;
    motionRequest_.speed = static_cast<int16_t>(speed);
    motionRequest_.angleDeg = static_cast<int16_t>(angle);
    motionRequest_.distanceMm = distanceMove ? static_cast<uint32_t>(angle) : 0U;
    motionRequest_.continuous = continuous;
    motionRequest_.deferredAck = deferredAck;
    motionRequest_.sessionId = correlatedMotion
        ? static_cast<uint32_t>(sessionId) : 0U;
    motionRequest_.operationId = correlatedMotion
        ? static_cast<uint32_t>(operationId) : 0U;
    motionRequested_ = true;
    motionRequestInFlight_ = deferredAck;
    lastSessionActivityMs_ = now;
    if (!deferredAck) serial_.print(ack);
    return;
  }

  serial_.print("<ERR,UNKNOWN_COMMAND>\r\n");
}

void RobotLinkServer::handleAsciiByte(char value,
                                      const RobotTelemetry& telemetry) {
  if (value == '<') {
    asciiReceiving_ = true;
    asciiLength_ = 0;
    return;
  }
  if (!asciiReceiving_) return;
  if (value == '>') {
    asciiBuffer_[asciiLength_] = '\0';
    asciiReceiving_ = false;
    handleAsciiFrame(asciiBuffer_, telemetry, false);
    return;
  }
  if (value == '\r' || value == '\n') return;
  if (asciiLength_ >= sizeof(asciiBuffer_) - 1U) {
    asciiReceiving_ = false;
    asciiLength_ = 0;
    return;
  }
  asciiBuffer_[asciiLength_++] = value;
}

void RobotLinkServer::handleFrame(const RobotLink::Frame& frame,
                                  const RobotTelemetry& telemetry) {
  ++validFrames_;
  lastConfirmedRxMs_ = millis();

  char body[RobotLink::MAX_FRAME] = {};
  if (frame.payload[0] == '\0') {
    snprintf(body, sizeof(body), "%s", frame.type);
  } else {
    snprintf(body, sizeof(body), "%s,%s", frame.type, frame.payload);
  }
  if (strcmp(body, "HELLO,PROTO,3") == 0) {
    rxSequenceInitialized_ = true;
    lastRxSequence_ = frame.sequence;
    helloReceived_ = true;
    motionSessionReady_ = false;
    motionLeaseActive_ = false;
    turnLeaseActive_ = false;
    handleAsciiFrame(body, telemetry, true);
    return;
  }

  const bool isStop = strcmp(frame.type, RobotLink::MessageType::STOP) == 0;
  if (!isStop && !acceptSequence(frame.sequence)) {
    serial_.print("<ERR,SEQ_REPLAY>\r\n");
    return;
  }

  handleAsciiFrame(body, telemetry, true);
}

bool RobotLinkServer::acceptSequence(uint16_t sequence) {
  if (!rxSequenceInitialized_) {
    rxSequenceInitialized_ = true;
    lastRxSequence_ = sequence;
    return true;
  }
  const uint16_t delta = static_cast<uint16_t>(sequence - lastRxSequence_);
  if (delta == 0U || delta >= 0x8000U) return false;
  lastRxSequence_ = sequence;
  return true;
}

void RobotLinkServer::update(const RobotTelemetry& telemetry) {
  lastObstacleZone_ = telemetry.obstacleZone;
  lastObstacleDistanceCm_ = telemetry.obstacleDistanceCm;
  lastObstacleFresh_ = telemetry.ultrasonicFresh;
  lastObstacleMotorsStopped_ =
      telemetry.leftCommand == 0 && telemetry.rightCommand == 0;

  RobotLink::Frame frame;
  uint8_t budget = 64U;
  while (budget-- > 0U && serial_.available() > 0) {
    ++receivedBytes_;
    const char value = static_cast<char>(serial_.read());
    handleAsciiByte(value, telemetry);
    if (parser_.push(value, frame)) handleFrame(frame, telemetry);
  }

  const uint32_t now = millis();
  if (connected() && telemetry.obstacleZoneSequence != 0U &&
      (!obstacleEventInitialized_ ||
       telemetry.obstacleZoneSequence != lastObstacleZoneSequence_)) {
    obstacleEventInitialized_ = true;
    lastObstacleZoneSequence_ = telemetry.obstacleZoneSequence;
    if (telemetry.obstacleZone >= 2U) {
      serial_.print("<EVENT,OBSTACLE,DETECTED,ZONE,");
      serial_.print(ObstacleZoneName(telemetry.obstacleZone));
      serial_.print(",DIST,");
      serial_.print(telemetry.obstacleDistanceCm, 1);
      serial_.print(">\r\n");
    } else if (telemetry.obstacleZone == 1U) {
      serial_.print("<EVENT,OBSTACLE,CLEAR,DIST,");
      serial_.print(telemetry.obstacleDistanceCm, 1);
      serial_.print(">\r\n");
    }
  }
  if (motionLeaseActive_ && telemetry.obstacleLimited &&
      telemetry.leftCommand == 0 && telemetry.rightCommand == 0) {
    serial_.print("<EVENT,OBSTACLE,STOPPED,ZONE,");
    serial_.print(ObstacleZoneName(telemetry.obstacleZone));
    serial_.print(",DIST,");
    serial_.print(telemetry.obstacleDistanceCm, 1);
    serial_.print(">\r\n");
    if (!turnLeaseActive_) motionLeaseActive_ = false;
  }
  const bool encoderFault = telemetry.encoderHealth >= 3U &&
                            telemetry.encoderHealth <= 5U;
  if (encoderFault && !encoderFaultReported_) {
    encoderFaultReported_ = true;
    serial_.print("<EVENT,ENCODER,FAULT,");
    serial_.print(EncoderHealthName(telemetry.encoderHealth));
    serial_.print(">\r\n");
    if (motionLeaseActive_) {
      motionLeaseActive_ = false;
      turnLeaseActive_ = false;
    }
  } else if (!encoderFault) {
    encoderFaultReported_ = false;
  }
  if (compassResetAwaitingSample_) {
    if (telemetry.compassConnected &&
        telemetry.compassZeroGeneration > compassResetGeneration_) {
      compassResetAwaitingSample_ = false;
      serial_.print("<EVENT,COMPASS,ZEROED,H,");
      serial_.print(telemetry.headingDeg, 1);
      serial_.print(">\r\n");
    } else if ((now - compassResetStartedMs_) >
               COMPASS_RESET_EVENT_TIMEOUT_MS) {
      compassResetAwaitingSample_ = false;
      // Compass is an optional sensor. A missing zero sample is diagnostic
      // information only; it must not be reported as a heading failure.
      serial_.print("<EVENT,COMPASS,UNAVAILABLE>\r\n");
    }
  }
  if (aiMode_ && telemetry.ps2CommandActive) {
    aiMode_ = false;
    motionRequested_ = false;
    motionRequest_ = {};
    motionRequestInFlight_ = false;
    if (motionLeaseActive_) {
      serial_.print("<EVENT,AI_CANCELLED,PS2_OVERRIDE>\r\n");
    }
    motionLeaseActive_ = false;
    turnLeaseActive_ = false;
  }
  if (aiMode_ && motionLeaseActive_ &&
      (now - lastMotionHeartbeatMs_) > ROBOT_MOTION_LEASE_TIMEOUT_MS) {
    motionRequested_ = false;
    motionRequest_ = {};
    motionRequestInFlight_ = false;
    stopRequested_ = true;
    serial_.print("<EVENT,STOP,MOTION_LEASE_TIMEOUT>\r\n");
    motionLeaseActive_ = false;
    turnLeaseActive_ = false;
    debug_.println("ROBOT_AI,MOTION_LEASE_TIMEOUT=1");
  }
  if (aiMode_ && lastSessionActivityMs_ != 0U &&
      (now - lastSessionActivityMs_) > ROBOT_AI_SESSION_TIMEOUT_MS) {
    aiMode_ = false;
    motionRequested_ = false;
    motionRequest_ = {};
    motionRequestInFlight_ = false;
    if (motionLeaseActive_) stopRequested_ = true;
    motionLeaseActive_ = false;
    turnLeaseActive_ = false;
    serial_.print("<EVENT,AI_SESSION,EXPIRED>\r\n");
    debug_.println("ROBOT_AI,SESSION_TIMEOUT=1,MODE=MANUAL");
  }
  if (aiMode_ && turnLeaseActive_ && telemetry.aiTurnActive &&
      (now - lastTurnProgressMs_) >= TURN_PROGRESS_MS) {
    lastTurnProgressMs_ = now;
    serial_.print("<PROGRESS,TURN,H,");
    serial_.print(telemetry.headingDeg, 1);
    serial_.print(",TGT,");
    serial_.print(telemetry.aiTurnTargetDeg, 1);
    serial_.print(",ERR,");
    serial_.print(telemetry.aiTurnErrorDeg, 1);
    serial_.print(",SPD,");
    serial_.print(telemetry.aiTurnSpeed);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  }
  if (aiMode_ && motionLeaseActive_ && telemetry.aiDistanceActive &&
      (now - lastTurnProgressMs_) >= ROBOT_AI_DISTANCE_PROGRESS_MS) {
    lastTurnProgressMs_ = now;
    serial_.print("<PROGRESS,MOVE,TARGET,");
    serial_.print(telemetry.aiDistanceTargetMm, 1);
    serial_.print(",TRAVEL,");
    serial_.print(telemetry.aiDistanceTravelledMm, 1);
    serial_.print(",L,");
    serial_.print(telemetry.leftCommand);
    serial_.print(",R,");
    serial_.print(telemetry.rightCommand);
    serial_.print(",LT,");
    serial_.print(static_cast<long>(telemetry.leftEncoderTicks));
    serial_.print(",RT,");
    serial_.print(static_cast<long>(telemetry.rightEncoderTicks));
    serial_.print(",LV,");
    serial_.print(telemetry.leftVelocityMmS, 1);
    serial_.print(",RV,");
    serial_.print(telemetry.rightVelocityMmS, 1);
    serial_.print(",ENC,");
    serial_.print(EncoderHealthName(telemetry.encoderHealth));
    serial_.print(",BRAKE,");
    serial_.print(telemetry.brakeEnabled ? 1 : 0);
    serial_.print(",RAMP,");
    serial_.print(telemetry.rampEnabled ? 1 : 0);
    serial_.print(",PS2CMD,");
    serial_.print(telemetry.ps2CommandActive ? 1 : 0);
    serial_.print(",OBS,");
    serial_.print(ObstacleZoneName(telemetry.obstacleZone));
    serial_.print(",LIMITED,");
    serial_.print(telemetry.obstacleLimited ? 1 : 0);
    PrintCorrelation(serial_, activeMotionSessionId_, activeMotionOperationId_);
    serial_.print(">\r\n");
  }
  if (connected() != linkReported_) {
    linkReported_ = connected();
    debug_.print("ROBOT_LINK,STATE=");
    debug_.println(linkReported_ ? "CONNECTED" : "LOST");
  }
  if ((now - lastDiagnosticMs_) >= 1000U) {
    lastDiagnosticMs_ = now;
    debug_.print("ROBOT_UART,RX_BYTES=");
    debug_.print(receivedBytes_);
    debug_.print(",VALID=");
    debug_.print(validFrames_);
    debug_.print(",BIDIR=");
    debug_.println(connected() ? 1 : 0);
  }
}

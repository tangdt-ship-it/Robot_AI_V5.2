#include <map/map_controller.h>

#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t kTeachSampleMs = 100U;       // 10 Hz observation.
constexpr float kAutoDistanceMm = 750.0f;
constexpr float kCornerTriggerDeg = 25.0f;
constexpr float kCornerReleaseDeg = 15.0f;
constexpr uint8_t kCornerStableSamples = 3U;
constexpr float kDuplicateDistanceMm = 20.0f;
constexpr float kDuplicateHeadingDeg = 2.0f;
constexpr float kEndpointDistanceMm = 50.0f;
constexpr float kEndpointHeadingDeg = 5.0f;
constexpr float kWaypointToleranceMm = 60.0f;
constexpr float kReplayPoseHoldToleranceMm = 100.0f;
constexpr float kReplayPoseHoldToleranceDeg = 15.0f;
constexpr float kMinimumSegmentMm = 20.0f;
constexpr float kMaximumSegmentMm = 5000.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr int16_t kReplaySpeed = 20;

const char* ActionName(Ps2MapAction action) {
  switch (action) {
    case Ps2MapAction::SLOT: return "SLOT";
    case Ps2MapAction::START: return "START";
    case Ps2MapAction::TRIANGLE: return "TRIANGLE";
    case Ps2MapAction::SQUARE: return "SQUARE";
    case Ps2MapAction::SQUARE_LONG: return "SQUARE_LONG";
    case Ps2MapAction::CIRCLE: return "CIRCLE";
    case Ps2MapAction::CROSS: return "CROSS";
    case Ps2MapAction::CROSS_LONG: return "CROSS_LONG";
    case Ps2MapAction::SELECT_LONG: return "SELECT_LONG";
  }
  return "UNKNOWN";
}

bool IsReplayModeAllowed(MapRouteType type, MapReplayMode mode) {
  if (type == MapRouteType::CLOSED) {
    return mode == MapReplayMode::ONCE || mode == MapReplayMode::LOOP;
  }
  return mode == MapReplayMode::ONCE || mode == MapReplayMode::RETURN ||
         mode == MapReplayMode::PING_PONG;
}

const char* HoldReasonName(MapHoldReason reason) {
  switch (reason) {
    case MapHoldReason::USER: return "USER";
    case MapHoldReason::OBSTACLE: return "OBSTACLE";
    case MapHoldReason::EXTERNAL_STOP: return "EXTERNAL_STOP";
    case MapHoldReason::PS2_TAKEOVER: return "PS2_TAKEOVER";
    case MapHoldReason::NONE: return "NONE";
  }
  return "NONE";
}
}  // namespace

MapController::MapController(RobotController& robot, Ps2Controller& ps2,
                             LcdDisplay& display, WheelOdometry& odometry,
                             HeadingFusion& fusion,
                             UltrasonicSensor& ultrasonic, Print& debugStream)
    : robot_(robot), ps2_(ps2), display_(display), odometry_(odometry),
      fusion_(fusion), ultrasonic_(ultrasonic), debug_(debugStream) {}

void MapController::begin() {
  if (!store_.begin()) {
    storeState_ = MapStoreState::STORAGE_ERROR;
    log("OWNER=STM32,STORE=ERROR");
    publishStatus();
    return;
  }
  const MapSlotMetadata metadata = store_.metadata(selectedSlot_);
  storeState_ = metadata.state;
  routeType_ = metadata.routeType;
  routeMode_ = IsReplayModeAllowed(routeType_, metadata.replayMode)
                   ? metadata.replayMode
                   : MapReplayMode::ONCE;
  mode_ = metadata.state == MapStoreState::SAVED
              ? MapControllerMode::SAVED
              : MapControllerMode::READY;
  log("OWNER=STM32,LEGACY_ESP32_RUNTIME=OFF,PS2=LOCAL,REPLAY=LOCAL");
  publishStatus();
}

void MapController::processInput() {
  Ps2MapEvent event;
  if (ps2_.takeMapEvent(event)) handleEvent(event);
}

void MapController::update() {
  processInput();

  if (teachFinishPending_) {
    if (robot_.motorsStopped() && !robot_.aiMotionActive() &&
        !ps2_.motionCommandActive()) {
      teachFinishPending_ = false;
      (void)finalizeTeach();
    }
  }
  if (mode_ == MapControllerMode::TEACHING && !teachFinishPending_) {
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - nextTeachSampleMs_) >= 0) {
      nextTeachSampleMs_ = now + kTeachSampleMs;
      sampleTeach();
    }
  }

  if (replayActive_) {
    if (odometry_.resetGeneration() != replayOriginResetGeneration_ ||
        robot_.headingResetGeneration() != replayOriginHeadingResetGeneration_ ||
        route_.header.generation != replayOriginRouteGeneration_) {
      abortReplay("RESET_BOUNDARY");
    } else if (ps2_.state().r3) {
      cancelReplay("R3");
    } else if (robot_.motionOwner() != MotionOwner::REPLAY &&
               !robot_.aiMotionActive() &&
               replayOperation_ != MapReplayOperation::NONE) {
      // External STOP/mission arbitration removed the owner without producing
      // a replay result. Do not infer success from a stopped motor.
      enterReplayHold(MapHoldReason::EXTERNAL_STOP, false);
    } else {
      updateReplay();
    }
  }

  serviceCancelTrace();
  serviceStorage();
  const uint32_t now = millis();
  if (statusDirty_ || (replayActive_ && (now - lastStatusMs_) >= 250U)) {
    publishStatus();
  }
}

void MapController::handleEvent(const Ps2MapEvent& event) {
  debug_.print("MAP,EVENT=");
  debug_.print(ActionName(event.action));
  debug_.print(",SLOT=");
  debug_.println(static_cast<unsigned>(event.slot));

  switch (event.action) {
    case Ps2MapAction::SLOT: handleSlot(event.slot); break;
    case Ps2MapAction::START: handleStart(); break;
    case Ps2MapAction::TRIANGLE: handleTriangle(); break;
    case Ps2MapAction::CIRCLE: handleCircle(); break;
    case Ps2MapAction::SQUARE: handleSquare(false); break;
    case Ps2MapAction::SQUARE_LONG: handleSquare(true); break;
    case Ps2MapAction::CROSS: handleCross(); break;
    case Ps2MapAction::CROSS_LONG: handleCrossLong(); break;
    case Ps2MapAction::SELECT_LONG:
      // Kept as a compatibility event but no longer aliases delete/cancel;
      // CROSS is the unambiguous local cancel/STOP action.
      break;
  }
  statusDirty_ = true;
}

void MapController::handleSlot(uint8_t slot) {
  if (mode_ == MapControllerMode::TEACHING || replayActive_ ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::REPLAY_HOLD) {
    return;
  }
  selectedSlot_ = slot == 2U ? MapSlot::MAP_2 : MapSlot::MAP_1;
  loadedValid_ = false;
  const MapSlotMetadata metadata = store_.metadata(selectedSlot_);
  storeState_ = metadata.state;
  routeType_ = metadata.routeType;
  routeMode_ = IsReplayModeAllowed(routeType_, metadata.replayMode)
                   ? metadata.replayMode
                   : MapReplayMode::ONCE;
  mode_ = metadata.state == MapStoreState::SAVED
              ? MapControllerMode::SAVED
              : MapControllerMode::READY;
}

void MapController::handleStart() {
  if (mode_ == MapControllerMode::TEACHING ||
      mode_ == MapControllerMode::DELETE_CONFIRM) {
    logStartReject(mode_ == MapControllerMode::TEACHING ? "TEACHING"
                                                        : "DELETE_CONFIRM");
    return;
  }
  if (replayActive_) {
    logStartReject("REPLAY_ACTIVE");
    return;
  }
  if (mode_ == MapControllerMode::REPLAY_HOLD) {
    debug_.println("MAP,START,ACTION=RESUME");
    debug_.print("MAP,RESUME,REQUEST,WP=");
    debug_.print(static_cast<unsigned>(replayTargetIndex_));
    debug_.print(",GEN=");
    debug_.println(replayGeneration_);
    const char* rejectReason = nullptr;
    if (canResumeReplay(rejectReason)) {
      // Keep the original route origin and route coordinates. The next
      // segment computes its target from the current live pose, so coast after
      // HOLD never turns an old remaining-distance value into ground truth.
      nextReplayGeneration();
      replaySegmentGeneration_ = 0U;
      replayActive_ = true;
      mode_ = MapControllerMode::REPLAY_RUNNING;
      replayOperation_ = MapReplayOperation::NONE;
      replayReason_ = "RESUME";
      holdReason_ = MapHoldReason::NONE;
      statusDirty_ = true;
      debug_.print("MAP,RESUME,ACCEPT,WP=");
      debug_.print(static_cast<unsigned>(replayTargetIndex_));
      debug_.print(",GEN=");
      debug_.println(replayGeneration_);
    } else {
      const char* reason = rejectReason != nullptr ? rejectReason
                                                    : "HOLD_NOT_RESUMABLE";
      debug_.print("MAP,RESUME,REJECT,REASON=");
      debug_.println(reason);
      logStartReject(reason);
    }
    return;
  }
  debug_.println("MAP,START,ACTION=RUN");
  const char* reason = nullptr;
  if (prepareReplay(reason)) {
    debug_.print("MAP,START,ACCEPT,GEN=");
    debug_.println(replayGeneration_);
  } else {
    logStartReject(reason != nullptr ? reason : "PRECHECK");
  }
}

void MapController::handleTriangle() {
  if (replayActive_ || mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::DELETE_CONFIRM) {
    return;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    markManualWaypoint();
    return;
  }
  (void)beginTeach();
}

void MapController::handleCircle() {
  if (mode_ == MapControllerMode::TEACHING) {
    requestTeachFinish();
    return;
  }
  if (mode_ == MapControllerMode::DELETE_CONFIRM) {
    deletePending_ = true;
    mode_ = MapControllerMode::READY;
    return;
  }
  if (replayActive_) return;
  if (storeState_ == MapStoreState::SAVED && !loadedValid_) {
    if (!loadSelected()) return;
  }
  if (storeState_ != MapStoreState::SAVED || !loadedValid_) return;
  const MapReplayMode previous = routeMode_;
  if (routeType_ == MapRouteType::CLOSED) {
    routeMode_ = previous == MapReplayMode::LOOP ? MapReplayMode::ONCE
                                                 : MapReplayMode::LOOP;
  } else {
    routeMode_ = previous == MapReplayMode::ONCE
                     ? MapReplayMode::RETURN
                     : previous == MapReplayMode::RETURN
                         ? MapReplayMode::PING_PONG
                         : MapReplayMode::ONCE;
  }
  modeBeforeSave_ = previous;
  modeSavePending_ = true;
  statusDirty_ = true;
}

void MapController::handleSquare(bool longPress) {
  if (longPress) {
    if (replayActive_ || mode_ == MapControllerMode::TEACHING ||
        mode_ == MapControllerMode::DELETE_CONFIRM ||
        !robot_.motorsStopped() || robot_.aiMotionActive()) {
      return;
    }
    mode_ = MapControllerMode::DELETE_CONFIRM;
    return;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    if (route_.header.waypointCount > 1U) {
      --route_.header.waypointCount;
      route_.header.routeLengthMm = routeLengthMm(route_);
      resetTeachTracking();
    }
  }
}

void MapController::handleCross() {
  if (mode_ == MapControllerMode::TEACHING) {
    cancelTeach();
  } else if (replayActive_) {
    // X-down is the immediate safety stop and enters a resumable USER HOLD.
    enterReplayHold(MapHoldReason::USER, true);
  } else if (mode_ == MapControllerMode::REPLAY_HOLD) {
    // A new short X while already held leaves the hold in place. The long
    // event for this press is handled separately by handleCrossLong().
  } else if (mode_ == MapControllerMode::DELETE_CONFIRM) {
    mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                                : MapControllerMode::READY;
  }
}

void MapController::handleCrossLong() {
  if (replayActive_) {
    // Be robust if the short event was delayed behind another input: safety
    // still stops first, then the same physical press escalates to CANCEL.
    enterReplayHold(MapHoldReason::USER, true);
  }
  if (mode_ == MapControllerMode::REPLAY_HOLD) {
    cancelReplay("X_LONG");
  }
}

bool MapController::loadSelected() {
  if (!store_.load(selectedSlot_, route_)) {
    loadedValid_ = false;
    storeState_ = MapStoreState::INVALID;
    mode_ = MapControllerMode::READY;
    return false;
  }
  const char* reason = nullptr;
  if (!validateRoute(route_, reason)) {
    loadedValid_ = false;
    storeState_ = MapStoreState::INVALID;
    mode_ = MapControllerMode::READY;
    return false;
  }
  loadedValid_ = true;
  storeState_ = MapStoreState::SAVED;
  routeType_ = static_cast<MapRouteType>(route_.header.routeType);
  routeMode_ = static_cast<MapReplayMode>(route_.header.replayMode);
  mode_ = MapControllerMode::SAVED;
  return true;
}

bool MapController::beginTeach() {
  if (replayActive_ || !robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.brakeEnabled() || ps2_.motionCommandActive()) {
    return false;
  }
  Pose origin;
  if (!readPose(origin)) return false;
  teachOrigin_ = origin;
  teachOriginValid_ = true;
  route_ = {};
  route_.header.waypointCount = 0U;
  routeType_ = MapRouteType::OPEN;
  routeMode_ = MapReplayMode::ONCE;
  const Pose localOrigin{};
  if (!appendWaypoint(localOrigin, MAP_WP_START)) return false;
  mode_ = MapControllerMode::TEACHING;
  storeState_ = MapStoreState::EMPTY;
  nextTeachSampleMs_ = millis() + kTeachSampleMs;
  resetTeachTracking();
  statusDirty_ = true;
  return true;
}

void MapController::cancelTeach() {
  route_ = {};
  teachOriginValid_ = false;
  teachFinishPending_ = false;
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                              : MapControllerMode::READY;
}

void MapController::markManualWaypoint() {
  Pose pose;
  if (!teachOriginValid_ || !readPose(pose)) return;
  (void)appendWaypoint(toTeachLocal(pose), MAP_WP_MANUAL_MARK);
}

void MapController::sampleTeach() {
  Pose pose;
  if (!teachOriginValid_ || !readPose(pose)) return;
  const Pose local = toTeachLocal(pose);
  if (!lastTeachSampleValid_) {
    lastTeachSample_ = local;
    lastTeachSampleValid_ = true;
    return;
  }
  const float moved = distanceMm(local.xMm, local.yMm, lastTeachSample_.xMm,
                                 lastTeachSample_.yMm);
  const float corner = fabsf(shortestDeltaDeg(
      local.headingDeg, lastTeachSample_.headingDeg));
  if (corner >= kCornerTriggerDeg) {
    if (cornerStableSamples_ < 255U) ++cornerStableSamples_;
  } else if (corner < kCornerReleaseDeg) {
    cornerStableSamples_ = 0U;
  }

  const uint16_t count = route_.header.waypointCount;
  const MapWaypoint& last = route_.waypoints[count - 1U];
  const float fromStored = distanceMm(local.xMm, local.yMm,
                                       static_cast<float>(last.xMm),
                                       static_cast<float>(last.yMm));
  if (fromStored >= kAutoDistanceMm) {
    (void)appendWaypoint(local, MAP_WP_AUTO_DISTANCE);
    cornerStableSamples_ = 0U;
  } else if (cornerStableSamples_ >= kCornerStableSamples &&
             moved >= kMinimumSegmentMm) {
    (void)appendWaypoint(local, MAP_WP_AUTO_CORNER);
    cornerStableSamples_ = 0U;
  }
  lastTeachSample_ = local;
}

bool MapController::appendWaypoint(const Pose& pose, uint8_t flags) {
  uint16_t& count = route_.header.waypointCount;
  if (count > STM32_MAP_MAX_WAYPOINTS) return false;
  if (count != 0U) {
    MapWaypoint& previous = route_.waypoints[count - 1U];
    const float separation = distanceMm(pose.xMm, pose.yMm,
                                        static_cast<float>(previous.xMm),
                                        static_cast<float>(previous.yMm));
    const float headingDelta = fabsf(shortestDeltaDeg(
        static_cast<float>(headingCdeg(pose.headingDeg)) / 100.0f,
        static_cast<float>(previous.headingCdeg) / 100.0f));
    if (separation <= kDuplicateDistanceMm &&
        headingDelta <= kDuplicateHeadingDeg) {
      if ((flags & (MAP_WP_MANUAL_MARK | MAP_WP_ENDPOINT)) != 0U) {
        previous.xMm = static_cast<int32_t>(lroundf(pose.xMm));
        previous.yMm = static_cast<int32_t>(lroundf(pose.yMm));
        previous.headingCdeg = headingCdeg(pose.headingDeg);
      }
      previous.flags |= flags;
      route_.header.routeLengthMm = routeLengthMm(route_);
      statusDirty_ = true;
      return true;
    }
  }
  if (count >= STM32_MAP_MAX_WAYPOINTS) return false;
  MapWaypoint& point = route_.waypoints[count++];
  point = {};
  point.xMm = static_cast<int32_t>(lroundf(pose.xMm));
  point.yMm = static_cast<int32_t>(lroundf(pose.yMm));
  point.headingCdeg = headingCdeg(pose.headingDeg);
  point.flags = flags;
  route_.header.routeLengthMm = routeLengthMm(route_);
  statusDirty_ = true;
  return true;
}

bool MapController::readPose(Pose& pose) const {
  if (!odometry_.ready() || !odometry_.healthy() || !fusion_.ready()) return false;
  pose.xMm = odometry_.data().xMm;
  pose.yMm = odometry_.data().yMm;
  pose.headingDeg = fusion_.headingDeg();
  return isfinite(pose.xMm) && isfinite(pose.yMm) &&
         isfinite(pose.headingDeg);
}

MapController::Pose MapController::toTeachLocal(const Pose& pose) const {
  const float dx = pose.xMm - teachOrigin_.xMm;
  const float dy = pose.yMm - teachOrigin_.yMm;
  const float c = cosf(-teachOrigin_.headingDeg * kDegToRad);
  const float s = sinf(-teachOrigin_.headingDeg * kDegToRad);
  Pose local;
  local.xMm = dx * c - dy * s;
  local.yMm = dx * s + dy * c;
  local.headingDeg = shortestDeltaDeg(pose.headingDeg,
                                      teachOrigin_.headingDeg);
  return local;
}

int16_t MapController::headingCdeg(float headingDegValue) {
  float normalized = normalizeDeg(headingDegValue);
  return static_cast<int16_t>(lroundf(normalized * 100.0f));
}

float MapController::normalizeDeg(float degrees) {
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees <= -180.0f) degrees += 360.0f;
  return degrees;
}

float MapController::shortestDeltaDeg(float targetDeg, float currentDeg) {
  return normalizeDeg(targetDeg - currentDeg);
}

float MapController::distanceMm(float ax, float ay, float bx, float by) {
  const float dx = ax - bx;
  const float dy = ay - by;
  return sqrtf(dx * dx + dy * dy);
}

bool MapController::approximatelyEqual(float lhs, float rhs, float tolerance) {
  return fabsf(lhs - rhs) <= tolerance;
}

void MapController::resetTeachTracking() {
  lastTeachSampleValid_ = false;
  cornerStableSamples_ = 0U;
}

void MapController::requestTeachFinish() {
  if (!teachOriginValid_) return;
  teachFinishPending_ = true;
  if (robot_.motorsStopped() && !robot_.aiMotionActive() &&
      !ps2_.motionCommandActive()) {
    teachFinishPending_ = false;
    (void)finalizeTeach();
  }
}

bool MapController::finalizeTeach() {
  Pose pose;
  if (!readPose(pose)) return false;
  if (!appendWaypoint(toTeachLocal(pose), MAP_WP_ENDPOINT)) return false;
  const char* reason = nullptr;
  const uint16_t count = route_.header.waypointCount;
  if (count >= 2U) {
    const float closeDistance = distanceMm(
        static_cast<float>(route_.waypoints[0].xMm),
        static_cast<float>(route_.waypoints[0].yMm),
        static_cast<float>(route_.waypoints[count - 1U].xMm),
        static_cast<float>(route_.waypoints[count - 1U].yMm));
    const float closeHeading = fabsf(shortestDeltaDeg(
        static_cast<float>(route_.waypoints[count - 1U].headingCdeg) /
            100.0f,
        static_cast<float>(route_.waypoints[0].headingCdeg) / 100.0f));
    routeType_ = closeDistance <= kEndpointDistanceMm &&
                         closeHeading <= kEndpointHeadingDeg && count >= 3U
                     ? MapRouteType::CLOSED
                     : MapRouteType::OPEN;
  }
  if (!IsReplayModeAllowed(routeType_, routeMode_)) routeMode_ = MapReplayMode::ONCE;
  route_.header.routeType = static_cast<uint8_t>(routeType_);
  route_.header.replayMode = static_cast<uint8_t>(routeMode_);
  updateRouteHeaderForSave(route_);
  if (!validateRoute(route_, reason)) {
    debug_.print("MAP,TEACH=REJECT,REASON=");
    debug_.println(reason != nullptr ? reason : "INVALID");
    mode_ = MapControllerMode::TEACHING;
    return false;
  }
  savePending_ = true;
  mode_ = MapControllerMode::READY;
  teachOriginValid_ = false;
  return true;
}

uint32_t MapController::routeLengthMm(const MapRouteData& route) const {
  const uint16_t count = route.header.waypointCount;
  if (count < 2U) return 0U;
  float total = 0.0f;
  for (uint16_t i = 1U; i < count; ++i) {
    total += distanceMm(static_cast<float>(route.waypoints[i - 1U].xMm),
                        static_cast<float>(route.waypoints[i - 1U].yMm),
                        static_cast<float>(route.waypoints[i].xMm),
                        static_cast<float>(route.waypoints[i].yMm));
  }
  if (static_cast<MapRouteType>(route.header.routeType) == MapRouteType::CLOSED) {
    total += distanceMm(static_cast<float>(route.waypoints[count - 1U].xMm),
                        static_cast<float>(route.waypoints[count - 1U].yMm),
                        static_cast<float>(route.waypoints[0].xMm),
                        static_cast<float>(route.waypoints[0].yMm));
  }
  return total <= 0.0f ? 0U : static_cast<uint32_t>(lroundf(total));
}

void MapController::updateRouteHeaderForSave(MapRouteData& route) const {
  route.header.slot = static_cast<uint8_t>(selectedSlot_);
  route.header.waypointCount =
      constrain(route.header.waypointCount, 0U, STM32_MAP_MAX_WAYPOINTS);
  route.header.payloadBytes = static_cast<uint16_t>(
      route.header.waypointCount * sizeof(MapWaypoint));
  route.header.routeLengthMm = routeLengthMm(route);
}

bool MapController::validateRoute(const MapRouteData& route,
                                  const char*& reason) const {
  const uint16_t count = route.header.waypointCount;
  if (count < 2U || count > STM32_MAP_MAX_WAYPOINTS) {
    reason = "POINTS";
    return false;
  }
  if ((route.waypoints[0].flags & MAP_WP_START) == 0U ||
      (route.waypoints[count - 1U].flags & MAP_WP_ENDPOINT) == 0U) {
    reason = "ENDPOINTS";
    return false;
  }
  const MapRouteType type = static_cast<MapRouteType>(route.header.routeType);
  const MapReplayMode mode = static_cast<MapReplayMode>(route.header.replayMode);
  if (static_cast<uint8_t>(type) >
          static_cast<uint8_t>(MapRouteType::CLOSED) ||
      static_cast<uint8_t>(mode) >
          static_cast<uint8_t>(MapReplayMode::PING_PONG) ||
      !IsReplayModeAllowed(type, mode)) {
    reason = "TYPE_MODE";
    return false;
  }
  for (uint16_t i = 1U; i < count; ++i) {
    const float length = distanceMm(
        static_cast<float>(route.waypoints[i - 1U].xMm),
        static_cast<float>(route.waypoints[i - 1U].yMm),
        static_cast<float>(route.waypoints[i].xMm),
        static_cast<float>(route.waypoints[i].yMm));
    if (length < kMinimumSegmentMm || length > kMaximumSegmentMm) {
      reason = "SEGMENT";
      return false;
    }
  }
  if (type == MapRouteType::CLOSED) {
    if (count < 3U) {
      reason = "CLOSED_POINTS";
      return false;
    }
    const float closure = distanceMm(
        static_cast<float>(route.waypoints[count - 1U].xMm),
        static_cast<float>(route.waypoints[count - 1U].yMm),
        static_cast<float>(route.waypoints[0].xMm),
        static_cast<float>(route.waypoints[0].yMm));
    // The endpoint may intentionally coincide with START after duplicate
    // suppression. It is a closure marker, not a travelled segment; adjacent
    // zero-length segments above remain invalid.
    if (closure > kMaximumSegmentMm) {
      reason = "CLOSURE";
      return false;
    }
  }
  const uint32_t computed = routeLengthMm(route);
  if (computed == 0U ||
      !approximatelyEqual(static_cast<float>(computed),
                          static_cast<float>(route.header.routeLengthMm),
                          50.0f)) {
    reason = "LENGTH";
    return false;
  }
  reason = "OK";
  return true;
}

void MapController::cycleReplayMode() {
  // Kept as a named extension point for a future voice/MCP MAP surface. The
  // current product intentionally exposes no MCP MAP tool.
}

uint32_t MapController::nextReplayGeneration() {
  ++replayGeneration_;
  if (replayGeneration_ == 0U) ++replayGeneration_;
  return replayGeneration_;
}

bool MapController::prepareReplay(const char*& rejectReason) {
  rejectReason = nullptr;
  if (!loadSelected()) {
    rejectReason = "NOT_SAVED";
    return false;
  }
  const char* reason = nullptr;
  if (!replayPrecheck(route_, reason)) {
    mode_ = MapControllerMode::SAVED;
    replayReason_ = reason != nullptr ? reason : "PRECHECK";
    rejectReason = replayReason_;
    debug_.print("MAP,REPLAY=REJECT,REASON=");
    debug_.println(replayReason_);
    return false;
  }
  Pose live;
  if (!readPose(live)) {
    rejectReason = "POSE";
    return false;
  }
  replayOrigin_ = live;
  replayOriginValid_ = true;
  replayContextSlot_ = selectedSlot_;
  replayOriginResetGeneration_ = odometry_.resetGeneration();
  replayOriginHeadingResetGeneration_ = robot_.headingResetGeneration();
  replayCurrentIndex_ = 0U;
  replayTargetIndex_ = 1U;
  replayDirection_ = 1;
  replayReturned_ = false;
  replayOriginRouteGeneration_ = route_.header.generation;
  replayTargetDistanceMm_ = 0U;
  replayTargetDeg_ = 0;
  replayTravelMm_ = 0U;
  replayErrorMm_ = 0U;
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayReason_ = "RUNNING";
  replayResumeAllowed_ = false;
  replayHoldPoseValid_ = false;
  holdReason_ = MapHoldReason::NONE;
  replayActive_ = true;
  replayOperation_ = MapReplayOperation::NONE;
  mode_ = MapControllerMode::REPLAY_CHECKED;
  statusDirty_ = true;
  return true;
}

bool MapController::replayPrecheck(const MapRouteData& route,
                                   const char*& reason) const {
  if (!validateRoute(route, reason)) return false;
  const uint32_t now = millis();
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    reason = "MOTION_OWNER";
    return false;
  }
  if (robot_.brakeEnabled()) {
    reason = "BRAKE";
    return false;
  }
  if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
      ps2_.motionCommandActive()) {
    reason = "PS2_NOT_NEUTRAL";
    return false;
  }
  if (!odometry_.ready() || !odometry_.healthy()) {
    reason = "ODOMETRY";
    return false;
  }
  if (!fusion_.ready() || fusion_.health() == FusionHealth::NO_SOURCE) {
    reason = "HEADING";
    return false;
  }
  const bool obstacleLiveClear =
      ultrasonic_.isFresh() && ultrasonic_.healthy() &&
      ultrasonic_.overallZone() == ObstacleZone::CLEAR;
  const bool obstacleGraceClear =
      ultrasonic_.overallZone() == ObstacleZone::CLEAR &&
      ultrasonic_.hasRecentClearWindow(now);
  if (!obstacleLiveClear && !obstacleGraceClear) {
    reason = "OBSTACLE_SENSOR";
    return false;
  }
  if (obstacleGraceClear && !obstacleLiveClear) {
    debug_.println("MAP,REPLAY=PRECHECK,OBSTACLE_GRACE=1");
  }
  reason = "OK";
  return true;
}

void MapController::updateReplay() {
  if (!replayActive_) return;
  if (replayOperation_ == MapReplayOperation::NONE) {
    if (mode_ == MapControllerMode::REPLAY_CHECKED) {
      mode_ = MapControllerMode::REPLAY_RUNNING;
    }
    (void)startNextReplaySegment();
  }
}

MapController::Pose MapController::routePointWorld(uint16_t index) const {
  if (index >= route_.header.waypointCount) return replayOrigin_;
  const MapWaypoint& local = route_.waypoints[index];
  const float c = cosf(replayOrigin_.headingDeg * kDegToRad);
  const float s = sinf(replayOrigin_.headingDeg * kDegToRad);
  Pose world;
  world.xMm = replayOrigin_.xMm + static_cast<float>(local.xMm) * c -
              static_cast<float>(local.yMm) * s;
  world.yMm = replayOrigin_.yMm + static_cast<float>(local.xMm) * s +
              static_cast<float>(local.yMm) * c;
  world.headingDeg = normalizeDeg(
      replayOrigin_.headingDeg + static_cast<float>(local.headingCdeg) / 100.0f);
  return world;
}

bool MapController::currentReplayPose(Pose& pose) const { return readPose(pose); }

bool MapController::targetWaypointReached(const Pose& pose,
                                           const Pose& target) const {
  return distanceMm(pose.xMm, pose.yMm, target.xMm, target.yMm) <=
         kWaypointToleranceMm;
}

bool MapController::startNextReplaySegment() {
  if (!replayActive_ || replayOperation_ != MapReplayOperation::NONE) return false;
  if (replayTargetIndex_ >= route_.header.waypointCount) {
    completeReplay();
    return true;
  }
  Pose current;
  if (!currentReplayPose(current)) {
    abortReplay("POSE");
    return false;
  }
  replayTarget_ = routePointWorld(replayTargetIndex_);
  const float targetDistance = distanceMm(current.xMm, current.yMm,
                                          replayTarget_.xMm, replayTarget_.yMm);
  replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(targetDistance));
  replayTargetDeg_ = 0;
  replayTravelMm_ = 0U;
  replayErrorMm_ = replayTargetDistanceMm_;
  if (targetDistance <= kWaypointToleranceMm) {
    advanceReplayAfterTarget();
    return true;
  }
  const float bearing = atan2f(replayTarget_.yMm - current.yMm,
                               replayTarget_.xMm - current.xMm) * kRadToDeg;
  const float turnDelta = shortestDeltaDeg(bearing, current.headingDeg);
  const uint32_t segmentGeneration = nextReplayGeneration();
  if (fabsf(turnDelta) > TURN_TOLERANCE_DEG) {
    replayTargetDeg_ = static_cast<int16_t>(lroundf(fabsf(turnDelta)));
    replayOperation_ = MapReplayOperation::TURN;
    if (!robot_.startReplayTurnRelative(turnDelta > 0.0f, fabsf(turnDelta),
                                        kReplaySpeed, segmentGeneration)) {
      replayOperation_ = MapReplayOperation::NONE;
      abortReplay("TURN_START");
      return false;
    }
  } else {
    replayOperation_ = MapReplayOperation::MOVE;
    if (!robot_.startReplayDistance(true, replayTargetDistanceMm_,
                                    kReplaySpeed, segmentGeneration)) {
      replayOperation_ = MapReplayOperation::NONE;
      abortReplay("MOVE_START");
      return false;
    }
  }
  replaySegmentGeneration_ = segmentGeneration;
  logSegmentStart(segmentGeneration, replayCurrentIndex_, replayTargetIndex_,
                  replayOperation_);
  statusDirty_ = true;
  return true;
}

void MapController::advanceReplayAfterTarget() {
  replayCurrentIndex_ = replayTargetIndex_;
  const uint16_t count = route_.header.waypointCount;
  if (replayDirection_ > 0) {
    if (replayCurrentIndex_ < count - 1U) {
      replayTargetIndex_ = replayCurrentIndex_ + 1U;
      return;
    }
    if (routeType_ == MapRouteType::CLOSED &&
        routeMode_ == MapReplayMode::LOOP &&
        replayCurrentIndex_ == count - 1U) {
      replayTargetIndex_ = 0U;
      return;
    }
    if (routeMode_ == MapReplayMode::ONCE) {
      completeReplay();
      return;
    }
    replayDirection_ = -1;
    replayReturned_ = true;
    replayTargetIndex_ = count - 2U;
    return;
  }

  if (replayCurrentIndex_ > 0U) {
    replayTargetIndex_ = replayCurrentIndex_ - 1U;
    return;
  }
  if (routeMode_ == MapReplayMode::RETURN) {
    completeReplay();
    return;
  }
  replayDirection_ = 1;
  replayReturned_ = false;
  replayTargetIndex_ = 1U;
}

void MapController::enterReplayHold(MapHoldReason reason, bool allowResume) {
  // X-down and safety holds use the existing RobotController stop path. This
  // changes no braking strategy and guarantees owner release before HOLD is
  // exposed to the rest of the loop.
  robot_.stopImmediately(true);
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayActive_ = false;
  replayOperation_ = MapReplayOperation::HOLD;
  replayReason_ = HoldReasonName(reason);
  holdReason_ = reason;
  replayResumeAllowed_ = allowResume;
  replayHoldPoseValid_ = readPose(replayHoldPose_);
  mode_ = MapControllerMode::REPLAY_HOLD;
  statusDirty_ = true;
  ps2_.holdMapInput();
  debug_.print("MAP,HOLD,REASON=");
  debug_.print(HoldReasonName(reason));
  debug_.print(",WP=");
  debug_.print(static_cast<unsigned>(replayTargetIndex_));
  debug_.print(",GEN=");
  debug_.println(replayGeneration_);
}

void MapController::abortReplay(const char* reason) {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  clearReplayResumeContext();
  replayActive_ = false;
  replayReason_ = reason != nullptr ? reason : "ERROR";
  mode_ = MapControllerMode::SAVED;
  statusDirty_ = true;
  debug_.print("MAP,REPLAY=ABORT,REASON=");
  debug_.println(replayReason_);
}

void MapController::completeReplay() {
  nextReplayGeneration();
  clearReplayResumeContext();
  replayActive_ = false;
  replayReason_ = "DONE";
  mode_ = MapControllerMode::REPLAY_COMPLETE;
  statusDirty_ = true;
  debug_.println("MAP,REPLAY_COMPLETE");
}

void MapController::cancelReplay(const char* reason) {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  clearReplayResumeContext();
  replayActive_ = false;
  replayReason_ = reason != nullptr ? reason : "CANCELLED";
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                              : MapControllerMode::READY;
  statusDirty_ = true;
  ps2_.disarmMapInput();
  debug_.print("MAP,CANCEL,REASON=");
  debug_.println(replayReason_);
  debug_.println("MAP,REPLAY_CANCEL");
  beginCancelTrace();
}

void MapController::clearReplayResumeContext() {
  replaySegmentGeneration_ = 0U;
  replayResumeAllowed_ = false;
  replayHoldPoseValid_ = false;
  replayOriginValid_ = false;
  holdReason_ = MapHoldReason::NONE;
  replayOperation_ = MapReplayOperation::NONE;
  replayCurrentIndex_ = 0U;
  replayTargetIndex_ = 1U;
  replayDirection_ = 1;
  replayReturned_ = false;
  replayOrigin_ = {};
  replayHoldPose_ = {};
  replayOriginRouteGeneration_ = 0U;
  replayOriginResetGeneration_ = 0U;
  replayOriginHeadingResetGeneration_ = 0U;
  replayTargetDistanceMm_ = 0U;
  replayTargetDeg_ = 0;
  replayTravelMm_ = 0U;
  replayErrorMm_ = 0U;
}

bool MapController::canResumeReplay(const char*& rejectReason) const {
  rejectReason = nullptr;
  if (!loadedValid_) {
    rejectReason = "NOT_SAVED";
    return false;
  }
  if (!replayResumeAllowed_ || mode_ != MapControllerMode::REPLAY_HOLD) {
    rejectReason = "HOLD_NOT_RESUMABLE";
    return false;
  }
  if (selectedSlot_ != replayContextSlot_) {
    rejectReason = "SLOT_CHANGED";
    return false;
  }
  if (!replayOriginValid_ ||
      !IsReplayModeAllowed(routeType_, routeMode_)) {
    rejectReason = "REPLAY_CONTEXT";
    return false;
  }
  if (replayTargetIndex_ >= route_.header.waypointCount ||
      replayCurrentIndex_ >= route_.header.waypointCount ||
      replayTargetIndex_ == replayCurrentIndex_ ||
      (replayDirection_ != 1 && replayDirection_ != -1)) {
    rejectReason = "TARGET_WAYPOINT";
    return false;
  }
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    rejectReason = "MOTION_OWNER";
    return false;
  }
  if (robot_.brakeEnabled()) {
    rejectReason = "BRAKE";
    return false;
  }
  if (odometry_.resetGeneration() != replayOriginResetGeneration_) {
    rejectReason = "RESET_BOUNDARY";
    return false;
  }
  if (robot_.headingResetGeneration() != replayOriginHeadingResetGeneration_) {
    rejectReason = "HEADING_RESET_BOUNDARY";
    return false;
  }
  if (route_.header.generation != replayOriginRouteGeneration_) {
    rejectReason = "ROUTE_CHANGED";
    return false;
  }
  const uint32_t now = millis();
  if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
      ps2_.motionCommandActive()) {
    rejectReason = "PS2_NOT_NEUTRAL";
    return false;
  }
  if (!odometry_.ready() || !odometry_.healthy()) {
    rejectReason = "ODOMETRY";
    return false;
  }
  if (!fusion_.ready() || fusion_.health() == FusionHealth::NO_SOURCE) {
    rejectReason = "HEADING";
    return false;
  }
  const bool obstacleLiveClear =
      ultrasonic_.isFresh() && ultrasonic_.healthy() &&
      ultrasonic_.overallZone() == ObstacleZone::CLEAR;
  const bool obstacleGraceClear =
      ultrasonic_.overallZone() == ObstacleZone::CLEAR &&
      ultrasonic_.hasRecentClearWindow(now);
  if (!obstacleLiveClear && !obstacleGraceClear) {
    rejectReason = "OBSTACLE_NOT_CLEAR";
    return false;
  }
  Pose pose;
  if (!replayHoldPoseValid_ || !readPose(pose)) {
    rejectReason = "POSE";
    return false;
  }
  if (distanceMm(pose.xMm, pose.yMm, replayHoldPose_.xMm,
                 replayHoldPose_.yMm) > kReplayPoseHoldToleranceMm ||
      fabsf(shortestDeltaDeg(pose.headingDeg, replayHoldPose_.headingDeg)) >
          kReplayPoseHoldToleranceDeg) {
    rejectReason = "POSE_DRIFT";
    return false;
  }
  rejectReason = "OK";
  return true;
}

bool MapController::consumeReplayTurnResult(const AiTurnResult& result) {
  if (result.owner != MotionOwner::REPLAY) return false;
  if (!replayActive_ || replayOperation_ != MapReplayOperation::TURN ||
      result.motionGeneration != replaySegmentGeneration_) {
    if (result.motionGeneration != 0U) {
      debug_.print("MAP,SEGMENT_DROP_STALE,GEN_OLD=");
      debug_.print(result.motionGeneration);
      debug_.print(",GEN_CURRENT=");
      debug_.println(replayGeneration_);
    }
    return false;
  }
  const uint16_t from = replayCurrentIndex_;
  const uint16_t to = replayTargetIndex_;
  const int16_t targetDeg = replayTargetDeg_;
  replayOperation_ = MapReplayOperation::NONE;
  replayErrorMm_ = static_cast<uint32_t>(lroundf(fabsf(result.errorDeg) * 10.0f));
  logSegmentDone(result.motionGeneration, from, to,
                 MapReplayOperation::TURN, 0.0f, 0.0f, targetDeg,
                 result.headingDeg);
  if (result.code == AiTurnResultCode::DONE) {
    return true;
  }
  if (result.code == AiTurnResultCode::OBSTACLE) {
    enterReplayHold(MapHoldReason::OBSTACLE, true);
  } else if (result.code == AiTurnResultCode::CANCELLED &&
             ps2_.motionCommandActive()) {
    cancelReplay("PS2_TAKEOVER");
  } else if (result.code == AiTurnResultCode::CANCELLED) {
    cancelReplay();
  } else {
    abortReplay(result.code == AiTurnResultCode::HEADING_LOST ? "HEADING_LOST"
                                                               : "TURN_ERROR");
  }
  return true;
}

bool MapController::consumeReplayDistanceResult(const AiDistanceResult& result) {
  if (result.owner != MotionOwner::REPLAY) return false;
  if (!replayActive_ || replayOperation_ != MapReplayOperation::MOVE ||
      result.motionGeneration != replaySegmentGeneration_) {
    if (result.motionGeneration != 0U) {
      debug_.print("MAP,SEGMENT_DROP_STALE,GEN_OLD=");
      debug_.print(result.motionGeneration);
      debug_.print(",GEN_CURRENT=");
      debug_.println(replayGeneration_);
    }
    return false;
  }
  const uint16_t from = replayCurrentIndex_;
  const uint16_t to = replayTargetIndex_;
  const uint32_t generation = result.motionGeneration;
  replayOperation_ = MapReplayOperation::NONE;
  replayTravelMm_ = result.travelledMm < 0.0f
                        ? 0U
                        : static_cast<uint32_t>(lroundf(result.travelledMm));
  replayErrorMm_ = replayTargetDistanceMm_ > replayTravelMm_
                       ? replayTargetDistanceMm_ - replayTravelMm_
                       : replayTravelMm_ - replayTargetDistanceMm_;
  logSegmentDone(generation, from, to, MapReplayOperation::MOVE,
                 static_cast<float>(replayTargetDistanceMm_),
                 static_cast<float>(replayTravelMm_), 0, 0.0f);
  if (result.code == AiDistanceResultCode::DONE) {
    advanceReplayAfterTarget();
  } else if (result.code == AiDistanceResultCode::OBSTACLE) {
    enterReplayHold(MapHoldReason::OBSTACLE, true);
  } else if (result.code == AiDistanceResultCode::CANCELLED &&
             ps2_.motionCommandActive()) {
    cancelReplay("PS2_TAKEOVER");
  } else if (result.code == AiDistanceResultCode::CANCELLED) {
    cancelReplay();
  } else {
    abortReplay(result.code == AiDistanceResultCode::ENCODER_FAULT
                    ? "ENCODER_FAULT"
                    : "MOVE_ERROR");
  }
  return true;
}

void MapController::beginCancelTrace() {
  cancelTraceActive_ = true;
  cancelTraceStartMs_ = millis();
  nextCancelTraceMs_ = cancelTraceStartMs_;
  cancelTraceLines_ = 0U;
}

void MapController::serviceCancelTrace() {
  if (!cancelTraceActive_) return;
  const uint32_t now = millis();
  if (cancelTraceLines_ >= 20U ||
      (now - cancelTraceStartMs_) > 2000U) {
    cancelTraceActive_ = false;
    return;
  }
  if (static_cast<int32_t>(now - nextCancelTraceMs_) < 0) return;

  Pose pose;
  if (!readPose(pose)) {
    pose.xMm = odometry_.data().xMm;
    pose.yMm = odometry_.data().yMm;
    pose.headingDeg = fusion_.headingDeg();
  }
  debug_.print("MAP,CANCEL_TRACE,T=");
  debug_.print(now - cancelTraceStartMs_);
  debug_.print(",STATE=");
  debug_.print(static_cast<uint8_t>(mode_));
  debug_.print(",OWNER=");
  debug_.print(robot_.motionOwnerText(robot_.motionOwner()));
  debug_.print(",MOTION_MODE=");
  debug_.print(robot_.aiMotionModeValue());
  debug_.print(",TARGET_L=");
  debug_.print(robot_.targetLeftCommand());
  debug_.print(",TARGET_R=");
  debug_.print(robot_.targetRightCommand());
  debug_.print(",CURRENT_L=");
  debug_.print(robot_.currentLeftCommand());
  debug_.print(",CURRENT_R=");
  debug_.print(robot_.currentRightCommand());
  debug_.print(",LV=");
  debug_.print(odometry_.data().leftVelocityMmS, 1);
  debug_.print(",RV=");
  debug_.print(odometry_.data().rightVelocityMmS, 1);
  debug_.print(",X=");
  debug_.print(pose.xMm, 1);
  debug_.print(",Y=");
  debug_.print(pose.yMm, 1);
  debug_.print(",H=");
  debug_.print(pose.headingDeg, 2);
  debug_.print(",GEN=");
  debug_.println(replayGeneration_);
  ++cancelTraceLines_;
  nextCancelTraceMs_ = now + 100U;
}

void MapController::serviceStorage() {
  if (deletePending_) {
    if (!robot_.motorsStopped() || robot_.aiMotionActive()) return;
    deletePending_ = false;
    if (store_.erase(selectedSlot_)) {
      loadedValid_ = false;
      storeState_ = MapStoreState::EMPTY;
      mode_ = MapControllerMode::READY;
      log("DELETE=OK");
    } else {
      storeState_ = MapStoreState::STORAGE_ERROR;
      mode_ = MapControllerMode::READY;
      log("DELETE=FAIL");
    }
    statusDirty_ = true;
    return;
  }
  if (savePending_) {
    if (!robot_.motorsStopped() || robot_.aiMotionActive()) return;
    savePending_ = false;
    const bool saved = store_.save(selectedSlot_, route_);
    if (saved) {
      loadedValid_ = true;
      storeState_ = MapStoreState::SAVED;
      routeType_ = static_cast<MapRouteType>(route_.header.routeType);
      routeMode_ = static_cast<MapReplayMode>(route_.header.replayMode);
      mode_ = MapControllerMode::SAVED;
      log("TEACH_SAVE=OK");
    } else {
      // The previous active A/B record remains untouched on a failed erase,
      // program or read-back. Drop the in-progress RAM record and expose the
      // failed save explicitly; a later START reloads the old Flash record.
      loadedValid_ = false;
      storeState_ = MapStoreState::STORAGE_ERROR;
      mode_ = MapControllerMode::READY;
      log("TEACH_SAVE=FAIL,OLD_ROUTE_RETAINED=1");
    }
    statusDirty_ = true;
    return;
  }
  if (modeSavePending_) {
    if (!loadedValid_ || !robot_.motorsStopped() || robot_.aiMotionActive()) return;
    modeSavePending_ = false;
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    updateRouteHeaderForSave(route_);
    if (!store_.save(selectedSlot_, route_)) {
      routeMode_ = modeBeforeSave_;
      route_.header.replayMode = static_cast<uint8_t>(routeMode_);
      storeState_ = MapStoreState::STORAGE_ERROR;
      log("REPLAY_MODE_SAVE=FAIL");
    } else {
      storeState_ = MapStoreState::SAVED;
      log("REPLAY_MODE_SAVE=OK");
    }
    statusDirty_ = true;
  }
}

void MapController::publishStatus() {
  MapSlotMetadata metadata = store_.metadata(selectedSlot_);
  if (storeState_ == MapStoreState::STORAGE_ERROR) {
    metadata.state = MapStoreState::STORAGE_ERROR;
  } else if (loadedValid_ || mode_ == MapControllerMode::TEACHING) {
    metadata.state = mode_ == MapControllerMode::TEACHING
                         ? MapStoreState::EMPTY
                         : MapStoreState::SAVED;
    metadata.routeType = routeType_;
    metadata.replayMode = routeMode_;
    metadata.waypointCount = route_.header.waypointCount;
    metadata.routeLengthMm = route_.header.routeLengthMm;
  } else {
    metadata.state = storeState_;
    metadata.routeType = routeType_;
    metadata.replayMode = routeMode_;
  }
  const uint16_t points = metadata.waypointCount;
  const uint16_t replayWp = replayTargetIndex_ + 1U;
  display_.setMapStatus(
      static_cast<uint8_t>(selectedSlot_), static_cast<uint8_t>(metadata.state),
      static_cast<uint8_t>(mode_), points, STM32_MAP_MAX_WAYPOINTS,
      metadata.routeLengthMm, replayWp,
      loadedValid_ ? route_.header.waypointCount : 0U,
      replayTargetDistanceMm_, replayTravelMm_, replayErrorMm_,
      static_cast<uint8_t>(replayOperation_),
      static_cast<uint8_t>(metadata.routeType),
      static_cast<uint8_t>(metadata.replayMode),
      static_cast<uint8_t>(holdReason_), replayTargetDeg_);
  lastStatusMs_ = millis();
  statusDirty_ = false;
}

void MapController::log(const char* message) const {
  debug_.print("MAP,");
  debug_.println(message != nullptr ? message : "");
}

void MapController::logStartReject(const char* reason) const {
  debug_.print("MAP,START,REJECT,REASON=");
  debug_.println(reason != nullptr ? reason : "UNKNOWN");
}

void MapController::logSegmentStart(uint32_t generation, uint16_t from,
                                     uint16_t to,
                                     MapReplayOperation operation) const {
  debug_.print("MAP,SEGMENT_START,GEN=");
  debug_.print(generation);
  debug_.print(",WP_FROM=");
  debug_.print(static_cast<unsigned>(from));
  debug_.print(",WP_TO=");
  debug_.print(static_cast<unsigned>(to));
  debug_.print(",OP=");
  debug_.println(operation == MapReplayOperation::TURN ? "TURN" : "MOVE");
}

void MapController::logSegmentDone(uint32_t generation, uint16_t from,
                                    uint16_t to,
                                    MapReplayOperation operation,
                                    float targetMm, float travelledMm,
                                    int16_t targetDeg,
                                    float headingDeg) const {
  debug_.print("MAP,SEGMENT_DONE,GEN=");
  debug_.print(generation);
  debug_.print(",WP_FROM=");
  debug_.print(static_cast<unsigned>(from));
  debug_.print(",WP_TO=");
  debug_.print(static_cast<unsigned>(to));
  debug_.print(",OP=");
  debug_.print(operation == MapReplayOperation::TURN ? "TURN" : "MOVE");
  debug_.print(",TARGET_MM=");
  debug_.print(targetMm, 1);
  debug_.print(",TRAVEL_MM=");
  debug_.print(travelledMm, 1);
  debug_.print(",TARGET_DEG=");
  debug_.print(targetDeg);
  debug_.print(",HEADING=");
  debug_.println(headingDeg, 2);
}

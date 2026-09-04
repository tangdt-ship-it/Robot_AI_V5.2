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
constexpr float kClosedAutoDistanceMm = 50.0f;
constexpr float kClosedAutoHeadingDeg = 5.0f;
constexpr float kClosedCandidateDistanceMm = 200.0f;
constexpr float kClosedCandidateHeadingDeg = 20.0f;
constexpr float kClosedClosureSkipDistanceMm = kDuplicateDistanceMm;
constexpr float kWaypointToleranceMm =
    static_cast<float>(MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM);
constexpr float kReplayPoseHoldToleranceMm = 100.0f;
constexpr float kReplayPoseHoldToleranceDeg = 15.0f;
constexpr float kMinimumSegmentMm = 20.0f;
constexpr float kMaximumSegmentMm = 5000.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr int16_t kReplaySpeed = 20;
// Kept as a named compatibility alias for the first-segment policy. The MAP
// pre-turn tolerance is now the single source of truth for replay guidance.
constexpr float kReplayStartupTurnDeadbandDeg =
    MAP_REPLAY_PRETURN_TOLERANCE_DEG;

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
      mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::CLOSED_CONFIRM) {
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
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::CLOSED_CONFIRM) {
    logStartReject(mode_ == MapControllerMode::TEACHING
                       ? "TEACHING"
                       : mode_ == MapControllerMode::DELETE_CONFIRM
                           ? "DELETE_CONFIRM"
                           : "CLOSED_CONFIRM");
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
      if (routeType_ == MapRouteType::CLOSED &&
          routeMode_ == MapReplayMode::LOOP) {
        debug_.print("MAP,LOOP,RESUME,LAP=");
        debug_.print(replayLapCounter_);
        debug_.print(",WP=");
        debug_.println(static_cast<unsigned>(replayTargetIndex_));
      }
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
    if (routeType_ == MapRouteType::CLOSED &&
        routeMode_ == MapReplayMode::LOOP) {
      debug_.println("MAP,LOOP,START");
    }
  } else {
    logStartReject(reason != nullptr ? reason : "PRECHECK");
  }
}

void MapController::handleTriangle() {
  if (replayActive_ || mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::CLOSED_CONFIRM) {
    return;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    markManualWaypoint();
    return;
  }
  (void)beginTeach();
}

void MapController::handleCircle() {
  debug_.print("MAP,CIRCLE,HANDLE,MODE=");
  debug_.println(static_cast<unsigned>(mode_));
  if (mode_ == MapControllerMode::TEACHING) {
    debug_.println("MAP,CIRCLE,ACTION=FINISH_TEACH");
    requestTeachFinish();
    return;
  }
  if (mode_ == MapControllerMode::CLOSED_CONFIRM) {
    debug_.println("MAP,CIRCLE,ACTION=CONFIRM_CLOSED");
    const uint32_t closeDistance = closeCandidateDistanceMm_;
    debug_.print("MAP,TYPE_SELECT,TYPE=CLOSED,CLOSE_DIST=");
    debug_.println(closeDistance);
    if (queueTeachSave(MapRouteType::CLOSED,
                       MapControllerMode::CLOSED_CONFIRM)) {
      debug_.println("MAP,CLOSE_CONFIRM,RESULT=CLOSED");
    }
    return;
  }
  if (mode_ == MapControllerMode::DELETE_CONFIRM) {
    debug_.println("MAP,CIRCLE,ACTION=DELETE_CONFIRM");
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
  debug_.println("MAP,CIRCLE,ACTION=CYCLE_MODE");
  modeBeforeSave_ = previous;
  modeSavePending_ = true;
  statusDirty_ = true;
}

void MapController::handleSquare(bool longPress) {
  if (longPress) {
    if (replayActive_ || mode_ == MapControllerMode::TEACHING ||
        mode_ == MapControllerMode::DELETE_CONFIRM ||
        mode_ == MapControllerMode::CLOSED_CONFIRM ||
        !robot_.motorsStopped() || robot_.aiMotionActive()) {
      return;
    }
    mode_ = MapControllerMode::DELETE_CONFIRM;
    return;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    if (teachMode_ != MapTeachMode::MANUAL_KEYFRAME) return;
    const uint16_t count = route_.header.waypointCount;
    if (count > 1U) {
      --route_.header.waypointCount;
      route_.header.routeLengthMm = routeLengthMm(route_);
      resetTeachTracking();
      debug_.print("MAP,KEYFRAME,UNDO,IDX=");
      debug_.println(static_cast<unsigned>(count - 1U));
    } else {
      debug_.println("MAP,KEYFRAME,REJECT,REASON=START_PROTECTED");
    }
  }
}

void MapController::handleCross() {
  if (mode_ == MapControllerMode::TEACHING) {
    cancelTeach();
  } else if (mode_ == MapControllerMode::CLOSED_CONFIRM) {
    debug_.println("MAP,TYPE_SELECT,TYPE=OPEN");
    if (queueTeachSave(MapRouteType::OPEN,
                       MapControllerMode::CLOSED_CONFIRM)) {
      debug_.println("MAP,CLOSE_CONFIRM,RESULT=OPEN");
    }
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
  if (mode_ == MapControllerMode::CLOSED_CONFIRM) {
    debug_.println("MAP,TYPE_SELECT,TYPE=OPEN");
    if (queueTeachSave(MapRouteType::OPEN,
                       MapControllerMode::CLOSED_CONFIRM)) {
      debug_.println("MAP,CLOSE_CONFIRM,RESULT=OPEN");
    }
    return;
  }
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
  teachMode_ = MapTeachMode::MANUAL_KEYFRAME;
  route_ = {};
  route_.header.waypointCount = 0U;
  routeType_ = MapRouteType::OPEN;
  routeMode_ = MapReplayMode::ONCE;
  closeCandidateDistanceMm_ = 0U;
  closeCandidateHeadingDeg_ = 0;
  const Pose localOrigin{};
  if (!appendWaypoint(localOrigin, MAP_WP_START)) return false;
  debug_.println("MAP,TEACH_MODE=MANUAL_KEYFRAME");
  debug_.print("MAP,KEYFRAME,START,IDX=0,X=");
  debug_.print(localOrigin.xMm, 1);
  debug_.print(",Y=");
  debug_.print(localOrigin.yMm, 1);
  debug_.print(",H=");
  debug_.println(localOrigin.headingDeg, 1);
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
  if (teachMode_ != MapTeachMode::MANUAL_KEYFRAME) return;
  const Pose local = toTeachLocal(pose);
  const uint16_t count = route_.header.waypointCount;
  if (count == 0U) return;
  const MapWaypoint& previous = route_.waypoints[count - 1U];
  const float separation = distanceMm(
      local.xMm, local.yMm, static_cast<float>(previous.xMm),
      static_cast<float>(previous.yMm));
  if (separation <= kMinimumSegmentMm) {
    debug_.print("MAP,KEYFRAME,REJECT,REASON=TOO_CLOSE,IDX=");
    debug_.println(static_cast<unsigned>(count));
    return;
  }
  if (!appendWaypoint(local, MAP_WP_MANUAL_MARK)) {
    debug_.print("MAP,KEYFRAME,REJECT,REASON=FULL,IDX=");
    debug_.println(static_cast<unsigned>(count));
    return;
  }
  const uint16_t index = route_.header.waypointCount - 1U;
  debug_.print("MAP,KEYFRAME,MARK,IDX=");
  debug_.print(static_cast<unsigned>(index));
  debug_.print(",X=");
  debug_.print(local.xMm, 1);
  debug_.print(",Y=");
  debug_.print(local.yMm, 1);
  debug_.print(",H=");
  debug_.println(local.headingDeg, 1);
}

void MapController::sampleTeach() {
  Pose pose;
  if (!teachOriginValid_ || !readPose(pose)) return;
  const Pose local = toTeachLocal(pose);
  if (teachMode_ == MapTeachMode::MANUAL_KEYFRAME) {
    // Manual Teach observes pose only for diagnostics. A timer sample must
    // never become an AUTO_DISTANCE/AUTO_CORNER waypoint.
    lastTeachSample_ = local;
    lastTeachSampleValid_ = true;
    cornerStableSamples_ = 0U;
    return;
  }
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
    if (separation <= kMinimumSegmentMm) {
      // A heading-only waypoint would become a zero-length MOVE during
      // Replay. Endpoint flags are merged into the existing point; all
      // other close points are rejected.
      if ((flags & MAP_WP_ENDPOINT) != 0U) {
        previous.flags |= flags;
        route_.header.routeLengthMm = routeLengthMm(route_);
        statusDirty_ = true;
        return true;
      }
      if ((flags & MAP_WP_MANUAL_MARK) != 0U) return false;
      if (headingDelta > kDuplicateHeadingDeg) return false;
    }
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
  const Pose localEndpoint = toTeachLocal(pose);
  uint16_t count = route_.header.waypointCount;
  if (count == 0U) return false;
  bool mergedEndpoint = false;
  MapWaypoint& last = route_.waypoints[count - 1U];
  const float endpointSeparation = distanceMm(
      localEndpoint.xMm, localEndpoint.yMm, static_cast<float>(last.xMm),
      static_cast<float>(last.yMm));
  if (endpointSeparation <= kMinimumSegmentMm) {
    // Do not manufacture a short/zero MOVE at the end. Keep a manual anchor
    // authoritative and mark that same point as the route endpoint.
    last.flags |= MAP_WP_ENDPOINT;
    mergedEndpoint = true;
    route_.header.routeLengthMm = routeLengthMm(route_);
    statusDirty_ = true;
  } else if (!appendWaypoint(localEndpoint, MAP_WP_ENDPOINT)) {
    return false;
  }
  count = route_.header.waypointCount;
  const MapWaypoint& endpoint = route_.waypoints[count - 1U];
  debug_.print("MAP,KEYFRAME,ENDPOINT,IDX=");
  debug_.print(static_cast<unsigned>(count - 1U));
  debug_.print(",X=");
  debug_.print(static_cast<float>(endpoint.xMm), 1);
  debug_.print(",Y=");
  debug_.print(static_cast<float>(endpoint.yMm), 1);
  debug_.print(",H=");
  debug_.print(static_cast<float>(endpoint.headingCdeg) / 100.0f, 1);
  debug_.print(",ACTION=");
  debug_.println(mergedEndpoint ? "MERGE" : "APPEND");
  float closeDistance = 0.0f;
  float closeHeading = 0.0f;
  if (count >= 2U) {
    closeDistance = distanceMm(
        static_cast<float>(route_.waypoints[0].xMm),
        static_cast<float>(route_.waypoints[0].yMm),
        static_cast<float>(route_.waypoints[count - 1U].xMm),
        static_cast<float>(route_.waypoints[count - 1U].yMm));
    closeHeading = fabsf(shortestDeltaDeg(
        static_cast<float>(route_.waypoints[count - 1U].headingCdeg) /
            100.0f,
        static_cast<float>(route_.waypoints[0].headingCdeg) / 100.0f));
  }
  debug_.print("MAP,TEACH,CLOSE_CHECK,D=");
  debug_.print(closeDistance, 1);
  debug_.print(",H=");
  debug_.print(closeHeading, 1);

  if (teachMode_ == MapTeachMode::MANUAL_KEYFRAME) {
    // Manual keyframes are authoritative. Do not infer OPEN/CLOSED from the
    // endpoint pose: the operator explicitly chooses the route type after
    // Teach, including routes whose final point is far from P0.
    optimizedRoute_ = route_;
    semanticRoute_ = route_;
    debug_.println();
    debug_.println("MAP,TEACH_MODE=MANUAL_KEYFRAME");
    debug_.println("MAP,SEMANTIC=BYPASS_MANUAL_KEYFRAME");
    uint16_t manualCount = 0U;
    for (uint16_t index = 0U; index < count; ++index) {
      if ((route_.waypoints[index].flags & MAP_WP_MANUAL_MARK) != 0U) {
        ++manualCount;
      }
    }
    debug_.print("MAP,TEACH,SUMMARY,POINTS=");
    debug_.print(static_cast<unsigned>(count));
    debug_.print(",MANUAL=");
    debug_.print(static_cast<unsigned>(manualCount));
    debug_.print(",LENGTH=");
    debug_.print(routeLengthMm(route_));
    debug_.println(",TYPE=USER_CONFIRM");
    closeCandidateDistanceMm_ = static_cast<uint32_t>(lroundf(closeDistance));
    closeCandidateHeadingDeg_ = static_cast<int16_t>(lroundf(closeHeading));
    routeType_ = MapRouteType::OPEN;
    routeMode_ = MapReplayMode::ONCE;
    route_.header.routeType = static_cast<uint8_t>(routeType_);
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    updateRouteHeaderForSave(route_);
    storeState_ = MapStoreState::EMPTY;
    loadedValid_ = false;
    teachOriginValid_ = false;
    teachFinishPending_ = false;
    mode_ = MapControllerMode::CLOSED_CONFIRM;
    debug_.print("MAP,TYPE_CONFIRM,POINTS=");
    debug_.print(static_cast<unsigned>(count));
    debug_.print(",CLOSE_DIST=");
    debug_.print(closeDistance, 1);
    debug_.print(",CLOSE_HEADING=");
    debug_.println(closeHeading, 1);
    statusDirty_ = true;
    return true;
  } else {
    const bool enoughPoints = count >= 3U;
    const bool autoClosed = enoughPoints &&
                            closeDistance <= kClosedAutoDistanceMm &&
                            closeHeading <= kClosedAutoHeadingDeg;
    const bool closedCandidate = enoughPoints && !autoClosed &&
                                 closeDistance <= kClosedCandidateDistanceMm &&
                                 closeHeading <= kClosedCandidateHeadingDeg;
    if (autoClosed) {
      debug_.println(",CLASS=AUTO_CLOSED");
    } else if (closedCandidate) {
      debug_.println(",CLASS=CANDIDATE");
    } else {
      debug_.println(",CLASS=OPEN");
    }
    const MapRouteType detectedType = (autoClosed || closedCandidate)
                                          ? MapRouteType::CLOSED
                                          : MapRouteType::OPEN;
    route_.header.routeType = static_cast<uint8_t>(detectedType);
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    RouteCleanerMetrics optimizeMetrics;
    const bool cleanAccepted =
        cleanMapRoute(route_, optimizedRoute_, optimizeMetrics);
    route_ = optimizedRoute_;
    logOptimizeSummary(optimizeMetrics);
    SemanticRouteMetrics semanticMetrics;
    semanticMetrics.rawPoints = optimizeMetrics.rawPoints;
    if (cleanAccepted) {
      (void)optimizeSemanticRoute(optimizedRoute_, semanticRoute_,
                                  semanticMetrics);
      semanticMetrics.rawPoints = optimizeMetrics.rawPoints;
    } else {
      semanticRoute_ = optimizedRoute_;
      semanticMetrics.cleanPoints = optimizedRoute_.header.waypointCount;
      semanticMetrics.semanticPoints = optimizedRoute_.header.waypointCount;
      semanticMetrics.cleanLengthMm = optimizedRoute_.header.routeLengthMm;
      semanticMetrics.semanticLengthMm = optimizedRoute_.header.routeLengthMm;
      semanticMetrics.fallbackReason = "CLEAN_FALLBACK";
    }
    route_ = semanticRoute_;
    logSemanticSummary(semanticMetrics);
    if (autoClosed) {
      return queueTeachSave(MapRouteType::CLOSED,
                            MapControllerMode::TEACHING);
    }
    if (closedCandidate) {
      closeCandidateDistanceMm_ = static_cast<uint32_t>(lroundf(closeDistance));
      closeCandidateHeadingDeg_ = static_cast<int16_t>(lroundf(closeHeading));
      routeType_ = MapRouteType::CLOSED;
      routeMode_ = MapReplayMode::ONCE;
      route_.header.routeType = static_cast<uint8_t>(routeType_);
      route_.header.replayMode = static_cast<uint8_t>(routeMode_);
      updateRouteHeaderForSave(route_);
      storeState_ = MapStoreState::EMPTY;
      loadedValid_ = false;
      teachOriginValid_ = false;
      teachFinishPending_ = false;
      mode_ = MapControllerMode::CLOSED_CONFIRM;
      statusDirty_ = true;
      return true;
    }
    return queueTeachSave(MapRouteType::OPEN, MapControllerMode::TEACHING);
  }
}

bool MapController::queueTeachSave(MapRouteType type,
                                   MapControllerMode failureMode) {
  const MapRouteType previousType = routeType_;
  const MapReplayMode previousMode = routeMode_;
  routeType_ = type;
  if (!IsReplayModeAllowed(routeType_, routeMode_)) {
    routeMode_ = MapReplayMode::ONCE;
  }
  route_.header.routeType = static_cast<uint8_t>(routeType_);
  route_.header.replayMode = static_cast<uint8_t>(routeMode_);
  updateRouteHeaderForSave(route_);
  const char* reason = nullptr;
  if (!validateRoute(route_, reason)) {
    routeType_ = previousType;
    routeMode_ = previousMode;
    route_.header.routeType = static_cast<uint8_t>(routeType_);
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    updateRouteHeaderForSave(route_);
    debug_.print("MAP,TEACH=REJECT,REASON=");
    debug_.println(reason != nullptr ? reason : "INVALID");
    if (type == MapRouteType::CLOSED) {
      debug_.print("MAP,CLOSE,REJECT,REASON=");
      debug_.println(reason != nullptr ? reason : "INVALID");
    }
    mode_ = failureMode;
    statusDirty_ = true;
    return false;
  }
  savePending_ = true;
  mode_ = MapControllerMode::READY;
  teachOriginValid_ = false;
  teachFinishPending_ = false;
  closeCandidateDistanceMm_ = 0U;
  closeCandidateHeadingDeg_ = 0;
  statusDirty_ = true;
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
    const float closure = distanceMm(
        static_cast<float>(route.waypoints[count - 1U].xMm),
        static_cast<float>(route.waypoints[count - 1U].yMm),
        static_cast<float>(route.waypoints[0].xMm),
        static_cast<float>(route.waypoints[0].yMm));
    if (closure > kClosedClosureSkipDistanceMm) total += closure;
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
    // A near-zero endpoint is a closure marker, not a travelled segment. Do
    // not manufacture an invalid 0-20 mm closing MOVE for an endpoint that is
    // already effectively at START. Adjacent non-closing segments remain
    // subject to the normal minimum/maximum checks above.
    if (closure > kClosedClosureSkipDistanceMm &&
        (closure < kMinimumSegmentMm || closure > kMaximumSegmentMm)) {
      reason = closure > kMaximumSegmentMm ? "CLOSURE_TOO_LONG"
                                           : "CLOSURE_TOO_SHORT";
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
  replayRealignReason_ = ReplayRealignReason::NONE;
  replayContextSlot_ = selectedSlot_;
  replayOriginResetGeneration_ = odometry_.resetGeneration();
  replayOriginHeadingResetGeneration_ = robot_.headingResetGeneration();
  replayCurrentIndex_ = 0U;
  replayTargetIndex_ = 1U;
  replayDirection_ = 1;
  replayReturned_ = false;
  replayLapCounter_ = 0U;
  replayOriginRouteGeneration_ = route_.header.generation;
  replayTargetDistanceMm_ = 0U;
  replayTargetDeg_ = 0;
  replayGuideBearingDeg_ = 0.0f;
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

float MapController::replayIncomingBearing(uint16_t fromIndex,
                                            uint16_t toIndex) const {
  const Pose segmentStart = routePointWorld(fromIndex);
  const Pose target = routePointWorld(toIndex);
  return atan2f(target.yMm - segmentStart.yMm,
                target.xMm - segmentStart.xMm) * kRadToDeg;
}

bool MapController::startNextReplaySegment() {
  if (!replayActive_ || replayOperation_ != MapReplayOperation::NONE) return false;
  if (replayTargetIndex_ >= route_.header.waypointCount) {
    completeReplay();
    return true;
  }
  const uint16_t count = route_.header.waypointCount;
  const bool logicalClosingEdge =
      routeType_ == MapRouteType::CLOSED && replayDirection_ > 0 &&
      replayCurrentIndex_ == count - 1U && replayTargetIndex_ == 0U;
  if (logicalClosingEdge) {
    // A closure that is already effectively at P0 is a logical edge, not a
    // zero-length Guided MOVE. Avoid atan2(0,0), a meaningless turn and any
    // motor command; advancing here completes ONCE or the current LOOP lap.
    const Pose closingStart = routePointWorld(count - 1U);
    const Pose closingTarget = routePointWorld(0U);
    const float closureDistance = distanceMm(
        closingStart.xMm, closingStart.yMm, closingTarget.xMm,
        closingTarget.yMm);
    if (closureDistance <= kClosedClosureSkipDistanceMm) {
      replayTarget_ = closingTarget;
      replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(closureDistance));
      replayTravelMm_ = 0U;
      replayErrorMm_ = replayTargetDistanceMm_;
      replayRealignReason_ = ReplayRealignReason::NONE;
      debug_.print("MAP,CLOSE_EDGE,SKIP,DIST=");
      debug_.println(closureDistance, 1);
      advanceReplayAfterTarget();
      return true;
    }
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
  const float targetBearing = atan2f(replayTarget_.yMm - current.yMm,
                                     replayTarget_.xMm - current.xMm) *
                              kRadToDeg;
  const float targetHeadingError =
      shortestDeltaDeg(targetBearing, current.headingDeg);

  // Arrival heading is the direction of the active route edge, not the
  // direction from a laterally displaced chassis position into the waypoint.
  // This keeps arbitrary-angle routes correct without using stored Teach
  // heading or snapping to orthogonal directions.
  const float incomingBearing =
      replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
  if (logicalClosingEdge) {
    debug_.print("MAP,CLOSE_EDGE,PLAN,FROM=");
    debug_.print(static_cast<unsigned>(replayCurrentIndex_));
    debug_.print(",TO=0,DIST=");
    debug_.print(targetDistance, 1);
    debug_.print(",BEARING=");
    debug_.println(incomingBearing, 2);
  }
  const float arrivalHeadingError =
      shortestDeltaDeg(incomingBearing, current.headingDeg);
  const bool inArrivalZone = targetDistance <= kWaypointToleranceMm;

  ReplayRealignReason actionReason = replayRealignReason_;
  float desiredBearing = targetBearing;
  float desiredHeadingError = targetHeadingError;
  bool coarsePreturn = false;
  bool realignAction = actionReason != ReplayRealignReason::NONE;

  if (actionReason == ReplayRealignReason::ARRIVAL) {
    realignAction = true;
    desiredBearing = incomingBearing;
    desiredHeadingError = arrivalHeadingError;
    if (!inArrivalZone) {
      // The chassis may move slightly while turning. Once it leaves the
      // arrival zone, resume guidance to the same target from the live pose.
      replayRealignReason_ = ReplayRealignReason::NONE;
      logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                          incomingBearing, current.headingDeg,
                          arrivalHeadingError, "GUIDED");
      desiredBearing = targetBearing;
      desiredHeadingError = targetHeadingError;
    } else if (fabsf(arrivalHeadingError) <=
               MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG) {
      replayRealignReason_ = ReplayRealignReason::NONE;
      logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                          incomingBearing, current.headingDeg,
                          arrivalHeadingError, "ADVANCE");
      advanceReplayAfterTarget();
      return true;
    } else {
      // ARRIVAL realign always turns toward the incoming route edge.
      coarsePreturn = true;
    }
  } else if (actionReason == ReplayRealignReason::PATH) {
    realignAction = true;
    if (inArrivalZone) {
      // A PATH turn can end inside the arrival zone. Re-evaluate the arrival
      // heading gate before deciding whether this waypoint is complete.
      desiredBearing = incomingBearing;
      desiredHeadingError = arrivalHeadingError;
      if (fabsf(arrivalHeadingError) <=
          MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG) {
        replayRealignReason_ = ReplayRealignReason::NONE;
        logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                            incomingBearing, current.headingDeg,
                            arrivalHeadingError, "ADVANCE");
        advanceReplayAfterTarget();
        return true;
      }
      actionReason = ReplayRealignReason::ARRIVAL;
      replayRealignReason_ = actionReason;
      coarsePreturn = true;
    } else if (fabsf(targetHeadingError) >
               MAP_REPLAY_PRETURN_TOLERANCE_DEG) {
      // PATH realign uses the live current-pose-to-target bearing.
      coarsePreturn = true;
    } else {
      replayRealignReason_ = ReplayRealignReason::NONE;
      logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                          targetBearing, current.headingDeg,
                          targetHeadingError, "GUIDED");
    }
  } else if (inArrivalZone) {
    desiredBearing = incomingBearing;
    desiredHeadingError = arrivalHeadingError;
    if (fabsf(arrivalHeadingError) <=
        MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG) {
      advanceReplayAfterTarget();
      return true;
    }
    actionReason = ReplayRealignReason::ARRIVAL;
    replayRealignReason_ = actionReason;
    realignAction = true;
    coarsePreturn = true;
  } else {
    const bool startupNoiseTurn = replayCurrentIndex_ == 0U &&
                                  replayTargetIndex_ == 1U &&
                                  fabsf(targetHeadingError) <=
                                      kReplayStartupTurnDeadbandDeg;
    coarsePreturn = fabsf(targetHeadingError) >
                        MAP_REPLAY_PRETURN_TOLERANCE_DEG &&
                    !startupNoiseTurn;
  }

  if (realignAction && coarsePreturn) {
    logGuideRealign(actionReason, replayTargetIndex_, targetDistance,
                    desiredBearing, current.headingDeg, desiredHeadingError);
  }
  replayGuideBearingDeg_ = desiredBearing;
  logGuidePlan(replayCurrentIndex_, replayTargetIndex_, targetDistance,
               desiredBearing, current.headingDeg, desiredHeadingError);
  const uint32_t segmentGeneration = nextReplayGeneration();
  if (coarsePreturn) {
    replayTargetDeg_ = static_cast<int16_t>(lroundf(fabsf(desiredHeadingError)));
    replayOperation_ = MapReplayOperation::TURN;
    logGuidePreturn(desiredHeadingError, realignAction ? "REALIGN" : "TURN");
    if (!robot_.startReplayTurnRelative(
            desiredHeadingError > 0.0f, fabsf(desiredHeadingError),
            kReplaySpeed, segmentGeneration, AiTurnProfile::MAP_COARSE)) {
      replayOperation_ = MapReplayOperation::NONE;
      abortReplay("TURN_START");
      return false;
    }
  } else {
    replayOperation_ = MapReplayOperation::MOVE;
    replayTargetDeg_ = static_cast<int16_t>(lroundf(desiredBearing));
    logGuidePreturn(desiredHeadingError, "SKIP");
    if (!robot_.startReplayGuidedWaypoint(
            replayTarget_.xMm, replayTarget_.yMm, current.xMm, current.yMm,
            kReplaySpeed, incomingBearing, segmentGeneration)) {
      replayOperation_ = MapReplayOperation::NONE;
      abortReplay("GUIDE_START");
      return false;
    }
  }
  replaySegmentGeneration_ = segmentGeneration;
  logSegmentStart(segmentGeneration, replayCurrentIndex_, replayTargetIndex_,
                  replayOperation_);
  if (replayOperation_ == MapReplayOperation::MOVE) {
    logGuideStart(replayTargetIndex_, targetDistance, desiredBearing);
  }
  statusDirty_ = true;
  return true;
}

void MapController::advanceReplayAfterTarget() {
  const uint16_t fromIndex = replayCurrentIndex_;
  const uint16_t targetIndex = replayTargetIndex_;
  const uint16_t count = route_.header.waypointCount;
  const bool logicalClosingEdge =
      routeType_ == MapRouteType::CLOSED && replayDirection_ > 0 &&
      fromIndex == count - 1U && targetIndex == 0U;
  replayCurrentIndex_ = targetIndex;
  if (logicalClosingEdge) {
    debug_.println("MAP,CLOSE_EDGE,DONE");
  }
  if (replayDirection_ > 0) {
    if (routeType_ == MapRouteType::CLOSED &&
        fromIndex == count - 1U && targetIndex == 0U) {
      // The segment from the final waypoint back to P0 is a real closing
      // edge for CLOSED ONCE and LOOP. A LOOP lap completes only after this
      // edge reaches P0; P0 is never treated as a terminal completion.
      if (routeMode_ == MapReplayMode::LOOP) {
        if (replayLapCounter_ != 0xFFFFFFFFUL) ++replayLapCounter_;
        debug_.print("MAP,LOOP,LAP_COMPLETE,LAP=");
        debug_.println(replayLapCounter_);
        replayTargetIndex_ = count > 1U ? 1U : 0U;
        return;
      }
      if (routeMode_ == MapReplayMode::ONCE) {
        completeReplay();
        return;
      }
    }
    if (replayCurrentIndex_ < count - 1U) {
      replayTargetIndex_ = replayCurrentIndex_ + 1U;
      return;
    }
    if (routeType_ == MapRouteType::CLOSED &&
        (routeMode_ == MapReplayMode::ONCE ||
         routeMode_ == MapReplayMode::LOOP) &&
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
  if (routeType_ == MapRouteType::CLOSED &&
      routeMode_ == MapReplayMode::LOOP) {
    debug_.print("MAP,LOOP,HOLD,LAP=");
    debug_.print(replayLapCounter_);
    debug_.print(",WP=");
    debug_.println(static_cast<unsigned>(replayTargetIndex_));
  }
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
  const bool wasClosedLoop = routeType_ == MapRouteType::CLOSED &&
                             routeMode_ == MapReplayMode::LOOP;
  const uint32_t cancelledLap = replayLapCounter_;
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
  if (wasClosedLoop) {
    debug_.print("MAP,LOOP,CANCEL,LAP=");
    debug_.println(cancelledLap);
  }
  debug_.println("MAP,REPLAY_CANCEL");
  beginCancelTrace();
}

void MapController::clearReplayResumeContext() {
  replaySegmentGeneration_ = 0U;
  replayResumeAllowed_ = false;
  replayHoldPoseValid_ = false;
  replayOriginValid_ = false;
  replayRealignReason_ = ReplayRealignReason::NONE;
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
  replayGuideBearingDeg_ = 0.0f;
  replayTravelMm_ = 0U;
  replayErrorMm_ = 0U;
  replayLapCounter_ = 0U;
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
    if (replayRealignReason_ != ReplayRealignReason::NONE) {
      debug_.print("MAP,GUIDE,REALIGN,TURN_DONE,TYPE=");
      debug_.println(realignReasonName(replayRealignReason_));
    }
    logGuidePreturn(result.errorDeg, "DONE");
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
  if (result.code == AiDistanceResultCode::REALIGN_REQUIRED) {
    Pose current;
    if (!currentReplayPose(current)) {
      abortReplay("POSE");
      return true;
    }
    const float targetDistance =
        distanceMm(current.xMm, current.yMm, replayTarget_.xMm,
                   replayTarget_.yMm);
    const float targetBearing =
        atan2f(replayTarget_.yMm - current.yMm,
               replayTarget_.xMm - current.xMm) * kRadToDeg;
    const float incomingBearing =
        replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
    const bool inArrivalZone = targetDistance <= kWaypointToleranceMm;
    replayRealignReason_ = inArrivalZone ? ReplayRealignReason::ARRIVAL
                                         : ReplayRealignReason::PATH;
    const float desiredBearing = inArrivalZone ? incomingBearing : targetBearing;
    const float headingError = shortestDeltaDeg(desiredBearing,
                                                current.headingDeg);
    replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(targetDistance));
    replayErrorMm_ = replayTargetDistanceMm_;
    logGuideRealign(replayRealignReason_, to, targetDistance, desiredBearing,
                    current.headingDeg, headingError);
  } else if (result.code == AiDistanceResultCode::DONE) {
    Pose current;
    if (!currentReplayPose(current)) {
      abortReplay("POSE");
      return true;
    }
    const float positionError =
        distanceMm(current.xMm, current.yMm, replayTarget_.xMm,
                   replayTarget_.yMm);
    const float targetBearing =
        atan2f(replayTarget_.yMm - current.yMm,
               replayTarget_.xMm - current.xMm) * kRadToDeg;
    const float arrivalBearing =
        replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
    const float arrivalHeadingError =
        shortestDeltaDeg(arrivalBearing, current.headingDeg);
    replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(positionError));
    replayErrorMm_ = replayTargetDistanceMm_;
    if (positionError <= kWaypointToleranceMm &&
        fabsf(arrivalHeadingError) <=
            MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG) {
      logSegmentDone(generation, from, to, MapReplayOperation::MOVE,
                     static_cast<float>(replayTargetDistanceMm_),
                     static_cast<float>(replayTravelMm_), 0, 0.0f);
      logGuideDone(to, positionError, arrivalBearing, current.headingDeg,
                   arrivalHeadingError);
      replayRealignReason_ = ReplayRealignReason::NONE;
      advanceReplayAfterTarget();
    } else if (positionError <= kWaypointToleranceMm) {
      // Do not let a distance-complete result bypass the arrival heading
      // gate. The next cycle performs ARRIVAL coarse correction.
      replayRealignReason_ = ReplayRealignReason::ARRIVAL;
      logGuideRealign(replayRealignReason_, to, positionError, arrivalBearing,
                      current.headingDeg, arrivalHeadingError);
    } else {
      // A live pose update can move the chassis outside the arrival zone
      // before this result is consumed. Re-enter PATH guidance to the same
      // target instead of advancing on a stale completion.
      replayRealignReason_ = ReplayRealignReason::PATH;
      const float targetHeadingError =
          shortestDeltaDeg(targetBearing, current.headingDeg);
      logGuideRealign(replayRealignReason_, to, positionError, targetBearing,
                      current.headingDeg, targetHeadingError);
    }
  } else if (result.code == AiDistanceResultCode::OBSTACLE) {
    enterReplayHold(MapHoldReason::OBSTACLE, true);
  } else if (result.code == AiDistanceResultCode::CANCELLED &&
             ps2_.motionCommandActive()) {
    cancelReplay("PS2_TAKEOVER");
  } else if (result.code == AiDistanceResultCode::CANCELLED) {
    cancelReplay();
  } else {
    if (result.code == AiDistanceResultCode::ENCODER_FAULT) {
      abortReplay("ENCODER_FAULT");
    } else if (result.code == AiDistanceResultCode::HEADING_LOST) {
      abortReplay("HEADING_LOST");
    } else {
      abortReplay("MOVE_ERROR");
    }
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
  } else if (loadedValid_ || mode_ == MapControllerMode::TEACHING ||
             mode_ == MapControllerMode::CLOSED_CONFIRM) {
    metadata.state = (mode_ == MapControllerMode::TEACHING ||
                      mode_ == MapControllerMode::CLOSED_CONFIRM)
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
      static_cast<uint8_t>(holdReason_), replayTargetDeg_, replayLapCounter_,
      closeCandidateDistanceMm_, closeCandidateHeadingDeg_);
  const uint32_t now = millis();
  if (replayActive_ && replayOperation_ == MapReplayOperation::MOVE &&
      robot_.guidedWaypointActive() &&
      (lastGuidanceLogMs_ == 0U ||
       now - lastGuidanceLogMs_ >= MAP_GUIDE_TELEMETRY_MS)) {
    logGuideUpdate();
    lastGuidanceLogMs_ = now;
  }
  lastStatusMs_ = now;
  statusDirty_ = false;
}

void MapController::logOptimizeSummary(
    const RouteCleanerMetrics& metrics) const {
  debug_.print("MAP,OPTIMIZE,RAW_POINTS=");
  debug_.print(static_cast<unsigned>(metrics.rawPoints));
  debug_.print(",CLEAN_POINTS=");
  debug_.print(static_cast<unsigned>(metrics.cleanPoints));
  debug_.print(",REMOVED=");
  debug_.print(static_cast<unsigned>(metrics.rawPoints - metrics.cleanPoints));
  debug_.print(",RAW_LEN=");
  debug_.print(metrics.rawLengthMm);
  debug_.print(",CLEAN_LEN=");
  debug_.print(metrics.cleanLengthMm);
  debug_.print(",MAX_DEV=");
  debug_.print(metrics.maxDeviationMm);
  debug_.print(",MANUAL_KEPT=");
  debug_.print(static_cast<unsigned>(metrics.manualKept));
  debug_.print(",CORNER_KEPT=");
  debug_.print(static_cast<unsigned>(metrics.cornerKept));
  debug_.print(",REMOVED_DUPLICATE=");
  debug_.print(static_cast<unsigned>(metrics.removedDuplicate));
  debug_.print(",REMOVED_SHORT=");
  debug_.print(static_cast<unsigned>(metrics.removedShort));
  debug_.print(",REMOVED_COLLINEAR=");
  debug_.print(static_cast<unsigned>(metrics.removedCollinear));
  debug_.print(",REMOVED_CORNER_CLUSTER=");
  debug_.print(static_cast<unsigned>(metrics.removedCornerCluster));
  debug_.print(",FITTED_CORNER=");
  debug_.print(static_cast<unsigned>(metrics.fittedCorner));
  debug_.print(",RESULT=");
  if (metrics.accepted) {
    debug_.println("ACCEPT");
  } else {
    debug_.print("RAW_FALLBACK,REASON=");
    debug_.println(metrics.fallbackReason != nullptr ? metrics.fallbackReason
                                                      : "UNKNOWN");
  }
  // Keep each optimized point on its own short diagnostic line. This makes
  // corner selection auditable over narrow serial terminals without changing
  // the route format or replay behavior.
  for (uint16_t index = 0U; index < route_.header.waypointCount; ++index) {
    const MapWaypoint& point = route_.waypoints[index];
    debug_.print("MAP,OPTIMIZE_WP,I=");
    debug_.print(static_cast<unsigned>(index));
    debug_.print(",X=");
    debug_.print(point.xMm);
    debug_.print(",Y=");
    debug_.print(point.yMm);
    debug_.print(",H=");
    debug_.print(point.headingCdeg);
    debug_.print(",F=");
    debug_.println(static_cast<unsigned>(point.flags));
  }
}

void MapController::logSemanticSummary(
    const SemanticRouteMetrics& metrics) const {
  debug_.print("MAP,SEMANTIC,RAW_POINTS=");
  debug_.print(static_cast<unsigned>(metrics.rawPoints));
  debug_.print(",CLEAN_POINTS=");
  debug_.print(static_cast<unsigned>(metrics.cleanPoints));
  debug_.print(",SEM_POINTS=");
  debug_.print(static_cast<unsigned>(metrics.semanticPoints));
  debug_.print(",STRAIGHTS=");
  debug_.print(static_cast<unsigned>(metrics.straightRuns));
  debug_.print(",TURN_REGIONS=");
  debug_.print(static_cast<unsigned>(metrics.turnRegions));
  debug_.print(",SYNTH_CORNERS=");
  debug_.print(static_cast<unsigned>(metrics.syntheticCorners));
  debug_.print(",RAW_CORNERS_USED=");
  debug_.print(static_cast<unsigned>(metrics.rawCornersUsed));
  debug_.print(",MANUAL_KEPT=");
  debug_.print(static_cast<unsigned>(metrics.manualKept));
  debug_.print(",MAX_DEV=");
  debug_.print(metrics.maxDeviationMm);
  debug_.print(",CLEAN_LEN=");
  debug_.print(metrics.cleanLengthMm);
  debug_.print(",SEM_LEN=");
  debug_.print(metrics.semanticLengthMm);
  debug_.print(",RESULT=");
  if (metrics.accepted) {
    debug_.println("ACCEPT");
  } else {
    debug_.print("FALLBACK_CLEAN,REASON=");
    debug_.println(metrics.fallbackReason != nullptr ? metrics.fallbackReason
                                                      : "UNKNOWN");
    debug_.print("MAP,SEMANTIC=FALLBACK_CLEAN,REASON=");
    debug_.println(metrics.fallbackReason != nullptr ? metrics.fallbackReason
                                                      : "UNKNOWN");
  }
  if (metrics.diagnosticOverflow) debug_.println("MAP,CORNER,DIAGNOSTIC_OVERFLOW=1");
  for (uint16_t index = 0U; index < metrics.cornerDiagnosticCount; ++index) {
    const SemanticCornerDiagnostic& corner = metrics.corners[index];
    debug_.print("MAP,CORNER,IDX=");
    debug_.print(static_cast<unsigned>(corner.index));
    debug_.print(",ANGLE=");
    debug_.print(static_cast<float>(corner.angleCdeg) / 100.0f, 2);
    debug_.print(",IN_DIR=");
    debug_.print(static_cast<float>(corner.incomingDirectionCdeg) / 100.0f,
                 2);
    debug_.print(",OUT_DIR=");
    debug_.print(static_cast<float>(corner.outgoingDirectionCdeg) / 100.0f,
                 2);
    debug_.print(",SHIFT=");
    debug_.print(corner.shiftMm);
    debug_.print(",DEV=");
    debug_.print(corner.deviationMm);
    debug_.print(",SOURCE=");
    switch (corner.source) {
      case SemanticCornerSource::SYNTHETIC: debug_.print("SYNTHETIC"); break;
      case SemanticCornerSource::MANUAL: debug_.print("MANUAL"); break;
      case SemanticCornerSource::RAW: debug_.print("RAW"); break;
    }
    debug_.print(",RESULT=");
    debug_.println(corner.accepted ? "ACCEPT" : "REJECT");
  }
  for (uint16_t index = 0U; index < route_.header.waypointCount; ++index) {
    const MapWaypoint& point = route_.waypoints[index];
    debug_.print("MAP,SEMANTIC_WP,I=");
    debug_.print(static_cast<unsigned>(index));
    debug_.print(",X=");
    debug_.print(point.xMm);
    debug_.print(",Y=");
    debug_.print(point.yMm);
    debug_.print(",H=");
    debug_.print(point.headingCdeg);
    debug_.print(",F=");
    debug_.println(static_cast<unsigned>(point.flags));
  }
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

void MapController::logGuidePlan(uint16_t from, uint16_t to, float distanceMm,
                                  float bearingDeg, float headingDeg,
                                  float headingErrorDeg) const {
  debug_.print("MAP,GUIDE,PLAN,WP_FROM=");
  debug_.print(static_cast<unsigned>(from));
  debug_.print(",WP_TO=");
  debug_.print(static_cast<unsigned>(to));
  debug_.print(",DIST_MM=");
  debug_.print(distanceMm, 1);
  debug_.print(",BEARING=");
  debug_.print(bearingDeg, 2);
  debug_.print(",HDG=");
  debug_.print(headingDeg, 2);
  debug_.print(",HDG_ERR=");
  debug_.print(headingErrorDeg, 2);
  debug_.print(",PRETURN_TOL=");
  debug_.print(MAP_REPLAY_PRETURN_TOLERANCE_DEG, 1);
  debug_.print(",REALIGN_THR=");
  debug_.println(MAP_GUIDE_REALIGN_THRESHOLD_DEG, 1);
}

void MapController::logGuidePreturn(float headingErrorDeg,
                                     const char* action) const {
  debug_.print("MAP,GUIDE,PRETURN,ERR=");
  debug_.print(headingErrorDeg, 2);
  debug_.print(",TOL=");
  debug_.print(MAP_REPLAY_PRETURN_TOLERANCE_DEG, 1);
  debug_.print(",SETTLE_MS=");
  debug_.print(MAP_REPLAY_PRETURN_SETTLE_MS);
  debug_.print(",ACTION=");
  debug_.println(action != nullptr ? action : "UNKNOWN");
}

void MapController::logGuideStart(uint16_t waypoint, float distanceMm,
                                  float bearingDeg) const {
  debug_.print("MAP,GUIDE,START,WP=");
  debug_.print(static_cast<unsigned>(waypoint));
  debug_.print(",DIST_MM=");
  debug_.print(distanceMm, 1);
  debug_.print(",BEARING=");
  debug_.print(bearingDeg, 2);
  debug_.print(",MIN_SPEED=");
  debug_.println(MAP_GUIDE_MIN_SPEED);
}

void MapController::logGuideUpdate() const {
  debug_.print("MAP,GUIDE,UPDATE,REMAIN_MM=");
  debug_.print(robot_.guidedRemainingMm(), 1);
  debug_.print(",HDG_ERR=");
  debug_.print(robot_.guidedHeadingErrorDeg(), 2);
  debug_.print(",XTRACK_MM=");
  debug_.print(robot_.guidedCrossTrackMm(), 1);
  debug_.print(",BASE=");
  debug_.print(robot_.guidedBaseSpeed());
  debug_.print(",STEER=");
  debug_.print(robot_.guidedSteering());
  debug_.print(",L=");
  debug_.print(robot_.targetLeftCommand());
  debug_.print(",R=");
  debug_.println(robot_.targetRightCommand());
}

const char* MapController::realignReasonName(
    ReplayRealignReason reason) {
  switch (reason) {
    case ReplayRealignReason::NONE: return "NONE";
    case ReplayRealignReason::PATH: return "PATH";
    case ReplayRealignReason::ARRIVAL: return "ARRIVAL";
  }
  return "NONE";
}

void MapController::logGuideDone(uint16_t waypoint, float positionErrorMm,
                                 float arrivalBearingDeg, float headingDeg,
                                 float headingErrorDeg) const {
  debug_.print("MAP,GUIDE,DONE,WP=");
  debug_.print(static_cast<unsigned>(waypoint));
  debug_.print(",POS_ERR_MM=");
  debug_.print(positionErrorMm, 1);
  debug_.print(",ARRIVAL_BEARING=");
  debug_.print(arrivalBearingDeg, 2);
  debug_.print(",HDG=");
  debug_.print(headingDeg, 2);
  debug_.print(",ARRIVAL_HDG_ERR=");
  debug_.print(headingErrorDeg, 2);
  debug_.print(",POS_TOL_MM=");
  debug_.print(MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM);
  debug_.print(",HDG_TOL=");
  debug_.println(MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG, 1);
}

void MapController::logGuideRealign(ReplayRealignReason reason,
                                    uint16_t waypoint, float distanceMmValue,
                                    float desiredBearingDeg, float headingDeg,
                                    float headingErrorDeg) const {
  debug_.print("MAP,GUIDE,REALIGN,WP=");
  debug_.print(static_cast<unsigned>(waypoint));
  debug_.print(",TYPE=");
  debug_.print(realignReasonName(reason));
  debug_.print(",DIST_MM=");
  debug_.print(distanceMmValue, 1);
  debug_.print(reason == ReplayRealignReason::ARRIVAL ? ",INCOMING="
                                                     : ",DESIRED=");
  debug_.print(desiredBearingDeg, 2);
  debug_.print(",HDG=");
  debug_.print(headingDeg, 2);
  debug_.print(",ERR=");
  debug_.print(headingErrorDeg, 2);
  debug_.print(",ARRIVAL_TOL=");
  debug_.print(MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG, 1);
  debug_.print(",PATH_THRESHOLD=");
  debug_.println(MAP_GUIDE_REALIGN_THRESHOLD_DEG, 1);
}

void MapController::logGuideRealignDone(
    ReplayRealignReason reason, uint16_t waypoint, float positionErrorMm,
    float desiredBearingDeg, float headingDeg, float headingErrorDeg,
    const char* action) const {
  debug_.print("MAP,GUIDE,REALIGN_DONE,TYPE=");
  debug_.print(realignReasonName(reason));
  debug_.print(",WP=");
  debug_.print(static_cast<unsigned>(waypoint));
  debug_.print(",POS_ERR_MM=");
  debug_.print(positionErrorMm, 1);
  debug_.print(reason == ReplayRealignReason::ARRIVAL ? ",INCOMING="
                                                     : ",DESIRED=");
  debug_.print(desiredBearingDeg, 2);
  debug_.print(",HDG=");
  debug_.print(headingDeg, 2);
  debug_.print(",HDG_ERR=");
  debug_.print(headingErrorDeg, 2);
  debug_.print(",ACTION=");
  debug_.println(action != nullptr ? action : "RETRY");
}

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
// A coarse ARRIVAL turn can translate the chassis by a few millimetres while
// correcting heading.  This bounded recovery window is only for the
// immediately-following ARRIVAL realign; the normal arrival gate remains the
// configured waypoint tolerance above.  Never restart a forward Guided MOVE
// for this small post-turn drift, because it can carry the chassis past the
// target and make point-pursuit turn back toward the same waypoint.
constexpr float kPostTurnArrivalRecoveryToleranceMm = 10.0f;
constexpr uint8_t kMaxArrivalTurnAttempts = 2U;
constexpr float kReplayPoseHoldToleranceMm = 100.0f;
constexpr float kReplayPoseHoldToleranceDeg = 15.0f;
constexpr float kMinimumSegmentMm = 20.0f;
constexpr float kMaximumSegmentMm = 5000.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
// Kept as a named compatibility alias for the first-segment policy. The MAP
// pre-turn tolerance is now the single source of truth for replay guidance.
constexpr float kReplayStartupTurnDeadbandDeg =
    MAP_REPLAY_PRETURN_TOLERANCE_DEG;

const char* ActionName(Ps2MapAction action) {
  switch (action) {
    case Ps2MapAction::SLOT: return "SLOT";
    case Ps2MapAction::START: return "START";
    case Ps2MapAction::UP: return "UP";
    case Ps2MapAction::DOWN: return "DOWN";
    case Ps2MapAction::LEFT: return "LEFT";
    case Ps2MapAction::RIGHT: return "RIGHT";
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

bool IsReplayModeValueValid(MapReplayMode mode) {
  return mode == MapReplayMode::ONCE || mode == MapReplayMode::LOOP ||
         mode == MapReplayMode::RETURN || mode == MapReplayMode::PING_PONG ||
         mode == MapReplayMode::CLOSED;
}

MapReplayMode EffectiveReplayMode(MapRouteType type, MapReplayMode mode) {
  // Old records encoded a closed route as routeType=CLOSED with ONCE/LOOP.
  // Translate the compatibility representation into the runtime mode.
  if (type == MapRouteType::CLOSED && mode == MapReplayMode::ONCE) {
    return MapReplayMode::CLOSED;
  }
  return mode;
}

bool IsReplayModeAllowed(MapRouteType type, MapReplayMode mode) {
  if (!IsReplayModeValueValid(mode)) return false;
  if (type == MapRouteType::CLOSED) {
    return mode == MapReplayMode::ONCE || mode == MapReplayMode::LOOP;
  }
  return mode == MapReplayMode::ONCE || mode == MapReplayMode::RETURN ||
         mode == MapReplayMode::PING_PONG || mode == MapReplayMode::CLOSED ||
         mode == MapReplayMode::LOOP;
}

bool IsClosingMode(MapReplayMode mode) {
  return mode == MapReplayMode::CLOSED || mode == MapReplayMode::LOOP;
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

const char* MissionInitiatorName(MapMissionInitiator initiator) {
  switch (initiator) {
    case MapMissionInitiator::PS2: return "PS2";
    case MapMissionInitiator::AI_VOICE: return "AI";
    case MapMissionInitiator::NONE: return "NONE";
  }
  return "NONE";
}

const char* ReturnP0SourceName(ReturnP0Source source) {
  switch (source) {
    case ReturnP0Source::PS2_START: return "PS2_START";
    case ReturnP0Source::AI_VOICE: return "AI_VOICE";
    case ReturnP0Source::INTERNAL: return "INTERNAL";
    case ReturnP0Source::NONE: return "NONE";
  }
  return "NONE";
}
}  // namespace

MapController::MapController(RobotController& robot, Ps2Controller& ps2,
                             LcdDisplay& display, WheelOdometry& odometry,
                             HeadingFusion& fusion,
                             UltrasonicSensor& ultrasonic,
                             ObstacleClassifier& obstacleClassifier,
                             Print& debugStream)
    : robot_(robot), ps2_(ps2), display_(display), odometry_(odometry),
      fusion_(fusion), ultrasonic_(ultrasonic),
      obstacleClassifier_(obstacleClassifier), debug_(debugStream) {}

const char* MapController::storageErrorReasonName(
    MapStorageErrorReason reason) {
  switch (reason) {
    case MapStorageErrorReason::SETTINGS_SAVE: return "SETTINGS_SAVE";
    case MapStorageErrorReason::TEACH_SAVE: return "TEACH_SAVE";
    case MapStorageErrorReason::MODE_SAVE: return "MODE_SAVE";
    case MapStorageErrorReason::STORAGE_INIT: return "STORAGE_INIT";
    case MapStorageErrorReason::GENERIC: return "GENERIC";
    case MapStorageErrorReason::NONE: return "NONE";
  }
  return "GENERIC";
}

const char* MapController::returnPhaseName(ReplayReturnPhase phase) {
  switch (phase) {
    case ReplayReturnPhase::OUTBOUND: return "OUT";
    case ReplayReturnPhase::INBOUND: return "BACK";
    case ReplayReturnPhase::NONE: return "NONE";
  }
  return "NONE";
}

MapReplayMode MapController::executionModeFor(MapUserMode mode,
                                              uint8_t repeatTarget) {
  switch (mode) {
    case MapUserMode::ONCE:
      return MapReplayMode::ONCE;
    case MapUserMode::SHUTTLE:
      return repeatTarget == MAP_LOOP_TARGET_MIN ? MapReplayMode::RETURN
                                                 : MapReplayMode::PING_PONG;
    case MapUserMode::LOOP:
      return repeatTarget == MAP_LOOP_TARGET_MIN ? MapReplayMode::CLOSED
                                                 : MapReplayMode::LOOP;
  }
  return MapReplayMode::ONCE;
}

MapUserMode MapController::userModeFromStored(MapRouteType type,
                                               MapReplayMode mode,
                                               bool shuttleRepeat) {
  (void)shuttleRepeat;
  if (type == MapRouteType::CLOSED || mode == MapReplayMode::CLOSED) {
    return MapUserMode::LOOP;
  }
  if (mode == MapReplayMode::RETURN || mode == MapReplayMode::PING_PONG) {
    return MapUserMode::SHUTTLE;
  }
  if (mode == MapReplayMode::LOOP) {
    // Accept the experimental OPEN+LOOP representation from older builds.
    return MapUserMode::LOOP;
  }
  return MapUserMode::ONCE;
}

const char* MapController::userModeName(MapUserMode mode) {
  switch (mode) {
    case MapUserMode::ONCE: return "ONCE";
    case MapUserMode::SHUTTLE: return "SHUTTLE";
    case MapUserMode::LOOP: return "LOOP";
  }
  return "ONCE";
}

bool MapController::userModeNeedsClosingEdge(MapUserMode mode) {
  return mode == MapUserMode::LOOP;
}

void MapController::applyStoredSettings(MapRouteType type, MapReplayMode mode,
                                         bool shuttleRepeat,
                                         uint8_t encodedTarget) {
  userMode_ = userModeFromStored(type, mode, shuttleRepeat);
  if (type == MapRouteType::CLOSED || mode == MapReplayMode::CLOSED) {
    loopTarget_ = mode == MapReplayMode::LOOP ? encodedTarget
                                              : MAP_LOOP_TARGET_MIN;
  } else if (mode == MapReplayMode::RETURN) {
    loopTarget_ = shuttleRepeat ? encodedTarget : MAP_LOOP_TARGET_MIN;
  } else if (mode == MapReplayMode::PING_PONG) {
    // Legacy PING_PONG is always infinite.
    userMode_ = MapUserMode::SHUTTLE;
    loopTarget_ = MAP_LOOP_TARGET_INF;
  } else if (mode == MapReplayMode::LOOP) {
    loopTarget_ = encodedTarget;
  } else {
    loopTarget_ = MAP_LOOP_TARGET_MIN;
  }
  routeMode_ = mode == MapReplayMode::PING_PONG
                   ? MapReplayMode::PING_PONG
                   : executionModeFor(userMode_, loopTarget_);
}

void MapController::begin() {
  ps2_.setMapUiCapture(false);
  homeContext_ = {};
  backP0UiDismissed_ = false;
  pendingHomeOrigin_ = {};
  pendingHomeResetGeneration_ = 0U;
  pendingHomeHeadingResetGeneration_ = 0U;
  pendingHomeContextValid_ = false;
  returnP0State_ = ReturnP0State::IDLE;
  returnP0Source_ = ReturnP0Source::NONE;
  returnP0Generation_ = 0U;
  returnP0SegmentGeneration_ = 0U;
  if (!store_.begin()) {
    storeState_ = MapStoreState::STORAGE_ERROR;
    storageErrorReason_ = MapStorageErrorReason::STORAGE_INIT;
    robot_.stopImmediately(true);
    log("OWNER=STM32,STORE=ERROR");
    publishStatus();
    return;
  }
  storageErrorReason_ = MapStorageErrorReason::NONE;
  const MapSlotMetadata metadata = store_.metadata(selectedSlot_);
  storeState_ = metadata.state;
  routeType_ = MapRouteType::OPEN;
  replaySpeed_ = metadata.replaySpeed;
  if (IsReplayModeAllowed(metadata.routeType, metadata.replayMode)) {
    applyStoredSettings(metadata.routeType, metadata.replayMode,
                        metadata.shuttleRepeat, metadata.loopTarget);
  } else {
    userMode_ = MapUserMode::ONCE;
    routeMode_ = MapReplayMode::ONCE;
    loopTarget_ = MAP_LOOP_TARGET_MIN;
  }
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

void MapController::notifyExternalStop() {
  // RobotLink STOP has already reached RobotController::stopImmediately() in
  // main.cpp. This notification is only the MAP safety boundary: it latches
  // autonomous resume off and never creates a new motion command.
  inhibitAutonomousResume("EXTERNAL_STOP");
  debug_.println("MAP,STOP,EXTERNAL,LATCH=1");
  if (returnP0InProgress()) {
    abortReturnToP0("EXTERNAL_STOP");
  } else if (obstacleDetourContextActive()) {
    abortObstacleDetour("EXTERNAL_STOP");
  } else if (replayActive_ && mode_ != MapControllerMode::REPLAY_HOLD) {
    enterReplayHold(MapHoldReason::EXTERNAL_STOP, false);
  }
}

void MapController::update() {
  processInput();

  if (homeContext_.valid) {
    if (storageErrorReason_ != MapStorageErrorReason::NONE) {
      invalidateHomeContext("STORAGE_ERROR");
    } else if (selectedSlot_ != homeContext_.slot ||
               (loadedValid_ &&
                route_.header.generation != homeContext_.routeGeneration)) {
      invalidateHomeContext("ROUTE_GENERATION");
    } else if (odometry_.resetGeneration() !=
                   homeContext_.odometryResetGeneration ||
               robot_.headingResetGeneration() !=
                   homeContext_.headingResetGeneration) {
      invalidateHomeContext("RESET_BOUNDARY");
    }
  }

  // A PS2 reconnect resets its transient parser state. Reassert the explicit
  // UI capture while Settings/Help/Delete is still active so a reconnect can
  // never reopen the manual motor path behind the menu.
  if ((mode_ == MapControllerMode::SETTINGS ||
       mode_ == MapControllerMode::HELP ||
       mode_ == MapControllerMode::DELETE_CONFIRM) &&
      !ps2_.mapUiCaptureActive()) {
    ps2_.setMapUiCapture(true);
  }

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

  // Track the obstacle clear window while held. Manual replay still requires
  // an explicit START; an accepted AI mission may pass the independent
  // autonomous-resume gate after the stricter clear period.
  serviceObstacleHold();

  if (postTeachBack_.valid && !postTeachBackActive_) {
    const bool resetChanged =
        odometry_.resetGeneration() != postTeachBack_.odometryResetGeneration ||
        robot_.headingResetGeneration() != postTeachBack_.headingResetGeneration;
    const bool motionStarted =
        ps2_.motionCommandActive() || robot_.aiMotionActive() ||
        robot_.motionOwner() != MotionOwner::NONE || !robot_.motorsStopped();
    if (resetChanged) {
      invalidatePostTeachBack("RESET_BOUNDARY");
    } else if (motionStarted) {
      invalidatePostTeachBack("MOTION_AFTER_SAVE");
    }
  }

  if (replayActive_) {
    if (returnP0InProgress()) {
      if (!homeContext_.valid || selectedSlot_ != homeContext_.slot ||
          route_.header.generation != homeContext_.routeGeneration ||
          odometry_.resetGeneration() != homeContext_.odometryResetGeneration ||
          robot_.headingResetGeneration() !=
              homeContext_.headingResetGeneration) {
        invalidateHomeContext("RESET_BOUNDARY");
        abortReturnToP0("RESET_BOUNDARY");
      } else if (ps2_.state().r3 || ps2_.motionCommandActive()) {
        abortReturnToP0(ps2_.state().r3 ? "R3" : "PS2_TAKEOVER");
      } else if (returnP0State_ == ReturnP0State::HOLD) {
        // AI Return uses the same safety HOLD boundary as replay, but its
        // clear/resume path is serviced below. PS2 Return remains manual-only.
      } else if (robot_.motionOwner() != MotionOwner::REPLAY &&
                 !robot_.aiMotionActive() &&
                 replayOperation_ != MapReplayOperation::NONE) {
        abortReturnToP0("EXTERNAL_STOP");
      } else {
        updateReturnToP0();
      }
    } else if (odometry_.resetGeneration() != replayOriginResetGeneration_ ||
        robot_.headingResetGeneration() != replayOriginHeadingResetGeneration_ ||
        route_.header.generation != replayOriginRouteGeneration_) {
      // This is intentionally diagnostic-only. The three origin snapshots
      // remain the exact safety gate below; emitting both sides of the
      // comparison lets a field HIL log identify which boundary changed.
      debug_.print("MAP,RESET_BOUNDARY,ODOM_ORIGIN=");
      debug_.print(replayOriginResetGeneration_);
      debug_.print(",ODOM_NOW=");
      debug_.print(odometry_.resetGeneration());
      debug_.print(",HEADING_ORIGIN=");
      debug_.print(replayOriginHeadingResetGeneration_);
      debug_.print(",HEADING_NOW=");
      debug_.print(robot_.headingResetGeneration());
      debug_.print(",ROUTE_ORIGIN=");
      debug_.print(replayOriginRouteGeneration_);
      debug_.print(",ROUTE_NOW=");
      debug_.println(route_.header.generation);
      if (obstacleDetourContextActive()) {
        abortObstacleDetour("RESET_BOUNDARY");
      } else {
        abortReplay("RESET_BOUNDARY");
      }
    } else if (ps2_.state().r3) {
      if (obstacleDetourContextActive()) {
        abortObstacleDetour("R3");
      } else {
        cancelReplay("R3");
      }
    } else if (obstacleDetourContextActive()) {
      if (ps2_.motionCommandActive()) {
        abortObstacleDetour("PS2_TAKEOVER");
      } else if (obstacleDetourInProgress()) {
        updateObstacleDetour();
      }
    } else if (robot_.motionOwner() != MotionOwner::REPLAY &&
               !robot_.aiMotionActive() &&
               replayOperation_ != MapReplayOperation::NONE &&
               !(mode_ == MapControllerMode::REPLAY_HOLD &&
                 holdReason_ == MapHoldReason::OBSTACLE &&
                 replayOperation_ == MapReplayOperation::HOLD)) {
      // External STOP/mission arbitration removed the owner without producing
      // a replay result. Do not infer success from a stopped motor.
      enterReplayHold(ps2_.motionCommandActive()
                          ? MapHoldReason::PS2_TAKEOVER
                          : MapHoldReason::EXTERNAL_STOP,
                      false);
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
    case Ps2MapAction::UP:
    case Ps2MapAction::DOWN:
    case Ps2MapAction::LEFT:
    case Ps2MapAction::RIGHT:
      handleSettingsInput(event.action);
      break;
    case Ps2MapAction::TRIANGLE: handleTriangle(); break;
    case Ps2MapAction::CIRCLE: handleCircle(); break;
    case Ps2MapAction::SQUARE: handleSquare(false); break;
    case Ps2MapAction::SQUARE_LONG: handleSquare(true); break;
    case Ps2MapAction::CROSS: handleCross(); break;
    case Ps2MapAction::CROSS_LONG: handleCrossLong(); break;
    case Ps2MapAction::SELECT_LONG:
      {
        const char* reason = nullptr;
        if (!enterSettings(reason)) {
          debug_.print("MAP,SETTINGS,REJECT,REASON=");
          debug_.println(reason != nullptr ? reason : "NOT_ALLOWED");
        }
      }
      break;
  }
  statusDirty_ = true;
}

void MapController::handleSlot(uint8_t slot) {
  if (storageErrorReason_ != MapStorageErrorReason::NONE ||
      mode_ == MapControllerMode::TEACHING || replayActive_ ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::CLOSED_CONFIRM ||
      mode_ == MapControllerMode::SETTINGS ||
      mode_ == MapControllerMode::HELP) {
    return;
  }
  invalidatePostTeachBack("SLOT_CHANGED");
  invalidateHomeContext("SLOT_CHANGED");
  selectedSlot_ = slot == 2U ? MapSlot::MAP_2 : MapSlot::MAP_1;
  loadedValid_ = false;
  const MapSlotMetadata metadata = store_.metadata(selectedSlot_);
  storeState_ = metadata.state;
  routeType_ = MapRouteType::OPEN;
  replaySpeed_ = metadata.replaySpeed;
  if (IsReplayModeAllowed(metadata.routeType, metadata.replayMode)) {
    applyStoredSettings(metadata.routeType, metadata.replayMode,
                        metadata.shuttleRepeat, metadata.loopTarget);
  } else {
    userMode_ = MapUserMode::ONCE;
    routeMode_ = MapReplayMode::ONCE;
    loopTarget_ = MAP_LOOP_TARGET_MIN;
  }
  storageErrorReason_ = MapStorageErrorReason::NONE;
  mode_ = metadata.state == MapStoreState::SAVED
              ? MapControllerMode::SAVED
              : MapControllerMode::READY;
}

bool MapController::enterSettings(const char*& reason) {
  reason = nullptr;
  if (!display_.isMapPage()) {
    reason = "NOT_MAP_PAGE";
    return false;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    reason = "TEACHING";
    return false;
  }
  if (storageErrorReason_ != MapStorageErrorReason::NONE) {
    reason = storageErrorReason_ == MapStorageErrorReason::STORAGE_INIT
                 ? "STORAGE_INIT"
                 : "STORAGE_ERROR";
    return false;
  }
  if (replayActive_ || mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::CLOSED_CONFIRM ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::SETTINGS ||
      mode_ == MapControllerMode::HELP) {
    reason = "BUSY";
    return false;
  }
  if (storeState_ != MapStoreState::SAVED &&
      mode_ != MapControllerMode::REPLAY_COMPLETE) {
    reason = "NOT_SAVED";
    return false;
  }
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    reason = "MOTION";
    return false;
  }
  if (ps2_.motionCommandActive()) {
    reason = "PS2_MOTION";
    return false;
  }
  if (!loadedValid_ && !loadSelected()) {
    reason = "LOAD";
    return false;
  }
  invalidatePostTeachBack("SETTINGS");
  settingsItem_ = MapSettingsItem::MODE;
  settingsUserMode_ = userMode_;
  settingsSpeed_ = replaySpeed_;
  settingsRepeatTarget_ = loopTarget_;
  helpPage_ = 0U;
  mode_ = MapControllerMode::SETTINGS;
  // Entering Settings is itself a safety boundary. It is currently allowed
  // only while stopped, but still clear any stale manual command and require
  // a fresh neutral frame before Manual can ever reacquire the chassis.
  robot_.stopImmediately(true);
  ps2_.setMapUiCapture(true);
  debug_.print("MAP,SETTINGS,ENTER,SLOT=");
  debug_.println(static_cast<unsigned>(selectedSlot_));
  statusDirty_ = true;
  return true;
}

void MapController::leaveMapUiCapture() {
  ps2_.setMapUiCapture(false);
  ps2_.disarmMapInput();
}

void MapController::saveSettingsAndExit() {
  if (mode_ != MapControllerMode::SETTINGS) return;
  invalidatePostTeachBack("SETTINGS_SAVE");
  const char* modeReason = nullptr;
  if (userModeNeedsClosingEdge(settingsUserMode_) &&
      !hasValidClosingEdge(route_, modeReason)) {
    debug_.println("MAP,SETTINGS,SAVE,REJECT=MODE");
    return;
  }
  const uint8_t normalizedTarget = settingsUserMode_ == MapUserMode::ONCE
                                       ? MAP_LOOP_TARGET_MIN
                                       : settingsRepeatTarget_;
  const MapReplayMode settingsExecution =
      executionModeFor(settingsUserMode_, normalizedTarget);
  const bool settingsChanged = settingsUserMode_ != userMode_ ||
                               settingsSpeed_ != replaySpeed_ ||
                               normalizedTarget != loopTarget_;
  if (!settingsChanged) {
    storeState_ = MapStoreState::SAVED;
    storageErrorReason_ = MapStorageErrorReason::NONE;
    mode_ = MapControllerMode::SAVED;
    debug_.println("MAP,SETTINGS,SAVE=NO_CHANGE");
    leaveMapUiCapture();
    statusDirty_ = true;
    return;
  }
  MapRouteData candidate = route_;
  // The route geometry remains canonical one-map data. Storage encoding is
  // applied by updateRouteHeaderForSave() so V5.2.10 can read every mode.
  const MapReplayMode oldMode = routeMode_;
  const MapUserMode oldUserMode = userMode_;
  const int16_t oldSpeed = replaySpeed_;
  const uint8_t oldLoopTarget = loopTarget_;
  replaySpeed_ = settingsSpeed_;
  loopTarget_ = normalizedTarget;
  updateRouteHeaderForSave(candidate, settingsUserMode_, normalizedTarget);
  if (!store_.save(selectedSlot_, candidate)) {
    routeMode_ = oldMode;
    userMode_ = oldUserMode;
    replaySpeed_ = oldSpeed;
    loopTarget_ = oldLoopTarget;
    storeState_ = MapStoreState::STORAGE_ERROR;
    storageErrorReason_ = MapStorageErrorReason::SETTINGS_SAVE;
    debug_.println("MAP,SETTINGS,SAVE=FAIL,OLD_ROUTE_RETAINED=1");
    mode_ = MapControllerMode::SAVED;
    leaveMapUiCapture();
    statusDirty_ = true;
    return;
  }
  route_ = candidate;
  normalizeRouteForRuntime(route_, settingsExecution);
  routeType_ = MapRouteType::OPEN;
  userMode_ = settingsUserMode_;
  routeMode_ = settingsExecution;
  replaySpeed_ = settingsSpeed_;
  loopTarget_ = normalizedTarget;
  loadedValid_ = true;
  storeState_ = MapStoreState::SAVED;
  storageErrorReason_ = MapStorageErrorReason::NONE;
  mode_ = MapControllerMode::SAVED;
  debug_.print("MAP,SETTINGS,SAVE=OK,MODE=");
  debug_.print(userModeName(userMode_));
  debug_.print(",EXEC=");
  debug_.print(MapRouteStore::replayModeName(routeMode_));
  debug_.print(",SPEED=");
  debug_.print(replaySpeed_);
  debug_.print(",COUNT=");
  if (loopTarget_ == MAP_LOOP_TARGET_INF) {
    debug_.println("INF");
  } else {
    debug_.println(static_cast<unsigned>(loopTarget_));
  }
  leaveMapUiCapture();
  statusDirty_ = true;
}

void MapController::cancelSettings() {
  if (mode_ != MapControllerMode::SETTINGS &&
      mode_ != MapControllerMode::HELP &&
      mode_ != MapControllerMode::DELETE_CONFIRM) {
    return;
  }
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                               : MapControllerMode::READY;
  debug_.println("MAP,SETTINGS,CANCEL");
  leaveMapUiCapture();
  statusDirty_ = true;
}

void MapController::leaveHelp() {
  if (mode_ != MapControllerMode::HELP) return;
  mode_ = MapControllerMode::SETTINGS;
  ps2_.setMapUiCapture(true);
  statusDirty_ = true;
}

void MapController::enterHelp() {
  if (mode_ != MapControllerMode::SETTINGS &&
      mode_ != MapControllerMode::HELP) {
    return;
  }
  if (mode_ == MapControllerMode::SETTINGS) helpPage_ = 0U;
  mode_ = MapControllerMode::HELP;
  ps2_.setMapUiCapture(true);
  statusDirty_ = true;
}

void MapController::cycleSettingsMode(int8_t direction) {
  if (direction == 0) return;
  const char* closeReason = nullptr;
  const bool closeAvailable = hasValidClosingEdge(route_, closeReason);
  if (closeAvailable) {
    constexpr MapUserMode kModes[] = {
        MapUserMode::ONCE, MapUserMode::SHUTTLE, MapUserMode::LOOP};
    int index = 0;
    for (int i = 0; i < 3; ++i) {
      if (kModes[i] == settingsUserMode_) {
        index = i;
        break;
      }
    }
    index = (index + (direction > 0 ? 1 : 2)) % 3;
    settingsUserMode_ = kModes[index];
    return;
  }
  if ((direction > 0 && settingsUserMode_ == MapUserMode::SHUTTLE) ||
      (direction < 0 && settingsUserMode_ == MapUserMode::ONCE)) {
    debug_.print("MAP,MODE,CLOSE_UNAVAILABLE,REASON=");
    debug_.println(closeReason != nullptr ? closeReason : "CLOSURE");
  }
  settingsUserMode_ = settingsUserMode_ == MapUserMode::ONCE
                          ? MapUserMode::SHUTTLE
                          : MapUserMode::ONCE;
}

void MapController::cycleSettingsSpeed(int8_t direction) {
  const int16_t next = static_cast<int16_t>(
      settingsSpeed_ + direction * MAP_REPLAY_SPEED_STEP);
  settingsSpeed_ = constrain(next, MAP_REPLAY_SPEED_MIN,
                             MAP_REPLAY_SPEED_MAX);
}

void MapController::cycleSettingsLap(int8_t direction) {
  if (settingsUserMode_ == MapUserMode::ONCE || direction == 0) return;
  if (direction > 0) {
    settingsRepeatTarget_ = settingsRepeatTarget_ == MAP_LOOP_TARGET_MAX
                                 ? MAP_LOOP_TARGET_INF
                                 : settingsRepeatTarget_ + 1U;
  } else {
    settingsRepeatTarget_ = settingsRepeatTarget_ == MAP_LOOP_TARGET_INF
                                 ? MAP_LOOP_TARGET_MAX
                                 : settingsRepeatTarget_ <= MAP_LOOP_TARGET_MIN
                                     ? MAP_LOOP_TARGET_INF
                                     : settingsRepeatTarget_ - 1U;
  }
}

bool MapController::settingsCanDelete() const {
  return mode_ == MapControllerMode::SETTINGS &&
         settingsItem_ == MapSettingsItem::DELETE_MAP &&
         loadedValid_ && storeState_ == MapStoreState::SAVED &&
         !replayActive_ && !robot_.aiMotionActive() &&
         robot_.motorsStopped() && robot_.motionOwner() == MotionOwner::NONE;
}

void MapController::handleSettingsInput(Ps2MapAction action) {
  if (mode_ == MapControllerMode::HELP ||
      mode_ == MapControllerMode::DELETE_CONFIRM) {
    return;
  }
  if (mode_ != MapControllerMode::SETTINGS) return;
  switch (action) {
    case Ps2MapAction::UP:
      settingsItem_ = static_cast<MapSettingsItem>(
          settingsItem_ == MapSettingsItem::MODE
              ? static_cast<uint8_t>(MapSettingsItem::DELETE_MAP)
              : static_cast<uint8_t>(settingsItem_) - 1U);
      break;
    case Ps2MapAction::DOWN:
      settingsItem_ = static_cast<MapSettingsItem>(
          (static_cast<uint8_t>(settingsItem_) + 1U) % 4U);
      break;
    case Ps2MapAction::LEFT:
      if (settingsItem_ == MapSettingsItem::MODE) {
        cycleSettingsMode(-1);
      } else if (settingsItem_ == MapSettingsItem::SPEED) {
        cycleSettingsSpeed(-1);
      } else if (settingsItem_ == MapSettingsItem::COUNT) {
        cycleSettingsLap(-1);
      }
      break;
    case Ps2MapAction::RIGHT:
      if (settingsItem_ == MapSettingsItem::MODE) {
        cycleSettingsMode(1);
      } else if (settingsItem_ == MapSettingsItem::SPEED) {
        cycleSettingsSpeed(1);
      } else if (settingsItem_ == MapSettingsItem::COUNT) {
        cycleSettingsLap(1);
      } else if (settingsCanDelete()) {
        mode_ = MapControllerMode::DELETE_CONFIRM;
        debug_.println("MAP,SETTINGS,DELETE_CONFIRM");
      }
      break;
    default:
      break;
  }
  statusDirty_ = true;
}

void MapController::handleStart() {
  (void)requestStart(MapMissionInitiator::PS2);
}

bool MapController::requestStart(MapMissionInitiator initiator) {
  if (initiator == MapMissionInitiator::NONE) {
    logStartReject("INITIATOR");
    return false;
  }
  if (storageErrorReason_ != MapStorageErrorReason::NONE) {
    robot_.stopImmediately(true);
    debug_.print("MAP,START,REJECT,REASON=");
    debug_.println(storageErrorReason_ == MapStorageErrorReason::STORAGE_INIT
                       ? "STORAGE_INIT"
                       : "STORAGE_ERROR");
    return false;
  }
  if (returnP0InProgress()) {
    logStartReject("RETURN_P0_ACTIVE");
    debug_.println("MAP,RETURN_P0,START,REJECT=RETURN_P0_ACTIVE");
    return false;
  }
  if (mode_ == MapControllerMode::SETTINGS) {
    if (initiator != MapMissionInitiator::PS2) {
      logStartReject("SETTINGS");
      return false;
    }
    saveSettingsAndExit();
    return true;
  }
  if (mode_ == MapControllerMode::HELP ||
      mode_ == MapControllerMode::DELETE_CONFIRM) {
    return false;
  }
  if (mode_ == MapControllerMode::TEACHING ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::CLOSED_CONFIRM) {
    logStartReject(mode_ == MapControllerMode::TEACHING
                       ? "TEACHING"
                       : mode_ == MapControllerMode::DELETE_CONFIRM
                           ? "DELETE_CONFIRM"
                           : "CLOSED_CONFIRM");
    return false;
  }
  if (initiator == MapMissionInitiator::PS2 &&
      mode_ == MapControllerMode::SAVED && homeContext_.valid &&
      !backP0UiDismissed_ && returnP0State_ != ReturnP0State::COMPLETE) {
    const char* backReason = nullptr;
    if (!backReadyP0Available(backReason)) {
      const char* reason = backReason != nullptr ? backReason : "UNAVAILABLE";
      robot_.stopImmediately(true);
      debug_.print("MAP,RETURN_P0,REJECT,REASON=");
      debug_.println(reason);
      logStartReject(reason);
      return false;
    }
    if (!requestReturnToP0(ReturnP0Source::PS2_START, backReason)) {
      const char* reason = backReason != nullptr ? backReason : "START";
      robot_.stopImmediately(true);
      debug_.print("MAP,RETURN_P0,REJECT,REASON=");
      debug_.println(reason);
      logStartReject(reason);
      return false;
    }
    missionInitiator_ = MapMissionInitiator::PS2;
    debug_.println("MAP,RETURN_P0,START,SOURCE=PS2_START");
    return true;
  }
  if (replayActive_ && mode_ != MapControllerMode::REPLAY_HOLD) {
    logStartReject("REPLAY_ACTIVE");
    return false;
  }
  if (mode_ == MapControllerMode::REPLAY_HOLD) {
    if (initiator != MapMissionInitiator::PS2) {
      logStartReject("AI_START_WHILE_HOLD");
      return false;
    }
    if (holdReason_ == MapHoldReason::OBSTACLE &&
        obstacleDetourPhase_ != ObstacleDetourPhase::IDLE) {
      debug_.print("OBS,DETOUR,START,REJECT=PHASE_");
      debug_.println(static_cast<unsigned>(obstacleDetourPhase_));
      logStartReject(obstacleDetourPhase_ == ObstacleDetourPhase::ABORTED
                         ? "DETOUR_ABORTED"
                         : "DETOUR_COMPLETE_HOLD");
      return false;
    }
    if (holdReason_ == MapHoldReason::OBSTACLE &&
        obstacleClassifier_.stable() &&
        (obstacleClassifier_.decision() == ObstacleDecision::AVOID_LEFT ||
         obstacleClassifier_.decision() == ObstacleDecision::AVOID_RIGHT)) {
      const char* detourReason = nullptr;
      if (armObstacleDetour(detourReason)) return true;
      debug_.print("OBS,DETOUR,NOT_ALLOWED,REASON=");
      debug_.println(detourReason != nullptr ? detourReason : "ENTRY_GATE");
      logStartReject(detourReason != nullptr ? detourReason
                                             : "DETOUR_NOT_ALLOWED");
      return false;
    }
    debug_.println("MAP,START,ACTION=RESUME");
    debug_.print("MAP,RESUME,REQUEST,WP=");
    debug_.print(static_cast<unsigned>(replayTargetIndex_));
    debug_.print(",GEN=");
    debug_.println(replayGeneration_);
    const char* rejectReason = nullptr;
    const bool resumed =
        resumeReplayFromHold(ReplayResumeSource::PS2_START, rejectReason);
    if (resumed) {
      debug_.print("MAP,RESUME,ACCEPT,WP=");
      debug_.print(static_cast<unsigned>(replayTargetIndex_));
      debug_.print(",GEN=");
      debug_.println(replayGeneration_);
      if (postTeachBackActive_) {
        debug_.print("MAP,BACK_P0,RESUME,WP=");
        debug_.println(static_cast<unsigned>(replayTargetIndex_));
      }
      if (routeMode_ == MapReplayMode::LOOP) {
        debug_.print("MAP,LOOP,RESUME,LAP=");
        debug_.print(replayLapCounter_);
        debug_.print(",WP=");
        debug_.println(static_cast<unsigned>(replayTargetIndex_));
      } else if (routeMode_ == MapReplayMode::RETURN) {
        debug_.print("MAP,RETURN,RESUME,PHASE=");
        debug_.print(returnPhaseName(replayReturnPhase_));
        debug_.print(",WP=");
        debug_.println(static_cast<unsigned>(replayTargetIndex_));
      } else if (routeMode_ == MapReplayMode::PING_PONG) {
        debug_.print("MAP,PING,RESUME,PHASE=");
        debug_.print(returnPhaseName(replayReturnPhase_));
        debug_.print(",CYCLE=");
        debug_.print(replayCycleCounter_ + 1U);
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
    return resumed;
  }
  debug_.println("MAP,START,ACTION=RUN");
  const char* reason = nullptr;
  const bool started = prepareReplay(reason, initiator);
  if (started) {
    missionInitiator_ = initiator;
    if (initiator == MapMissionInitiator::AI_VOICE) {
      // This is the only latch-clear boundary in Phase 1.
      autonomousResumeInhibited_ = false;
    }
    debug_.print("MAP,MISSION,INITIATOR=");
    debug_.println(MissionInitiatorName(missionInitiator_));
    debug_.print("MAP,START,ACCEPT,GEN=");
    debug_.println(replayGeneration_);
    if (routeMode_ == MapReplayMode::LOOP) {
      debug_.println("MAP,LOOP,START");
    } else if (routeMode_ == MapReplayMode::RETURN) {
      debug_.print("MAP,RETURN,START,POINTS=");
      debug_.println(static_cast<unsigned>(route_.header.waypointCount));
      debug_.println("MAP,RETURN,PHASE=OUT");
    } else if (routeMode_ == MapReplayMode::PING_PONG) {
      debug_.print("MAP,PING,START,CYCLE_TARGET=");
      if (loopTarget_ == MAP_LOOP_TARGET_INF) {
        debug_.println("INF");
      } else {
        debug_.println(static_cast<unsigned>(loopTarget_));
      }
      debug_.println("MAP,PING,PHASE=OUT,CYCLE=1");
    }
  } else {
    logStartReject(reason != nullptr ? reason : "PRECHECK");
  }
  return started;
}

bool MapController::requestRunMap(uint8_t slot, MapMissionInitiator initiator,
                                   const char*& reason) {
  reason = nullptr;
  if (slot != 1U && slot != 2U) {
    reason = "MAP_NOT_AVAILABLE";
    return false;
  }
  if (initiator == MapMissionInitiator::NONE) {
    reason = "INITIATOR";
    return false;
  }
  if (returnP0InProgress() || replayActive_ ||
      mode_ == MapControllerMode::TEACHING ||
      mode_ == MapControllerMode::SETTINGS ||
      mode_ == MapControllerMode::HELP ||
      mode_ == MapControllerMode::REPLAY_HOLD) {
    reason = "MAP_BUSY";
    return false;
  }
  if (static_cast<uint8_t>(selectedSlot_) != slot) {
    handleSlot(slot);
    if (static_cast<uint8_t>(selectedSlot_) != slot) {
      reason = "MAP_BUSY";
      return false;
    }
  }
  if (!loadSelected()) {
    reason = "MAP_NOT_AVAILABLE";
    return false;
  }
  if (!requestStart(initiator)) {
    reason = "MAP_COMMAND_REJECTED";
    return false;
  }
  if (initiator == MapMissionInitiator::AI_VOICE) {
    armAiRunHomeContextIfNeeded();
  }
  reason = "OK";
  return true;
}

bool MapController::requestReturnToP0(ReturnP0Source source,
                                       const char*& reason) {
  return requestReturnToP0Internal(source, reason);
}

void MapController::logReturnP0RejectSnapshot(ReturnP0Source source,
                                              const char* reason,
                                              const char* boundary) const {
  // One physical COM12 line.  Keep this read-only: it is deliberately emitted
  // before the caller's stop/invalidate/abort action so a rejected AI request
  // preserves the state that caused the rejection.
  debug_.print("MAP,RETURN_P0,REJECT_SNAPSHOT,REASON=");
  debug_.print(reason != nullptr ? reason : "REJECTED");
  debug_.print(",SOURCE=");
  debug_.print(ReturnP0SourceName(source));
  if (boundary != nullptr) {
    debug_.print(",BOUNDARY=");
    debug_.print(boundary);
  }
  debug_.print(",HOME_VALID=");
  debug_.print(homeContext_.valid ? 1 : 0);
  debug_.print(",HOME_SLOT=");
  debug_.print(static_cast<unsigned>(homeContext_.slot));
  debug_.print(",SELECTED_SLOT=");
  debug_.print(static_cast<unsigned>(selectedSlot_));
  debug_.print(",HOME_ROUTE_GEN=");
  debug_.print(homeContext_.routeGeneration);
  debug_.print(",ROUTE_GEN=");
  debug_.print(route_.header.generation);
  debug_.print(",HOME_ODOM_GEN=");
  debug_.print(homeContext_.odometryResetGeneration);
  debug_.print(",ODOM_GEN=");
  debug_.print(odometry_.resetGeneration());
  debug_.print(",HOME_HEADING_GEN=");
  debug_.print(homeContext_.headingResetGeneration);
  debug_.print(",HEADING_GEN=");
  debug_.print(robot_.headingResetGeneration());
  debug_.print(",LOADED=");
  debug_.print(loadedValid_ ? 1 : 0);
  debug_.print(",MODE=");
  debug_.print(static_cast<unsigned>(mode_));
  debug_.print(",REPLAY_ACTIVE=");
  debug_.print(replayActive_ ? 1 : 0);
  debug_.print(",RETURN_STATE=");
  debug_.print(static_cast<unsigned>(returnP0State_));
  debug_.print(",OWNER=");
  debug_.print(static_cast<unsigned>(robot_.motionOwner()));
  debug_.print(",MOTORS_STOPPED=");
  debug_.print(robot_.motorsStopped() ? 1 : 0);
  debug_.print(",AI_ACTIVE=");
  debug_.print(robot_.aiMotionActive() ? 1 : 0);
  debug_.print(",PS2_MOTION=");
  debug_.println((ps2_.motionCommandActive() || ps2_.state().r3) ? 1 : 0);
}

void MapController::logReturnP0TraceBase(const char* event,
                                         const Pose& live) const {
  const Pose& p0 = homeContext_.p0WorldPose;
  const float dx = live.xMm - p0.xMm;
  const float dy = live.yMm - p0.yMm;
  const float positionError = distanceMm(live.xMm, live.yMm, p0.xMm, p0.yMm);
  const float headingError = shortestDeltaDeg(p0.headingDeg, live.headingDeg);
  debug_.print("MAP,RETURN_P0,TRACE,EVENT=");
  debug_.print(event);
  debug_.print(",STATE=");
  debug_.print(returnP0StateName(returnP0State_));
  debug_.print(",LIVE_X=");
  debug_.print(live.xMm, 1);
  debug_.print(",LIVE_Y=");
  debug_.print(live.yMm, 1);
  debug_.print(",LIVE_H=");
  debug_.print(live.headingDeg, 2);
  debug_.print(",P0_X=");
  debug_.print(p0.xMm, 1);
  debug_.print(",P0_Y=");
  debug_.print(p0.yMm, 1);
  debug_.print(",P0_H=");
  debug_.print(p0.headingDeg, 2);
  debug_.print(",P0_DX=");
  debug_.print(dx, 1);
  debug_.print(",P0_DY=");
  debug_.print(dy, 1);
  debug_.print(",P0_POS_ERR=");
  debug_.print(positionError, 1);
  debug_.print(",P0_HEADING_ERR=");
  debug_.print(headingError, 2);
  debug_.print(",ROUTE_GEN=");
  debug_.print(route_.header.generation);
  debug_.print(",ODOM_GEN=");
  debug_.print(odometry_.resetGeneration());
  debug_.print(",HEADING_GEN=");
  debug_.print(robot_.headingResetGeneration());
}

void MapController::logReturnP0TraceMotion(const char* event,
                                           const Pose& live) const {
  logReturnP0TraceBase(event, live);
  debug_.print(",MOTOR_L=");
  debug_.print(robot_.currentLeftCommand());
  debug_.print(",MOTOR_R=");
  debug_.print(robot_.currentRightCommand());
  debug_.print(",LEFT_TICKS=");
  debug_.print(static_cast<long>(odometry_.data().leftTicks));
  debug_.print(",RIGHT_TICKS=");
  debug_.print(static_cast<long>(odometry_.data().rightTicks));
  debug_.print(",FUSED_HEADING=");
  debug_.print(fusion_.headingDeg(), 2);
  debug_.print(",ENCODER_HEALTH=");
  debug_.print(odometry_.healthText());
  debug_.print(",FUSION_HEALTH=");
  debug_.print(fusion_.healthText());
}

void MapController::logReturnP0TraceP0Error(const Pose& live) const {
  const Pose& p0 = homeContext_.p0WorldPose;
  debug_.print(",DX=");
  debug_.print(live.xMm - p0.xMm, 1);
  debug_.print(",DY=");
  debug_.print(live.yMm - p0.yMm, 1);
  debug_.print(",POS_ERR=");
  debug_.print(distanceMm(live.xMm, live.yMm, p0.xMm, p0.yMm), 1);
  debug_.print(",HEADING_ERR=");
  debug_.print(shortestDeltaDeg(p0.headingDeg, live.headingDeg), 2);
}

float MapController::diagnosticCrossTrackToSegment(
    const Pose& live, const Pose& segmentStart, const Pose& segmentEnd) {
  const float dx = segmentEnd.xMm - segmentStart.xMm;
  const float dy = segmentEnd.yMm - segmentStart.yMm;
  const float lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= 1.0e-3f) return distanceMm(
      live.xMm, live.yMm, segmentStart.xMm, segmentStart.yMm);
  float t = ((live.xMm - segmentStart.xMm) * dx +
             (live.yMm - segmentStart.yMm) * dy) / lengthSquared;
  t = constrain(t, 0.0f, 1.0f);
  return distanceMm(live.xMm, live.yMm, segmentStart.xMm + t * dx,
                    segmentStart.yMm + t * dy);
}

bool MapController::requestReturnToP0Internal(ReturnP0Source source,
                                              const char*& reason) {
  reason = nullptr;
  if (returnP0InProgress()) {
    reason = "RETURN_P0_ACTIVE";
    logReturnP0RejectSnapshot(source, reason);
    return false;
  }
  if (source == ReturnP0Source::NONE) {
    reason = "SOURCE";
    logReturnP0RejectSnapshot(source, reason);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  returnP0State_ = ReturnP0State::VALIDATE_HOME;
  if (!homeContext_.valid) {
    reason = "HOME_CONTEXT_INVALID";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    returnP0State_ = ReturnP0State::ABORTED;
    debug_.println("MAP,RETURN_P0,ABORT,REASON=HOME_CONTEXT_INVALID");
    return false;
  }
  if (selectedSlot_ != homeContext_.slot) {
    reason = "SLOT_CHANGED";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    invalidateHomeContext(reason);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (odometry_.resetGeneration() != homeContext_.odometryResetGeneration) {
    reason = "RESET_BOUNDARY";
    logReturnP0RejectSnapshot(source, reason, "ODOM");
    robot_.stopImmediately(true);
    invalidateHomeContext(reason);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (robot_.headingResetGeneration() != homeContext_.headingResetGeneration) {
    reason = "RESET_BOUNDARY";
    logReturnP0RejectSnapshot(source, reason, "HEADING");
    robot_.stopImmediately(true);
    invalidateHomeContext(reason);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (!loadedValid_ && !loadSelected()) {
    reason = "ROUTE_INVALID";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (route_.header.generation != homeContext_.routeGeneration) {
    reason = "ROUTE_CHANGED";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    invalidateHomeContext(reason);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (robot_.motionOwner() != MotionOwner::NONE &&
      robot_.motionOwner() != MotionOwner::REPLAY) {
    reason = "MOTION_OWNER";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }
  if (robot_.motionOwner() == MotionOwner::REPLAY || replayActive_ ||
      mode_ == MapControllerMode::REPLAY_HOLD) {
    robot_.stopImmediately(true);
  } else if (!robot_.motorsStopped() || robot_.aiMotionActive()) {
    reason = "MOTION_OWNER";
    logReturnP0RejectSnapshot(source, reason);
    robot_.stopImmediately(true);
    returnP0State_ = ReturnP0State::ABORTED;
    return false;
  }

  returnP0State_ = ReturnP0State::LOCATE_ON_ROUTE;
  RouteProjection projection;
  if (!locateRouteProjection(projection, reason)) {
    logReturnP0RejectSnapshot(source, reason != nullptr ? reason : "LOCATE");
    robot_.stopImmediately(true);
    returnP0State_ = ReturnP0State::ABORTED;
    debug_.print("MAP,RETURN_P0,ABORT,REASON=");
    debug_.println(reason != nullptr ? reason : "LOCATE");
    return false;
  }

  // A universal Return request owns the local MAP replay channel from this
  // point. The saved route and HomeContext remain untouched; only the old
  // operation/generation is invalidated.
  invalidatePostTeachBack("RETURN_P0");
  nextReplayGeneration();
  returnP0Generation_ = replayGeneration_;
  returnP0SegmentGeneration_ = 0U;
  returnP0Source_ = source;
  if (source == ReturnP0Source::AI_VOICE) {
    missionInitiator_ = MapMissionInitiator::AI_VOICE;
    // An accepted AI MAP request is the explicit boundary that clears the
    // previous autonomous-stop latch. PS2 requests never clear it.
    autonomousResumeInhibited_ = false;
  }
  returnP0Projection_ = projection;
  returnP0TargetIndex_ = projection.segmentStartIndex;
  returnP0SegmentStartIndex_ = projection.segmentStartIndex;
  returnP0ReacquireAttempts_ = 0U;
  returnP0PositionCorrectionAttempts_ = 0U;
  returnP0HeadingAttempts_ = 0U;
  returnP0TurnPending_ = false;
  returnP0HeldState_ = ReturnP0State::IDLE;
  returnP0HeldTargetIndex_ = 0U;
  returnP0HeldSegmentStartIndex_ = 0U;
  returnP0SettleSinceMs_ = 0U;
  replayOrigin_ = homeContext_.p0WorldPose;
  replayOriginValid_ = true;
  replayContextSlot_ = homeContext_.slot;
  replayOriginRouteGeneration_ = homeContext_.routeGeneration;
  replayOriginResetGeneration_ = homeContext_.odometryResetGeneration;
  replayOriginHeadingResetGeneration_ = homeContext_.headingResetGeneration;
  replayCurrentIndex_ = projection.segmentEndIndex;
  replayTargetIndex_ = returnP0TargetIndex_;
  replayDirection_ = -1;
  replayOperation_ = MapReplayOperation::NONE;
  replayResumeAllowed_ = false;
  replayHoldPoseValid_ = false;
  holdReason_ = MapHoldReason::NONE;
  replayReason_ = "RETURN_P0";
  replayActive_ = true;
  mode_ = MapControllerMode::REPLAY_CHECKED;
  statusDirty_ = true;
  Pose acceptedPose;
  if (readPose(acceptedPose)) {
    logReturnP0TraceBase("ACCEPT", acceptedPose);
    logReturnP0TraceP0Error(acceptedPose);
    debug_.print(",SOURCE=");
    debug_.println(ReturnP0SourceName(source));
    logReturnP0TraceBase("PROJECTION", acceptedPose);
    debug_.print(",SEG_START=");
    debug_.print(static_cast<unsigned>(projection.segmentStartIndex));
    debug_.print(",SEG_END=");
    debug_.print(static_cast<unsigned>(projection.segmentEndIndex));
    debug_.print(",T=");
    debug_.print(projection.t, 3);
    debug_.print(",PROJ_X=");
    debug_.print(projection.projectedPose.xMm, 1);
    debug_.print(",PROJ_Y=");
    debug_.print(projection.projectedPose.yMm, 1);
    debug_.print(",CROSS_TRACK=");
    debug_.print(projection.crossTrackMm, 1);
    debug_.print(",DIST_TO_PROJECTION=");
    debug_.print(projection.distanceMm, 1);
    debug_.println(",AMBIGUOUS=0");
  }
  debug_.print("MAP,RETURN_P0,REQUEST,SOURCE=");
  debug_.print(ReturnP0SourceName(source));
  debug_.print(",GEN=");
  debug_.print(returnP0Generation_);
  debug_.print(",SEG=");
  debug_.print(static_cast<unsigned>(projection.segmentStartIndex));
  debug_.print(",T=");
  debug_.print(projection.t, 3);
  debug_.print(",XT=");
  debug_.println(projection.crossTrackMm, 1);

  if (projection.crossTrackMm >
      static_cast<float>(MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM)) {
    returnP0State_ = ReturnP0State::REACQUIRE_ROUTE;
    if (!startReturnReacquire()) {
      reason = "REACQUIRE_START";
      logReturnP0RejectSnapshot(source, reason);
      abortReturnToP0(reason);
      return false;
    }
  } else {
    returnP0State_ = ReturnP0State::RETURN_WAYPOINT;
    if (!startReturnWaypoint()) {
      reason = "RETURN_START";
      logReturnP0RejectSnapshot(source, reason);
      abortReturnToP0(reason);
      return false;
    }
  }
  return true;
}

void MapController::handleTriangle() {
  if (storageErrorReason_ != MapStorageErrorReason::NONE) {
    debug_.print("MAP,TEACH,REJECT,REASON=");
    debug_.println(storageErrorReasonName(storageErrorReason_));
    return;
  }
  if (mode_ == MapControllerMode::SETTINGS ||
      mode_ == MapControllerMode::HELP) {
    if (mode_ == MapControllerMode::HELP) {
      helpPage_ = static_cast<uint8_t>((helpPage_ + 1U) % 3U);
    }
    enterHelp();
    return;
  }
  if (replayActive_ || mode_ == MapControllerMode::REPLAY_HOLD ||
      mode_ == MapControllerMode::DELETE_CONFIRM ||
      mode_ == MapControllerMode::CLOSED_CONFIRM) {
    return;
  }
  if (mode_ == MapControllerMode::TEACHING) {
    markManualWaypoint();
    return;
  }
  invalidatePostTeachBack("NEW_TEACH");
  (void)beginTeach();
}

void MapController::handleCircle() {
  debug_.print("MAP,CIRCLE,HANDLE,MODE=");
  debug_.println(static_cast<unsigned>(mode_));
  if (storageErrorReason_ != MapStorageErrorReason::NONE) {
    debug_.print("MAP,CIRCLE,REJECT,REASON=");
    debug_.println(storageErrorReasonName(storageErrorReason_));
    return;
  }
  if (mode_ == MapControllerMode::SETTINGS) {
    if (settingsCanDelete()) {
      mode_ = MapControllerMode::DELETE_CONFIRM;
      debug_.println("MAP,SETTINGS,DELETE_CONFIRM");
      statusDirty_ = true;
    }
    return;
  }
  if (mode_ == MapControllerMode::HELP) return;
  if (mode_ == MapControllerMode::TEACHING) {
    debug_.println("MAP,CIRCLE,ACTION=FINISH_TEACH");
    requestTeachFinish();
    return;
  }
  if (mode_ == MapControllerMode::DELETE_CONFIRM) {
    if (!loadedValid_ || replayActive_ || teachFinishPending_ ||
        robot_.aiMotionActive() || !robot_.motorsStopped() ||
        robot_.motionOwner() != MotionOwner::NONE) {
      debug_.println("MAP,CIRCLE,REJECT,REASON=DELETE_SAFETY");
      return;
    }
    debug_.println("MAP,CIRCLE,ACTION=DELETE_CONFIRM");
    deletePending_ = true;
    mode_ = MapControllerMode::READY;
    leaveMapUiCapture();
    return;
  }
  if (replayActive_) return;
  if (storeState_ == MapStoreState::SAVED && !loadedValid_) {
    if (!loadSelected()) return;
  }
  if (storeState_ != MapStoreState::SAVED || !loadedValid_) return;
  // MODE is now a transactional Settings item. Keeping the main page free of
  // an undocumented mode toggle prevents an accidental route rewrite.
  debug_.println("MAP,CIRCLE,REJECT,REASON=SETTINGS_REQUIRED");
}

void MapController::handleSquare(bool longPress) {
  if (longPress) {
    // Destructive deletion is available only from the explicit Settings
    // DELETE MAP item. A long SQUARE on the main MAP page is inert.
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
  } else if (homeContext_.valid && !backP0UiDismissed_ &&
             returnP0State_ != ReturnP0State::COMPLETE &&
             !returnP0InProgress()) {
    const char* backReason = nullptr;
    if (backReadyP0Available(backReason)) {
      backP0UiDismissed_ = true;
      invalidatePostTeachBack("DISMISS");
      debug_.println("MAP,RETURN_P0,DISMISS");
      statusDirty_ = true;
    }
  } else if (postTeachBack_.valid && !postTeachBackActive_) {
    debug_.println("MAP,BACK_P0,DISMISS");
    invalidatePostTeachBack("DISMISS");
  } else if (mode_ == MapControllerMode::HELP) {
    leaveHelp();
  } else if (mode_ == MapControllerMode::SETTINGS ||
             mode_ == MapControllerMode::DELETE_CONFIRM) {
    cancelSettings();
  } else if (storageErrorReason_ != MapStorageErrorReason::NONE &&
             storageErrorReason_ != MapStorageErrorReason::STORAGE_INIT &&
             loadedValid_) {
    // X BACK exists only when the old Flash route was restored in RAM. It
    // acknowledges the runtime error without deleting or rewriting Flash.
    robot_.stopImmediately(true);
    storageErrorReason_ = MapStorageErrorReason::NONE;
    storeState_ = MapStoreState::SAVED;
    mode_ = MapControllerMode::SAVED;
    debug_.println("MAP,STORAGE_ERROR,ACK=BACK,OLD_ROUTE=1");
    leaveMapUiCapture();
    statusDirty_ = true;
  } else if (replayActive_) {
    // X-down is the immediate safety stop and enters a resumable USER HOLD.
    if (obstacleDetourInProgress()) {
      abortObstacleDetour("USER_STOP");
    } else if (!obstacleDetourContextActive()) {
      if (returnP0InProgress()) {
        // Return-to-P0 has no implicit resume contract in Phase 2. Preserve
        // Home, but make a manual stop a non-resumable hold.
        returnP0State_ = ReturnP0State::HOLD;
      }
      enterReplayHold(MapHoldReason::USER, true);
    }
  } else if (mode_ == MapControllerMode::REPLAY_HOLD) {
    // A new short X while already held leaves the hold in place. The long
    // event for this press is handled separately by handleCrossLong().
  } else if (mode_ == MapControllerMode::DELETE_CONFIRM) {
    mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                                : MapControllerMode::READY;
  }
}

void MapController::handleCrossLong() {
  if (mode_ == MapControllerMode::HELP) {
    leaveHelp();
    return;
  }
  if (mode_ == MapControllerMode::SETTINGS ||
      mode_ == MapControllerMode::DELETE_CONFIRM) {
    cancelSettings();
    return;
  }
  if (homeContext_.valid && !backP0UiDismissed_ &&
      returnP0State_ != ReturnP0State::COMPLETE &&
      !returnP0InProgress()) {
    const char* backReason = nullptr;
    if (backReadyP0Available(backReason)) {
      backP0UiDismissed_ = true;
      invalidatePostTeachBack("DISMISS_LONG");
      debug_.println("MAP,RETURN_P0,DISMISS_LONG");
      statusDirty_ = true;
    }
    return;
  }
  if (postTeachBack_.valid && !postTeachBackActive_) {
    debug_.println("MAP,BACK_P0,DISMISS_LONG");
    invalidatePostTeachBack("DISMISS_LONG");
    return;
  }
  if (obstacleDetourInProgress()) {
    // A long X is still a highest-priority production stop. Keep the detour
    // generation fenced before the existing cancel path is considered.
    abortObstacleDetour("X_LONG");
    return;
  }
  if (returnP0InProgress()) {
    cancelReplay("X_LONG");
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
  invalidatePostTeachBack("ROUTE_LOAD");
  if (!store_.load(selectedSlot_, route_)) {
    invalidateHomeContext("STORAGE_INVALID");
    loadedValid_ = false;
    storeState_ = MapStoreState::INVALID;
    mode_ = MapControllerMode::READY;
    return false;
  }
  const char* reason = nullptr;
  if (!validateRoute(route_, reason)) {
    invalidateHomeContext("STORAGE_INVALID");
    loadedValid_ = false;
    storeState_ = MapStoreState::INVALID;
    mode_ = MapControllerMode::READY;
    return false;
  }
  loadedValid_ = true;
  if (homeContext_.valid &&
      (homeContext_.slot != selectedSlot_ ||
       homeContext_.routeGeneration != route_.header.generation)) {
    invalidateHomeContext("ROUTE_GENERATION");
  }
  storeState_ = MapStoreState::SAVED;
  const MapRouteType storedType =
      static_cast<MapRouteType>(route_.header.routeType);
  const MapReplayMode storedMode =
      static_cast<MapReplayMode>(route_.header.replayMode);
  routeType_ = MapRouteType::OPEN;
  const uint8_t encodedTarget =
      mapLoopTargetFromReserved(route_.header.reserved);
  applyStoredSettings(storedType, storedMode,
                      mapShuttleRepeatFromReserved(route_.header.reserved),
                      encodedTarget);
  // Normalize the compatibility representation in RAM. Flash remains in the
  // V5.2.9-readable encoding unless a real setting change is saved.
  route_.header.routeType = static_cast<uint8_t>(MapRouteType::OPEN);
  route_.header.replayMode = static_cast<uint8_t>(routeMode_);
  route_.header.routeLengthMm = routeLengthMm(route_);
  replaySpeed_ = mapReplaySpeedFromReserved(route_.header.reserved);
  storageErrorReason_ = MapStorageErrorReason::NONE;
  mode_ = MapControllerMode::SAVED;
  return true;
}

bool MapController::beginTeach() {
  if (storageErrorReason_ != MapStorageErrorReason::NONE) {
    debug_.print("MAP,TEACH,REJECT,REASON=");
    debug_.println(storageErrorReasonName(storageErrorReason_));
    return false;
  }
  if (replayActive_ || !robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.brakeEnabled() || ps2_.motionCommandActive()) {
    return false;
  }
  invalidatePostTeachBack("NEW_TEACH");
  invalidateHomeContext("NEW_TEACH");
  teachOldRouteAvailable_ = storeState_ == MapStoreState::SAVED &&
                            (loadedValid_ ||
                             store_.metadata(selectedSlot_).state ==
                                 MapStoreState::SAVED);
  Pose origin;
  if (!readPose(origin)) return false;
  teachOrigin_ = origin;
  teachOriginValid_ = true;
  teachOriginResetGeneration_ = odometry_.resetGeneration();
  teachOriginHeadingResetGeneration_ = robot_.headingResetGeneration();
  pendingHomeOrigin_ = origin;
  pendingHomeResetGeneration_ = teachOriginResetGeneration_;
  pendingHomeHeadingResetGeneration_ = teachOriginHeadingResetGeneration_;
  pendingHomeContextValid_ = true;
  teachMode_ = MapTeachMode::MANUAL_KEYFRAME;
  route_ = {};
  route_.header.waypointCount = 0U;
  routeType_ = MapRouteType::OPEN;
  userMode_ = MapUserMode::ONCE;
  routeMode_ = MapReplayMode::ONCE;
  replaySpeed_ = MAP_REPLAY_SPEED_DEFAULT;
  loopTarget_ = MAP_LOOP_TARGET_MIN;
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
  pendingTeachBackValid_ = false;
  pendingHomeContextValid_ = false;
  teachFinishPending_ = false;
  teachOldRouteAvailable_ = false;
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                              : MapControllerMode::READY;
}

void MapController::stagePostTeachBackSnapshot() {
  if (!teachOriginValid_ || teachMode_ != MapTeachMode::MANUAL_KEYFRAME) {
    pendingTeachBackValid_ = false;
    return;
  }
  pendingTeachBackOrigin_ = teachOrigin_;
  pendingTeachBackResetGeneration_ = teachOriginResetGeneration_;
  pendingTeachBackHeadingResetGeneration_ = teachOriginHeadingResetGeneration_;
  pendingTeachBackValid_ = true;
}

void MapController::armPostTeachBackAfterSave() {
  if (!pendingTeachBackValid_ || route_.header.waypointCount < 2U) {
    pendingTeachBackValid_ = false;
    return;
  }
  postTeachBack_ = {};
  postTeachBack_.valid = true;
  postTeachBack_.slot = selectedSlot_;
  postTeachBack_.endpointIndex = route_.header.waypointCount - 1U;
  postTeachBack_.routeGeneration = route_.header.generation;
  postTeachBack_.odometryResetGeneration = pendingTeachBackResetGeneration_;
  postTeachBack_.headingResetGeneration =
      pendingTeachBackHeadingResetGeneration_;
  postTeachBack_.teachOrigin = pendingTeachBackOrigin_;
  pendingTeachBackValid_ = false;
  postTeachBackActive_ = false;
  debug_.print("MAP,BACK_P0,AVAILABLE,SLOT=");
  debug_.print(static_cast<unsigned>(selectedSlot_));
  debug_.print(",FROM=");
  debug_.print(static_cast<unsigned>(postTeachBack_.endpointIndex));
  debug_.print(",GEN=");
  debug_.println(postTeachBack_.routeGeneration);
  statusDirty_ = true;
}

void MapController::armHomeContextAfterSave() {
  if (!pendingHomeContextValid_ || route_.header.waypointCount < 2U ||
      route_.header.generation == 0U) {
    pendingHomeContextValid_ = false;
    return;
  }
  homeContext_ = {};
  homeContext_.valid = true;
  homeContext_.slot = selectedSlot_;
  homeContext_.routeGeneration = route_.header.generation;
  homeContext_.odometryResetGeneration = pendingHomeResetGeneration_;
  homeContext_.headingResetGeneration = pendingHomeHeadingResetGeneration_;
  homeContext_.p0WorldPose = pendingHomeOrigin_;
  backP0UiDismissed_ = false;
  returnP0State_ = ReturnP0State::IDLE;
  returnP0Source_ = ReturnP0Source::NONE;
  pendingHomeContextValid_ = false;
  debug_.print("MAP,HOME,ARM,SLOT=");
  debug_.print(static_cast<unsigned>(homeContext_.slot));
  debug_.print(",GEN=");
  debug_.print(homeContext_.routeGeneration);
  debug_.print(",X=");
  debug_.print(homeContext_.p0WorldPose.xMm, 1);
  debug_.print(",Y=");
  debug_.print(homeContext_.p0WorldPose.yMm, 1);
  debug_.print(",H=");
  debug_.println(homeContext_.p0WorldPose.headingDeg, 1);
}

void MapController::armAiRunHomeContextIfNeeded() {
  // Normal replay already binds route-local P0 to replayOrigin_.  Reuse that
  // exact frame for a Voice Return-P0 command after boot, without persisting
  // a new HOME record or altering the established post-Teach PS2 flow.
  if (homeContext_.valid || !replayActive_ || !replayOriginValid_ ||
      route_.header.waypointCount < 2U || route_.header.generation == 0U) {
    return;
  }
  homeContext_ = {};
  homeContext_.valid = true;
  homeContext_.slot = selectedSlot_;
  homeContext_.routeGeneration = route_.header.generation;
  homeContext_.odometryResetGeneration = odometry_.resetGeneration();
  homeContext_.headingResetGeneration = robot_.headingResetGeneration();
  homeContext_.p0WorldPose = replayOrigin_;
  // This session context is for authenticated AI Return-P0 only.  It must
  // not advertise or arm the physical PS2 BACK-P0 action that is reserved
  // for the existing post-Teach workflow.
  backP0UiDismissed_ = true;
  returnP0State_ = ReturnP0State::IDLE;
  returnP0Source_ = ReturnP0Source::NONE;
  debug_.print("MAP,HOME,ARM,REASON=AI_RUN,SLOT=");
  debug_.print(static_cast<unsigned>(homeContext_.slot));
  debug_.print(",GEN=");
  debug_.println(homeContext_.routeGeneration);
}

void MapController::invalidateHomeContext(const char* reason) {
  const bool hadContext = homeContext_.valid || pendingHomeContextValid_;
  homeContext_ = {};
  pendingHomeOrigin_ = {};
  pendingHomeResetGeneration_ = 0U;
  pendingHomeHeadingResetGeneration_ = 0U;
  pendingHomeContextValid_ = false;
  if (hadContext) {
    debug_.print("MAP,HOME,INVALIDATE,REASON=");
    debug_.println(reason != nullptr ? reason : "UNKNOWN");
  }
}

void MapController::invalidatePostTeachBack(const char* reason) {
  const bool hadContext = postTeachBack_.valid || postTeachBackActive_ ||
                          pendingTeachBackValid_;
  postTeachBack_ = {};
  postTeachBackActive_ = false;
  postTeachBackComplete_ = false;
  pendingTeachBackValid_ = false;
  if (hadContext) {
    debug_.print("MAP,BACK_P0,INVALIDATE,REASON=");
    debug_.println(reason != nullptr ? reason : "UNKNOWN");
  }
  statusDirty_ = true;
}

bool MapController::postTeachBackRejectShouldInvalidate(
    const char* reason) const {
  if (reason == nullptr) return true;
  // These failures describe a temporary runtime precondition. Keep the
  // Teach snapshot so the user can correct the condition and press START
  // again; nothing is retried automatically.
  const char* transientReasons[] = {
      "NOT_MAP_PAGE", "PS2_NOT_NEUTRAL", "BRAKE", "ODOMETRY", "HEADING",
      "OBSTACLE_SENSOR", "OBSTACLE_NOT_CLEAR", "POSE"};
  for (const char* transient : transientReasons) {
    if (strcmp(reason, transient) == 0) return false;
  }
  return true;
}

bool MapController::postTeachBackAvailable(const char*& reason) const {
  reason = nullptr;
  if (!display_.isMapPage()) {
    reason = "NOT_MAP_PAGE";
    return false;
  }
  if (!postTeachBack_.valid || postTeachBackActive_) {
    reason = "NOT_AVAILABLE";
    return false;
  }
  if (mode_ != MapControllerMode::SAVED || !loadedValid_ ||
      storeState_ != MapStoreState::SAVED) {
    reason = "STATE";
    return false;
  }
  if (selectedSlot_ != postTeachBack_.slot) {
    reason = "SLOT_CHANGED";
    return false;
  }
  if (route_.header.waypointCount < 2U ||
      postTeachBack_.endpointIndex != route_.header.waypointCount - 1U) {
    reason = "ROUTE_POINTS";
    return false;
  }
  if (route_.header.generation != postTeachBack_.routeGeneration) {
    reason = "ROUTE_CHANGED";
    return false;
  }
  if (odometry_.resetGeneration() != postTeachBack_.odometryResetGeneration) {
    reason = "RESET_BOUNDARY";
    return false;
  }
  if (robot_.headingResetGeneration() != postTeachBack_.headingResetGeneration) {
    reason = "HEADING_RESET_BOUNDARY";
    return false;
  }
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    reason = "MOTION_OWNER";
    return false;
  }
  if (robot_.brakeEnabled()) {
    reason = "BRAKE";
    return false;
  }
  const uint32_t now = millis();
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
  Pose current;
  if (!readPose(current)) {
    reason = "POSE";
    return false;
  }
  const Pose endpoint = routePointWorldFromOrigin(
      postTeachBack_.teachOrigin, postTeachBack_.endpointIndex);
  if (distanceMm(current.xMm, current.yMm, endpoint.xMm, endpoint.yMm) >
      kReplayPoseHoldToleranceMm) {
    reason = "NOT_AT_ENDPOINT";
    return false;
  }
  reason = "OK";
  return true;
}

bool MapController::backReadyP0Available(const char*& reason) const {
  reason = nullptr;
  if (!display_.isMapPage()) {
    reason = "NOT_MAP_PAGE";
    return false;
  }
  if (!homeContext_.valid) {
    reason = "HOME_CONTEXT_INVALID";
    return false;
  }
  if (backP0UiDismissed_) {
    reason = "DISMISSED";
    return false;
  }
  if (returnP0InProgress()) {
    reason = "RETURN_P0_ACTIVE";
    return false;
  }
  if (returnP0State_ == ReturnP0State::COMPLETE) {
    reason = "RETURN_P0_COMPLETE";
    return false;
  }
  if (mode_ != MapControllerMode::SAVED || !loadedValid_ ||
      storeState_ != MapStoreState::SAVED) {
    reason = "STATE";
    return false;
  }
  if (selectedSlot_ != homeContext_.slot) {
    reason = "SLOT_CHANGED";
    return false;
  }
  if (route_.header.generation != homeContext_.routeGeneration) {
    reason = "ROUTE_CHANGED";
    return false;
  }
  if (route_.header.waypointCount < 2U ||
      (route_.waypoints[0].flags & MAP_WP_START) == 0U) {
    reason = "ROUTE_POINTS";
    return false;
  }
  if (odometry_.resetGeneration() != homeContext_.odometryResetGeneration) {
    reason = "RESET_BOUNDARY";
    return false;
  }
  if (robot_.headingResetGeneration() != homeContext_.headingResetGeneration) {
    reason = "HEADING_RESET_BOUNDARY";
    return false;
  }
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    reason = "MOTION_OWNER";
    return false;
  }
  if (robot_.brakeEnabled()) {
    reason = "BRAKE";
    return false;
  }
  const uint32_t now = millis();
  if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
      ps2_.motionCommandActive() || ps2_.state().r3) {
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
  if (!ultrasonic_.isFresh() || !ultrasonic_.healthy() ||
      ultrasonic_.overallZone() != ObstacleZone::CLEAR) {
    reason = "OBSTACLE_SENSOR";
    return false;
  }
  reason = "OK";
  return true;
}

bool MapController::startPostTeachBack(const char*& reason) {
  if (!postTeachBackAvailable(reason)) return false;
  if (!replayPrecheck(route_, reason, MapMissionInitiator::PS2)) return false;

  replayOrigin_ = postTeachBack_.teachOrigin;
  replayOriginValid_ = true;
  replayRealignReason_ = ReplayRealignReason::NONE;
  replayArrivalHeadingViolationSinceMs_ = 0U;
  replayArrivalTurnPending_ = false;
  replayArrivalTurnWaypoint_ = 0U;
  replayArrivalTurnAttempts_ = 0U;
  replayContextSlot_ = postTeachBack_.slot;
  replayOriginResetGeneration_ = postTeachBack_.odometryResetGeneration;
  replayOriginHeadingResetGeneration_ = postTeachBack_.headingResetGeneration;
  replayOriginRouteGeneration_ = postTeachBack_.routeGeneration;
  replayCurrentIndex_ = postTeachBack_.endpointIndex;
  replayTargetIndex_ = postTeachBack_.endpointIndex - 1U;
  replayDirection_ = -1;
  replayReturned_ = true;
  replayReturnPhase_ = ReplayReturnPhase::INBOUND;
  replayLapCounter_ = 0U;
  replayCycleCounter_ = 0U;
  replayTargetDistanceMm_ = 0U;
  replayTargetDeg_ = 0;
  replayGuideBearingDeg_ = 0.0f;
  replayTravelMm_ = 0U;
  replayErrorMm_ = 0U;
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayReason_ = "BACK_P0";
  replayResumeAllowed_ = false;
  replayHoldPoseValid_ = false;
  holdReason_ = MapHoldReason::NONE;
  postTeachBackActive_ = true;
  replayActive_ = true;
  replayOperation_ = MapReplayOperation::NONE;
  mode_ = MapControllerMode::REPLAY_CHECKED;
  statusDirty_ = true;
  debug_.print("MAP,BACK_P0,START,FROM=");
  debug_.print(static_cast<unsigned>(replayCurrentIndex_));
  debug_.print(",TO=");
  debug_.println(static_cast<unsigned>(replayTargetIndex_));
  return true;
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
    // Manual keyframes are authoritative. The saved record is always the
    // canonical open geometry; CLOSED/LOOP is selected later in Settings.
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
    debug_.println(",TYPE=OPEN_CANONICAL");
    closeCandidateDistanceMm_ = static_cast<uint32_t>(lroundf(closeDistance));
    closeCandidateHeadingDeg_ = static_cast<int16_t>(lroundf(closeHeading));
    routeType_ = MapRouteType::OPEN;
    userMode_ = MapUserMode::ONCE;
    routeMode_ = MapReplayMode::ONCE;
    route_.header.routeType = static_cast<uint8_t>(routeType_);
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    updateRouteHeaderForSave(route_);
    storeState_ = MapStoreState::EMPTY;
    loadedValid_ = false;
    stagePostTeachBackSnapshot();
    teachOriginValid_ = false;
    teachFinishPending_ = false;
    mode_ = MapControllerMode::READY;
    debug_.print("MAP,TEACH,SAVED_PENDING,POINTS=");
    debug_.print(static_cast<unsigned>(count));
    debug_.println(",MODE=ONCE");
    savePending_ = true;
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
    // Optimizers may still use the compatibility route type while cleaning,
    // but the persisted route is normalized below to OPEN geometry.
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
    if (autoClosed || closedCandidate) {
      closeCandidateDistanceMm_ = static_cast<uint32_t>(lroundf(closeDistance));
      closeCandidateHeadingDeg_ = static_cast<int16_t>(lroundf(closeHeading));
    }
    return queueTeachSave(MapRouteType::OPEN, MapControllerMode::TEACHING);
  }
}

bool MapController::queueTeachSave(MapRouteType type,
                                   MapControllerMode failureMode) {
  (void)type;
  const MapRouteType previousType = routeType_;
  const MapReplayMode previousMode = routeMode_;
  routeType_ = MapRouteType::OPEN;
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
    pendingTeachBackValid_ = false;
    mode_ = failureMode;
    statusDirty_ = true;
    return false;
  }
  stagePostTeachBackSnapshot();
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
  // This is always the canonical open geometry length. Storage compatibility
  // adds the logical Pn->P0 edge only for persisted CLOSED/LOOP records.
  return total <= 0.0f ? 0U : static_cast<uint32_t>(lroundf(total));
}

uint32_t MapController::persistedRouteLengthMm(
    const MapRouteData& route, MapRouteType persistedType) const {
  const uint32_t baseLength = routeLengthMm(route);
  if (persistedType != MapRouteType::CLOSED ||
      route.header.waypointCount < 2U) {
    return baseLength;
  }
  const uint16_t last = route.header.waypointCount - 1U;
  const float closure = distanceMm(
      static_cast<float>(route.waypoints[last].xMm),
      static_cast<float>(route.waypoints[last].yMm),
      static_cast<float>(route.waypoints[0].xMm),
      static_cast<float>(route.waypoints[0].yMm));
  if (closure <= kClosedClosureSkipDistanceMm) return baseLength;
  return baseLength + static_cast<uint32_t>(lroundf(closure));
}

MapRouteType MapController::persistedRouteType(MapReplayMode mode) {
  return IsClosingMode(mode) ? MapRouteType::CLOSED : MapRouteType::OPEN;
}

MapReplayMode MapController::persistedReplayMode(MapReplayMode mode) {
  return mode == MapReplayMode::CLOSED ? MapReplayMode::ONCE : mode;
}

void MapController::updateRouteHeaderForSave(MapRouteData& route,
                                             MapUserMode userMode,
                                             uint8_t repeatTarget) const {
  const MapRouteType storedType = userMode == MapUserMode::LOOP
                                      ? MapRouteType::CLOSED
                                      : MapRouteType::OPEN;
  MapReplayMode storedMode = MapReplayMode::ONCE;
  if (userMode == MapUserMode::SHUTTLE) {
    // New records deliberately use OPEN+RETURN for every shuttle count. The
    // marker bit makes multi-cycle shuttle records degrade safely to one
    // RETURN on V5.2.10 instead of becoming legacy infinite PING_PONG.
    storedMode = MapReplayMode::RETURN;
  } else if (userMode == MapUserMode::LOOP &&
             repeatTarget != MAP_LOOP_TARGET_MIN) {
    storedMode = MapReplayMode::LOOP;
  }
  route.header.slot = static_cast<uint8_t>(selectedSlot_);
  route.header.waypointCount =
      constrain(route.header.waypointCount, 0U, STM32_MAP_MAX_WAYPOINTS);
  route.header.payloadBytes = static_cast<uint16_t>(
      route.header.waypointCount * sizeof(MapWaypoint));
  route.header.routeType = static_cast<uint8_t>(storedType);
  route.header.replayMode = static_cast<uint8_t>(storedMode);
  route.header.routeLengthMm = persistedRouteLengthMm(route, storedType);
  route.header.reserved = mapReplaySpeedToReserved(
      route.header.reserved, replaySpeed_);
  route.header.reserved = mapLoopTargetToReserved(
      route.header.reserved, repeatTarget);
  route.header.reserved = mapShuttleRepeatToReserved(
      route.header.reserved,
      userMode == MapUserMode::SHUTTLE && repeatTarget != MAP_LOOP_TARGET_MIN);
}

void MapController::updateRouteHeaderForSave(MapRouteData& route) const {
  updateRouteHeaderForSave(route, userMode_, loopTarget_);
}

void MapController::normalizeRouteForRuntime(MapRouteData& route,
                                             MapReplayMode runtimeMode) const {
  route.header.routeType = static_cast<uint8_t>(MapRouteType::OPEN);
  route.header.replayMode = static_cast<uint8_t>(runtimeMode);
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
      !IsReplayModeValueValid(mode) ||
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
  const MapReplayMode effectiveMode = EffectiveReplayMode(type, mode);
  if (type == MapRouteType::CLOSED || IsClosingMode(effectiveMode)) {
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
  bool lengthValid = approximatelyEqual(
      static_cast<float>(computed), static_cast<float>(route.header.routeLengthMm),
      50.0f);
  // Accept the old CLOSED record length (which included Pn->P0) during load.
  // No new save writes that representation.
  if (!lengthValid && type == MapRouteType::CLOSED) {
    const float closure = distanceMm(
        static_cast<float>(route.waypoints[count - 1U].xMm),
        static_cast<float>(route.waypoints[count - 1U].yMm),
        static_cast<float>(route.waypoints[0].xMm),
        static_cast<float>(route.waypoints[0].yMm));
    const uint32_t legacyLength = static_cast<uint32_t>(lroundf(
        static_cast<float>(computed) +
        (closure > kClosedClosureSkipDistanceMm ? closure : 0.0f)));
    lengthValid = approximatelyEqual(
        static_cast<float>(legacyLength),
        static_cast<float>(route.header.routeLengthMm), 50.0f);
  }
  if (computed == 0U || !lengthValid) {
    reason = "LENGTH";
    return false;
  }
  reason = "OK";
  return true;
}

bool MapController::hasValidClosingEdge(const MapRouteData& route,
                                        const char*& reason) const {
  const uint16_t count = route.header.waypointCount;
  if (count < 3U) {
    reason = "CLOSED_POINTS";
    return false;
  }
  const float closure = distanceMm(
      static_cast<float>(route.waypoints[count - 1U].xMm),
      static_cast<float>(route.waypoints[count - 1U].yMm),
      static_cast<float>(route.waypoints[0].xMm),
      static_cast<float>(route.waypoints[0].yMm));
  if (closure > kMaximumSegmentMm) {
    reason = "CLOSURE_TOO_LONG";
    return false;
  }
  if (closure > kClosedClosureSkipDistanceMm && closure < kMinimumSegmentMm) {
    reason = "CLOSURE_TOO_SHORT";
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

bool MapController::prepareReplay(const char*& rejectReason,
                                  MapMissionInitiator initiator) {
  rejectReason = nullptr;
  invalidatePostTeachBack("NORMAL_REPLAY");
  if (!loadSelected()) {
    rejectReason = "NOT_SAVED";
    return false;
  }
  const char* reason = nullptr;
  if (!replayPrecheck(route_, reason, initiator)) {
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
  replayArrivalHeadingViolationSinceMs_ = 0U;
  replayArrivalTurnPending_ = false;
  replayArrivalTurnWaypoint_ = 0U;
  replayArrivalTurnAttempts_ = 0U;
  replayContextSlot_ = selectedSlot_;
  replayOriginResetGeneration_ = odometry_.resetGeneration();
  replayOriginHeadingResetGeneration_ = robot_.headingResetGeneration();
  replayCurrentIndex_ = 0U;
  replayTargetIndex_ = 1U;
  replayDirection_ = 1;
  replayReturned_ = false;
  replayReturnPhase_ = (routeMode_ == MapReplayMode::RETURN ||
                        routeMode_ == MapReplayMode::PING_PONG)
                           ? ReplayReturnPhase::OUTBOUND
                           : ReplayReturnPhase::NONE;
  replayLapCounter_ = 0U;
  replayCycleCounter_ = 0U;
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
                                   const char*& reason,
                                   MapMissionInitiator initiator) const {
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
  if (initiator == MapMissionInitiator::PS2) {
    if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
        ps2_.motionCommandActive()) {
      reason = "PS2_NOT_NEUTRAL";
      return false;
    }
  } else if (ps2_.motionCommandActive() || ps2_.state().r3) {
    // AI may start without a fresh receiver frame, but never while the
    // operator has taken over the chassis or asserted the PS2 stop boundary.
    reason = "PS2_TAKEOVER";
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

bool MapController::obstacleDetourContextActive() const {
  return obstacleDetourPhase_ != ObstacleDetourPhase::IDLE;
}

bool MapController::obstacleDetourInProgress() const {
  return obstacleDetourPhase_ != ObstacleDetourPhase::IDLE &&
         obstacleDetourPhase_ != ObstacleDetourPhase::COMPLETE_HOLD &&
         obstacleDetourPhase_ != ObstacleDetourPhase::ABORTED;
}

bool MapController::obstacleDetourSensorsReady() const {
  // Directional detour is intentionally unavailable with the single centred
  // SR04. Normal MAP obstacle STOP/HOLD/resume continues through the
  // aggregate ultrasonic safety gate.
  if (!ultrasonic_.directionalSensingAvailable()) return false;
  const auto valid = [](const UltrasonicReading& reading) {
    return reading.fresh && reading.health == SensorHealth::HEALTHY &&
           reading.valid && reading.echoValid && isfinite(reading.distanceCm) &&
           reading.distanceCm >= ULTRASONIC_MIN_CM &&
           reading.distanceCm <= ULTRASONIC_MAX_CM &&
           reading.zone != ObstacleZone::UNKNOWN;
  };
  const UltrasonicReading& left = ultrasonic_.frontLeft();
  const UltrasonicReading& right = ultrasonic_.frontRight();
  return valid(left) && valid(right) && left.zone != ObstacleZone::EMERGENCY &&
         right.zone != ObstacleZone::EMERGENCY;
}

bool MapController::obstacleDetourPathClear() const {
  const UltrasonicReading& left = ultrasonic_.frontLeft();
  const UltrasonicReading& right = ultrasonic_.frontRight();
  const bool strictClear =
      obstacleDetourSensorsReady() && ultrasonic_.isFresh() &&
      ultrasonic_.healthy() && ultrasonic_.overallZone() == ObstacleZone::CLEAR;
  // Once Phase 2 has selected a direction from strict evidence, a short
  // dropout during the already bounded detour may use the sensor subsystem's
  // existing qualified clear window. This is still fail-closed: a channel
  // must not be UNKNOWN/BLOCKED/EMERGENCY, and the window itself enforces
  // prior real Echo, distance, timeout-count, and age limits.
  const auto boundedClearZone = [](const UltrasonicReading& reading) {
    return reading.zone != ObstacleZone::UNKNOWN &&
           reading.zone != ObstacleZone::BLOCKED &&
           reading.zone != ObstacleZone::EMERGENCY;
  };
  const bool boundedDegradedClear =
      ultrasonic_.hasRecentClearWindow(millis()) &&
      boundedClearZone(left) && boundedClearZone(right) &&
      ultrasonic_.overallZone() != ObstacleZone::UNKNOWN &&
      ultrasonic_.overallZone() != ObstacleZone::BLOCKED &&
      ultrasonic_.overallZone() != ObstacleZone::EMERGENCY;
  return strictClear || boundedDegradedClear;
}

bool MapController::obstacleDetourEntryGates(const char*& rejectReason) const {
  rejectReason = nullptr;
  if (!replayActive_ || mode_ != MapControllerMode::REPLAY_HOLD ||
      holdReason_ != MapHoldReason::OBSTACLE) {
    rejectReason = "OBSTACLE_HOLD";
    return false;
  }
  if (obstacleDetourContextActive()) {
    rejectReason = "DETOUR_ALREADY_HANDLED";
    return false;
  }
  if (!replayResumeAllowed_ || replayOperation_ != MapReplayOperation::HOLD) {
    rejectReason = "REPLAY_CONTEXT";
    return false;
  }
  if (!robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE) {
    rejectReason = "MOTION_OWNER";
    return false;
  }
  const uint32_t now = millis();
  if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
      ps2_.motionCommandActive() || ps2_.state().r3) {
    rejectReason = "PS2_NOT_NEUTRAL";
    return false;
  }
  if (!loadedValid_ || !replayOriginValid_ ||
      !validateRoute(route_, rejectReason)) {
    if (rejectReason == nullptr) rejectReason = "ROUTE_CONTEXT";
    return false;
  }
  if (selectedSlot_ != replayContextSlot_ ||
      route_.header.generation != replayOriginRouteGeneration_) {
    rejectReason = "ROUTE_CONTEXT";
    return false;
  }
  if (replayCurrentIndex_ >= route_.header.waypointCount ||
      replayTargetIndex_ >= route_.header.waypointCount ||
      replayTargetIndex_ == replayCurrentIndex_ ||
      (replayDirection_ != 1 && replayDirection_ != -1)) {
    rejectReason = "TARGET_WAYPOINT";
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
  if (!odometry_.ready() || !odometry_.healthy()) {
    rejectReason = "ODOMETRY";
    return false;
  }
  if (!fusion_.ready() || fusion_.health() == FusionHealth::NO_SOURCE) {
    rejectReason = "HEADING";
    return false;
  }
  if (!obstacleClassifier_.stable() ||
      (obstacleClassifier_.decision() != ObstacleDecision::AVOID_LEFT &&
       obstacleClassifier_.decision() != ObstacleDecision::AVOID_RIGHT)) {
    rejectReason = "CLASSIFIER_NOT_STABLE_AVOID";
    return false;
  }
  if (!obstacleDetourSensorsReady()) {
    rejectReason = "OBSTACLE_SENSOR";
    return false;
  }
  if (obstacleDetourAttempts_ >= OBSTACLE_DETOUR_MAX_ATTEMPTS) {
    rejectReason = "MAX_ATTEMPTS";
    return false;
  }
  Pose pose;
  if (!readPose(pose)) {
    rejectReason = "POSE";
    return false;
  }
  rejectReason = "OK";
  return true;
}

bool MapController::armObstacleDetour(const char*& rejectReason) {
  // Keep the capability boundary at the arm point as well as at the sensor
  // readiness gate. Future callers cannot accidentally make a left/right
  // detour actionable while production has only one centred SR04.
  if (!ultrasonic_.directionalSensingAvailable()) {
    rejectReason = "DIRECTIONAL_SENSORS_DISABLED";
    return false;
  }
  if (!obstacleDetourEntryGates(rejectReason)) return false;

  Pose pose;
  if (!readPose(pose)) {
    rejectReason = "POSE";
    return false;
  }
  obstacleDetourStartPose_ = pose;
  obstacleDetourOriginalCurrentIndex_ = replayCurrentIndex_;
  obstacleDetourOriginalTargetIndex_ = replayTargetIndex_;
  obstacleDetourOriginalDirection_ = replayDirection_;
  obstacleDetourOriginalRouteGeneration_ = route_.header.generation;
  obstacleDetourOriginalReplayGeneration_ = replayGeneration_;
  obstacleDetourDecision_ = obstacleClassifier_.decision();
  obstacleDetourAwayRight_ =
      obstacleDetourDecision_ == ObstacleDecision::AVOID_RIGHT;
  obstacleDetourAttempts_ = 1U;
  obstacleDetourStartMs_ = millis();
  obstacleDetourClearSinceMs_ = 0U;
  obstacleDetourGeneration_ = 0U;
  obstacleDetourTravelBudgetUsedMm_ = 0U;
  obstacleDetourTurnBudgetUsedDeg_ = 0.0f;
  obstacleDetourPhase_ = ObstacleDetourPhase::ARMED;
  replayResumeAllowed_ = false;
  replayOperation_ = MapReplayOperation::NONE;
  mode_ = MapControllerMode::REPLAY_RUNNING;
  holdReason_ = MapHoldReason::NONE;
  debug_.print("OBS,DETOUR,ARM,SIDE=");
  debug_.print(obstacleDetourAwayRight_ ? "RIGHT" : "LEFT");
  debug_.print(",WP=");
  debug_.println(static_cast<unsigned>(obstacleDetourOriginalTargetIndex_));

  const bool awayLeft = !obstacleDetourAwayRight_;
  if (!startObstacleDetourTurn(awayLeft, ObstacleDetourPhase::TURN_AWAY,
                               "TURN_AWAY")) {
    rejectReason = "TURN_AWAY_START";
    abortObstacleDetour(rejectReason);
    return false;
  }
  return true;
}

bool MapController::startObstacleDetourTurn(bool left,
                                             ObstacleDetourPhase phase,
                                             const char* phaseName) {
  const uint32_t now = millis();
  if (obstacleDetourStartMs_ == 0U ||
      now - obstacleDetourStartMs_ >= OBSTACLE_DETOUR_TIMEOUT_MS ||
      obstacleDetourTurnBudgetUsedDeg_ + OBSTACLE_DETOUR_TURN_AWAY_DEG >
          OBSTACLE_DETOUR_MAX_TOTAL_TURN_DEG) {
    return false;
  }
  const uint32_t generation = nextReplayGeneration();
  replaySegmentGeneration_ = generation;
  obstacleDetourGeneration_ = generation;
  replayOperation_ = MapReplayOperation::TURN;
  replayTargetDeg_ = static_cast<int16_t>(
      lroundf(OBSTACLE_DETOUR_TURN_AWAY_DEG));
  if (!robot_.startReplayTurnRelative(
          left, OBSTACLE_DETOUR_TURN_AWAY_DEG, OBSTACLE_DETOUR_SPEED,
          generation, AiTurnProfile::MAP_COARSE)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    obstacleDetourGeneration_ = 0U;
    return false;
  }
  obstacleDetourTurnBudgetUsedDeg_ += OBSTACLE_DETOUR_TURN_AWAY_DEG;
  obstacleDetourPhase_ = phase;
  debug_.print("OBS,DETOUR,");
  debug_.print(phaseName != nullptr ? phaseName : "TURN");
  debug_.print(",GEN=");
  debug_.println(generation);
  statusDirty_ = true;
  return true;
}

bool MapController::startObstacleDetourDistance(uint32_t distanceMm,
                                                 ObstacleDetourPhase phase,
                                                 const char* phaseName) {
  const uint32_t now = millis();
  if (obstacleDetourStartMs_ == 0U ||
      now - obstacleDetourStartMs_ >= OBSTACLE_DETOUR_TIMEOUT_MS ||
      distanceMm == 0U ||
      obstacleDetourTravelBudgetUsedMm_ + distanceMm >
          OBSTACLE_DETOUR_MAX_TOTAL_DISTANCE_MM) {
    return false;
  }
  const uint32_t generation = nextReplayGeneration();
  replaySegmentGeneration_ = generation;
  obstacleDetourGeneration_ = generation;
  replayOperation_ = MapReplayOperation::MOVE;
  replayTargetDistanceMm_ = distanceMm;
  replayTravelMm_ = 0U;
  replayErrorMm_ = distanceMm;
  if (!robot_.startReplayDistance(true, distanceMm, OBSTACLE_DETOUR_SPEED,
                                  generation)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    obstacleDetourGeneration_ = 0U;
    return false;
  }
  obstacleDetourTravelBudgetUsedMm_ += distanceMm;
  obstacleDetourPhase_ = phase;
  debug_.print("OBS,DETOUR,");
  debug_.print(phaseName != nullptr ? phaseName : "MOVE");
  debug_.print(",MM=");
  debug_.print(distanceMm);
  debug_.print(",GEN=");
  debug_.println(generation);
  statusDirty_ = true;
  return true;
}

void MapController::updateObstacleDetour() {
  if (!obstacleDetourInProgress()) return;
  const uint32_t now = millis();
  if (obstacleDetourStartMs_ == 0U ||
      now - obstacleDetourStartMs_ >= OBSTACLE_DETOUR_TIMEOUT_MS) {
    abortObstacleDetour("TIMEOUT");
    return;
  }

  const bool turnPhase = obstacleDetourPhase_ == ObstacleDetourPhase::TURN_AWAY ||
                         obstacleDetourPhase_ == ObstacleDetourPhase::TURN_PARALLEL;
  const bool movePhase = obstacleDetourPhase_ == ObstacleDetourPhase::MOVE_AWAY ||
                         obstacleDetourPhase_ == ObstacleDetourPhase::MOVE_BYPASS;
  if (turnPhase &&
      (replayOperation_ != MapReplayOperation::TURN ||
       !robot_.aiTurnActive() || robot_.motionOwner() != MotionOwner::REPLAY)) {
    abortObstacleDetour("EXTERNAL_STOP");
    return;
  }
  if (movePhase &&
      (replayOperation_ != MapReplayOperation::MOVE ||
       !robot_.aiDistanceActive() ||
       robot_.motionOwner() != MotionOwner::REPLAY)) {
    abortObstacleDetour("EXTERNAL_STOP");
    return;
  }
  if (!turnPhase && !movePhase &&
      (replayOperation_ != MapReplayOperation::NONE ||
       robot_.aiMotionActive() || robot_.motionOwner() != MotionOwner::NONE ||
       !robot_.motorsStopped())) {
    abortObstacleDetour("EXTERNAL_STOP");
    return;
  }

  if (obstacleDetourPhase_ != ObstacleDetourPhase::WAIT_AWAY_CLEAR &&
      obstacleDetourPhase_ != ObstacleDetourPhase::WAIT_BYPASS_CLEAR) {
    return;
  }
  if (!obstacleDetourPathClear()) {
    obstacleDetourClearSinceMs_ = 0U;
    return;
  }
  if (obstacleDetourClearSinceMs_ == 0U) {
    obstacleDetourClearSinceMs_ = now;
    return;
  }
  if (now - obstacleDetourClearSinceMs_ < OBSTACLE_DETOUR_CLEAR_STABLE_MS) {
    return;
  }
  obstacleDetourClearSinceMs_ = 0U;
  if (obstacleDetourPhase_ == ObstacleDetourPhase::WAIT_AWAY_CLEAR) {
    if (!startObstacleDetourDistance(OBSTACLE_DETOUR_MOVE_AWAY_MM,
                                     ObstacleDetourPhase::MOVE_AWAY,
                                     "MOVE_AWAY")) {
      abortObstacleDetour("MOVE_AWAY_START");
    }
  } else if (!startObstacleDetourDistance(
                 OBSTACLE_DETOUR_BYPASS_MM,
                 ObstacleDetourPhase::MOVE_BYPASS, "MOVE_BYPASS")) {
    abortObstacleDetour("MOVE_BYPASS_START");
  }
}

bool MapController::consumeObstacleDetourTurnResult(
    const AiTurnResult& result) {
  if (result.owner != MotionOwner::REPLAY) return false;
  const bool expected =
      (obstacleDetourPhase_ == ObstacleDetourPhase::TURN_AWAY ||
       obstacleDetourPhase_ == ObstacleDetourPhase::TURN_PARALLEL) &&
      replayOperation_ == MapReplayOperation::TURN &&
      result.motionGeneration != 0U &&
      result.motionGeneration == obstacleDetourGeneration_;
  if (!expected) {
    debug_.print("OBS,DETOUR,STALE_RESULT,TYPE=TURN,GEN=");
    debug_.print(result.motionGeneration);
    debug_.print(",EXPECTED=");
    debug_.println(obstacleDetourGeneration_);
    return true;
  }
  replayOperation_ = MapReplayOperation::NONE;
  replaySegmentGeneration_ = 0U;
  obstacleDetourGeneration_ = 0U;
  if (result.code == AiTurnResultCode::DONE) {
    obstacleDetourClearSinceMs_ = 0U;
    if (obstacleDetourPhase_ == ObstacleDetourPhase::TURN_AWAY) {
      obstacleDetourPhase_ = ObstacleDetourPhase::WAIT_AWAY_CLEAR;
      debug_.println("OBS,DETOUR,WAIT_AWAY_CLEAR");
    } else {
      obstacleDetourPhase_ = ObstacleDetourPhase::WAIT_BYPASS_CLEAR;
      debug_.println("OBS,DETOUR,WAIT_BYPASS_CLEAR");
    }
    statusDirty_ = true;
    return true;
  }
  if (result.code == AiTurnResultCode::CANCELLED &&
      ps2_.motionCommandActive()) {
    abortObstacleDetour("PS2_TAKEOVER");
  } else if (result.code == AiTurnResultCode::HEADING_LOST) {
    abortObstacleDetour("HEADING_LOST");
  } else if (result.code == AiTurnResultCode::MOTION_FAULT) {
    abortObstacleDetour("MOTION_FAULT");
  } else if (result.code == AiTurnResultCode::OBSTACLE) {
    abortObstacleDetour("OBSTACLE");
  } else if (result.code == AiTurnResultCode::TIMEOUT) {
    abortObstacleDetour("TIMEOUT");
  } else if (result.code == AiTurnResultCode::CANCELLED) {
    abortObstacleDetour("CANCELLED");
  } else {
    abortObstacleDetour("UNEXPECTED_TURN_RESULT");
  }
  return true;
}

bool MapController::consumeObstacleDetourDistanceResult(
    const AiDistanceResult& result) {
  if (result.owner != MotionOwner::REPLAY) return false;
  const bool expected =
      (obstacleDetourPhase_ == ObstacleDetourPhase::MOVE_AWAY ||
       obstacleDetourPhase_ == ObstacleDetourPhase::MOVE_BYPASS) &&
      replayOperation_ == MapReplayOperation::MOVE &&
      result.motionGeneration != 0U &&
      result.motionGeneration == obstacleDetourGeneration_;
  if (!expected) {
    debug_.print("OBS,DETOUR,STALE_RESULT,TYPE=DISTANCE,GEN=");
    debug_.print(result.motionGeneration);
    debug_.print(",EXPECTED=");
    debug_.println(obstacleDetourGeneration_);
    return true;
  }
  replayOperation_ = MapReplayOperation::NONE;
  replaySegmentGeneration_ = 0U;
  obstacleDetourGeneration_ = 0U;
  replayTravelMm_ = result.travelledMm < 0.0f
                        ? 0U
                        : static_cast<uint32_t>(lroundf(result.travelledMm));
  if (result.code == AiDistanceResultCode::DONE) {
    if (obstacleDetourPhase_ == ObstacleDetourPhase::MOVE_AWAY) {
      const bool parallelLeft = obstacleDetourAwayRight_;
      if (!startObstacleDetourTurn(parallelLeft,
                                   ObstacleDetourPhase::TURN_PARALLEL,
                                   "TURN_PARALLEL")) {
        abortObstacleDetour("TURN_PARALLEL_START");
      }
    } else {
      completeObstacleDetour();
    }
    return true;
  }
  if (result.code == AiDistanceResultCode::CANCELLED &&
      ps2_.motionCommandActive()) {
    abortObstacleDetour("PS2_TAKEOVER");
  } else if (result.code == AiDistanceResultCode::OBSTACLE) {
    abortObstacleDetour("OBSTACLE");
  } else if (result.code == AiDistanceResultCode::ENCODER_FAULT) {
    abortObstacleDetour("ENCODER_FAULT");
  } else if (result.code == AiDistanceResultCode::HEADING_LOST) {
    abortObstacleDetour("HEADING_LOST");
  } else if (result.code == AiDistanceResultCode::TIMEOUT) {
    abortObstacleDetour("TIMEOUT");
  } else if (result.code == AiDistanceResultCode::CANCELLED) {
    abortObstacleDetour("CANCELLED");
  } else {
    abortObstacleDetour("UNEXPECTED_DISTANCE_RESULT");
  }
  return true;
}

void MapController::completeObstacleDetour() {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  obstacleDetourGeneration_ = 0U;
  replayOperation_ = MapReplayOperation::HOLD;
  replayActive_ = true;
  replayResumeAllowed_ = false;
  mode_ = MapControllerMode::REPLAY_HOLD;
  holdReason_ = MapHoldReason::OBSTACLE;
  obstacleDetourPhase_ = ObstacleDetourPhase::COMPLETE_HOLD;
  obstacleDetourClearSinceMs_ = 0U;
  replayReason_ = "DETOUR_COMPLETE_HOLD";
  ps2_.holdMapInput();
  debug_.print("OBS,DETOUR,COMPLETE_HOLD,SIDE=");
  debug_.print(obstacleDetourAwayRight_ ? "RIGHT" : "LEFT");
  debug_.print(",WP=");
  debug_.print(static_cast<unsigned>(obstacleDetourOriginalTargetIndex_));
  debug_.print(",TARGET=");
  debug_.print(static_cast<unsigned>(obstacleDetourOriginalTargetIndex_));
  debug_.print(",DIST_TOTAL=");
  debug_.print(obstacleDetourTravelBudgetUsedMm_);
  debug_.print(",TURN_TOTAL=");
  debug_.println(obstacleDetourTurnBudgetUsedDeg_, 1);
  statusDirty_ = true;
}

void MapController::abortObstacleDetour(const char* reason) {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  obstacleDetourGeneration_ = 0U;
  replayOperation_ = MapReplayOperation::HOLD;
  replayActive_ = loadedValid_ && replayOriginValid_;
  replayResumeAllowed_ = false;
  mode_ = MapControllerMode::REPLAY_HOLD;
  holdReason_ = MapHoldReason::OBSTACLE;
  obstacleDetourPhase_ = ObstacleDetourPhase::ABORTED;
  obstacleDetourClearSinceMs_ = 0U;
  replayReason_ = reason != nullptr ? reason : "DETOUR_ABORTED";
  ps2_.holdMapInput();
  debug_.print("OBS,DETOUR,ABORT,REASON=");
  debug_.println(replayReason_);
  statusDirty_ = true;
}

bool MapController::returnP0InProgress() const {
  return returnP0State_ != ReturnP0State::IDLE &&
         returnP0State_ != ReturnP0State::COMPLETE &&
         returnP0State_ != ReturnP0State::ABORTED;
}

const char* MapController::returnP0StateName(ReturnP0State state) {
  switch (state) {
    case ReturnP0State::IDLE: return "IDLE";
    case ReturnP0State::VALIDATE_HOME: return "VALIDATE_HOME";
    case ReturnP0State::LOCATE_ON_ROUTE: return "LOCATE_ON_ROUTE";
    case ReturnP0State::REACQUIRE_ROUTE: return "REACQUIRE_ROUTE";
    case ReturnP0State::RETURN_WAYPOINT: return "RETURN_WAYPOINT";
    case ReturnP0State::P0_POSITION_APPROACH: return "P0_POSITION_APPROACH";
    case ReturnP0State::P0_POSITION_SETTLE: return "P0_POSITION_SETTLE";
    case ReturnP0State::P0_HEADING_RESTORE: return "P0_HEADING_RESTORE";
    case ReturnP0State::P0_HEADING_SETTLE: return "P0_HEADING_SETTLE";
    case ReturnP0State::HOLD: return "HOLD";
    case ReturnP0State::COMPLETE: return "COMPLETE";
    case ReturnP0State::ABORTED: return "ABORTED";
  }
  return "ABORTED";
}

bool MapController::locateRouteProjection(RouteProjection& projection,
                                           const char*& reason) const {
  projection = {};
  reason = nullptr;
  if (!homeContext_.valid) {
    reason = "HOME_CONTEXT_INVALID";
    return false;
  }
  if (!loadedValid_ || route_.header.waypointCount < 2U ||
      route_.header.generation != homeContext_.routeGeneration) {
    reason = "ROUTE_CHANGED";
    return false;
  }
  Pose current;
  if (!readPose(current)) {
    reason = "POSE";
    return false;
  }

  float bestDistance = 1.0e30f;
  uint16_t bestSegment = 0U;
  bool found = false;
  const uint16_t count = route_.header.waypointCount;
  for (uint16_t index = 0U; index + 1U < count; ++index) {
    const Pose start = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                 index);
    const Pose end = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                               index + 1U);
    const float dx = end.xMm - start.xMm;
    const float dy = end.yMm - start.yMm;
    const float lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1.0e-3f) continue;
    float t = ((current.xMm - start.xMm) * dx +
               (current.yMm - start.yMm) * dy) /
              lengthSquared;
    t = constrain(t, 0.0f, 1.0f);
    const float projectedX = start.xMm + t * dx;
    const float projectedY = start.yMm + t * dy;
    const float distance = distanceMm(current.xMm, current.yMm, projectedX,
                                      projectedY);
    // At a shared adjacent corner, prefer the earlier segment. This makes
    // P1/P2 resolve to their predecessor while non-adjacent crossings are
    // handled by the explicit ambiguity gate below.
    if (!found || distance < bestDistance - 0.001f ||
        (fabsf(distance - bestDistance) <= 0.001f && index < bestSegment)) {
      bestDistance = distance;
      bestSegment = index;
      found = true;
    }
  }
  if (!found) {
    reason = "DEGENERATE_ROUTE";
    return false;
  }
  for (uint16_t index = 0U; index + 1U < count; ++index) {
    if (index == bestSegment ||
        (index > bestSegment ? index - bestSegment : bestSegment - index) <=
            1U) {
      continue;
    }
    const Pose start = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                 index);
    const Pose end = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                               index + 1U);
    const float dx = end.xMm - start.xMm;
    const float dy = end.yMm - start.yMm;
    const float lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1.0e-3f) continue;
    float t = ((current.xMm - start.xMm) * dx +
               (current.yMm - start.yMm) * dy) /
              lengthSquared;
    t = constrain(t, 0.0f, 1.0f);
    const float projectedX = start.xMm + t * dx;
    const float projectedY = start.yMm + t * dy;
    const float distance = distanceMm(current.xMm, current.yMm, projectedX,
                                      projectedY);
    if (distance <= bestDistance + MAP_RETURN_P0_AMBIGUITY_MARGIN_MM) {
      reason = "AMBIGUOUS_SEGMENT";
      return false;
    }
  }
  if (bestDistance > MAP_RETURN_P0_MAX_CROSSTRACK_MM) {
    reason = "OFF_ROUTE";
    return false;
  }
  const Pose bestStart = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                   bestSegment);
  const Pose bestEnd = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                 bestSegment + 1U);
  const float dx = bestEnd.xMm - bestStart.xMm;
  const float dy = bestEnd.yMm - bestStart.yMm;
  const float lengthSquared = dx * dx + dy * dy;
  float t = ((current.xMm - bestStart.xMm) * dx +
             (current.yMm - bestStart.yMm) * dy) /
            lengthSquared;
  t = constrain(t, 0.0f, 1.0f);
  projection.valid = true;
  projection.segmentStartIndex = bestSegment;
  projection.segmentEndIndex = bestSegment + 1U;
  projection.t = t;
  projection.projectedPose.xMm = bestStart.xMm + t * dx;
  projection.projectedPose.yMm = bestStart.yMm + t * dy;
  projection.projectedPose.headingDeg =
      atan2f(dy, dx) * kRadToDeg;
  projection.crossTrackMm = bestDistance;
  projection.distanceMm = bestDistance;
  reason = "OK";
  return true;
}

bool MapController::startReturnReacquire() {
  if (!returnP0Projection_.valid || !returnP0InProgress()) return false;
  if (returnP0ReacquireAttempts_ >= MAP_RETURN_P0_MAX_REACQUIRE_ATTEMPTS) {
    return false;
  }
  Pose current;
  if (!readPose(current)) return false;
  const Pose target = returnP0Projection_.projectedPose;
  const float targetDistance = distanceMm(current.xMm, current.yMm,
                                          target.xMm, target.yMm);
  if (targetDistance <=
      static_cast<float>(MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM)) {
    returnP0State_ = ReturnP0State::RETURN_WAYPOINT;
    return startReturnWaypoint();
  }
  ++returnP0ReacquireAttempts_;
  const Pose segmentStart = returnP0Projection_.t <= 0.01f
                                ? routePointWorldFromOrigin(
                                      homeContext_.p0WorldPose,
                                      returnP0Projection_.segmentEndIndex)
                                : routePointWorldFromOrigin(
                                      homeContext_.p0WorldPose,
                                      returnP0Projection_.segmentStartIndex);
  const float bearing = atan2f(target.yMm - segmentStart.yMm,
                               target.xMm - segmentStart.xMm) * kRadToDeg;
  const uint32_t generation = nextReplayGeneration();
  returnP0Generation_ = generation;
  returnP0SegmentGeneration_ = generation;
  replaySegmentGeneration_ = generation;
  replayOperation_ = MapReplayOperation::MOVE;
  replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(targetDistance));
  replayTravelMm_ = 0U;
  replayErrorMm_ = replayTargetDistanceMm_;
  if (!robot_.startReplayGuidedWaypoint(
          target.xMm, target.yMm, segmentStart.xMm, segmentStart.yMm,
          replaySpeed_, bearing,
          MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM, generation)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    returnP0SegmentGeneration_ = 0U;
    return false;
  }
  logReturnP0TraceMotion("REACQUIRE_START", current);
  debug_.print(",TARGET_X=");
  debug_.print(target.xMm, 1);
  debug_.print(",TARGET_Y=");
  debug_.print(target.yMm, 1);
  debug_.print(",DIST=");
  debug_.print(targetDistance, 1);
  debug_.print(",HEADING=");
  debug_.print(current.headingDeg, 2);
  debug_.print(",TARGET_BEARING=");
  debug_.println(bearing, 2);
  debug_.print("MAP,RETURN_P0,REACQUIRE,SEG=");
  debug_.print(static_cast<unsigned>(returnP0Projection_.segmentStartIndex));
  debug_.print(",T=");
  debug_.print(returnP0Projection_.t, 3);
  debug_.print(",XT=");
  debug_.print(returnP0Projection_.crossTrackMm, 1);
  debug_.print(",ATTEMPT=");
  debug_.println(static_cast<unsigned>(returnP0ReacquireAttempts_));
  return true;
}

bool MapController::startReturnWaypoint() {
  if (!returnP0InProgress()) return false;
  if (returnP0TargetIndex_ == 0U) return startReturnP0Position();
  if (returnP0TargetIndex_ + 1U >= route_.header.waypointCount) {
    return false;
  }
  Pose current;
  if (!readPose(current)) return false;
  const Pose target = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                returnP0TargetIndex_);
  const float targetDistance = distanceMm(current.xMm, current.yMm,
                                          target.xMm, target.yMm);
  if (targetDistance <=
      static_cast<float>(MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM)) {
    replayCurrentIndex_ = returnP0TargetIndex_;
    --returnP0TargetIndex_;
    replayTargetIndex_ = returnP0TargetIndex_;
    return startReturnWaypoint();
  }
  const Pose segmentStart = routePointWorldFromOrigin(
      homeContext_.p0WorldPose, returnP0TargetIndex_ + 1U);
  const float bearing = atan2f(target.yMm - segmentStart.yMm,
                               target.xMm - segmentStart.xMm) * kRadToDeg;
  const uint32_t generation = nextReplayGeneration();
  returnP0Generation_ = generation;
  returnP0SegmentGeneration_ = generation;
  replaySegmentGeneration_ = generation;
  replayOperation_ = MapReplayOperation::MOVE;
  replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(targetDistance));
  replayTravelMm_ = 0U;
  replayErrorMm_ = replayTargetDistanceMm_;
  replayTargetIndex_ = returnP0TargetIndex_;
  replayCurrentIndex_ = returnP0TargetIndex_ + 1U;
  if (!robot_.startReplayGuidedWaypoint(
          target.xMm, target.yMm, segmentStart.xMm, segmentStart.yMm,
          replaySpeed_, bearing, MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM,
          generation)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    returnP0SegmentGeneration_ = 0U;
    return false;
  }
  logReturnP0TraceMotion("WP_START", current);
  debug_.print(",FROM_INDEX=");
  debug_.print(static_cast<unsigned>(returnP0TargetIndex_ + 1U));
  debug_.print(",TARGET_INDEX=");
  debug_.print(static_cast<unsigned>(returnP0TargetIndex_));
  debug_.print(",TARGET_X=");
  debug_.print(target.xMm, 1);
  debug_.print(",TARGET_Y=");
  debug_.print(target.yMm, 1);
  debug_.print(",DIST=");
  debug_.print(targetDistance, 1);
  debug_.print(",POS_ERR=");
  debug_.print(targetDistance, 1);
  debug_.print(",TARGET_BEARING=");
  debug_.print(bearing, 2);
  debug_.print(",CROSS_TRACK=");
  debug_.println(diagnosticCrossTrackToSegment(current, segmentStart, target),
                 1);
  debug_.print("MAP,RETURN_P0,WP=");
  debug_.print(static_cast<unsigned>(returnP0TargetIndex_));
  debug_.print(",GEN=");
  debug_.println(generation);
  return true;
}

bool MapController::startReturnP0Position() {
  if (!returnP0InProgress() || route_.header.waypointCount < 2U) {
    return false;
  }
  Pose current;
  if (!readPose(current)) return false;
  const Pose target = homeContext_.p0WorldPose;
  const float positionError = distanceMm(current.xMm, current.yMm,
                                         target.xMm, target.yMm);
  replayTargetIndex_ = 0U;
  logReturnP0TraceMotion("P0_POSITION_START", current);
  logReturnP0TraceP0Error(current);
  debug_.print(",P0_X=");
  debug_.print(target.xMm, 1);
  debug_.print(",P0_Y=");
  debug_.print(target.yMm, 1);
  debug_.print(",P0_H=");
  debug_.println(target.headingDeg, 2);
  if (positionError <= MAP_RETURN_P0_POSITION_TOLERANCE_MM) {
    robot_.stopImmediately(true);
    replayOperation_ = MapReplayOperation::NONE;
    returnP0State_ = ReturnP0State::P0_POSITION_SETTLE;
    returnP0SettleSinceMs_ = millis();
    logReturnP0TraceMotion("P0_POSITION_ARRIVAL", current);
    logReturnP0TraceP0Error(current);
    debug_.println(",ARRIVAL=1");
    logReturnP0TraceMotion("P0_POSITION_SETTLE", current);
    logReturnP0TraceP0Error(current);
    debug_.println(",PHASE=ENTER,SETTLE_MS=0");
    debug_.print("MAP,RETURN_P0,P0_POSITION,ERR=");
    debug_.println(positionError, 1);
    return true;
  }
  const Pose segmentStart = routePointWorldFromOrigin(homeContext_.p0WorldPose,
                                                       1U);
  const float bearing = atan2f(target.yMm - segmentStart.yMm,
                               target.xMm - segmentStart.xMm) * kRadToDeg;
  const uint32_t generation = nextReplayGeneration();
  returnP0Generation_ = generation;
  returnP0SegmentGeneration_ = generation;
  replaySegmentGeneration_ = generation;
  replayOperation_ = MapReplayOperation::MOVE;
  replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(positionError));
  replayTravelMm_ = 0U;
  replayErrorMm_ = replayTargetDistanceMm_;
  returnP0State_ = ReturnP0State::P0_POSITION_APPROACH;
  if (!robot_.startReplayGuidedWaypoint(
          target.xMm, target.yMm, segmentStart.xMm, segmentStart.yMm,
          replaySpeed_, bearing, MAP_RETURN_P0_POSITION_TOLERANCE_MM,
          generation)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    returnP0SegmentGeneration_ = 0U;
    return false;
  }
  debug_.print("MAP,RETURN_P0,P0_POSITION,START,ERR=");
  debug_.print(positionError, 1);
  debug_.print(",GEN=");
  debug_.println(generation);
  return true;
}

bool MapController::startReturnP0Heading() {
  if (!returnP0InProgress()) return false;
  Pose current;
  if (!readPose(current)) return false;
  const float headingError = shortestDeltaDeg(
      homeContext_.p0WorldPose.headingDeg, current.headingDeg);
  logReturnP0TraceMotion("P0_HEADING_START", current);
  logReturnP0TraceP0Error(current);
  debug_.print(",TARGET_H=");
  debug_.print(homeContext_.p0WorldPose.headingDeg, 2);
  debug_.print(",HEADING_ERR=");
  debug_.println(headingError, 2);
  if (fabsf(headingError) <= MAP_RETURN_P0_HEADING_TOLERANCE_DEG) {
    robot_.stopImmediately(true);
    replayOperation_ = MapReplayOperation::NONE;
    returnP0State_ = ReturnP0State::P0_HEADING_SETTLE;
    returnP0SettleSinceMs_ = millis();
    return true;
  }
  if (returnP0HeadingAttempts_ >= MAP_RETURN_P0_MAX_HEADING_ATTEMPTS) {
    return false;
  }
  ++returnP0HeadingAttempts_;
  const uint32_t generation = nextReplayGeneration();
  returnP0Generation_ = generation;
  returnP0SegmentGeneration_ = generation;
  replaySegmentGeneration_ = generation;
  replayOperation_ = MapReplayOperation::TURN;
  returnP0TurnPending_ = true;
  returnP0State_ = ReturnP0State::P0_HEADING_RESTORE;
  if (!robot_.startReplayTurnRelative(
          headingError > 0.0f, fabsf(headingError), replaySpeed_, generation,
          AiTurnProfile::PRECISE)) {
    replayOperation_ = MapReplayOperation::NONE;
    replaySegmentGeneration_ = 0U;
    returnP0SegmentGeneration_ = 0U;
    returnP0TurnPending_ = false;
    return false;
  }
  debug_.print("MAP,RETURN_P0,P0_HEADING,START,ERR=");
  debug_.print(headingError, 2);
  debug_.print(",TARGET=");
  debug_.println(homeContext_.p0WorldPose.headingDeg, 2);
  return true;
}

void MapController::updateReturnToP0() {
  if (!returnP0InProgress() || returnP0State_ == ReturnP0State::HOLD ||
      replayOperation_ != MapReplayOperation::NONE) {
    return;
  }
  if (!homeContext_.valid || selectedSlot_ != homeContext_.slot ||
      route_.header.generation != homeContext_.routeGeneration ||
      odometry_.resetGeneration() != homeContext_.odometryResetGeneration ||
      robot_.headingResetGeneration() != homeContext_.headingResetGeneration) {
    invalidateHomeContext("RESET_BOUNDARY");
    abortReturnToP0("RESET_BOUNDARY");
    return;
  }
  if (!odometry_.ready() || !odometry_.healthy() || !fusion_.ready() ||
      fusion_.health() == FusionHealth::NO_SOURCE) {
    abortReturnToP0(!odometry_.healthy() ? "ENCODER_FAULT" : "HEADING_LOST");
    return;
  }
  const uint32_t now = millis();
  switch (returnP0State_) {
    case ReturnP0State::REACQUIRE_ROUTE:
      if (!startReturnReacquire()) abortReturnToP0("REACQUIRE_LIMIT");
      break;
    case ReturnP0State::RETURN_WAYPOINT:
      if (!startReturnWaypoint()) abortReturnToP0("RETURN_START");
      break;
    case ReturnP0State::P0_POSITION_APPROACH:
      if (!startReturnP0Position()) abortReturnToP0("P0_POSITION");
      break;
    case ReturnP0State::P0_POSITION_SETTLE: {
      if (now - returnP0SettleSinceMs_ < MAP_RETURN_P0_SETTLE_MS) break;
      Pose current;
      if (!readPose(current)) {
        abortReturnToP0("POSE");
        break;
      }
      const float positionError = distanceMm(
          current.xMm, current.yMm, homeContext_.p0WorldPose.xMm,
          homeContext_.p0WorldPose.yMm);
      logReturnP0TraceMotion("P0_POSITION_SETTLE", current);
      logReturnP0TraceP0Error(current);
      debug_.print(",PHASE=DONE,SETTLE_MS=");
      debug_.println(now - returnP0SettleSinceMs_);
      if (positionError > MAP_RETURN_P0_POSITION_TOLERANCE_MM) {
        if (returnP0PositionCorrectionAttempts_ >=
            MAP_RETURN_P0_MAX_POSITION_CORRECTIONS) {
          abortReturnToP0("P0_POSITION_LIMIT");
        } else {
          ++returnP0PositionCorrectionAttempts_;
          returnP0State_ = ReturnP0State::P0_POSITION_APPROACH;
          if (!startReturnP0Position()) abortReturnToP0("P0_POSITION");
        }
        break;
      }
      returnP0State_ = ReturnP0State::P0_HEADING_RESTORE;
      if (!startReturnP0Heading()) abortReturnToP0("P0_HEADING");
      break;
    }
    case ReturnP0State::P0_HEADING_RESTORE:
      if (!startReturnP0Heading()) abortReturnToP0("P0_HEADING");
      break;
    case ReturnP0State::P0_HEADING_SETTLE: {
      if (now - returnP0SettleSinceMs_ < MAP_RETURN_P0_SETTLE_MS) break;
      Pose current;
      if (!readPose(current)) {
        abortReturnToP0("POSE");
        break;
      }
      const float positionError = distanceMm(
          current.xMm, current.yMm, homeContext_.p0WorldPose.xMm,
          homeContext_.p0WorldPose.yMm);
      const float headingError = fabsf(shortestDeltaDeg(
          homeContext_.p0WorldPose.headingDeg, current.headingDeg));
      const bool sensorSafe = ultrasonic_.isFresh() && ultrasonic_.healthy() &&
                              ultrasonic_.overallZone() != ObstacleZone::UNKNOWN;
      const bool finalGatePass =
          positionError <= MAP_RETURN_P0_POSITION_TOLERANCE_MM &&
          headingError <= MAP_RETURN_P0_HEADING_TOLERANCE_DEG &&
          robot_.motorsStopped() && !robot_.aiMotionActive() &&
          robot_.motionOwner() == MotionOwner::NONE && odometry_.healthy() &&
          fusion_.health() != FusionHealth::NO_SOURCE && sensorSafe;
      logReturnP0TraceMotion("FINAL_GATE", current);
      logReturnP0TraceP0Error(current);
      debug_.print(",POS_LIMIT=");
      debug_.print(MAP_RETURN_P0_POSITION_TOLERANCE_MM);
      debug_.print(",HEADING_LIMIT=");
      debug_.print(MAP_RETURN_P0_HEADING_TOLERANCE_DEG, 2);
      debug_.print(",MOTORS_STOPPED=");
      debug_.print(robot_.motorsStopped() ? 1 : 0);
      debug_.print(",GATE_RESULT=");
      debug_.println(finalGatePass ? "PASS" : "FAIL");
      if (finalGatePass) {
        logReturnP0TraceMotion("COMPLETE", current);
        logReturnP0TraceP0Error(current);
        debug_.println(",GATE_RESULT=PASS");
        completeReturnToP0();
      } else if (positionError > MAP_RETURN_P0_POSITION_TOLERANCE_MM) {
        if (returnP0PositionCorrectionAttempts_ >=
            MAP_RETURN_P0_MAX_POSITION_CORRECTIONS) {
          abortReturnToP0("P0_POSITION_LIMIT");
        } else {
          ++returnP0PositionCorrectionAttempts_;
          returnP0State_ = ReturnP0State::P0_POSITION_APPROACH;
          if (!startReturnP0Position()) abortReturnToP0("P0_POSITION");
        }
      } else if (headingError > MAP_RETURN_P0_HEADING_TOLERANCE_DEG) {
        returnP0State_ = ReturnP0State::P0_HEADING_RESTORE;
        if (!startReturnP0Heading()) abortReturnToP0("P0_HEADING");
      } else {
        abortReturnToP0("FINAL_VERIFY");
      }
      break;
    }
    default:
      break;
  }
}

void MapController::abortReturnToP0(const char* reason) {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayOperation_ = MapReplayOperation::NONE;
  replayActive_ = false;
  replayReason_ = reason != nullptr ? reason : "RETURN_ABORT";
  returnP0SegmentGeneration_ = 0U;
  returnP0TurnPending_ = false;
  returnP0HeldState_ = ReturnP0State::IDLE;
  returnP0State_ = ReturnP0State::ABORTED;
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                               : MapControllerMode::READY;
  debug_.print("MAP,RETURN_P0,ABORT,REASON=");
  debug_.println(replayReason_);
  statusDirty_ = true;
}

void MapController::completeReturnToP0() {
  robot_.stopImmediately(true);
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayOperation_ = MapReplayOperation::NONE;
  replayActive_ = false;
  replayReason_ = "RETURN_P0_COMPLETE";
  returnP0SegmentGeneration_ = 0U;
  returnP0TurnPending_ = false;
  returnP0HeldState_ = ReturnP0State::IDLE;
  returnP0State_ = ReturnP0State::COMPLETE;
  mode_ = MapControllerMode::REPLAY_COMPLETE;
  debug_.println("MAP,RETURN_P0,COMPLETE");
  statusDirty_ = true;
}

bool MapController::consumeReturnTurnResult(const AiTurnResult& result) {
  if (!returnP0InProgress() ||
      replayOperation_ != MapReplayOperation::TURN ||
      result.motionGeneration != returnP0SegmentGeneration_) {
    if (result.motionGeneration != 0U) {
      debug_.print("MAP,RETURN_P0,DROP_STALE,GEN=");
      debug_.println(result.motionGeneration);
    }
    return true;
  }
  replayOperation_ = MapReplayOperation::NONE;
  if (result.code == AiTurnResultCode::DONE) {
    if (returnP0State_ == ReturnP0State::P0_HEADING_RESTORE) {
      Pose current;
      if (readPose(current)) {
        logReturnP0TraceMotion("P0_HEADING_DONE", current);
        logReturnP0TraceP0Error(current);
        debug_.print(",TARGET_H=");
        debug_.print(homeContext_.p0WorldPose.headingDeg, 2);
        debug_.print(",TURN_FINAL_ERROR=");
        debug_.print(result.errorDeg, 2);
        debug_.print(",TURN_OVERSHOOT=");
        debug_.println("NA");
      }
      returnP0TurnPending_ = false;
      returnP0State_ = ReturnP0State::P0_HEADING_SETTLE;
      returnP0SettleSinceMs_ = millis();
    } else if (returnP0TurnPending_) {
      returnP0TurnPending_ = false;
      returnP0State_ = ReturnP0State::RETURN_WAYPOINT;
    }
    return true;
  }
  if (result.code == AiTurnResultCode::OBSTACLE) {
    returnP0HeldState_ = returnP0State_;
    returnP0HeldTargetIndex_ = returnP0TargetIndex_;
    returnP0HeldSegmentStartIndex_ = returnP0SegmentStartIndex_;
    returnP0State_ = ReturnP0State::HOLD;
    enterReplayHold(MapHoldReason::OBSTACLE, false);
    return true;
  }
  abortReturnToP0(result.code == AiTurnResultCode::HEADING_LOST
                      ? "HEADING_LOST"
                      : "TURN_ERROR");
  return true;
}

bool MapController::consumeReturnDistanceResult(
    const AiDistanceResult& result) {
  if (!returnP0InProgress() ||
      (replayOperation_ != MapReplayOperation::MOVE) ||
      result.motionGeneration != returnP0SegmentGeneration_) {
    if (result.motionGeneration != 0U) {
      debug_.print("MAP,RETURN_P0,DROP_STALE,GEN=");
      debug_.println(result.motionGeneration);
    }
    return true;
  }
  replayOperation_ = MapReplayOperation::NONE;
  replayTravelMm_ = result.travelledMm < 0.0f
                        ? 0U
                        : static_cast<uint32_t>(lroundf(result.travelledMm));
  if (result.code == AiDistanceResultCode::REALIGN_REQUIRED) {
    if (++returnP0ReacquireAttempts_ > MAP_RETURN_P0_MAX_REACQUIRE_ATTEMPTS) {
      abortReturnToP0("REALIGN_LIMIT");
      return true;
    }
    Pose current;
    if (!readPose(current) || returnP0TargetIndex_ + 1U >=
                                  route_.header.waypointCount) {
      abortReturnToP0("POSE");
      return true;
    }
    const Pose target = routePointWorldFromOrigin(
        homeContext_.p0WorldPose, returnP0TargetIndex_);
    const Pose segmentStart = routePointWorldFromOrigin(
        homeContext_.p0WorldPose, returnP0TargetIndex_ + 1U);
    const float desiredBearing = atan2f(target.yMm - segmentStart.yMm,
                                        target.xMm - segmentStart.xMm) * kRadToDeg;
    const float error = shortestDeltaDeg(desiredBearing, current.headingDeg);
    const uint32_t generation = nextReplayGeneration();
    returnP0Generation_ = generation;
    returnP0SegmentGeneration_ = generation;
    replaySegmentGeneration_ = generation;
    replayOperation_ = MapReplayOperation::TURN;
    returnP0TurnPending_ = true;
    if (!robot_.startReplayTurnRelative(
            error > 0.0f, fabsf(error), replaySpeed_, generation,
            AiTurnProfile::MAP_COARSE)) {
      abortReturnToP0("REALIGN_TURN");
    }
    return true;
  }
  if (result.code == AiDistanceResultCode::DONE) {
    Pose current;
    if (!readPose(current)) {
      abortReturnToP0("POSE");
      return true;
    }
    const Pose target = returnP0TargetIndex_ == 0U
                            ? homeContext_.p0WorldPose
                            : routePointWorldFromOrigin(
                                  homeContext_.p0WorldPose,
                                  returnP0TargetIndex_);
    const float positionError = distanceMm(current.xMm, current.yMm,
                                           target.xMm, target.yMm);
    const float tolerance = returnP0TargetIndex_ == 0U
                                ? static_cast<float>(
                                      MAP_RETURN_P0_POSITION_TOLERANCE_MM)
                                : static_cast<float>(
                                      MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM);
    if (returnP0State_ == ReturnP0State::REACQUIRE_ROUTE) {
      logReturnP0TraceMotion("REACQUIRE_DONE", current);
      debug_.print(",TARGET_X=");
      debug_.print(returnP0Projection_.projectedPose.xMm, 1);
      debug_.print(",TARGET_Y=");
      debug_.print(returnP0Projection_.projectedPose.yMm, 1);
      debug_.print(",CROSS_TRACK=");
      const float projectionError = distanceMm(
          current.xMm, current.yMm, returnP0Projection_.projectedPose.xMm,
          returnP0Projection_.projectedPose.yMm);
      debug_.print(projectionError, 1);
      debug_.print(",POS_ERR=");
      debug_.println(projectionError, 1);
    } else if (returnP0TargetIndex_ != 0U) {
      const Pose segmentStart = routePointWorldFromOrigin(
          homeContext_.p0WorldPose, returnP0TargetIndex_ + 1U);
      const float routeBearing = atan2f(target.yMm - segmentStart.yMm,
                                         target.xMm - segmentStart.xMm) *
                                 kRadToDeg;
      logReturnP0TraceMotion("WP_DONE", current);
      debug_.print(",TARGET_INDEX=");
      debug_.print(static_cast<unsigned>(returnP0TargetIndex_));
      debug_.print(",TARGET_X=");
      debug_.print(target.xMm, 1);
      debug_.print(",TARGET_Y=");
      debug_.print(target.yMm, 1);
      debug_.print(",POS_ERR=");
      debug_.print(positionError, 1);
      debug_.print(",HEADING_ERR_TO_ROUTE=");
      debug_.print(shortestDeltaDeg(routeBearing, current.headingDeg), 2);
      debug_.print(",CROSS_TRACK=");
      debug_.println(diagnosticCrossTrackToSegment(current, segmentStart,
                                                    target),
                     1);
    }
    if (positionError > tolerance) {
      if (returnP0TargetIndex_ == 0U &&
          returnP0PositionCorrectionAttempts_ <
              MAP_RETURN_P0_MAX_POSITION_CORRECTIONS) {
        ++returnP0PositionCorrectionAttempts_;
        returnP0State_ = ReturnP0State::P0_POSITION_APPROACH;
        if (!startReturnP0Position()) abortReturnToP0("P0_POSITION");
      } else {
        abortReturnToP0("POSITION_ERROR");
      }
      return true;
    }
    if (returnP0TargetIndex_ == 0U) {
      logReturnP0TraceMotion("P0_POSITION_ARRIVAL", current);
      logReturnP0TraceP0Error(current);
      debug_.println(",ARRIVAL=1");
      returnP0State_ = ReturnP0State::P0_POSITION_SETTLE;
      returnP0SettleSinceMs_ = millis();
      logReturnP0TraceMotion("P0_POSITION_SETTLE", current);
      logReturnP0TraceP0Error(current);
      debug_.println(",PHASE=ENTER,SETTLE_MS=0");
    } else {
      replayCurrentIndex_ = returnP0TargetIndex_;
      --returnP0TargetIndex_;
      replayTargetIndex_ = returnP0TargetIndex_;
      returnP0State_ = ReturnP0State::RETURN_WAYPOINT;
    }
    return true;
  }
  if (result.code == AiDistanceResultCode::OBSTACLE) {
    returnP0HeldState_ = returnP0State_;
    returnP0HeldTargetIndex_ = returnP0TargetIndex_;
    returnP0HeldSegmentStartIndex_ = returnP0SegmentStartIndex_;
    returnP0State_ = ReturnP0State::HOLD;
    enterReplayHold(MapHoldReason::OBSTACLE, false);
    return true;
  }
  abortReturnToP0(result.code == AiDistanceResultCode::ENCODER_FAULT
                      ? "ENCODER_FAULT"
                      : result.code == AiDistanceResultCode::HEADING_LOST
                          ? "HEADING_LOST"
                          : "MOVE_ERROR");
  return true;
}

MapController::Pose MapController::routePointWorld(uint16_t index) const {
  return routePointWorldFromOrigin(replayOrigin_, index);
}

MapController::Pose MapController::routePointWorldFromOrigin(
    const Pose& origin, uint16_t index) const {
  if (index >= route_.header.waypointCount) return origin;
  const MapWaypoint& local = route_.waypoints[index];
  // This is the same transform previously expressed directly with
  // replayOrigin_.headingDeg; the explicit origin parameter is what lets the
  // post-Teach action use the saved Teach origin without recapturing Pn.
  const float c = cosf(origin.headingDeg * kDegToRad);
  const float s = sinf(origin.headingDeg * kDegToRad);
  Pose world;
  world.xMm = origin.xMm + static_cast<float>(local.xMm) * c -
              static_cast<float>(local.yMm) * s;
  world.yMm = origin.yMm + static_cast<float>(local.xMm) * s +
              static_cast<float>(local.yMm) * c;
  world.headingDeg = normalizeDeg(
      origin.headingDeg + static_cast<float>(local.headingCdeg) / 100.0f);
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
      IsClosingMode(routeMode_) && replayDirection_ > 0 &&
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
  // Arrival heading is the direction of the active route edge, not the
  // direction from a laterally displaced chassis position into the waypoint.
  // This keeps arbitrary-angle routes correct without using stored Teach
  // heading or snapping to orthogonal directions.  The same canonical edge
  // geometry is also used for the normal replay turn and guided cross-track
  // reference; otherwise a lateral arrival error changes the next corner's
  // bearing (for example, 60 mm over a 700 mm edge looks like a 4.9 degree
  // short turn).
  const float incomingBearing =
      replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
  const float routeHeadingError =
      shortestDeltaDeg(incomingBearing, current.headingDeg);
  // Every replay leg uses the immutable saved edge as its reference.  Starting
  // a leg from the live arrival pose shifts the reference line and changes the
  // next corner bearing, so lateral error can accumulate while live
  // cross-track appears small.  Live pose remains the feedback used for
  // distance, heading and cross-track correction.
  const bool canonicalBackSegment = postTeachBackActive_;
  const float waypointToleranceMm = static_cast<float>(
      canonicalBackSegment ? MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM
                           : MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM);
  const Pose canonicalSegmentStart = routePointWorld(replayCurrentIndex_);
  const Pose guidedSegmentStart = canonicalSegmentStart;
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
  const bool inArrivalZone = targetDistance <= waypointToleranceMm;
  const bool arrivalHeadingWithinTolerance =
      fabsf(arrivalHeadingError) <= MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG;
  const float routeRad = incomingBearing * kDegToRad;
  const float targetAlongRouteMm =
      (replayTarget_.xMm - current.xMm) * cosf(routeRad) +
      (replayTarget_.yMm - current.yMm) * sinf(routeRad);
  const uint32_t nowMs = millis();
  bool arrivalHeadingViolationStable = false;
  if (!inArrivalZone || arrivalHeadingWithinTolerance) {
    replayArrivalHeadingViolationSinceMs_ = 0U;
  } else if (replayArrivalHeadingViolationSinceMs_ == 0U) {
    replayArrivalHeadingViolationSinceMs_ = nowMs;
  } else {
    arrivalHeadingViolationStable =
        (nowMs - replayArrivalHeadingViolationSinceMs_) >=
        MAP_GUIDE_ARRIVAL_HEADING_DEBOUNCE_MS;
  }

  ReplayRealignReason actionReason = replayRealignReason_;
  float desiredBearing = incomingBearing;
  float desiredHeadingError = routeHeadingError;
  bool coarsePreturn = false;
  bool realignAction = actionReason != ReplayRealignReason::NONE;

  if (actionReason == ReplayRealignReason::ARRIVAL) {
    realignAction = true;
    desiredBearing = incomingBearing;
    desiredHeadingError = arrivalHeadingError;
    if (!inArrivalZone) {
      const bool postTurnRecovery =
          targetDistance <= kPostTurnArrivalRecoveryToleranceMm;
      if (postTurnRecovery && arrivalHeadingWithinTolerance) {
        // A heading correction may move the chassis just outside the strict
        // 5 mm gate.  It is already at the same waypoint and correctly
        // aligned, so advancing is safer than restarting forward guidance.
        replayRealignReason_ = ReplayRealignReason::NONE;
        logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                            incomingBearing, current.headingDeg,
                            arrivalHeadingError, "ADVANCE_RECOVERED");
        advanceReplayAfterTarget();
        return true;
      }
      if (postTurnRecovery) {
        // Stay on the arrival correction path.  Do not issue a forward MOVE
        // while the target is only a few millimetres away.
        coarsePreturn = true;
      } else {
        if (!arrivalHeadingWithinTolerance) {
          // The old path resumed forward guidance here even though the
          // arrival heading was still invalid.  Keep this waypoint in the
          // in-place correction path until its heading is valid.
          coarsePreturn = true;
        } else if (targetAlongRouteMm < -kWaypointToleranceMm) {
          // Guided waypoint motion is forward-only.  If an arrival turn has
          // carried the chassis beyond the target, restarting it would drive
          // away from the same waypoint and can create a REALIGN loop.
          debug_.print("MAP,GUIDE,REALIGN,ABORT,WP=");
          debug_.print(static_cast<unsigned>(replayTargetIndex_));
          debug_.print(",REASON=ARRIVAL_OVERSHOOT,DIST_MM=");
          debug_.println(targetDistance, 1);
          abortReplay("ARRIVAL_OVERSHOOT");
          return false;
        } else {
          // The target is still ahead and the route heading is valid, so a
          // bounded forward recovery to the same waypoint is safe.
          replayRealignReason_ = ReplayRealignReason::NONE;
          replayArrivalTurnPending_ = false;
          replayArrivalTurnAttempts_ = 0U;
          logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                              incomingBearing, current.headingDeg,
                              arrivalHeadingError, "GUIDED");
          desiredBearing = incomingBearing;
          desiredHeadingError = routeHeadingError;
        }
      }
    } else if (arrivalHeadingWithinTolerance) {
      replayRealignReason_ = ReplayRealignReason::NONE;
      logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                          incomingBearing, current.headingDeg,
                          arrivalHeadingError, "ADVANCE");
      advanceReplayAfterTarget();
      return true;
    } else if (arrivalHeadingViolationStable) {
      // ARRIVAL realign always turns toward the incoming route edge.
      coarsePreturn = true;
    } else {
      // Stay stopped for one short bounded observation window. This rejects
      // a noisy sample just outside the strict completion gate without
      // weakening the +/-0.5-degree contract.
      return true;
    }
  } else if (actionReason == ReplayRealignReason::PATH) {
    realignAction = true;
    if (inArrivalZone) {
      // A PATH turn can end inside the arrival zone. Re-evaluate the arrival
      // heading gate before deciding whether this waypoint is complete.
      desiredBearing = incomingBearing;
      desiredHeadingError = arrivalHeadingError;
      if (arrivalHeadingWithinTolerance) {
        replayRealignReason_ = ReplayRealignReason::NONE;
        logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                            incomingBearing, current.headingDeg,
                            arrivalHeadingError, "ADVANCE");
        advanceReplayAfterTarget();
        return true;
      }
      if (arrivalHeadingViolationStable) {
        actionReason = ReplayRealignReason::ARRIVAL;
        replayRealignReason_ = actionReason;
        coarsePreturn = true;
      } else {
        return true;
      }
    } else if (fabsf(routeHeadingError) >
               MAP_REPLAY_PRETURN_TOLERANCE_DEG) {
      // PATH realign uses the canonical route-edge bearing.
      coarsePreturn = true;
    } else {
      replayRealignReason_ = ReplayRealignReason::NONE;
      logGuideRealignDone(actionReason, replayTargetIndex_, targetDistance,
                          incomingBearing, current.headingDeg,
                          routeHeadingError, "GUIDED");
    }
  } else if (inArrivalZone) {
    desiredBearing = incomingBearing;
    desiredHeadingError = arrivalHeadingError;
    if (arrivalHeadingWithinTolerance) {
      advanceReplayAfterTarget();
      return true;
    }
    if (!arrivalHeadingViolationStable) return true;
    actionReason = ReplayRealignReason::ARRIVAL;
    replayRealignReason_ = actionReason;
    realignAction = true;
    coarsePreturn = true;
  } else {
    const bool startupNoiseTurn = replayCurrentIndex_ == 0U &&
                                  replayTargetIndex_ == 1U &&
                                  fabsf(routeHeadingError) <=
                                      kReplayStartupTurnDeadbandDeg;
    coarsePreturn = fabsf(desiredHeadingError) >
                        MAP_REPLAY_PRETURN_TOLERANCE_DEG &&
                    !startupNoiseTurn;
  }

  if (realignAction && coarsePreturn) {
    if (actionReason == ReplayRealignReason::ARRIVAL) {
      if (replayArrivalTurnPending_ &&
          replayArrivalTurnWaypoint_ == replayTargetIndex_ &&
          replayArrivalTurnAttempts_ >= kMaxArrivalTurnAttempts) {
        debug_.print("MAP,GUIDE,REALIGN,ABORT,WP=");
        debug_.print(static_cast<unsigned>(replayTargetIndex_));
        debug_.println(",REASON=REALIGN_STUCK");
        abortReplay("REALIGN_STUCK");
        return false;
      }
      if (!replayArrivalTurnPending_ ||
          replayArrivalTurnWaypoint_ != replayTargetIndex_) {
        replayArrivalTurnPending_ = true;
        replayArrivalTurnWaypoint_ = replayTargetIndex_;
        replayArrivalTurnAttempts_ = 0U;
      }
      ++replayArrivalTurnAttempts_;
    }
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
    const int16_t mapTurnSpeed = min(replaySpeed_, TURN_MAX_SPEED);
    if (!robot_.startReplayTurnRelative(
            desiredHeadingError > 0.0f, fabsf(desiredHeadingError),
            mapTurnSpeed, segmentGeneration, AiTurnProfile::MAP_COARSE)) {
      replayOperation_ = MapReplayOperation::NONE;
      abortReplay("TURN_START");
      return false;
    }
  } else {
    replayOperation_ = MapReplayOperation::MOVE;
    replayTargetDeg_ = static_cast<int16_t>(lroundf(desiredBearing));
    logGuidePreturn(desiredHeadingError, "SKIP");
    if (!robot_.startReplayGuidedWaypoint(
            replayTarget_.xMm, replayTarget_.yMm, guidedSegmentStart.xMm,
            guidedSegmentStart.yMm,
            replaySpeed_, incomingBearing,
            static_cast<uint32_t>(waypointToleranceMm), segmentGeneration)) {
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
  replayArrivalHeadingViolationSinceMs_ = 0U;
  replayArrivalTurnPending_ = false;
  replayArrivalTurnWaypoint_ = 0U;
  replayArrivalTurnAttempts_ = 0U;
  const uint16_t count = route_.header.waypointCount;
  const bool logicalClosingEdge =
      IsClosingMode(routeMode_) && replayDirection_ > 0 &&
      fromIndex == count - 1U && targetIndex == 0U;
  replayCurrentIndex_ = targetIndex;
  if (logicalClosingEdge) {
    debug_.println("MAP,CLOSE_EDGE,DONE");
  }
  if (replayDirection_ > 0) {
    if (IsClosingMode(routeMode_) &&
        fromIndex == count - 1U && targetIndex == 0U) {
      // The segment from the final waypoint back to P0 is a real closing
      // edge for CLOSED and LOOP. A LOOP lap completes only after this
      // edge reaches P0; P0 is never treated as a terminal completion.
      if (routeMode_ == MapReplayMode::LOOP) {
        if (replayLapCounter_ != 0xFFFFFFFFUL) ++replayLapCounter_;
        debug_.print("MAP,LOOP,LAP_COMPLETE,LAP=");
        debug_.println(replayLapCounter_);
        if (loopTarget_ != MAP_LOOP_TARGET_INF &&
            replayLapCounter_ >= loopTarget_) {
          debug_.print("MAP,LOOP,TARGET_REACHED,LAP=");
          debug_.println(replayLapCounter_);
          completeReplay();
          return;
        }
        replayTargetIndex_ = count > 1U ? 1U : 0U;
        return;
      }
      if (routeMode_ == MapReplayMode::CLOSED) {
        completeReplay();
        return;
      }
    }
    if (replayCurrentIndex_ < count - 1U) {
      replayTargetIndex_ = replayCurrentIndex_ + 1U;
      return;
    }
    if (IsClosingMode(routeMode_) &&
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
    if (routeMode_ == MapReplayMode::RETURN ||
        routeMode_ == MapReplayMode::PING_PONG) {
      replayReturnPhase_ = ReplayReturnPhase::INBOUND;
      const bool ping = routeMode_ == MapReplayMode::PING_PONG;
      debug_.print(ping ? "MAP,PING,TURNAROUND,WP="
                       : "MAP,RETURN,TURNAROUND,WP=");
      debug_.println(static_cast<unsigned>(replayCurrentIndex_));
      debug_.print(ping ? "MAP,PING,PHASE=BACK,CYCLE="
                       : "MAP,RETURN,PHASE=BACK,FROM=");
      if (ping) debug_.print(replayCycleCounter_ + 1U);
      if (!ping) {
        debug_.print(static_cast<unsigned>(replayCurrentIndex_));
        debug_.print(",TO=");
      } else {
        debug_.print(",FROM=");
        debug_.print(static_cast<unsigned>(replayCurrentIndex_));
        debug_.print(",TO=");
      }
      debug_.println(static_cast<unsigned>(count - 2U));
    } else {
      replayReturnPhase_ = ReplayReturnPhase::NONE;
    }
    replayTargetIndex_ = count - 2U;
    return;
  }

  if (replayCurrentIndex_ > 0U) {
    replayTargetIndex_ = replayCurrentIndex_ - 1U;
    return;
  }
  if (postTeachBackActive_) {
    debug_.println("MAP,BACK_P0,COMPLETE,WP=0");
    completeReplay();
    return;
  }
  if (routeMode_ == MapReplayMode::RETURN) {
    completeReplay();
    return;
  }
  if (routeMode_ == MapReplayMode::PING_PONG) {
    if (replayCycleCounter_ != 0xFFFFFFFFUL) ++replayCycleCounter_;
    debug_.print("MAP,PING,CYCLE_COMPLETE,CYCLE=");
    debug_.println(replayCycleCounter_);
    if (loopTarget_ != MAP_LOOP_TARGET_INF &&
        replayCycleCounter_ >= loopTarget_) {
      debug_.print("MAP,PING,TARGET_REACHED,CYCLE=");
      debug_.println(replayCycleCounter_);
      completeReplay();
      return;
    }
    replayDirection_ = 1;
    replayReturned_ = false;
    replayReturnPhase_ = ReplayReturnPhase::OUTBOUND;
    replayTargetIndex_ = 1U;
    debug_.print("MAP,PING,PHASE=OUT,CYCLE=");
    debug_.println(replayCycleCounter_ + 1U);
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
  if (reason != MapHoldReason::OBSTACLE) {
    inhibitAutonomousResume(HoldReasonName(reason));
  }
  if (reason == MapHoldReason::OBSTACLE &&
      !obstacleDetourContextActive()) {
    // A fresh production obstacle result starts a new one-attempt detour
    // lifecycle.  An ABORTED/COMPLETE detour is deliberately not reset here;
    // it can only be replaced by a new replay/obstacle event.
    obstacleDetourAttempts_ = 0U;
    obstacleDetourDecision_ = ObstacleDecision::HOLD;
    obstacleDetourStartMs_ = 0U;
    obstacleDetourClearSinceMs_ = 0U;
    obstacleDetourGeneration_ = 0U;
    obstacleDetourTravelBudgetUsedMm_ = 0U;
    obstacleDetourTurnBudgetUsedDeg_ = 0.0f;
  }
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  // Keep the valid MAP mission active while an obstacle is held.  This makes
  // the Phase 3 entry contract explicit without allowing normal replay to
  // advance: HOLD remains the current replay operation until START decides
  // between the Phase 1 resume path and the bounded detour.
  replayActive_ = reason == MapHoldReason::OBSTACLE;
  replayOperation_ = MapReplayOperation::HOLD;
  replayReason_ = HoldReasonName(reason);
  holdReason_ = reason;
  replayResumeAllowed_ = allowResume;
  if (returnP0InProgress() && reason != MapHoldReason::OBSTACLE) {
    // Phase 2 deliberately keeps Return-to-P0 non-resumable after a user or
    // external stop. Home remains valid for a fresh request.
    returnP0State_ = ReturnP0State::HOLD;
    replayResumeAllowed_ = false;
  }
  obstacleClearSinceMs_ = 0U;
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
  if (postTeachBackActive_) {
    debug_.print("MAP,BACK_P0,HOLD,WP=");
    debug_.println(static_cast<unsigned>(replayTargetIndex_));
  }
  if (routeMode_ == MapReplayMode::LOOP) {
    debug_.print("MAP,LOOP,HOLD,LAP=");
    debug_.print(replayLapCounter_);
    debug_.print(",WP=");
    debug_.println(static_cast<unsigned>(replayTargetIndex_));
  } else if (routeMode_ == MapReplayMode::RETURN) {
    debug_.print("MAP,RETURN,HOLD,PHASE=");
    debug_.print(returnPhaseName(replayReturnPhase_));
    debug_.print(",WP=");
    debug_.println(static_cast<unsigned>(replayTargetIndex_));
  } else if (routeMode_ == MapReplayMode::PING_PONG) {
    debug_.print("MAP,PING,HOLD,PHASE=");
    debug_.print(returnPhaseName(replayReturnPhase_));
    debug_.print(",CYCLE=");
    debug_.print(replayCycleCounter_ + 1U);
    debug_.print(",WP=");
    debug_.println(static_cast<unsigned>(replayTargetIndex_));
  }
}

void MapController::abortReplay(const char* reason) {
  const bool wasReturnP0 = returnP0InProgress();
  const bool wasPostTeachBack = postTeachBackActive_;
  robot_.stopImmediately(true);
  nextReplayGeneration();
  clearReplayResumeContext();
  replayActive_ = false;
  replayReason_ = reason != nullptr ? reason : "ERROR";
  if (wasPostTeachBack) {
    invalidatePostTeachBack(reason != nullptr ? reason : "ERROR");
  }
  mode_ = MapControllerMode::SAVED;
  if (wasReturnP0) {
    returnP0State_ = ReturnP0State::ABORTED;
    returnP0SegmentGeneration_ = 0U;
    returnP0TurnPending_ = false;
  }
  statusDirty_ = true;
  debug_.print("MAP,REPLAY=ABORT,REASON=");
  debug_.println(replayReason_);
}

void MapController::completeReplay() {
  const bool wasReturnP0 = returnP0InProgress();
  const bool wasPostTeachBack = postTeachBackActive_;
  const uint32_t completedLap = replayLapCounter_;
  const uint32_t completedCycle = replayCycleCounter_;
  const bool wasReturn = routeMode_ == MapReplayMode::RETURN;
  const bool wasPing = routeMode_ == MapReplayMode::PING_PONG;
  nextReplayGeneration();
  clearReplayResumeContext();
  replayLapCounter_ = completedLap;
  replayCycleCounter_ = completedCycle;
  replayActive_ = false;
  replayReason_ = "DONE";
  if (wasPostTeachBack) {
    invalidatePostTeachBack("COMPLETE");
    replayReason_ = "BACK_COMPLETE";
    replayCurrentIndex_ = 0U;
    replayTargetIndex_ = 0U;
    postTeachBackComplete_ = true;
  }
  mode_ = MapControllerMode::REPLAY_COMPLETE;
  statusDirty_ = true;
  debug_.println("MAP,REPLAY_COMPLETE");
  if (wasPostTeachBack) debug_.println("MAP,BACK_P0,COMPLETE");
  if (wasReturn) {
    debug_.println("MAP,RETURN,COMPLETE,WP=0");
    debug_.println("MAP,RETURN,PHASE=NONE");
  } else if (wasPing) {
    debug_.print("MAP,PING,COMPLETE,CYCLES=");
    debug_.println(replayCycleCounter_);
  }
  if (wasReturnP0) {
    returnP0State_ = ReturnP0State::COMPLETE;
    returnP0SegmentGeneration_ = 0U;
    returnP0TurnPending_ = false;
  }
}

void MapController::cancelReplay(const char* reason) {
  const bool wasReturnP0 = returnP0InProgress();
  const bool wasPostTeachBack = postTeachBackActive_;
  const bool wasClosedLoop = routeMode_ == MapReplayMode::LOOP;
  const bool wasReturn = routeMode_ == MapReplayMode::RETURN;
  const bool wasPing = routeMode_ == MapReplayMode::PING_PONG;
  const ReplayReturnPhase cancelledReturnPhase = replayReturnPhase_;
  const uint32_t cancelledLap = replayLapCounter_;
  robot_.stopImmediately(true);
  nextReplayGeneration();
  clearReplayResumeContext();
  replayActive_ = false;
  replayReason_ = reason != nullptr ? reason : "CANCELLED";
  if (wasPostTeachBack) {
    invalidatePostTeachBack(reason != nullptr ? reason : "CANCELLED");
    debug_.print("MAP,BACK_P0,CANCEL,REASON=");
    debug_.println(replayReason_);
  }
  mode_ = storeState_ == MapStoreState::SAVED ? MapControllerMode::SAVED
                                              : MapControllerMode::READY;
  if (wasReturnP0) {
    returnP0State_ = ReturnP0State::ABORTED;
    returnP0SegmentGeneration_ = 0U;
    returnP0TurnPending_ = false;
  }
  statusDirty_ = true;
  ps2_.disarmMapInput();
  debug_.print("MAP,CANCEL,REASON=");
  debug_.println(replayReason_);
  if (wasClosedLoop) {
    debug_.print("MAP,LOOP,CANCEL,LAP=");
    debug_.println(cancelledLap);
  } else if (wasReturn) {
    debug_.print("MAP,RETURN,CANCEL,PHASE=");
    debug_.println(returnPhaseName(cancelledReturnPhase));
  } else if (wasPing) {
    debug_.print("MAP,PING,CANCEL,PHASE=");
    debug_.print(returnPhaseName(cancelledReturnPhase));
    debug_.print(",CYCLE=");
    debug_.println(replayCycleCounter_ + 1U);
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
  replayArrivalHeadingViolationSinceMs_ = 0U;
  replayArrivalTurnPending_ = false;
  replayArrivalTurnWaypoint_ = 0U;
  replayArrivalTurnAttempts_ = 0U;
  holdReason_ = MapHoldReason::NONE;
  obstacleClearSinceMs_ = 0U;
  replayOperation_ = MapReplayOperation::NONE;
  replayCurrentIndex_ = 0U;
  replayTargetIndex_ = 1U;
  replayDirection_ = 1;
  replayReturned_ = false;
  replayReturnPhase_ = ReplayReturnPhase::NONE;
  replayOrigin_ = {};
  replayHoldPose_ = {};
  replayOriginRouteGeneration_ = 0U;
  replayOriginResetGeneration_ = 0U;
  replayOriginHeadingResetGeneration_ = 0U;
  returnP0HeldState_ = ReturnP0State::IDLE;
  returnP0HeldTargetIndex_ = 0U;
  returnP0HeldSegmentStartIndex_ = 0U;
  replayTargetDistanceMm_ = 0U;
  replayTargetDeg_ = 0;
  replayGuideBearingDeg_ = 0.0f;
  replayTravelMm_ = 0U;
  replayErrorMm_ = 0U;
  replayLapCounter_ = 0U;
  replayCycleCounter_ = 0U;
  missionInitiator_ = MapMissionInitiator::NONE;
  autonomousResumeInhibited_ = true;
  obstacleDetourPhase_ = ObstacleDetourPhase::IDLE;
  obstacleDetourDecision_ = ObstacleDecision::HOLD;
  obstacleDetourAwayRight_ = false;
  obstacleDetourAttempts_ = 0U;
  obstacleDetourStartMs_ = 0U;
  obstacleDetourClearSinceMs_ = 0U;
  obstacleDetourGeneration_ = 0U;
  obstacleDetourTravelBudgetUsedMm_ = 0U;
  obstacleDetourTurnBudgetUsedDeg_ = 0.0f;
  obstacleDetourStartPose_ = {};
  obstacleDetourOriginalCurrentIndex_ = 0U;
  obstacleDetourOriginalTargetIndex_ = 0U;
  obstacleDetourOriginalDirection_ = 1;
  obstacleDetourOriginalRouteGeneration_ = 0U;
  obstacleDetourOriginalReplayGeneration_ = 0U;
}

void MapController::serviceObstacleHold() {
  if (mode_ != MapControllerMode::REPLAY_HOLD ||
      holdReason_ != MapHoldReason::OBSTACLE) {
    obstacleClearSinceMs_ = 0U;
    return;
  }

  const uint32_t now = millis();
  const bool aiAutoResume =
      missionInitiator_ == MapMissionInitiator::AI_VOICE;
  if (aiAutoResume && (!odometry_.ready() || !odometry_.healthy())) {
    inhibitAutonomousResume("ODOMETRY");
  } else if (aiAutoResume &&
             (!fusion_.ready() || fusion_.health() == FusionHealth::NO_SOURCE)) {
    inhibitAutonomousResume("FUSION");
  } else if (aiAutoResume &&
             odometry_.resetGeneration() != replayOriginResetGeneration_) {
    inhibitAutonomousResume("RESET_BOUNDARY");
  } else if (aiAutoResume && robot_.headingResetGeneration() !=
                                 replayOriginHeadingResetGeneration_) {
    inhibitAutonomousResume("HEADING_RESET_BOUNDARY");
  } else if (aiAutoResume && route_.header.generation !=
                                 replayOriginRouteGeneration_) {
    inhibitAutonomousResume("ROUTE_CHANGED");
  } else if (aiAutoResume && selectedSlot_ != replayContextSlot_) {
    inhibitAutonomousResume("SLOT_CHANGED");
  } else if (aiAutoResume &&
             (!robot_.motorsStopped() || robot_.aiMotionActive() ||
              robot_.motionOwner() != MotionOwner::NONE ||
              robot_.brakeEnabled())) {
    inhibitAutonomousResume("MOTION_CONTEXT");
  } else if (aiAutoResume && !replayHoldPoseValid_) {
    inhibitAutonomousResume("POSE");
  } else if (aiAutoResume) {
    Pose current;
    if (!readPose(current) ||
        distanceMm(current.xMm, current.yMm, replayHoldPose_.xMm,
                   replayHoldPose_.yMm) > kReplayPoseHoldToleranceMm ||
        fabsf(shortestDeltaDeg(current.headingDeg,
                               replayHoldPose_.headingDeg)) >
            kReplayPoseHoldToleranceDeg) {
      inhibitAutonomousResume("POSE_DRIFT");
    }
  }
  if (aiAutoResume && !ultrasonic_.isFresh()) {
    inhibitAutonomousResume("SENSOR_STALE");
  } else if (aiAutoResume && !ultrasonic_.healthy()) {
    inhibitAutonomousResume("SENSOR_UNHEALTHY");
  } else if (aiAutoResume && ultrasonic_.overallZone() == ObstacleZone::UNKNOWN) {
    inhibitAutonomousResume("SENSOR_UNKNOWN");
  }
  const bool obstacleLiveClear =
      ultrasonic_.isFresh() && ultrasonic_.healthy() &&
      ultrasonic_.overallZone() == ObstacleZone::CLEAR;
  if (!obstacleLiveClear) {
    if (obstacleClearSinceMs_ != 0U) {
      debug_.println("OBS,HOLD,WAIT");
    }
    obstacleClearSinceMs_ = 0U;
    return;
  }
  if (obstacleClearSinceMs_ == 0U) {
    obstacleClearSinceMs_ = now;
    debug_.println("OBS,HOLD,CLEAR_PENDING");
    if (aiAutoResume && !autonomousResumeInhibited_) {
      debug_.println("OBS,HOLD,AI_AUTO_CLEAR_PENDING");
    }
  }
  if (aiAutoResume && !autonomousResumeInhibited_ &&
      (now - obstacleClearSinceMs_) >= AI_OBSTACLE_AUTO_RESUME_CLEAR_MS) {
    const char* rejectReason = nullptr;
    const bool resumed = returnP0InProgress()
                             ? resumeReturnP0FromObstacleHold(rejectReason)
                             : resumeReplayFromHold(ReplayResumeSource::AI_AUTO,
                                                    rejectReason);
    if (!resumed) {
      inhibitAutonomousResume(rejectReason != nullptr ? rejectReason
                                                       : "AUTO_RESUME_REJECT");
    }
  }
}

bool MapController::resumeReturnP0FromObstacleHold(const char*& rejectReason) {
  rejectReason = nullptr;
  if (returnP0Source_ != ReturnP0Source::AI_VOICE ||
      returnP0State_ != ReturnP0State::HOLD ||
      holdReason_ != MapHoldReason::OBSTACLE) {
    rejectReason = "RETURN_HOLD_CONTEXT";
    return false;
  }
  if (autonomousResumeInhibited_ || !homeContext_.valid ||
      selectedSlot_ != homeContext_.slot ||
      route_.header.generation != homeContext_.routeGeneration ||
      odometry_.resetGeneration() != homeContext_.odometryResetGeneration ||
      robot_.headingResetGeneration() != homeContext_.headingResetGeneration ||
      !odometry_.ready() || !odometry_.healthy() || !fusion_.ready() ||
      fusion_.health() == FusionHealth::NO_SOURCE ||
      !ultrasonic_.isFresh() || !ultrasonic_.healthy() ||
      ultrasonic_.overallZone() != ObstacleZone::CLEAR ||
      !robot_.motorsStopped() || robot_.aiMotionActive() ||
      robot_.motionOwner() != MotionOwner::NONE || ps2_.motionCommandActive() ||
      ps2_.state().r3 || returnP0HeldTargetIndex_ >= route_.header.waypointCount) {
    rejectReason = "RETURN_RESUME_GATE";
    return false;
  }
  returnP0TargetIndex_ = returnP0HeldTargetIndex_;
  returnP0SegmentStartIndex_ = returnP0HeldSegmentStartIndex_;
  returnP0State_ = returnP0HeldState_;
  nextReplayGeneration();
  returnP0Generation_ = replayGeneration_;
  returnP0SegmentGeneration_ = 0U;
  replaySegmentGeneration_ = 0U;
  replayActive_ = true;
  replayOperation_ = MapReplayOperation::NONE;
  replayResumeAllowed_ = false;
  holdReason_ = MapHoldReason::NONE;
  obstacleClearSinceMs_ = 0U;
  mode_ = MapControllerMode::REPLAY_RUNNING;
  statusDirty_ = true;

  bool started = false;
  switch (returnP0State_) {
    case ReturnP0State::REACQUIRE_ROUTE:
      started = startReturnReacquire();
      break;
    case ReturnP0State::RETURN_WAYPOINT:
      started = startReturnWaypoint();
      break;
    case ReturnP0State::P0_POSITION_APPROACH:
    case ReturnP0State::P0_POSITION_SETTLE:
    case ReturnP0State::P0_HEADING_RESTORE:
    case ReturnP0State::P0_HEADING_SETTLE:
      returnP0State_ = ReturnP0State::P0_POSITION_APPROACH;
      started = startReturnP0Position();
      break;
    default:
      rejectReason = "RETURN_HOLD_STATE";
      started = false;
      break;
  }
  if (!started) {
    rejectReason = "RETURN_RESUME_START";
    abortReturnToP0(rejectReason);
    return false;
  }
  debug_.print("MAP,RETURN_P0,AI_AUTO_RESUME,STATE=");
  debug_.println(returnP0StateName(returnP0State_));
  rejectReason = "OK";
  return true;
}

bool MapController::canResumeReplay(const char*& rejectReason) {
  return canResumeReplay(ReplayResumeSource::PS2_START, rejectReason);
}

bool MapController::canResumeReplay(ReplayResumeSource source,
                                    const char*& rejectReason) {
  rejectReason = nullptr;
  if (!loadedValid_) {
    rejectReason = "NOT_SAVED";
    return false;
  }
  if (!replayResumeAllowed_ || mode_ != MapControllerMode::REPLAY_HOLD) {
    rejectReason = "HOLD_NOT_RESUMABLE";
    return false;
  }
  if (source == ReplayResumeSource::AI_AUTO) {
    if (missionInitiator_ != MapMissionInitiator::AI_VOICE) {
      rejectReason = "INITIATOR";
      return false;
    }
    if (autonomousResumeInhibited_) {
      rejectReason = "AUTO_RESUME_INHIBITED";
      return false;
    }
    if (holdReason_ != MapHoldReason::OBSTACLE) {
      rejectReason = "HOLD_REASON";
      return false;
    }
  }
  if (selectedSlot_ != replayContextSlot_) {
    rejectReason = "SLOT_CHANGED";
    return false;
  }
  if (!replayOriginValid_ ||
      !IsReplayModeAllowed(MapRouteType::OPEN, routeMode_)) {
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
  if (source == ReplayResumeSource::PS2_START) {
    if (!ps2_.state().frameFresh || ps2_.frameTimedOut(now) ||
        ps2_.motionCommandActive()) {
      rejectReason = "PS2_NOT_NEUTRAL";
      return false;
    }
  } else if (ps2_.motionCommandActive() || ps2_.state().r3) {
    // AI resume is independent of receiver freshness, but never outranks an
    // operator takeover or the PS2 STOP boundary.
    rejectReason = "PS2_TAKEOVER";
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
  if (holdReason_ == MapHoldReason::OBSTACLE) {
    if (!obstacleLiveClear) {
      obstacleClearSinceMs_ = 0U;
      rejectReason = "OBSTACLE_NOT_CLEAR";
      return false;
    }
    if (obstacleClearSinceMs_ == 0U) {
      obstacleClearSinceMs_ = now;
      debug_.println("OBS,HOLD,CLEAR_PENDING");
    }
    const uint32_t requiredClearMs =
        source == ReplayResumeSource::AI_AUTO
            ? AI_OBSTACLE_AUTO_RESUME_CLEAR_MS
            : OBSTACLE_CLEAR_STABLE_MS;
    if ((now - obstacleClearSinceMs_) < requiredClearMs) {
      rejectReason = "OBSTACLE_NOT_CLEAR";
      return false;
    }
  } else {
    const bool obstacleGraceClear =
        ultrasonic_.overallZone() == ObstacleZone::CLEAR &&
        ultrasonic_.hasRecentClearWindow(now);
    if (!obstacleLiveClear && !obstacleGraceClear) {
      rejectReason = "OBSTACLE_NOT_CLEAR";
      return false;
    }
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

bool MapController::resumeReplayFromHold(ReplayResumeSource source,
                                         const char*& rejectReason) {
  if (!canResumeReplay(source, rejectReason)) return false;
  // Keep the original route origin and route coordinates. The next segment
  // computes its target from the current live pose, so coast after HOLD never
  // turns an old remaining-distance value into ground truth.
  const bool obstacleHold = holdReason_ == MapHoldReason::OBSTACLE;
  nextReplayGeneration();
  replaySegmentGeneration_ = 0U;
  replayActive_ = true;
  mode_ = MapControllerMode::REPLAY_RUNNING;
  replayOperation_ = MapReplayOperation::NONE;
  replayReason_ = "RESUME";
  holdReason_ = MapHoldReason::NONE;
  obstacleClearSinceMs_ = 0U;
  statusDirty_ = true;
  if (obstacleHold && source == ReplayResumeSource::AI_AUTO) {
    debug_.print("OBS,HOLD,AI_AUTO_RESUME,WP=");
    debug_.print(static_cast<unsigned>(replayTargetIndex_));
    debug_.print(",GEN=");
    debug_.println(replayGeneration_);
  } else if (obstacleHold) {
    debug_.println("OBS,HOLD,RESUME");
  }
  rejectReason = "OK";
  return true;
}

void MapController::inhibitAutonomousResume(const char* reason) {
  if (autonomousResumeInhibited_) return;
  autonomousResumeInhibited_ = true;
  debug_.print("OBS,HOLD,AI_AUTO_INHIBIT,REASON=");
  debug_.println(reason != nullptr ? reason : "UNKNOWN");
}

bool MapController::consumeReplayTurnResult(const AiTurnResult& result) {
  if (result.owner != MotionOwner::REPLAY) return false;
  if (returnP0InProgress()) return consumeReturnTurnResult(result);
  if (obstacleDetourContextActive()) {
    return consumeObstacleDetourTurnResult(result);
  }
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
  if (returnP0InProgress()) return consumeReturnDistanceResult(result);
  if (obstacleDetourContextActive()) {
    return consumeObstacleDetourDistanceResult(result);
  }
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
  const float waypointToleranceMm = static_cast<float>(
      postTeachBackActive_ ? MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM
                           : MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM);
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
    const float incomingBearing =
        replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
    const bool inArrivalZone = targetDistance <= waypointToleranceMm;
  replayRealignReason_ = inArrivalZone ? ReplayRealignReason::ARRIVAL
                                         : ReplayRealignReason::PATH;
    if (replayRealignReason_ != ReplayRealignReason::ARRIVAL) {
      replayArrivalTurnPending_ = false;
      replayArrivalTurnWaypoint_ = 0U;
      replayArrivalTurnAttempts_ = 0U;
    }
    // A guided result must never reintroduce a live-pose-to-target bearing:
    // that geometry is displaced by the accumulated cross-track error and
    // would make the next MAP or BACK corner turn short.  Both PATH and
    // ARRIVAL realign continue toward the immutable saved edge.
    const float desiredBearing = incomingBearing;
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
    const float arrivalBearing =
        replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_);
    const float arrivalHeadingError =
        shortestDeltaDeg(arrivalBearing, current.headingDeg);
    replayTargetDistanceMm_ = static_cast<uint32_t>(lroundf(positionError));
    replayErrorMm_ = replayTargetDistanceMm_;
    if (positionError <= waypointToleranceMm &&
        fabsf(arrivalHeadingError) <=
            MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG) {
      logSegmentDone(generation, from, to, MapReplayOperation::MOVE,
                     static_cast<float>(replayTargetDistanceMm_),
                     static_cast<float>(replayTravelMm_), 0, 0.0f);
      logGuideDone(to, positionError, arrivalBearing, current.headingDeg,
                   arrivalHeadingError);
      replayRealignReason_ = ReplayRealignReason::NONE;
      replayArrivalTurnPending_ = false;
      replayArrivalTurnWaypoint_ = 0U;
      replayArrivalTurnAttempts_ = 0U;
      advanceReplayAfterTarget();
    } else if (positionError <= waypointToleranceMm) {
      // Do not let a distance-complete result bypass the arrival heading
      // gate. The next cycle performs ARRIVAL coarse correction.
      replayRealignReason_ = ReplayRealignReason::ARRIVAL;
      replayArrivalTurnPending_ = false;
      replayArrivalTurnWaypoint_ = to;
      replayArrivalTurnAttempts_ = 0U;
      logGuideRealign(replayRealignReason_, to, positionError, arrivalBearing,
                      current.headingDeg, arrivalHeadingError);
    } else {
      // A live pose update can move the chassis outside the arrival zone
      // before this result is consumed. Re-enter PATH guidance to the same
      // target instead of advancing on a stale completion.
      replayRealignReason_ = ReplayRealignReason::PATH;
      replayArrivalTurnPending_ = false;
      replayArrivalTurnWaypoint_ = 0U;
      replayArrivalTurnAttempts_ = 0U;
      const float routeHeadingError =
          shortestDeltaDeg(arrivalBearing, current.headingDeg);
      logGuideRealign(replayRealignReason_, to, positionError, arrivalBearing,
                      current.headingDeg, routeHeadingError);
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
      invalidatePostTeachBack("DELETE");
      invalidateHomeContext("DELETE");
      loadedValid_ = false;
      storeState_ = MapStoreState::EMPTY;
      storageErrorReason_ = MapStorageErrorReason::NONE;
      mode_ = MapControllerMode::READY;
      log("DELETE=OK");
    } else {
      invalidateHomeContext("STORAGE_ERROR");
      storeState_ = MapStoreState::STORAGE_ERROR;
      storageErrorReason_ = MapStorageErrorReason::GENERIC;
      mode_ = MapControllerMode::READY;
      log("DELETE=FAIL");
    }
    statusDirty_ = true;
    return;
  }
  if (savePending_) {
    if (!robot_.motorsStopped() || robot_.aiMotionActive()) return;
    savePending_ = false;
    const MapReplayMode runtimeMode = routeMode_;
    const bool saved = store_.save(selectedSlot_, route_);
    if (saved) {
      loadedValid_ = true;
      storeState_ = MapStoreState::SAVED;
      storageErrorReason_ = MapStorageErrorReason::NONE;
      teachOldRouteAvailable_ = false;
      routeType_ = MapRouteType::OPEN;
      routeMode_ = runtimeMode;
      normalizeRouteForRuntime(route_, runtimeMode);
      mode_ = MapControllerMode::SAVED;
      log("TEACH_SAVE=OK");
      armHomeContextAfterSave();
      armPostTeachBackAfterSave();
    } else {
      // The previous active A/B record remains untouched on a failed erase,
      // program or read-back. Restore it into RAM when this Teach session
      // started from a valid route; otherwise discard the failed new route.
      invalidatePostTeachBack("STORAGE_ERROR");
      invalidateHomeContext("STORAGE_ERROR");
      const bool restored = teachOldRouteAvailable_ && loadSelected();
      teachOldRouteAvailable_ = false;
      storageErrorReason_ = MapStorageErrorReason::TEACH_SAVE;
      storeState_ = MapStoreState::STORAGE_ERROR;
      if (restored) {
        mode_ = MapControllerMode::SAVED;
        log("TEACH_SAVE=FAIL,OLD_ROUTE_RETAINED=1");
      } else {
        route_ = {};
        loadedValid_ = false;
        mode_ = MapControllerMode::READY;
        log("TEACH_SAVE=FAIL,NO_VALID_ROUTE=1");
      }
    }
    statusDirty_ = true;
    return;
  }
  if (modeSavePending_) {
    if (!loadedValid_ || !robot_.motorsStopped() || robot_.aiMotionActive()) return;
    modeSavePending_ = false;
    route_.header.routeType = static_cast<uint8_t>(MapRouteType::OPEN);
    route_.header.replayMode = static_cast<uint8_t>(routeMode_);
    updateRouteHeaderForSave(route_);
    if (!store_.save(selectedSlot_, route_)) {
      invalidateHomeContext("STORAGE_ERROR");
      routeMode_ = modeBeforeSave_;
      normalizeRouteForRuntime(route_, routeMode_);
      storeState_ = MapStoreState::STORAGE_ERROR;
      storageErrorReason_ = MapStorageErrorReason::MODE_SAVE;
      mode_ = MapControllerMode::SAVED;
      log("REPLAY_MODE_SAVE=FAIL");
    } else {
      invalidateHomeContext("ROUTE_GENERATION");
      storeState_ = MapStoreState::SAVED;
      storageErrorReason_ = MapStorageErrorReason::NONE;
      normalizeRouteForRuntime(route_, routeMode_);
      log("REPLAY_MODE_SAVE=OK");
    }
    statusDirty_ = true;
  }
}

void MapController::publishStatus() {
  const char* backReadyReason = nullptr;
  const bool backReady = backReadyP0Available(backReadyReason);
  const bool returnActive = returnP0InProgress();
  const bool returnComplete = returnP0State_ == ReturnP0State::COMPLETE;
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
  const bool settingsUi = mode_ == MapControllerMode::SETTINGS ||
                          mode_ == MapControllerMode::HELP;
  const MapUserMode displayUserMode = settingsUi ? settingsUserMode_ : userMode_;
  const MapReplayMode displayMode =
      executionModeFor(displayUserMode,
                       settingsUi ? settingsRepeatTarget_ : loopTarget_);
  const int16_t displaySpeed = settingsUi ? settingsSpeed_ : replaySpeed_;
  const uint8_t displayLoopTarget = settingsUi ? settingsRepeatTarget_
                                                : loopTarget_;
  display_.setMapStatus(
      static_cast<uint8_t>(selectedSlot_), static_cast<uint8_t>(metadata.state),
      static_cast<uint8_t>(mode_), points, STM32_MAP_MAX_WAYPOINTS,
      metadata.routeLengthMm, replayWp,
      loadedValid_ ? route_.header.waypointCount : 0U,
      replayTargetDistanceMm_, replayTravelMm_, replayErrorMm_,
      static_cast<uint8_t>(replayOperation_),
      static_cast<uint8_t>(metadata.routeType),
      static_cast<uint8_t>(displayMode),
      static_cast<uint8_t>(replayReturnPhase_),
      static_cast<uint8_t>(holdReason_), replayTargetDeg_, replayLapCounter_,
      closeCandidateDistanceMm_, closeCandidateHeadingDeg_,
      static_cast<uint8_t>(settingsItem_), displaySpeed, displayLoopTarget,
      helpPage_, static_cast<uint8_t>(storageErrorReason_), loadedValid_,
      static_cast<uint8_t>(displayUserMode), replayCycleCounter_,
      backReady, postTeachBackActive_ || returnActive,
      postTeachBackComplete_ || returnComplete);
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
  debug_.print(",ARR_BLEND=");
  debug_.print(robot_.guidedArrivalBlend(), 2);
  debug_.print(",PID_P=");
  debug_.print(robot_.guidedPidP(), 2);
  debug_.print(",PID_I=");
  debug_.print(robot_.guidedPidI(), 2);
  debug_.print(",PID_D=");
  debug_.print(robot_.guidedPidD(), 2);
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
  debug_.print(postTeachBackActive_
                   ? MAP_GUIDE_BACK_ARRIVAL_POSITION_TOLERANCE_MM
                   : MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM);
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

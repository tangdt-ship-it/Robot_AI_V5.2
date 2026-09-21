#ifndef STM32_LOCAL_MAP_CONTROLLER_H
#define STM32_LOCAL_MAP_CONTROLLER_H

#include <Arduino.h>

#include <control/robot_controller.h>
#include <display/lcd_display.h>
#include <encoders/wheel_odometry.h>
#include <localization/heading_fusion.h>
#include <map/route_cleaner.h>
#include <map/map_types.h>
#include <map/semantic_route_optimizer.h>
#include <map/route_store.h>
#include <ps2/ps2_controller.h>
#include <sensors/obstacle_classifier.h>
#include <sensors/ultrasonic_sensor.h>

// STM32-local Teach/Replay controller. It observes PS2, odometry, Heading and
// obstacle state, and is the only runtime MAP owner. It never sends MCP
// correlation frames and it never runs motor code from a worker/task.
class MapController {
 public:
  MapController(RobotController& robot, Ps2Controller& ps2,
                LcdDisplay& display, WheelOdometry& odometry,
                HeadingFusion& fusion, UltrasonicSensor& ultrasonic,
                ObstacleClassifier& obstacleClassifier, Print& debugStream);

  void begin();
  // Drain only the physical MAP input boundary. Main calls this before
  // asynchronous motion-result delivery so X can invalidate stale results;
  // the normal update then advances replay after results are consumed.
  void processInput();
  void update();

  // Unified MAP mission entry for physical PS2 and authenticated AI voice.
  bool requestStart(MapMissionInitiator initiator);
  bool requestRunMap(uint8_t slot, MapMissionInitiator initiator,
                     const char*& reason);
  bool requestReturnToP0(ReturnP0Source source, const char*& reason);
  // Called at the existing RobotLink STOP boundary after the electrical stop
  // has been consumed. It never starts motion or changes the wire protocol.
  void notifyExternalStop();

  // Main uses these hooks to consume REPLAY results internally. Returning
  // false leaves normal MCP SID/OP RobotLink routing unchanged.
  bool consumeReplayTurnResult(const AiTurnResult& result);
  bool consumeReplayDistanceResult(const AiDistanceResult& result);

  bool replayActive() const { return replayActive_; }
  MapControllerMode mode() const { return mode_; }
  MapSlot selectedSlot() const { return selectedSlot_; }
  MapReplayMode replayMode() const { return routeMode_; }
  MapUserMode userMode() const { return userMode_; }
  ReturnP0State returnP0State() const { return returnP0State_; }
  bool homeContextValid() const { return homeContext_.valid; }

 private:
  enum class ReplayResumeSource : uint8_t { PS2_START, AI_AUTO };
  enum class ReplayRealignReason : uint8_t { NONE, PATH, ARRIVAL };
  enum class ReplayReturnPhase : uint8_t {
    NONE = 0U,
    OUTBOUND = 1U,
    INBOUND = 2U,
  };
  enum class ObstacleDetourPhase : uint8_t {
    IDLE = 0U,
    ARMED = 1U,
    TURN_AWAY = 2U,
    WAIT_AWAY_CLEAR = 3U,
    MOVE_AWAY = 4U,
    TURN_PARALLEL = 5U,
    WAIT_BYPASS_CLEAR = 6U,
    MOVE_BYPASS = 7U,
    COMPLETE_HOLD = 8U,
    ABORTED = 9U,
  };
  enum class MapStorageErrorReason : uint8_t {
    NONE = 0U,
    SETTINGS_SAVE = 1U,
    TEACH_SAVE = 2U,
    MODE_SAVE = 3U,
    STORAGE_INIT = 4U,
    GENERIC = 5U,
  };
  enum class MapSettingsItem : uint8_t { MODE = 0U, SPEED = 1U, COUNT = 2U,
                                         DELETE_MAP = 3U };

  struct Pose {
    float xMm = 0.0f;
    float yMm = 0.0f;
    float headingDeg = 0.0f;
  };

  // Transient post-Teach action. This is deliberately not part of
  // MapRouteData: it must never be persisted or become a user replay mode.
  struct PostTeachBackContext {
    bool valid = false;
    MapSlot slot = MapSlot::MAP_1;
    uint16_t endpointIndex = 0U;
    uint32_t routeGeneration = 0U;
    uint32_t odometryResetGeneration = 0U;
    uint32_t headingResetGeneration = 0U;
    Pose teachOrigin{};
  };

  struct HomeContext {
    bool valid = false;
    MapSlot slot = MapSlot::MAP_1;
    uint32_t routeGeneration = 0U;
    uint32_t odometryResetGeneration = 0U;
    uint32_t headingResetGeneration = 0U;
    Pose p0WorldPose{};
  };

  struct RouteProjection {
    bool valid = false;
    uint16_t segmentStartIndex = 0U;
    uint16_t segmentEndIndex = 0U;
    float t = 0.0f;
    Pose projectedPose{};
    float crossTrackMm = 0.0f;
    float distanceMm = 0.0f;
  };

  void handleEvent(const Ps2MapEvent& event);
  void handleStart();
  void handleTriangle();
  void handleCircle();
  void handleSquare(bool longPress);
  void handleCross();
  void handleCrossLong();
  void handleSlot(uint8_t slot);
  void handleSettingsInput(Ps2MapAction action);
  bool enterSettings(const char*& reason);
  void saveSettingsAndExit();
  void cancelSettings();
  void enterHelp();
  void leaveHelp();
  void leaveMapUiCapture();
  void cycleSettingsMode(int8_t direction);
  void cycleSettingsSpeed(int8_t direction);
  void cycleSettingsLap(int8_t direction);
  bool settingsCanDelete() const;

  bool loadSelected();
  bool beginTeach();
  void cancelTeach();
  void markManualWaypoint();
  void sampleTeach();
  bool appendWaypoint(const Pose& pose, uint8_t flags);
  bool readPose(Pose& pose) const;
  Pose toTeachLocal(const Pose& pose) const;
  static int16_t headingCdeg(float headingDeg);
  static float normalizeDeg(float degrees);
  static float shortestDeltaDeg(float targetDeg, float currentDeg);
  static float distanceMm(float ax, float ay, float bx, float by);
  static bool approximatelyEqual(float lhs, float rhs, float tolerance);
  void resetTeachTracking();
  void requestTeachFinish();
  bool finalizeTeach();
  bool queueTeachSave(MapRouteType type, MapControllerMode failureMode);
  void stagePostTeachBackSnapshot();
  void armPostTeachBackAfterSave();
  void invalidatePostTeachBack(const char* reason);
  bool postTeachBackRejectShouldInvalidate(const char* reason) const;
  bool postTeachBackAvailable(const char*& reason) const;
  bool startPostTeachBack(const char*& reason);
  bool backReadyP0Available(const char*& reason) const;
  void armHomeContextAfterSave();
  // A persisted route has no world-frame P0 after a reboot.  An accepted AI
  // MAP run defines that session's P0 from the same replay origin used by the
  // normal route executor; it is intentionally RAM-only and never enables
  // the PS2 post-Teach BACK offer.
  void armAiRunHomeContextIfNeeded();
  void invalidateHomeContext(const char* reason);
  bool locateRouteProjection(RouteProjection& projection,
                             const char*& reason) const;
  bool requestReturnToP0Internal(ReturnP0Source source, const char*& reason);
  void updateReturnToP0();
  bool startReturnReacquire();
  bool startReturnWaypoint();
  bool startReturnP0Position();
  bool startReturnP0Heading();
  bool consumeReturnTurnResult(const AiTurnResult& result);
  bool consumeReturnDistanceResult(const AiDistanceResult& result);
  void abortReturnToP0(const char* reason);
  void completeReturnToP0();
  bool returnP0InProgress() const;
  static const char* returnP0StateName(ReturnP0State state);
  bool validateRoute(const MapRouteData& route, const char*& reason) const;
  bool hasValidClosingEdge(const MapRouteData& route,
                           const char*& reason) const;
  uint32_t routeLengthMm(const MapRouteData& route) const;
  uint32_t persistedRouteLengthMm(const MapRouteData& route,
                                  MapRouteType persistedType) const;
  static MapRouteType persistedRouteType(MapReplayMode mode);
  static MapReplayMode persistedReplayMode(MapReplayMode mode);
  static MapReplayMode executionModeFor(MapUserMode mode,
                                        uint8_t repeatTarget);
  static MapUserMode userModeFromStored(MapRouteType type,
                                        MapReplayMode mode,
                                        bool shuttleRepeat);
  static const char* userModeName(MapUserMode mode);
  static bool userModeNeedsClosingEdge(MapUserMode mode);
  void applyStoredSettings(MapRouteType type, MapReplayMode mode,
                           bool shuttleRepeat, uint8_t encodedTarget);
  void updateRouteHeaderForSave(MapRouteData& route, MapUserMode mode,
                                uint8_t repeatTarget) const;
  void updateRouteHeaderForSave(MapRouteData& route) const;
  void normalizeRouteForRuntime(MapRouteData& route,
                                MapReplayMode runtimeMode) const;

  void cycleReplayMode();
  bool prepareReplay(const char*& rejectReason,
                     MapMissionInitiator initiator);
  bool replayPrecheck(const MapRouteData& route, const char*& reason,
                      MapMissionInitiator initiator) const;
  void updateReplay();
  bool startNextReplaySegment();
  bool obstacleDetourContextActive() const;
  bool obstacleDetourInProgress() const;
  bool armObstacleDetour(const char*& rejectReason);
  bool obstacleDetourEntryGates(const char*& rejectReason) const;
  bool obstacleDetourSensorsReady() const;
  bool obstacleDetourPathClear() const;
  bool startObstacleDetourTurn(bool left, ObstacleDetourPhase phase,
                               const char* phaseName);
  bool startObstacleDetourDistance(uint32_t distanceMm,
                                   ObstacleDetourPhase phase,
                                   const char* phaseName);
  void updateObstacleDetour();
  bool consumeObstacleDetourTurnResult(const AiTurnResult& result);
  bool consumeObstacleDetourDistanceResult(const AiDistanceResult& result);
  void completeObstacleDetour();
  void abortObstacleDetour(const char* reason);
  bool currentReplayPose(Pose& pose) const;
  Pose routePointWorld(uint16_t index) const;
  Pose routePointWorldFromOrigin(const Pose& origin, uint16_t index) const;
  float replayIncomingBearing(uint16_t fromIndex, uint16_t toIndex) const;
  void advanceReplayAfterTarget();
  void enterReplayHold(MapHoldReason reason, bool allowResume);
  void abortReplay(const char* reason);
  void completeReplay();
  void cancelReplay(const char* reason = "CANCELLED");
  void serviceObstacleHold();
  bool canResumeReplay(const char*& rejectReason);
  bool canResumeReplay(ReplayResumeSource source,
                       const char*& rejectReason);
  bool resumeReplayFromHold(ReplayResumeSource source,
                            const char*& rejectReason);
  bool resumeReturnP0FromObstacleHold(const char*& rejectReason);
  void inhibitAutonomousResume(const char* reason);
  void clearReplayResumeContext();
  uint32_t nextReplayGeneration();
  void beginCancelTrace();
  void serviceCancelTrace();
  void logStartReject(const char* reason) const;
  void logSegmentStart(uint32_t generation, uint16_t from, uint16_t to,
                       MapReplayOperation operation) const;
  void logSegmentDone(uint32_t generation, uint16_t from, uint16_t to,
                      MapReplayOperation operation, float targetMm,
                      float travelledMm, int16_t targetDeg,
                      float headingDeg) const;
  void logGuidePlan(uint16_t from, uint16_t to, float distanceMm,
                    float bearingDeg, float headingDeg,
                    float headingErrorDeg) const;
  void logGuidePreturn(float headingErrorDeg, const char* action) const;
  void logGuideStart(uint16_t waypoint, float distanceMm,
                     float bearingDeg) const;
  void logGuideUpdate() const;
  void logGuideDone(uint16_t waypoint, float positionErrorMm,
                    float arrivalBearingDeg, float headingDeg,
                    float headingErrorDeg) const;
  void logGuideRealign(ReplayRealignReason reason, uint16_t waypoint,
                       float distanceMm, float desiredBearingDeg,
                       float headingDeg, float headingErrorDeg) const;
  void logGuideRealignDone(ReplayRealignReason reason, uint16_t waypoint,
                           float positionErrorMm, float desiredBearingDeg,
                           float headingDeg, float headingErrorDeg,
                           const char* action) const;
  static const char* realignReasonName(ReplayRealignReason reason);
  static const char* returnPhaseName(ReplayReturnPhase phase);

  void serviceStorage();
  void publishStatus();
  void logOptimizeSummary(const RouteCleanerMetrics& metrics) const;
  void logSemanticSummary(const SemanticRouteMetrics& metrics) const;
  void log(const char* message) const;
  static const char* storageErrorReasonName(MapStorageErrorReason reason);

  RobotController& robot_;
  Ps2Controller& ps2_;
  LcdDisplay& display_;
  WheelOdometry& odometry_;
  HeadingFusion& fusion_;
  UltrasonicSensor& ultrasonic_;
  ObstacleClassifier& obstacleClassifier_;
  Print& debug_;
  MapRouteStore store_;

  MapSlot selectedSlot_ = MapSlot::MAP_1;
  MapControllerMode mode_ = MapControllerMode::READY;
  MapRouteType routeType_ = MapRouteType::OPEN;
  MapUserMode userMode_ = MapUserMode::ONCE;
  MapReplayMode routeMode_ = MapReplayMode::ONCE;
  MapMissionInitiator missionInitiator_ = MapMissionInitiator::NONE;
  bool autonomousResumeInhibited_ = true;
  int16_t replaySpeed_ = MAP_REPLAY_SPEED_DEFAULT;
  uint8_t loopTarget_ = MAP_LOOP_TARGET_MIN;
  MapTeachMode teachMode_ = MapTeachMode::MANUAL_KEYFRAME;
  MapStoreState storeState_ = MapStoreState::EMPTY;
  MapStorageErrorReason storageErrorReason_ = MapStorageErrorReason::NONE;
  // route_ is the active Replay buffer. optimizedRoute_ and semanticRoute_
  // are fixed-size working buffers used only after Teach stops; the previous
  // A/B record remains in Flash if an optimization or save step fails.
  MapRouteData route_{};
  MapRouteData optimizedRoute_{};
  MapRouteData semanticRoute_{};
  bool loadedValid_ = false;

  Pose teachOrigin_{};
  uint32_t teachOriginResetGeneration_ = 0U;
  uint32_t teachOriginHeadingResetGeneration_ = 0U;
  Pose pendingTeachBackOrigin_{};
  uint32_t pendingTeachBackResetGeneration_ = 0U;
  uint32_t pendingTeachBackHeadingResetGeneration_ = 0U;
  bool pendingTeachBackValid_ = false;
  PostTeachBackContext postTeachBack_{};
  bool postTeachBackActive_ = false;
  bool postTeachBackComplete_ = false;
  HomeContext homeContext_{};
  bool backP0UiDismissed_ = false;
  Pose pendingHomeOrigin_{};
  uint32_t pendingHomeResetGeneration_ = 0U;
  uint32_t pendingHomeHeadingResetGeneration_ = 0U;
  bool pendingHomeContextValid_ = false;
  Pose lastTeachSample_{};
  bool teachOriginValid_ = false;
  bool lastTeachSampleValid_ = false;
  uint8_t cornerStableSamples_ = 0U;
  uint32_t nextTeachSampleMs_ = 0U;
  bool teachFinishPending_ = false;
  bool teachOldRouteAvailable_ = false;
  bool savePending_ = false;
  bool deletePending_ = false;
  bool modeSavePending_ = false;
  MapReplayMode modeBeforeSave_ = MapReplayMode::ONCE;

  MapSettingsItem settingsItem_ = MapSettingsItem::MODE;
  MapUserMode settingsUserMode_ = MapUserMode::ONCE;
  int16_t settingsSpeed_ = MAP_REPLAY_SPEED_DEFAULT;
  uint8_t settingsRepeatTarget_ = MAP_LOOP_TARGET_MIN;
  uint8_t helpPage_ = 0U;

  bool replayActive_ = false;
  bool replayResumeAllowed_ = false;
  MapReplayOperation replayOperation_ = MapReplayOperation::NONE;
  uint16_t replayCurrentIndex_ = 0U;
  uint16_t replayTargetIndex_ = 0U;
  int8_t replayDirection_ = 1;
  bool replayReturned_ = false;
  ReplayReturnPhase replayReturnPhase_ = ReplayReturnPhase::NONE;
  uint32_t replayOriginRouteGeneration_ = 0U;
  uint32_t replayOriginResetGeneration_ = 0U;
  uint32_t replayOriginHeadingResetGeneration_ = 0U;
  uint32_t replayGeneration_ = 0U;
  uint32_t replaySegmentGeneration_ = 0U;
  MapSlot replayContextSlot_ = MapSlot::MAP_1;
  bool replayOriginValid_ = false;
  ReplayRealignReason replayRealignReason_ = ReplayRealignReason::NONE;
  uint32_t replayArrivalHeadingViolationSinceMs_ = 0U;
  bool replayArrivalTurnPending_ = false;
  uint16_t replayArrivalTurnWaypoint_ = 0U;
  uint8_t replayArrivalTurnAttempts_ = 0U;
  Pose replayOrigin_{};
  Pose replayTarget_{};
  Pose replayHoldPose_{};
  bool replayHoldPoseValid_ = false;
  uint32_t replayTargetDistanceMm_ = 0U;
  int16_t replayTargetDeg_ = 0;
  float replayGuideBearingDeg_ = 0.0f;
  uint32_t replayTravelMm_ = 0U;
  uint32_t replayErrorMm_ = 0U;
  uint32_t replayLapCounter_ = 0U;
  uint32_t replayCycleCounter_ = 0U;
  const char* replayReason_ = "NONE";
  MapHoldReason holdReason_ = MapHoldReason::NONE;
  uint32_t obstacleClearSinceMs_ = 0U;
  ObstacleDetourPhase obstacleDetourPhase_ = ObstacleDetourPhase::IDLE;
  ObstacleDecision obstacleDetourDecision_ = ObstacleDecision::HOLD;
  bool obstacleDetourAwayRight_ = false;
  uint8_t obstacleDetourAttempts_ = 0U;
  uint32_t obstacleDetourStartMs_ = 0U;
  uint32_t obstacleDetourClearSinceMs_ = 0U;
  uint32_t obstacleDetourGeneration_ = 0U;
  uint32_t obstacleDetourTravelBudgetUsedMm_ = 0U;
  float obstacleDetourTurnBudgetUsedDeg_ = 0.0f;
  Pose obstacleDetourStartPose_{};
  uint16_t obstacleDetourOriginalCurrentIndex_ = 0U;
  uint16_t obstacleDetourOriginalTargetIndex_ = 0U;
  int8_t obstacleDetourOriginalDirection_ = 1;
  uint32_t obstacleDetourOriginalRouteGeneration_ = 0U;
  uint32_t obstacleDetourOriginalReplayGeneration_ = 0U;
  ReturnP0State returnP0State_ = ReturnP0State::IDLE;
  ReturnP0Source returnP0Source_ = ReturnP0Source::NONE;
  uint32_t returnP0Generation_ = 0U;
  uint32_t returnP0SegmentGeneration_ = 0U;
  uint16_t returnP0TargetIndex_ = 0U;
  uint16_t returnP0SegmentStartIndex_ = 0U;
  uint8_t returnP0ReacquireAttempts_ = 0U;
  uint8_t returnP0PositionCorrectionAttempts_ = 0U;
  uint8_t returnP0HeadingAttempts_ = 0U;
  bool returnP0TurnPending_ = false;
  ReturnP0State returnP0HeldState_ = ReturnP0State::IDLE;
  uint16_t returnP0HeldTargetIndex_ = 0U;
  uint16_t returnP0HeldSegmentStartIndex_ = 0U;
  uint32_t returnP0SettleSinceMs_ = 0U;
  RouteProjection returnP0Projection_{};
  uint32_t closeCandidateDistanceMm_ = 0U;
  int16_t closeCandidateHeadingDeg_ = 0;
  bool cancelTraceActive_ = false;
  uint32_t cancelTraceStartMs_ = 0U;
  uint32_t nextCancelTraceMs_ = 0U;
  uint8_t cancelTraceLines_ = 0U;
  bool statusDirty_ = true;
  uint32_t lastStatusMs_ = 0U;
  uint32_t lastGuidanceLogMs_ = 0U;
};

#endif  // STM32_LOCAL_MAP_CONTROLLER_H

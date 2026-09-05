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
#include <sensors/ultrasonic_sensor.h>

// STM32-local Teach/Replay controller. It observes PS2, odometry, Heading and
// obstacle state, and is the only runtime MAP owner. It never sends MCP
// correlation frames and it never runs motor code from a worker/task.
class MapController {
 public:
  MapController(RobotController& robot, Ps2Controller& ps2,
                LcdDisplay& display, WheelOdometry& odometry,
                HeadingFusion& fusion, UltrasonicSensor& ultrasonic,
                Print& debugStream);

  void begin();
  // Drain only the physical MAP input boundary. Main calls this before
  // asynchronous motion-result delivery so X can invalidate stale results;
  // the normal update then advances replay after results are consumed.
  void processInput();
  void update();

  // Main uses these hooks to consume REPLAY results internally. Returning
  // false leaves normal MCP SID/OP RobotLink routing unchanged.
  bool consumeReplayTurnResult(const AiTurnResult& result);
  bool consumeReplayDistanceResult(const AiDistanceResult& result);

  bool replayActive() const { return replayActive_; }
  MapControllerMode mode() const { return mode_; }
  MapSlot selectedSlot() const { return selectedSlot_; }
  MapReplayMode replayMode() const { return routeMode_; }

 private:
  enum class ReplayRealignReason : uint8_t { NONE, PATH, ARRIVAL };
  enum class MapStorageErrorReason : uint8_t {
    NONE = 0U,
    SETTINGS_SAVE = 1U,
    TEACH_SAVE = 2U,
    MODE_SAVE = 3U,
    STORAGE_INIT = 4U,
    GENERIC = 5U,
  };
  enum class MapSettingsItem : uint8_t { MODE = 0U, SPEED = 1U, LAP = 2U,
                                         DELETE_MAP = 3U };

  struct Pose {
    float xMm = 0.0f;
    float yMm = 0.0f;
    float headingDeg = 0.0f;
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
  bool validateRoute(const MapRouteData& route, const char*& reason) const;
  uint32_t routeLengthMm(const MapRouteData& route) const;
  void updateRouteHeaderForSave(MapRouteData& route) const;

  void cycleReplayMode();
  bool prepareReplay(const char*& rejectReason);
  bool replayPrecheck(const MapRouteData& route, const char*& reason) const;
  void updateReplay();
  bool startNextReplaySegment();
  bool currentReplayPose(Pose& pose) const;
  Pose routePointWorld(uint16_t index) const;
  float replayIncomingBearing(uint16_t fromIndex, uint16_t toIndex) const;
  void advanceReplayAfterTarget();
  void enterReplayHold(MapHoldReason reason, bool allowResume);
  void abortReplay(const char* reason);
  void completeReplay();
  void cancelReplay(const char* reason = "CANCELLED");
  bool canResumeReplay(const char*& rejectReason) const;
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
  Print& debug_;
  MapRouteStore store_;

  MapSlot selectedSlot_ = MapSlot::MAP_1;
  MapControllerMode mode_ = MapControllerMode::READY;
  MapRouteType routeType_ = MapRouteType::OPEN;
  MapReplayMode routeMode_ = MapReplayMode::ONCE;
  int16_t replaySpeed_ = MAP_REPLAY_SPEED_DEFAULT;
  uint8_t loopTarget_ = MAP_LOOP_TARGET_INF;
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
  MapReplayMode settingsMode_ = MapReplayMode::ONCE;
  int16_t settingsSpeed_ = MAP_REPLAY_SPEED_DEFAULT;
  uint8_t settingsLoopTarget_ = MAP_LOOP_TARGET_INF;
  uint8_t helpPage_ = 0U;

  bool replayActive_ = false;
  bool replayResumeAllowed_ = false;
  MapReplayOperation replayOperation_ = MapReplayOperation::NONE;
  uint16_t replayCurrentIndex_ = 0U;
  uint16_t replayTargetIndex_ = 0U;
  int8_t replayDirection_ = 1;
  bool replayReturned_ = false;
  uint32_t replayOriginRouteGeneration_ = 0U;
  uint32_t replayOriginResetGeneration_ = 0U;
  uint32_t replayOriginHeadingResetGeneration_ = 0U;
  uint32_t replayGeneration_ = 0U;
  uint32_t replaySegmentGeneration_ = 0U;
  MapSlot replayContextSlot_ = MapSlot::MAP_1;
  bool replayOriginValid_ = false;
  ReplayRealignReason replayRealignReason_ = ReplayRealignReason::NONE;
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
  const char* replayReason_ = "NONE";
  MapHoldReason holdReason_ = MapHoldReason::NONE;
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

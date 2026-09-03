#include <ps2/ps2_controller.h>
#include <debug/debug_output.h>
#include <robot_config.h>
#include <display/lcd_display.h>
#include <string.h>

// These objects are owned by main.cpp. Map page/slot presentation and action
// delivery are kept on STM32 when local MAP is enabled.
extern LcdDisplay display;
extern HardwareSerial robotAiSerial;
extern DebugOutput robotDebug;

namespace {
constexpr uint32_t kMapPageToggleGuardMs = 250U;
constexpr uint8_t kMapNeutralFramesToArm = 2U;
// X is the safety HOLD/CANCEL boundary. Give the operator more time to make
// an intentional long press without changing SELECT/SQUARE semantics.
constexpr uint32_t kMapCrossLongPressMs = 1200U;
constexpr uint32_t kMapLongPressMs = 800U;

void emitLegacyMapEvent(const char* action, uint8_t slot) {
  const uint8_t normalizedSlot = slot == 2U ? 2U : 1U;
  if (strcmp(action, "SLOT") == 0) {
    robotAiSerial.print("<EVENT,MAP,SLOT,");
    robotAiSerial.print(normalizedSlot);
    robotAiSerial.print(">\r\n");
    return;
  }
  robotAiSerial.print("<EVENT,MAP,");
  robotAiSerial.print(action);
  robotAiSerial.print(",SLOT,");
  robotAiSerial.print(normalizedSlot);
  robotAiSerial.print(">\r\n");
}
}  // namespace

Ps2Controller::Ps2Controller()
    : driver_(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN) {}

bool Ps2Controller::queueMapEvent(Ps2MapAction action) {
  // The main loop drains one debounced event per iteration. A second event in
  // the same frame cannot be meaningful for the physical controller. CROSS is
  // the local safety action and must win over a queued START or any other MAP
  // action in that frame.
  if (mapEventPending_) {
    if (action == Ps2MapAction::CROSS_LONG &&
        mapEvent_.action != Ps2MapAction::CROSS_LONG) {
      // A long X is the strongest local MAP action. It must replace a queued
      // short X/START/UI event so HOLD can never mask the requested CANCEL.
      mapEvent_.action = action;
      mapEvent_.slot = display.mapSlot();
#if ROBOT_DEBUG
      robotDebug.println("MAP,INPUT,CROSS_LONG_ACTION,PRIORITY_REPLACE=1");
#endif
      return true;
    }
    if (action == Ps2MapAction::CROSS &&
        mapEvent_.action != Ps2MapAction::CROSS &&
        mapEvent_.action != Ps2MapAction::CROSS_LONG) {
#if ROBOT_DEBUG
      if (mapEvent_.action == Ps2MapAction::START) {
        robotDebug.println("MAP,START,REJECT,REASON=SAFETY_PRIORITY");
      }
#endif
      mapEvent_.action = action;
      mapEvent_.slot = display.mapSlot();
      return true;
    }
    if (action == Ps2MapAction::START &&
        mapEvent_.action != Ps2MapAction::CROSS &&
        mapEvent_.action != Ps2MapAction::START) {
      // START is the primary MAP action. Do not lose its first edge behind a
      // lower-priority UI event generated in the same polling window.
      mapEvent_.action = action;
      mapEvent_.slot = display.mapSlot();
#if ROBOT_DEBUG
      robotDebug.println("MAP,INPUT,START_ACTION,PRIORITY_REPLACE=1");
#endif
      return true;
    }
#if ROBOT_DEBUG
    if (action == Ps2MapAction::START) {
      robotDebug.println("MAP,INPUT,START_ACTION,DROPPED=QUEUE_FULL");
      robotDebug.println("MAP,START,REJECT,REASON=QUEUE_FULL");
    }
#endif
    return false;
  }
  mapEvent_.action = action;
  mapEvent_.slot = display.mapSlot();
  mapEventPending_ = true;
#if ROBOT_DEBUG
  if (action == Ps2MapAction::START) {
    robotDebug.println("MAP,INPUT,START_ACTION");
  }
#endif
#if !STM32_LOCAL_MAP_ENABLE
  const char* legacyAction = action == Ps2MapAction::START ? "START" :
                             action == Ps2MapAction::TRIANGLE ? "TRIANGLE" :
                             action == Ps2MapAction::SQUARE ? "SQUARE" :
                             action == Ps2MapAction::SQUARE_LONG ? "SQUARE_LONG" :
                             action == Ps2MapAction::CIRCLE ? "CIRCLE" :
                             action == Ps2MapAction::CROSS ? "CROSS" :
                             action == Ps2MapAction::CROSS_LONG ? "CROSS_LONG" :
                             action == Ps2MapAction::SELECT_LONG ? "SELECT_LONG" :
                             "SLOT";
  emitLegacyMapEvent(legacyAction, mapEvent_.slot);
#endif
  return true;
}

bool Ps2Controller::takeMapEvent(Ps2MapEvent& event) {
  if (!mapEventPending_) return false;
  event = mapEvent_;
  mapEventPending_ = false;
  return true;
}

void Ps2Controller::disarmMapInput() {
  mapEventPending_ = false;
  mapActionsArmed_ = false;
  mapNeutralReleaseFrames_ = 0U;
  resetMapPressTracking();
  resetMapCrossTracking();
#if ROBOT_DEBUG
  robotDebug.println("MAP,INPUT,DISARM_AFTER_CANCEL");
#endif
}

void Ps2Controller::holdMapInput() {
  mapEventPending_ = false;
  mapActionsArmed_ = false;
  mapNeutralReleaseFrames_ = 0U;
  // Preserve X press timing so a held X can still escalate HOLD to CANCEL.
  resetMapPressTracking();
#if ROBOT_DEBUG
  robotDebug.println("MAP,INPUT,DISARM_AFTER_HOLD");
#endif
}

void Ps2Controller::begin() {
  const uint32_t now = millis();
  reconnect(now);
  lastPollMs_ = now;
}

void Ps2Controller::resetMapPressTracking() {
  mapSelectPressActive_ = false;
  mapSelectLongFired_ = false;
  mapSelectStartedMs_ = 0U;
  mapSquarePressActive_ = false;
  mapSquareLongFired_ = false;
  mapSquareStartedMs_ = 0U;
}

void Ps2Controller::resetMapCrossTracking() {
  mapCrossPressActive_ = false;
  mapCrossLongFired_ = false;
  mapCrossStartedMs_ = 0U;
}

void Ps2Controller::reconnect(uint32_t nowMs) {
  configResult_ = driver_.config_gamepad(false, false);
  state_.receiverConnected = (configResult_ == 0);
  state_.frameFresh = false;
  lastReconnectMs_ = nowMs;

  mapEdgesInitialized_ = false;
  previousMapL3_ = false;
  previousMapStart_ = false;
  previousMapSelect_ = false;
  previousMapTriangle_ = false;
  previousMapSquare_ = false;
  previousMapCircle_ = false;
  previousMapCross_ = false;
  resetMapCrossTracking();
  mapActionsArmed_ = false;
  mapNeutralReleaseFrames_ = 0U;
  lastMapPageToggleMs_ = 0U;
  mapEventPending_ = false;
  resetMapPressTracking();
}

int16_t Ps2Controller::processAxis(uint8_t raw, bool invert) {
  int16_t delta = static_cast<int16_t>(raw) - 128;
  if (invert) delta = -delta;
  if (abs(delta) <= JOYSTICK_DEADZONE_RAW) return 0;
  const int16_t sign = delta < 0 ? -1 : 1;
  const int16_t available = 127 - JOYSTICK_DEADZONE_RAW;
  int32_t linear = static_cast<int32_t>(abs(delta) - JOYSTICK_DEADZONE_RAW) *
                   1000L / available;
  linear = constrain(linear, 0L, 1000L);
  const int32_t cubic = linear * linear * linear / 1000000L;
  const int32_t shaped = (linear * 40L + cubic * 60L) / 100L;
  return static_cast<int16_t>(sign * shaped);
}

void Ps2Controller::updateActivity(uint32_t nowMs, uint16_t buttons,
                                   uint8_t lx, uint8_t ly, uint8_t rx,
                                   uint8_t ry) {
  if (!contentInitialized_) {
    previousFingerprint_ = buttons;
    previousLx_ = lx; previousLy_ = ly; previousRx_ = rx; previousRy_ = ry;
    lastActivityMs_ = nowMs;
    contentInitialized_ = true;
    return;
  }
  const bool changed = buttons != previousFingerprint_ ||
      abs(static_cast<int>(lx) - previousLx_) >= 2 ||
      abs(static_cast<int>(ly) - previousLy_) >= 2 ||
      abs(static_cast<int>(rx) - previousRx_) >= 2 ||
      abs(static_cast<int>(ry) - previousRy_) >= 2;
  if (changed) lastActivityMs_ = nowMs;
  previousFingerprint_ = buttons;
  previousLx_ = lx; previousLy_ = ly; previousRx_ = rx; previousRy_ = ry;
}

void Ps2Controller::captureState(uint32_t nowMs) {
  state_.up = driver_.Button(PSB_PAD_UP);
  state_.down = driver_.Button(PSB_PAD_DOWN);
  state_.left = driver_.Button(PSB_PAD_LEFT);
  state_.right = driver_.Button(PSB_PAD_RIGHT);
  state_.l1 = driver_.Button(PSB_L1);
  state_.l2 = driver_.Button(PSB_L2);
  state_.r1 = driver_.Button(PSB_R1);
  state_.r2 = driver_.Button(PSB_R2);
  state_.l3 = driver_.Button(PSB_L3);
  state_.r3 = driver_.Button(PSB_R3);
  state_.start = driver_.Button(PSB_START);
  state_.select = driver_.Button(PSB_SELECT);
  state_.triangle = driver_.Button(PSB_TRIANGLE);
  state_.circle = driver_.Button(PSB_CIRCLE);
  state_.cross = driver_.Button(PSB_CROSS);
  state_.square = driver_.Button(PSB_SQUARE);

  if (!mapEdgesInitialized_) {
    previousMapL3_ = state_.l3;
    previousMapStart_ = state_.start;
    previousMapSelect_ = state_.select;
    previousMapTriangle_ = state_.triangle;
    previousMapSquare_ = state_.square;
    previousMapCircle_ = state_.circle;
    previousMapCross_ = state_.cross;
    mapEdgesInitialized_ = true;
    mapActionsArmed_ = false;
    mapNeutralReleaseFrames_ = 0U;
    resetMapPressTracking();
  } else {
    const bool l3Pressed = state_.l3 && !previousMapL3_;
    const bool startPressed = state_.start && !previousMapStart_;
    const bool selectPressed = state_.select && !previousMapSelect_;
    const bool selectReleased = !state_.select && previousMapSelect_;
    const bool trianglePressed = state_.triangle && !previousMapTriangle_;
    const bool squarePressed = state_.square && !previousMapSquare_;
    const bool squareReleased = !state_.square && previousMapSquare_;
    const bool circlePressed = state_.circle && !previousMapCircle_;
    const bool crossPressed = state_.cross && !previousMapCross_;
    const bool crossReleased = !state_.cross && previousMapCross_;
    const bool allMapButtonsReleased =
        !state_.l3 && !state_.start && !state_.select && !state_.triangle &&
        !state_.square && !state_.circle && !state_.cross;

#if ROBOT_DEBUG
    if (startPressed) {
      robotDebug.println("MAP,INPUT,START_RAW_DOWN");
      robotDebug.println("MAP,INPUT,START_EDGE");
    }
#endif

    // X is a safety boundary, so its down edge is handled independently of
    // MAP arming. The down event produces HOLD immediately; the timer event
    // later escalates the same physical press to long-CANCEL.
    if (crossPressed && display.isMapPage()) {
      mapCrossPressActive_ = true;
      mapCrossLongFired_ = false;
      mapCrossStartedMs_ = nowMs;
      queueMapEvent(Ps2MapAction::CROSS);
    }
    if (mapCrossPressActive_ && state_.cross && !mapCrossLongFired_ &&
        display.isMapPage() &&
        (nowMs - mapCrossStartedMs_) >= kMapCrossLongPressMs) {
      queueMapEvent(Ps2MapAction::CROSS_LONG);
      mapCrossLongFired_ = true;
    }
    if (crossReleased) resetMapCrossTracking();

    if (l3Pressed) {
      if (lastMapPageToggleMs_ == 0U ||
          (nowMs - lastMapPageToggleMs_) >= kMapPageToggleGuardMs) {
        display.togglePage();
        lastMapPageToggleMs_ = nowMs;
      }
      mapActionsArmed_ = false;
      mapNeutralReleaseFrames_ = 0U;
      resetMapPressTracking();
      resetMapCrossTracking();
#if ROBOT_DEBUG
      if (startPressed) {
        robotDebug.println("MAP,START,REJECT,REASON=PAGE_TRANSITION");
      }
#endif
    } else if (!display.isMapPage()) {
      mapActionsArmed_ = false;
      mapNeutralReleaseFrames_ = 0U;
      resetMapPressTracking();
      resetMapCrossTracking();
#if ROBOT_DEBUG
      if (startPressed) {
        robotDebug.println("MAP,START,REJECT,REASON=NOT_MAP_PAGE");
      }
#endif
    } else if (!mapActionsArmed_) {
#if ROBOT_DEBUG
      if (startPressed) {
        robotDebug.println("MAP,START,REJECT,REASON=NOT_ARMED");
      }
#endif
      if (allMapButtonsReleased) {
        if (mapNeutralReleaseFrames_ < kMapNeutralFramesToArm) {
          ++mapNeutralReleaseFrames_;
        }
        if (mapNeutralReleaseFrames_ >= kMapNeutralFramesToArm) {
          mapActionsArmed_ = true;
          resetMapPressTracking();
#if ROBOT_DEBUG
          robotDebug.println("MAP,INPUT,ARMED");
#endif
        }
      } else {
        mapNeutralReleaseFrames_ = 0U;
      }
    } else {
      // START runs the selected route. TRIANGLE is reserved for Teach/marks.
      if (startPressed) queueMapEvent(Ps2MapAction::START);

      if (selectPressed) {
        mapSelectPressActive_ = true;
        mapSelectLongFired_ = false;
        mapSelectStartedMs_ = nowMs;
      }
      if (mapSelectPressActive_ && state_.select && !mapSelectLongFired_ &&
          (nowMs - mapSelectStartedMs_) >= kMapLongPressMs) {
        queueMapEvent(Ps2MapAction::SELECT_LONG);
        mapSelectLongFired_ = true;
      }
      if (selectReleased && mapSelectPressActive_) {
        if (!mapSelectLongFired_ && !display.mapSlotLocked()) {
          display.toggleMapSlot();
          queueMapEvent(Ps2MapAction::SLOT);
        }
        mapSelectPressActive_ = false;
        mapSelectLongFired_ = false;
        mapSelectStartedMs_ = 0U;
      }

      if (squarePressed) {
        mapSquarePressActive_ = true;
        mapSquareLongFired_ = false;
        mapSquareStartedMs_ = nowMs;
      }
      if (mapSquarePressActive_ && state_.square && !mapSquareLongFired_ &&
          (nowMs - mapSquareStartedMs_) >= kMapLongPressMs) {
        queueMapEvent(Ps2MapAction::SQUARE_LONG);
        mapSquareLongFired_ = true;
      }
      if (squareReleased && mapSquarePressActive_) {
        if (!mapSquareLongFired_) queueMapEvent(Ps2MapAction::SQUARE);
        mapSquarePressActive_ = false;
        mapSquareLongFired_ = false;
        mapSquareStartedMs_ = 0U;
      }

      if (trianglePressed) queueMapEvent(Ps2MapAction::TRIANGLE);
      if (circlePressed) queueMapEvent(Ps2MapAction::CIRCLE);
    }

    previousMapL3_ = state_.l3;
    previousMapStart_ = state_.start;
    previousMapSelect_ = state_.select;
    previousMapTriangle_ = state_.triangle;
    previousMapSquare_ = state_.square;
    previousMapCircle_ = state_.circle;
    previousMapCross_ = state_.cross;
  }

  const bool analogFrame = driver_.mode() == 0x73 || driver_.mode() == 0x79;
  const uint8_t lxRaw = analogFrame ? driver_.Analog(PSS_LX) : 128U;
  const uint8_t lyRaw = analogFrame ? driver_.Analog(PSS_LY) : 128U;
  const uint8_t rxRaw = analogFrame ? driver_.Analog(PSS_RX) : 128U;
  const uint8_t ryRaw = analogFrame ? driver_.Analog(PSS_RY) : 128U;
  state_.lx = processAxis(lxRaw, false);
  state_.ly = processAxis(lyRaw, true);
  state_.rx = processAxis(rxRaw, false);
  state_.ry = processAxis(ryRaw, true);

  uint16_t fingerprint = 0;
  static const uint16_t buttons[] = {
      PSB_SELECT, PSB_L3, PSB_R3, PSB_START, PSB_PAD_UP, PSB_PAD_RIGHT,
      PSB_PAD_DOWN, PSB_PAD_LEFT, PSB_L2, PSB_R2, PSB_L1, PSB_R1,
      PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE};
  for (uint16_t button : buttons) {
    if (driver_.Button(button)) fingerprint |= button;
  }
  updateActivity(nowMs, fingerprint, lxRaw, lyRaw, rxRaw, ryRaw);
  lastFrameIntervalMs_ = state_.lastGoodFrameMs == 0U
                             ? 0U
                             : nowMs - state_.lastGoodFrameMs;
  state_.lastGoodFrameMs = nowMs;
  ++goodFrameCount_;
  state_.frameFresh = true;
}

void Ps2Controller::update() {
  const uint32_t now = millis();
  state_.frameFresh = false;
  if (!state_.receiverConnected) {
    if ((now - lastReconnectMs_) >= PS2_RECONNECT_MS) reconnect(now);
    return;
  }
  if ((now - lastPollMs_) < PS2_POLL_MS) return;
  lastPollMs_ = now;
  if (!driver_.read_gamepad()) {
    state_.receiverConnected = driver_.connected();
    return;
  }
  state_.receiverConnected = driver_.connected();
  if (state_.receiverConnected) captureState(now);
}

uint16_t Ps2Controller::maskFor(Ps2Button button) {
  switch (button) {
    case Ps2Button::UP: return PSB_PAD_UP;
    case Ps2Button::DOWN: return PSB_PAD_DOWN;
    case Ps2Button::LEFT: return PSB_PAD_LEFT;
    case Ps2Button::RIGHT: return PSB_PAD_RIGHT;
    case Ps2Button::L1: return PSB_L1;
    case Ps2Button::L2: return PSB_L2;
    case Ps2Button::R1: return PSB_R1;
    case Ps2Button::R2: return PSB_R2;
    case Ps2Button::L3: return PSB_L3;
    case Ps2Button::R3: return PSB_R3;
    case Ps2Button::START: return PSB_START;
    case Ps2Button::SELECT: return PSB_SELECT;
    case Ps2Button::TRIANGLE: return PSB_TRIANGLE;
    case Ps2Button::CIRCLE: return PSB_CIRCLE;
    case Ps2Button::CROSS: return PSB_CROSS;
    case Ps2Button::SQUARE: return PSB_SQUARE;
  }
  return 0;
}

bool Ps2Controller::buttonPressed(Ps2Button button) const {
  if (!state_.frameFresh) return false;
  // RobotController still asks for the legacy START brake edge. Remap that
  // query to R3 so START itself is completely free for Map controls.
  if (button == Ps2Button::START) return driver_.ButtonPressed(PSB_R3);
  // CROSS no longer owns any Robot-page function (legacy ramp toggle removed).
  // Its raw state remains available above for Map event generation.
  if (button == Ps2Button::CROSS) return false;
  return driver_.ButtonPressed(maskFor(button));
}

bool Ps2Controller::buttonReleased(Ps2Button button) const {
  if (!state_.frameFresh) return false;
  if (button == Ps2Button::START) return driver_.ButtonReleased(PSB_R3);
  if (button == Ps2Button::CROSS) return false;
  return driver_.ButtonReleased(maskFor(button));
}

bool Ps2Controller::motionCommandActive() const {
  if (!state_.receiverConnected) return false;
  if (state_.up || state_.down || state_.left || state_.right) return true;
  return state_.lx != 0 || state_.ly != 0;
}

bool Ps2Controller::controlActive(uint32_t nowMs) const {
  if (!state_.receiverConnected || frameTimedOut(nowMs)) return false;
  // A standard wireless PS2 receiver has no transmitter-power/status bit: it
  // returns the same centered frame while the handheld is off or idle. Use a
  // live command or a short post-input hold as the safe software proxy.
  return motionCommandActive() ||
         (contentInitialized_ && (nowMs - lastActivityMs_) <= PS2_ACTIVITY_MS);
}

bool Ps2Controller::frameTimedOut(uint32_t nowMs) const {
  return !state_.receiverConnected || state_.lastGoodFrameMs == 0U ||
         (nowMs - state_.lastGoodFrameMs) > PS2_FAILSAFE_MS;
}

uint32_t Ps2Controller::frameAgeMs(uint32_t nowMs) const {
  return state_.lastGoodFrameMs == 0U ? 0U : nowMs - state_.lastGoodFrameMs;
}

Ps2ReceiverStatus Ps2Controller::receiverStatus(uint32_t nowMs) const {
  if (state_.lastGoodFrameMs == 0U) return Ps2ReceiverStatus::WAIT;
  if (!state_.receiverConnected || frameTimedOut(nowMs)) return Ps2ReceiverStatus::LOST;
  if (contentInitialized_ && (nowMs - lastActivityMs_) <= PS2_ACTIVITY_MS)
    return Ps2ReceiverStatus::ACT;
  return Ps2ReceiverStatus::RX;
}

const char* Ps2Controller::receiverStatusText(uint32_t nowMs) const {
  switch (receiverStatus(nowMs)) {
    case Ps2ReceiverStatus::WAIT: return "WAIT";
    case Ps2ReceiverStatus::ACT: return "ACT";
    case Ps2ReceiverStatus::RX: return "RX";
    case Ps2ReceiverStatus::LOST: return "LOST";
  }
  return "LOST";
}

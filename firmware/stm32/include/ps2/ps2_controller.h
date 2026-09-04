#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <Arduino.h>
#include <PS2X_STM32.h>

struct Ps2State {
  bool receiverConnected = false;
  bool frameFresh = false;
  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
  bool l1 = false;
  bool l2 = false;
  bool r1 = false;
  bool r2 = false;
  bool l3 = false;
  bool r3 = false;
  bool start = false;
  bool select = false;
  bool triangle = false;
  bool circle = false;
  bool cross = false;
  bool square = false;
  int16_t lx = 0;
  int16_t ly = 0;
  int16_t rx = 0;
  int16_t ry = 0;
  uint32_t lastGoodFrameMs = 0;
};

enum class Ps2Button : uint8_t {
  UP, DOWN, LEFT, RIGHT, L1, L2, R1, R2, L3, R3, START, SELECT,
  TRIANGLE, CIRCLE, CROSS, SQUARE
};

enum class Ps2ReceiverStatus : uint8_t { WAIT, ACT, RX, LOST };

enum class Ps2MapAction : uint8_t {
  SLOT,
  START,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  TRIANGLE,
  SQUARE,
  SQUARE_LONG,
  CIRCLE,
  CROSS,
  CROSS_LONG,
  SELECT_LONG,
};

struct Ps2MapEvent {
  Ps2MapAction action = Ps2MapAction::SLOT;
  uint8_t slot = 1U;
};

class Ps2Controller {
 public:
  Ps2Controller();
  void begin();
  void update();
  bool takeMapEvent(Ps2MapEvent& event);
  // Safety boundary used by local MAP cancellation. It drops any queued MAP
  // action and requires released buttons plus fresh neutral frames before a
  // new START edge can be generated.
  void disarmMapInput();
  // Same re-arm boundary for USER HOLD, but preserve the currently-held X so
  // the long-press timer can still escalate HOLD to CANCEL.
  void holdMapInput();
  // While a MAP Settings/Help/Delete screen owns the UI, the RobotController
  // must fail closed and D-pad/joystick input must stay inside the menu.
  void setMapUiCapture(bool enabled);
  bool mapUiCaptureActive() const { return mapUiCapture_; }

  const Ps2State& state() const { return state_; }
  bool buttonPressed(Ps2Button button) const;
  bool buttonReleased(Ps2Button button) const;
  bool motionCommandActive() const;
  // The wireless receiver can keep returning valid neutral frames after the
  // handheld transmitter is powered off. This reports operator activity, not
  // merely electrical presence of the receiver module.
  bool controlActive(uint32_t nowMs) const;
  bool frameTimedOut(uint32_t nowMs) const;
  Ps2ReceiverStatus receiverStatus(uint32_t nowMs) const;
  const char* receiverStatusText(uint32_t nowMs) const;
  uint32_t frameAgeMs(uint32_t nowMs) const;
  uint32_t lastFrameIntervalMs() const { return lastFrameIntervalMs_; }
  uint8_t configResult() const { return configResult_; }
  bool configured() const { return configResult_ == 0; }
  uint8_t mode() const { return driver_.mode(); }
  uint16_t rawButtons() const { return driver_.rawButtons(); }
  uint8_t consecutiveErrors() const { return driver_.consecutiveErrors(); }
  uint32_t goodFrameCount() const { return goodFrameCount_; }
  uint8_t rawByte(uint8_t index) const { return driver_.rawByte(index); }

 private:
  static uint16_t maskFor(Ps2Button button);
  static int16_t processAxis(uint8_t raw, bool invert);
  void reconnect(uint32_t nowMs);
  void captureState(uint32_t nowMs);
  void updateActivity(uint32_t nowMs, uint16_t buttons, uint8_t lx,
                      uint8_t ly, uint8_t rx, uint8_t ry);
  bool queueMapEvent(Ps2MapAction action);
  void resetMapPressTracking();
  void resetMapCrossTracking();

  PS2X_STM32 driver_;
  Ps2State state_;
  uint32_t lastPollMs_ = 0;
  uint32_t lastReconnectMs_ = 0;
  uint32_t lastActivityMs_ = 0;
  uint16_t previousFingerprint_ = 0xFFFF;
  uint8_t previousLx_ = 128;
  uint8_t previousLy_ = 128;
  uint8_t previousRx_ = 128;
  uint8_t previousRy_ = 128;
  bool contentInitialized_ = false;
  uint32_t lastFrameIntervalMs_ = 0;
  uint32_t goodFrameCount_ = 0;
  uint8_t configResult_ = 1;
  bool mapEventPending_ = false;
  Ps2MapEvent mapEvent_{};

  // Map UI events use explicit controller-level edges instead of the PS2X
  // ButtonPressed() helper. This prevents a stale helper edge from leaking a
  // Map action across an L3 page transition.
  bool mapEdgesInitialized_ = false;
  bool previousMapL3_ = false;
  bool previousMapStart_ = false;
  bool previousMapUp_ = false;
  bool previousMapDown_ = false;
  bool previousMapLeft_ = false;
  bool previousMapRight_ = false;
  bool previousMapSelect_ = false;
  bool previousMapTriangle_ = false;
  bool previousMapSquare_ = false;
  bool previousMapCircle_ = false;
  bool previousMapCross_ = false;

  bool mapCrossPressActive_ = false;
  bool mapCrossLongFired_ = false;
  uint32_t mapCrossStartedMs_ = 0U;

  // Map actions are deliberately disarmed on every page transition and after
  // reconnect. They are armed only after two consecutive fresh frames show
  // every Map-related button released while the LCD is on MAP.
  bool mapActionsArmed_ = false;
  bool mapUiCapture_ = false;
  uint8_t mapNeutralReleaseFrames_ = 0;
  uint32_t lastMapPageToggleMs_ = 0;

  // SELECT and SQUARE need mutually-exclusive short/long semantics. Short
  // actions fire on release; a held press emits one *_LONG event and suppresses
  // the short action on release.
  bool mapSelectPressActive_ = false;
  bool mapSelectLongFired_ = false;
  uint32_t mapSelectStartedMs_ = 0;
  bool mapSquarePressActive_ = false;
  bool mapSquareLongFired_ = false;
  uint32_t mapSquareStartedMs_ = 0;
};

#endif

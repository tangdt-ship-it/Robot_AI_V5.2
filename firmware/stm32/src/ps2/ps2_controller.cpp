#include <ps2/ps2_controller.h>
#include <robot_config.h>
#include <display/lcd_display.h>
#include <string.h>

// These objects are owned by main.cpp. Map page/slot presentation is kept on
// STM32 so L3/SELECT never depend on ESP32 polling. Short Map actions are sent
// upward as lightweight unsolicited RobotLink events; ESP32 can observe them
// without owning the raw PS2 polling loop.
extern LcdDisplay display;
extern HardwareSerial robotAiSerial;

namespace {
void emitMapEvent(const char* action, uint8_t slot) {
  const uint8_t normalizedSlot = slot == 2U ? 2U : 1U;
  if (strcmp(action, "SLOT") == 0) {
    // Slot-selection event has the compact canonical form
    // <EVENT,MAP,SLOT,1|2>. Do not emit the duplicated SLOT token.
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

void Ps2Controller::begin() {
  const uint32_t now = millis();
  reconnect(now);
  lastPollMs_ = now;
}

void Ps2Controller::reconnect(uint32_t nowMs) {
  configResult_ = driver_.config_gamepad(false, false);
  state_.receiverConnected = (configResult_ == 0);
  state_.frameFresh = false;
  lastReconnectMs_ = nowMs;

  // Re-arm Map edges from the first fresh frame after reconnect. This avoids
  // synthesizing a press from stale pre-disconnect state.
  mapEdgesInitialized_ = false;
  previousMapL3_ = false;
  previousMapSelect_ = false;
  previousMapTriangle_ = false;
  previousMapSquare_ = false;
  previousMapCircle_ = false;
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

  // Use explicit edges from the captured boolean state rather than the
  // library ButtonPressed() helper for Map controls. The latter can retain a
  // transition long enough to leak an action across an L3 page change.
  if (!mapEdgesInitialized_) {
    previousMapL3_ = state_.l3;
    previousMapSelect_ = state_.select;
    previousMapTriangle_ = state_.triangle;
    previousMapSquare_ = state_.square;
    previousMapCircle_ = state_.circle;
    mapEdgesInitialized_ = true;
  } else {
    const bool l3Pressed = state_.l3 && !previousMapL3_;
    const bool selectPressed = state_.select && !previousMapSelect_;
    const bool trianglePressed = state_.triangle && !previousMapTriangle_;
    const bool squarePressed = state_.square && !previousMapSquare_;
    const bool circlePressed = state_.circle && !previousMapCircle_;

    // L3 is deliberately 100% local to STM32: it only changes the LCD page and
    // emits no ESP32 Map action. If L3 changes page in this frame, suppress all
    // other Map actions until the next fresh frame so no event can cross the
    // page boundary.
    if (l3Pressed) {
      display.togglePage();
    } else if (display.isMapPage()) {
      if (selectPressed) {
        display.toggleMapSlot();
        emitMapEvent("SLOT", display.mapSlot());
      }
      if (trianglePressed) {
        emitMapEvent("TRIANGLE", display.mapSlot());
      }
      if (squarePressed) {
        emitMapEvent("SQUARE", display.mapSlot());
      }
      if (circlePressed) {
        emitMapEvent("CIRCLE", display.mapSlot());
      }
    }

    previousMapL3_ = state_.l3;
    previousMapSelect_ = state_.select;
    previousMapTriangle_ = state_.triangle;
    previousMapSquare_ = state_.square;
    previousMapCircle_ = state_.circle;
  }

  // A digital 0x41 frame contains no axis bytes; the unused bytes commonly
  // read 0xFF and must never be interpreted as full joystick deflection.
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
  return state_.frameFresh && driver_.ButtonPressed(maskFor(button));
}
bool Ps2Controller::buttonReleased(Ps2Button button) const {
  return state_.frameFresh && driver_.ButtonReleased(maskFor(button));
}

bool Ps2Controller::motionCommandActive() const {
  if (!state_.receiverConnected) return false;
  if (state_.up || state_.down || state_.left || state_.right) return true;
  // Processed zero corresponds to the configured raw deadzone. The larger raw
  // release threshold is retained as a documented safety margin in config.
  return state_.lx != 0 || state_.ly != 0;
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

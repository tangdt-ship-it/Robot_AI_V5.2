#include "PS2X_STM32.h"

namespace {
const uint8_t ENTER_CONFIG[] = {0x01, 0x43, 0x00, 0x01, 0x00};
const uint8_t SET_ANALOG_MODE[] =
    {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const uint8_t EXIT_CONFIG[] =
    {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
}

PS2X_STM32::PS2X_STM32(uint32_t clkPin, uint32_t cmdPin,
                       uint32_t attPin, uint32_t datPin)
    : clkPin_(clkPin), cmdPin_(cmdPin), attPin_(attPin), datPin_(datPin),
      buttons_(0xFFFF), lastButtons_(0xFFFF), mode_(0), connected_(false),
      lastGoodReadMs_(0), consecutiveErrors_(0) {
  for (uint8_t& value : data_) value = 0xFF;
}

uint8_t PS2X_STM32::transfer(uint8_t value) {
  uint8_t result = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    digitalWrite(cmdPin_, (value & (1U << bit)) ? HIGH : LOW);
    delayMicroseconds(CLOCK_HALF_US);
    digitalWrite(clkPin_, LOW);
    delayMicroseconds(CLOCK_HALF_US);
    if (digitalRead(datPin_) == HIGH) result |= static_cast<uint8_t>(1U << bit);
    digitalWrite(clkPin_, HIGH);
    delayMicroseconds(CLOCK_HALF_US);
  }
  digitalWrite(cmdPin_, HIGH);
  delayMicroseconds(BYTE_GAP_US);
  return result;
}

void PS2X_STM32::beginTransaction() {
  digitalWrite(cmdPin_, HIGH);
  digitalWrite(clkPin_, HIGH);
  digitalWrite(attPin_, LOW);
  delayMicroseconds(BYTE_GAP_US);
}

void PS2X_STM32::endTransaction() {
  digitalWrite(attPin_, HIGH);
  digitalWrite(cmdPin_, HIGH);
  digitalWrite(clkPin_, HIGH);
  delayMicroseconds(BYTE_GAP_US);
}

void PS2X_STM32::sendCommand(const uint8_t* command, size_t length) {
  beginTransaction();
  for (size_t i = 0; i < length; ++i) (void)transfer(command[i]);
  endTransaction();
  delay(commandDelayMs_);
}

bool PS2X_STM32::responseModeValid(uint8_t mode) const {
  return mode == 0x41 || mode == 0x73 || mode == 0x79;
}

uint8_t PS2X_STM32::config_gamepad(bool pressures, bool rumble) {
  (void)pressures;
  (void)rumble;
  pinMode(clkPin_, OUTPUT);
  pinMode(cmdPin_, OUTPUT);
  pinMode(attPin_, OUTPUT);
  pinMode(datPin_, INPUT_PULLUP);
  digitalWrite(clkPin_, HIGH);
  digitalWrite(cmdPin_, HIGH);
  digitalWrite(attPin_, HIGH);
  delay(100);

  // PS2X v1.8 probes twice before deciding that a receiver is absent.
  const bool firstReadOk = read_gamepad();
  const bool secondReadOk = read_gamepad();
  if (!firstReadOk && !secondReadOk) {
    connected_ = false;
    return 1;
  }
  commandDelayMs_ = 1;
  for (uint8_t attempt = 0; attempt <= 10; ++attempt) {
    sendCommand(ENTER_CONFIG, sizeof(ENTER_CONFIG));
    sendCommand(SET_ANALOG_MODE, sizeof(SET_ANALOG_MODE));
    sendCommand(EXIT_CONFIG, sizeof(EXIT_CONFIG));
    if (read_gamepad() && (mode_ == 0x73 || mode_ == 0x79)) {
      connected_ = true;
      return 0;
    }
    if (commandDelayMs_ < 11U) ++commandDelayMs_;
  }
  connected_ = false;
  return responseModeValid(mode_) ? 2 : 1;
}

bool PS2X_STM32::read_gamepad() {
  lastButtons_ = buttons_;
  beginTransaction();
  static const uint8_t request[9] =
      {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  for (uint8_t i = 0; i < 9; ++i) data_[i] = transfer(request[i]);
  endTransaction();

  const uint8_t newMode = data_[1];
  if (!responseModeValid(newMode) || data_[2] != 0x5A) {
    if (consecutiveErrors_ < 255U) ++consecutiveErrors_;
    if (consecutiveErrors_ >= MAX_CONSECUTIVE_ERRORS) connected_ = false;
    return false;
  }
  consecutiveErrors_ = 0;
  mode_ = newMode;
  buttons_ = static_cast<uint16_t>(data_[3]) |
             (static_cast<uint16_t>(data_[4]) << 8);
  connected_ = true;
  lastGoodReadMs_ = millis();
  return true;
}

bool PS2X_STM32::connected() const {
  return connected_ && (millis() - lastGoodReadMs_ < 250U);
}
bool PS2X_STM32::Button(uint16_t button) const { return ((~buttons_) & button) != 0; }
bool PS2X_STM32::ButtonPressed(uint16_t button) const {
  return (((lastButtons_ ^ buttons_) & button) != 0) && (((~buttons_) & button) != 0);
}
bool PS2X_STM32::ButtonReleased(uint16_t button) const {
  return (((lastButtons_ ^ buttons_) & button) != 0) && (((~lastButtons_) & button) != 0);
}
bool PS2X_STM32::NewButtonState() const { return lastButtons_ != buttons_; }
bool PS2X_STM32::NewButtonState(uint16_t button) const {
  return ((lastButtons_ ^ buttons_) & button) != 0;
}
uint8_t PS2X_STM32::Analog(uint8_t index) const {
  return index < sizeof(data_) ? data_[index] : 128;
}
uint8_t PS2X_STM32::mode() const { return mode_; }
uint32_t PS2X_STM32::lastGoodReadMs() const { return lastGoodReadMs_; }
uint16_t PS2X_STM32::rawButtons() const { return buttons_; }
uint8_t PS2X_STM32::consecutiveErrors() const { return consecutiveErrors_; }

uint8_t PS2X_STM32::rawByte(uint8_t index) const {
  return index < 9U ? data_[index] : 0xFFU;
}

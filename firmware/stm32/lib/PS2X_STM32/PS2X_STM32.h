#ifndef PS2X_STM32_H
#define PS2X_STM32_H

#include <Arduino.h>

#define PSB_SELECT 0x0001
#define PSB_L3 0x0002
#define PSB_R3 0x0004
#define PSB_START 0x0008
#define PSB_PAD_UP 0x0010
#define PSB_PAD_RIGHT 0x0020
#define PSB_PAD_DOWN 0x0040
#define PSB_PAD_LEFT 0x0080
#define PSB_L2 0x0100
#define PSB_R2 0x0200
#define PSB_L1 0x0400
#define PSB_R1 0x0800
#define PSB_TRIANGLE 0x1000
#define PSB_CIRCLE 0x2000
#define PSB_CROSS 0x4000
#define PSB_SQUARE 0x8000

#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

class PS2X_STM32 {
 public:
  PS2X_STM32(uint32_t clkPin, uint32_t cmdPin, uint32_t attPin,
             uint32_t datPin);
  uint8_t config_gamepad(bool pressures = false, bool rumble = false);
  bool read_gamepad();
  bool connected() const;
  bool Button(uint16_t button) const;
  bool ButtonPressed(uint16_t button) const;
  bool ButtonReleased(uint16_t button) const;
  bool NewButtonState() const;
  bool NewButtonState(uint16_t button) const;
  uint8_t Analog(uint8_t index) const;
  uint8_t mode() const;
  uint32_t lastGoodReadMs() const;
  uint16_t rawButtons() const;
  uint8_t consecutiveErrors() const;
  uint8_t rawByte(uint8_t index) const;

 private:
  uint8_t transfer(uint8_t value);
  void beginTransaction();
  void endTransaction();
  void sendCommand(const uint8_t* command, size_t length);
  bool responseModeValid(uint8_t mode) const;

  uint32_t clkPin_;
  uint32_t cmdPin_;
  uint32_t attPin_;
  uint32_t datPin_;
  uint8_t data_[21];
  uint16_t buttons_;
  uint16_t lastButtons_;
  uint8_t mode_;
  bool connected_;
  uint32_t lastGoodReadMs_;
  uint8_t consecutiveErrors_;

  // STM32 digitalWrite has different edge overhead from AVR direct-port I/O.
  // This waveform is the measured-good equivalent on the F103 board.
  static constexpr uint16_t CLOCK_HALF_US = 5;
  static constexpr uint16_t BYTE_GAP_US = 10;
  static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 3;
  uint8_t commandDelayMs_ = 1;
};

#endif

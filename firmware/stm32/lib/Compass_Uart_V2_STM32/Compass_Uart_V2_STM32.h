#ifndef COMPASS_UART_V2_STM32_H
#define COMPASS_UART_V2_STM32_H

#include <Arduino.h>

class Compass_Uart_V2 {
 public:
  Compass_Uart_V2(uint32_t rxPin = PA3, uint32_t txPin = PA2);
  void begin(uint32_t baud);
  void reset();
  int16_t read();
  bool lastReadOk() const;
  bool requestRead();
  bool pollRead(int16_t& value);
  bool readPending() const;
  bool readTimedOut() const;
  void cancelRead();

 private:
  enum class RxState : uint8_t { IDLE, WAIT_HIGH, WAIT_LOW };
  void clearRx();
  bool waitForBytes(size_t count, uint32_t timeoutUs);

  HardwareSerial serial_;
  bool lastReadOk_;
  RxState rxState_;
  uint32_t requestStartUs_;
  uint32_t byteStartUs_;
  uint8_t highByte_;
  bool timedOut_;
  static constexpr uint32_t RESPONSE_TIMEOUT_US = 12000U;
  static constexpr uint32_t BYTE_GAP_TIMEOUT_US = 3000U;
};

#endif


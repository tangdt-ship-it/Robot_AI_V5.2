#ifndef DEBUG_OUTPUT_H
#define DEBUG_OUTPUT_H

#include <Arduino.h>

// SEGGER-compatible RTT channel. OpenOCD reads it through the existing ST-Link,
// so diagnostics remain available when no USB-UART COM port is present.
class RttStream : public Print {
 public:
  size_t write(uint8_t value) override;
  size_t write(const uint8_t* buffer, size_t size) override;
  using Print::write;
};

class DebugOutput : public Print {
 public:
  DebugOutput(HardwareSerial& uart, RttStream& rtt) : uart_(uart), rtt_(rtt) {}
  size_t write(uint8_t value) override;
  size_t write(const uint8_t* buffer, size_t size) override;
  using Print::write;

 private:
  HardwareSerial& uart_;
  RttStream& rtt_;
};

#endif


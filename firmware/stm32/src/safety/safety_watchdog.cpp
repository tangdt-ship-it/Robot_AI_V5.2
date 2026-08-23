#include <safety/safety_watchdog.h>

#include <Arduino.h>
#include <IWatchdog.h>
#include <robot_config.h>
#include <stm32f1xx.h>

extern HardwareSerial robotAiSerial;

namespace {
const char* ResetCauseName(uint32_t csr) {
  if ((csr & RCC_CSR_IWDGRSTF) != 0U) return "IWDG";
  if ((csr & RCC_CSR_WWDGRSTF) != 0U) return "WWDG";
  if ((csr & RCC_CSR_SFTRSTF) != 0U) return "SOFTWARE";
  if ((csr & RCC_CSR_LPWRRSTF) != 0U) return "LOWPOWER";
  if ((csr & RCC_CSR_PORRSTF) != 0U) return "POWER_ON";
  if ((csr & RCC_CSR_PINRSTF) != 0U) return "RESET_PIN";
  return "UNKNOWN";
}

void ReportAndClearResetCause() {
  const uint32_t csr = RCC->CSR;
  robotAiSerial.print("<EVENT,STM32,RESET,CAUSE,");
  robotAiSerial.print(ResetCauseName(csr));
  robotAiSerial.print(",CSR,0x");
  robotAiSerial.print(csr, HEX);
  robotAiSerial.print(">\r\n");

  // Clear latched reset flags only after reporting them, so any later reboot
  // can be attributed to the new runtime session rather than the flash/reset
  // operation that started this one.
  RCC->CSR |= RCC_CSR_RMVF;
}
}  // namespace

void SafetyWatchdog::begin() {
  // RobotLink UART is already started immediately before this call in setup().
  // Emit the hardware reset cause on every boot before arming a fresh IWDG.
  ReportAndClearResetCause();

  if (!IWDG_ENABLED) {
    enabled_ = false;
    return;
  }

  // STM32duino IWatchdog expects the timeout in microseconds.
  const uint32_t timeoutUs = IWDG_TIMEOUT_MS * 1000UL;
  IWatchdog.begin(timeoutUs);
  enabled_ = IWatchdog.isEnabled();
}

void SafetyWatchdog::kick() {
  if (enabled_) {
    IWatchdog.reload();
  }
}

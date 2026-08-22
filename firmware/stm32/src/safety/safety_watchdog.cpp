#include <safety/safety_watchdog.h>

#include <IWatchdog.h>
#include <robot_config.h>

void SafetyWatchdog::begin() {
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

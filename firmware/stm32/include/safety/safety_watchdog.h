#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

class SafetyWatchdog {
 public:
  void begin();
  void kick();
  bool enabled() const { return enabled_; }

 private:
  bool enabled_ = false;
};

#endif

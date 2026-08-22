# V4.3 upgrade report

- Baseline: STM32 and host self-tests passed. STM32 V4.3 build passed (RAM
  5,896 B / 64 KB, Flash 67,784 B / 512 KB); ESP32 build is compiling with the
  pinned ESP-IDF 5.5.2 environment.
- Fixed: ESP32 reboot/sequence-zero could be rejected before HELLO; fusion had
  a fixed encoder-yaw weight and fixed high confidence; robot MCP registered
  32 tools.
- Changed: RobotLink server/session gate, heading fusion/config/telemetry,
  Xiaozhi MCP registration, version/changelog.
- Verification: STM32 build and ST-Link upload/verify passed. ESP32-S3 build
  and COM4 flash passed; the ESP32 console continued reporting normal system
  status after boot. COM12 was not present on this PC during the final check,
  so STM32 UART diagnostics could not be captured.
- Limitation: a clone PS2 receiver can prove receiver frames but cannot prove
  RF pairing while neutral; only observed controller activity may preempt AI.

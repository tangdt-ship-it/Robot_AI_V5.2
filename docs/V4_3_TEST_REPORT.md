# V4.3 test report

## Runtime evidence (ESP32 COM4, current flash)

- RobotLink: `HELLO/PING`, `GET STATE`, `MODE AI`, and `STOP` passed; no
  sequence replay was reported.
- Sensors: compass status and compass reset/zero event passed; HC-SR04 was
  fresh/valid at 20.9 cm (CAUTION); heading, speed, brake, and ramp checks
  passed.
- PS2: receiver communication passed (`RX`, fresh frame, enabled). This only
  proves the clone receiver link; RF pairing/manual activity was not exercised.
- MCP: robot registrations are allowlisted to 16 operational tools; detailed
  diagnostics stay on the local console. The boot log contains no tool-limit
  error.

## Build/flash

- STM32 production build and ST-Link upload/verify: passed.
- ESP32-S3 CAM build and COM4 flash: passed.

## Not exercised

- MOVE_DISTANCE, TURN, obstacle stop under motion, PS2 manual driving, and
  Return Home were not run. The forward ultrasonic zone was CAUTION at 20.9 cm,
  so motion was intentionally not forced. No claim of PASS is made for these.

## Return Home commissioning (current run)

- HOME snapshot: PASS. `GET,ODOMETRY` snapshot was X=5288.6 mm, Y=-5386.2 mm,
  H=-148.6 deg; `GET_HOME` returned local X=0 cm, Y=0 cm, H about 6.5 deg.
- Encoder reset invariance: PASS. After `ENCODER,RESET`, LT/RT were 0 and
  absolute ODOM X/Y plus HomePose were unchanged.
- Heading reset invariance: PASS. Voice `COMPASS,RESET` ACKed, heading read
  0.0 deg, and HomePose remained unchanged.
- The planned 300 mm test was invalidated by two consecutive 300 mm voice
  commands (about 600 mm total); it is not counted as a 300 mm PASS.
- Return Home on the resulting approximately 600 mm route: PASS. Mission
  ended `return_completed`; final ODOM X=5272.9 mm, Y=-5372.0 mm,
  H=-149.5 deg. Position error was approximately 21 mm and heading error
  approximately 0.9 deg. Robot stopped and returned to MANUAL.
- Obstacle events remained active during the mission; no safety bypass was
  used. No Return Home source patch was required in this run.

## Camera Vision commissioning (B1, production YUV422/SVGA)

- PASS. The OV2640 remained on `PIXFORMAT_YUV422` at `FRAMESIZE_SVGA`
  (800x600); TFT preview remained active during capture.
- JPEG conversion streamed one PSRAM-owned chunk at a time over the existing
  chunked multipart Vision request. Runtime validation recorded matching
  payload/on-wire write counts, JPEG SOI/EOI `FFD8`/`FFD9`, and safe memory
  telemetry throughout the upload.
- The real Xiaozhi Vision service returned image descriptions for two physical
  scenes. In the second it described an orange-and-black electric mosquito
  racket, light brick wall, a hand holding a dark device and a green light.
- No RGB565/VGA diagnostic fallback was needed. No STM32, Return Home, PS2,
  RobotLink, MCP tool registration, TFT UI, audio, or Wi-Fi changes were made.

## Camera Vision SRAM commissioning

- PATCH 1 PASS: the JPEG encoder thread now completes and joins before the
  Vision HTTP client opens. This reclaims the 3072-byte pthread stack before
  the HTTP TCP receive task (4096-byte stack) and its required internal-memory
  allocations are active. The JPEG remains a single PSRAM-owned queue chunk;
  no image format, resolution, quality, preview, audio, or network behavior
  changed.
- Baseline versus PATCH 1 runtime: internal free before HTTP improved from
  about 13 KB to 16.0--17.9 KB; after HTTP open from 6.5--7.6 KB to
  10.3--11.4 KB; and during JPEG upload from about 5 KB to 10.2--11.6 KB.
  The system minimum SRAM improved from about 475 B to 1595 B.
- Five consecutive physical `self.camera.take_photo` runs passed without a
  reboot. JPEG SOI/EOI, chunked writes, upload, TFT preview, and Xiaozhi
  descriptions all passed. Internal and PSRAM free values did not trend down
  between runs; no camera-vision memory leak was observed.

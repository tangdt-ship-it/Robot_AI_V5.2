# Robot_AI_V4_3

- RobotLink V3 now treats `HELLO,PROTO,3` as a new peer session before
  anti-replay, then requires `PING` before AI motion.
- Heading fusion adaptively fades encoder-yaw assistance during disagreement.
- The Xiaozhi robot MCP surface is limited to 16 operational tools.
- Restored `self.robot.reset_compass` within the 16-tool allowlist; COM4
  runtime self-test confirms reset/zero succeeds.

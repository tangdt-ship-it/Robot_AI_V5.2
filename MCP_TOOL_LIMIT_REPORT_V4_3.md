# MCP tool-limit fix report

- MCP tools before: 43 registered (32 were advertised after the earlier guard).
- MCP tools after: 38 registered; the read-only robot diagnostics group is now one tool and reset is one tool. The cloud-facing list remains capped at 32 to match Xiaozhi's hard limit.
- Merged: `get_state`, `get_encoder_status`, `get_odometry`, `get_imu_status`, `get_fusion_status`, `get_obstacle`, `get_heading`, `get_compass_status`, `get_ps2_status` -> `self.robot.get_diagnostics(target=...)`.
- Merged: `self.robot.reset_compass` (including encoder target) -> `self.robot.reset(target=heading|encoder)`.
- Removed from MCP list: only the old diagnostic/reset names; underlying RobotLink functions remain present: YES.
- Camera tool present: `self.camera.take_photo`.
- Build: PASS (incremental ESP32).
- Flash COM4: PASS.
- Boot: TFT, camera init, RobotLink HELLO/PING/STATE, PS2, Wi-Fi/MQTT and audio observed PASS.
- No STM32 flash performed for this MCP-only change.

Voice ACK tests still require runtime speech confirmation for `get_diagnostics` and `reset` encoder target.

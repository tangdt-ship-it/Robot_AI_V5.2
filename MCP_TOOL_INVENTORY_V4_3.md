# MCP tool inventory before consolidation

| Tool | Function / voice intent | Underlying command | Keep / merge into |
|---|---|---|---|
| self.robot.get_state | Robot state, mode, heading, motor status | GET,STATE | merge `self.robot.get_diagnostics(target=state)` |
| self.robot.get_encoder_status | Encoder health/speed | GET,ENCODER_STATUS | merge diagnostics/encoder |
| self.robot.get_odometry | Position, distance, ticks | GET,ODOMETRY | keep |
| self.robot.get_imu_status | IMU health/gyro/accel | GET,IMU_STATUS | merge diagnostics/imu |
| self.robot.get_fusion_status | Fused heading/confidence | GET,FUSION_STATUS | merge diagnostics/fusion |
| self.robot.get_obstacle | Front obstacle state | GET,OBSTACLE | merge diagnostics/obstacle |
| self.robot.move_distance | Move exact distance | MOVE,DISTANCE | keep |
| self.robot.navigate_autonomously | Autonomous mission | mission manager / motion RobotLink | keep |
| self.robot.set_home | Set Home | STM32 odometry | keep |
| self.robot.get_home | Read Home | mission manager | merge navigation status |
| self.robot.return_home | Return Home | mission manager / motion RobotLink | keep |
| self.robot.navigation_map | Read local map | mission manager | merge navigation status |
| self.robot.scan_obstacle | Shadow scan | mission manager / motion RobotLink | keep |
| self.robot.get_obstacle_plan | Read scan plan | mission manager | merge navigation status |
| self.robot.get_mission_state | Read mission state | mission manager | merge navigation status |
| self.robot.cancel_mission | Cancel mission + STOP | STOP, MODE,MANUAL | keep |
| self.robot.get_speed | Read speed | GET,SPEED | merge diagnostics/config |
| self.robot.set_speed | Set speed | SET,SPEED | merge `self.robot.configure(target=speed)` |
| self.robot.set_brake | Set brake | SET,BRAKE | merge configure/brake |
| self.robot.set_ramp | Set ramp | SET,RAMP | merge configure/ramp |
| self.robot.get_heading | Read heading | GET,HEADING | merge diagnostics/heading |
| self.robot.get_compass_status | Compass status | GET,COMPASS_STATUS | merge diagnostics/compass |
| self.robot.reset_compass | Reset heading or encoders | COMPASS,RESET / ENCODER,RESET | rename `self.robot.reset(target=heading|encoder)` |
| self.robot.get_ps2_status | PS2 state | PS2,STATUS | merge diagnostics/ps2 |
| self.robot.stop | Immediate stop | STOP | keep |
| self.robot.move_forward | Manual forward | motion manager / MOVE | merge `self.robot.manual_motion(action=forward...)` |
| self.robot.move_backward | Manual backward | motion manager / MOVE | merge manual_motion |
| self.robot.turn_left | Manual left turn | motion manager / TURN | merge manual_motion |
| self.robot.turn_right | Manual right turn | motion manager / TURN | merge manual_motion |
| self.robot.rotate_continuous | Continuous rotation | motion manager / TURN | merge manual_motion |
| self.robot.turn_relative | Relative turn | TURN,RELATIVE | keep |
| self.robot.turn_to_heading | Absolute turn | TURN,ABSOLUTE | keep |
| self.get_device_status | Device status | ESP32 local | keep |
| self.audio_speaker.set_volume | Speaker volume | ESP32 local | keep |
| self.screen.set_brightness | Screen brightness | ESP32 local | merge screen.configure |
| self.screen.set_theme | Screen theme | ESP32 local | merge screen.configure |
| self.camera.take_photo | Capture and explain photo | OV2640/Xiaozhi vision | keep (required) |
| self.get_system_info | System information | ESP32 local | merge device_admin |
| self.reboot | Reboot ESP32 | ESP32 local | merge device_admin |
| self.upgrade_firmware | OTA upgrade | ESP32 local | merge device_admin |
| self.screen.get_info | Screen information | ESP32 local | merge screen_tools |
| self.screen.snapshot | Screen snapshot | ESP32 local | merge screen_tools |
| self.screen.preview_image | Preview image | ESP32 local | merge screen_tools |
| self.assets.set_download_url | Asset URL | ESP32 local | keep |

This inventory was captured before changing registrations. No STM32 command is removed; consolidation only changes the MCP surface.

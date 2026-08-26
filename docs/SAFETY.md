# Robot_AI — Safety model

Priority nền: `Safety > PS2 > AI mission > idle`.

- boot luôn STOP;
- IWDG production bật;
- RobotLink motion cần CRC/sequence/heartbeat;
- HC-SR04 hard limits forward translation tại STM32;
- encoder stall hủy AI motion;
- PS2 hợp lệ preempt AI ngay tại STM32;
- mất MPU6050 **không tự làm robot chết** nếu fusion còn nguồn khác; trạng thái chuyển DEGRADED;
- mất mọi heading source -> turn/heading-owned AI motion bị từ chối/dừng;
- Return Home/Replay không được bỏ qua obstacle/brake/PS2 safety.

V5 bổ sung thêm fail-closed sensor health, motion ownership, reset-boundary handling, replay preflight và strict `(SID, OP)` correlation. Chi tiết xem `ROBOT_AI_V5_0_PROJECT.md`.

Hardware E-stop cắt EN/power motor vẫn là nâng cấp phần cứng nên có; software STOP không tương đương E-stop điện.

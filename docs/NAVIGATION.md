# Robot_AI_V4.2 — Navigation / Return Home

## Nguồn pose

MissionManager dùng `GET,ODOMETRY` từ STM32:

```text
x_mm, y_mm, heading_rad, left_ticks, right_ticks
```

Không còn dùng `motor time -> estimated distance` làm nguồn localization production.

## HOME và Breadcrumb

- `set_home`: lưu odometry hiện tại làm gốc (0,0,H0), xóa trail cũ.
- Trong autonomous navigation, breadcrumb được thêm khi di chuyển khoảng 12.5 cm hoặc heading thay đổi khoảng 12 độ.
- Tối đa 512 breadcrumb; HOME luôn được giữ.

## RETURN_HOME

ESP32 duyệt breadcrumb từ cuối về đầu. Với mỗi waypoint:

1. đồng bộ pose STM32;
2. tính heading tới waypoint bằng `atan2`;
3. STM32 quay vòng kín tới heading;
4. kiểm tra HC-SR04/camera;
5. `MOVE_DISTANCE` tối đa 60 cm mỗi bước;
6. nếu bị chặn, chạy local avoidance có giới hạn rồi quay lại waypoint;
7. về HOME trong tolerance khoảng 12 cm và quay lại H0.

Return Home vẫn là dead-reckoning bằng encoder/fusion, **không phải SLAM**. Trượt bánh tạo sai số tích lũy.

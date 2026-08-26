# Robot_AI — Navigation / Return Home

## Nguồn pose

MissionManager dùng `GET,ODOMETRY` từ STM32:

```text
x_mm, y_mm, heading_rad, left_ticks, right_ticks
```

Không dùng `motor time -> estimated distance` làm nguồn localization production.

## HOME và Breadcrumb

- `set_home`: lưu odometry hiện tại làm gốc (0,0,H0), xóa trail cũ.
- Breadcrumb được tạo theo quãng đường/thay đổi heading theo policy của MissionManager.
- HOME luôn được bảo vệ trong trail.

## RETURN_HOME

ESP32 duyệt breadcrumb từ cuối về đầu. Với mỗi waypoint:

1. đồng bộ pose STM32;
2. tính heading tới waypoint bằng `atan2`;
3. STM32 quay vòng kín tới heading;
4. kiểm tra safety/obstacle;
5. dùng `MOVE_DISTANCE` cho đoạn dịch chuyển;
6. dừng hoặc chuyển trạng thái theo policy nếu precondition an toàn không đạt;
7. về HOME trong tolerance và khôi phục heading khi khả dụng.

Return Home vẫn là dead-reckoning bằng encoder/fusion, **không phải SLAM**. Trượt bánh tạo sai số tích lũy.

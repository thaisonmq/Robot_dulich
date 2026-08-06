# Motion Safety Runtime

Container này là producer duy nhất của `/cmd_vel`. `twist_mux` ưu tiên joystick > web > Nav2, `nav2_velocity_smoother` giới hạn gia tốc cho chuyển động bình thường, rồi `rovera_motion_safety` kiểm tra hình học footprint thật trước khi xuất lệnh cuối.

E-stop, bumper, cliff, range, mất `/scan` quá 280 ms hoặc mất command đều xuất zero ngay, không qua smoothing. Vùng STOP dùng footprint 0,30 × 0,10 m cộng clearance động (tối thiểu 0,10 m); vùng SLOW nằm xa hơn 0,20 m. Sau khi hết vật cản phải sạch liên tục 400 ms.

Interface mở rộng: `/safety/cliff`, `/safety/bumper`, `/safety/range`, `/safety/directional_mask`, `/safety/health`. Hiện Pi chưa có cliff sensor; tuyệt đối không coi đây là chống rơi cầu thang.

Chạy test pure Python: `pytest -q tests`. Test ROS trong container bằng `ros2 topic echo /safety/health` và xác nhận `/cmd_vel` chỉ có publisher `rovera_motion_safety` trước khi cho phép motor.

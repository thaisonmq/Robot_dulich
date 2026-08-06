# Navigation Stack

Runtime này tách biệt hoàn toàn Robot Agent/WebRTC. `NAVIGATION_MODE=MAPPING` chạy TF, sensor normalizer, EKF và SLAM Toolbox `online_async`. `NAVIGATION_MODE=NAVIGATION` chạy Map Server, AMCL và Nav2; hai mode không chạy cùng lúc.

TF dự kiến: `map → odom → base_footprint → base_link → laser_frame/imu_frame`. `odom_frame` của vendor được normalizer đổi thành `odom`; chỉ EKF publish `odom → base_footprint`. Offset X/Y của LiDAR và IMU đang cấu hình 0 vì phần cứng được mô tả gần tâm, nhưng phải đo lại phụ kiện/offset trước khi test chuyển động. Orientation/gyro-Z IMU hiện không đáng tin nên chưa fusion yaw; cần kê bánh, xoay tay và xác minh trục trước khi bật.

Adapter cung cấp JSON-RPC qua Unix socket cho Robot Agent và gọi trực tiếp các action/service chuẩn: `ComputePathToPose`, `NavigateToPose`, `LoadMap`, initial pose, save map và serialized pose graph. Map lưu staging, tạo SHA-256 per-file, preview và bundle, rồi atomic rename.

Chỉ kiểm tra plan/non-motion trước. Không phát goal thật cho tới khi có người cạnh E-stop, khu vực trống và xác nhận riêng.

# Navigation Stack

Runtime này tách biệt hoàn toàn Robot Agent/WebRTC. `NAVIGATION_MODE=MAPPING` chạy TF, sensor normalizer, EKF và SLAM Toolbox `online_async`. `NAVIGATION_MODE=NAVIGATION` chạy Map Server, AMCL và Nav2; hai mode không chạy cùng lúc.

TF dự kiến: `map → odom → base_footprint → base_link → laser_frame/imu_frame`. Trong cả runtime độc lập và chế độ tương thích Yahboom, `odom_frame` của vendor được normalizer đổi thành `odom`, đúng một EKF Rovera publish `odom → base_footprint`, và robot-state Rovera giữ các fixed TF còn lại. Vì vậy Navigation không mất cây TF khi container tương thích Yahboom dừng nhưng micro-ROS vẫn còn cung cấp cảm biến thô. Thân xe là `0,30 × 0,20 × 0,15 m`; LiDAR nằm giữa mặt trên tại `x=0`, `y=0`, cao `z=0,15 m` so với `base_footprint`. IMU hiện gần tâm ở `z=0.03 m` so với `base_link`. Orientation/gyro-Z IMU hiện không đáng tin nên chưa fusion yaw; cần kê bánh, xoay tay và xác minh trục trước khi bật.

Adapter cung cấp JSON-RPC qua Unix socket cho Robot Agent và gọi trực tiếp các action/service chuẩn: `ComputePathToPose`, `NavigateToPose`, `LoadMap`, initial pose, save map và serialized pose graph. Map lưu staging, tạo SHA-256 per-file, preview và bundle, rồi atomic rename.

Chỉ kiểm tra plan/non-motion trước. Không phát goal thật cho tới khi có người cạnh E-stop, khu vực trống và xác nhận riêng.

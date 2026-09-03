# Tích hợp điều khiển xe ROS 2

Phần này nối lệnh từ WebSocket của Rovera vào xe mà không để công việc camera
chặn lệnh điều khiển:

```text
Web -> robot-simulator -> Unix datagram (latest only) -> control_bridge
                                                     -> /cmd_vel_web ─┐
Managed Yahboom joystick ----------------------------> /cmd_vel_joy ─┼─> mux
Nav2 ------------------------------------------------> /cmd_vel_nav ─┘
                                                            ↓
                                                smoother + motion-safety
                                                            ↓
                                                      /cmd_vel -> MCU
Obstacle detector -> /rovera/obstacle_* -> motion-safety (mọi nguồn)
```

## Đặc tính vận hành

- `MOTION_BACKEND=disabled` là chế độ coexistence read-only: camera/mapping
  vẫn chạy nhưng lệnh Web bị reject rõ ràng. `MOTION_BACKEND=ros2` chỉ được
  script managed-motion bật sau khi kiểm tra quyền sở hữu `/cmd_vel`.
- IPC là Unix datagram không blocking, mỗi lệnh có `boot_id`, sequence và TTL.
  Bridge bỏ lệnh cũ/trùng/hết hạn và chỉ giữ lệnh mới nhất chưa xử lý.
- Edge và bridge có watchdog độc lập. Mất WebSocket, tiến trình hoặc lệnh mới quá
  250 ms đều phát lệnh dừng; STOP được gửi thành một cụm ba gói.
- Software E-stop được latch khi nhận emergency stop. Velocity mới không thể
  nhả latch; chỉ datagram `estop_reset` riêng mới được phép reset và transition
  này vẫn phát zero. Edge chỉ báo hoàn tất về Center sau khi odometry mới xác
  nhận vận tốc tuyến tính/góc bằng zero trong toàn bộ dwell cấu hình.
- Managed-motion thay đúng tiến trình joystick vendor bằng cùng image/launch,
  chỉ remap `cmd_vel` sang `/cmd_vel_joy`; `/joy`, `/JoyState` và `/rpi5_ip`
  được giữ. Nó cũng chạy lại `yahboomcar_bringup` để giữ `/odom`, `/imu/data`,
  `/joint_states`, `/robot_description`, `/tf` và `/tf_static`; mapping-stack
  tái sử dụng các node này thay vì publish TF/EKF trùng. Agent legacy và toàn
  bộ topic cảm biến không bị thay thế.
- Các service ROS dùng `network_mode: host` và `ipc: host`; những service cùng
  UID truyền shared memory xuyên container. Yahboom/Agent chạy root cho phần
  cứng và dùng UDPv4 LAN để nối với service UID 1000. RViz2 cùng domain ở máy
  LAN nhìn được graph. Chỉ chạy một runtime Yahboom/Agent trên domain 20.
  Entry point Yahboom có memory guard 900 MiB và giới hạn virtual memory để
  bảo vệ cả các kernel bỏ qua Docker `mem_limit`.
- `motion-safety` là publisher duy nhất của `/cmd_vel`. Bridge bị entrypoint
  từ chối nếu Web không ra `/cmd_vel_web` hoặc nếu cố tự bật mux thứ hai.
- Giới hạn tốc độ được kẹp ở cả edge và bridge.
- Chương trình chống vật cản phát `std_msgs/Bool` lên
  `/rovera/obstacle_stop`: `true` khóa và phát zero liên tục, `false` mở khóa.
  Lệnh velocity của Web không thể tự xóa khóa này. Motion safety nhận tín hiệu
  trực tiếp nên khóa áp dụng cho Web, joystick và Nav2.
- Production mặc định `ROS_OBSTACLE_WATCHDOG_MS=500`. Bridge nhận heartbeat
  `/safety/bridge_interlock` từ motion-safety, vì vậy nó khóa ngay khi node
  safety chưa sẵn sàng, phát hard-stop hoặc mất heartbeat. Các nguồn cảm biến
  ngoài vẫn publish định kỳ vào `/rovera/obstacle_*`; motion-safety hợp nhất
  chúng với LiDAR rồi mới phát heartbeat cuối. Chỉ đặt watchdog bằng `0` trong
  bench/service mode khi motion output đã bị cô lập.
- Để chỉ khóa chiều gần vật cản, publish `std_msgs/UInt8` lên
  `/rovera/obstacle_directions`: bit 0 chặn tiến, bit 1 chặn lùi, bit 2 chặn
  quay trái và bit 3 chặn quay phải. Các thành phần vận tốc không hướng vào vật
  cản vẫn được giữ nguyên.

## Giao thức dừng do vật cản

Chương trình cảm biến không publish trực tiếp vào `/cmd_vel`. Nó publish ở
10--20 Hz:

```bash
# Có vật cản: giữ khóa cho tới khi vùng an toàn thực sự thông thoáng.
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_stop std_msgs/msg/Bool '{data: true}' -r 10

# Hết vật cản: tiếp tục heartbeat an toàn nếu watchdog đang bật.
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_stop std_msgs/msg/Bool '{data: false}' -r 10
```

Khóa theo hướng dùng bitmask:

```bash
# Cản phía trước (1): chặn tiến, vẫn cho lùi/quay.
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 1}' -r 10

# Cản trước + trái (1 + 4 = 5): chặn tiến và quay trái.
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 5}' -r 10

# Không có hướng bị chặn.
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 0}' -r 10
```

Với lệnh `-r 10`, phải dừng publisher giá trị `1` bằng `Ctrl+C` trước khi chạy
publisher giá trị `0`; không để hai publisher 1/0 chạy đồng thời. Mask được giữ
cho tới khi nhận giá trị mới. Khi chuyển `1 -> 0`, Web được mở lại sau khoảng
400 ms clear hysteresis nếu LaserScan vẫn an toàn.

## Dịch vụ Compose

Bridge nằm sau profile `managed-motion`:

- `ros-control-bridge`: bridge IPC, watchdog và physical-override guard.

Managed-motion dùng:

- `yahboom-joystick`: launch Yahboom hiện có nhưng remap `cmd_vel` thành
  `/cmd_vel_joy`.
- `motion-safety`: mux/smoother/safety và publisher duy nhất của `/cmd_vel`.

Nó cố ý **không** bật `micro-ros-agent`; Agent guarded legacy đang phục vụ
`/scan`, `/imu`, `/odom_raw` được giữ nguyên.

## Trình tự bật trên Pi sau này

1. Chạy preflight không thay đổi container:

   ```bash
   ./scripts/cutover_managed_motion.sh
   ```

2. Khi preflight đạt và bánh đã được kê, thực hiện cutover có xác nhận:

   ```bash
   export ROVERA_EXCLUSIVE_CMD_VEL_ACK=I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP
   ./scripts/cutover_managed_motion.sh --apply
   ```

Script build trước khi dừng joystick cũ, giữ Agent serial, rồi xác minh đúng một
publisher `/cmd_vel`. Nếu xác minh lỗi, script dừng các service mới và khởi động
lại container joystick cũ. Autostart vendor `uros.desktop` cũng được đổi tên
sang `uros.desktop.rovera-disabled` trong lúc apply để reboot không tạo lại một
publisher trực tiếp; rollback tự khôi phục file này.

Các giá trị mặc định và giới hạn nằm trong `edge.env.example`. `ROS_DOMAIN_ID`
mặc định là 20 theo graph đã khảo sát trên xe.

## Camera và độ trễ điều khiển

Room LiveKit chính của robot tắt auto-subscribe và chỉ chủ động nhận audio từ
identity `user:*`. Vì camera encoded dùng một identity khác, Pi không còn tải
ngược chính luồng video mình vừa phát. Cấu hình mẫu dùng 1080p25 ở 6 Mbps để
chừa băng thông Wi-Fi cho gói điều khiển; pipeline camera vẫn dùng FPS V4L2 đã
thương lượng và hàng đợi chỉ giữ khung mới nhất ở đường raw.

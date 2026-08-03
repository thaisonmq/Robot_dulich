# Tích hợp điều khiển xe ROS 2

Phần này nối lệnh từ WebSocket của Rovera vào xe mà không để công việc camera
chặn lệnh điều khiển:

```text
Web -> robot-simulator -> Unix datagram (latest only) -> control_bridge
                                                     -> /cmd_vel
Joystick/Yahboom hiện có ----------------------------> /cmd_vel -> YB_Car_Node
                           \-> /joy -> physical-override guard
```

## Đặc tính vận hành

- `MOTION_BACKEND=simulator` là mặc định. Build hoặc bật profile ROS 2 chưa thể
  làm xe chạy.
- IPC là Unix datagram không blocking, mỗi lệnh có `boot_id`, sequence và TTL.
  Bridge bỏ lệnh cũ/trùng/hết hạn và chỉ giữ lệnh mới nhất chưa xử lý.
- Edge và bridge có watchdog độc lập. Mất WebSocket, tiến trình hoặc lệnh mới quá
  250 ms đều phát lệnh dừng; STOP được gửi thành một cụm ba gói.
- Ở chế độ song song, bridge theo dõi `/joy` và ngừng hoàn toàn việc phát lệnh
  web trong 350 ms sau mỗi thao tác joystick vượt deadzone. Bridge cũng theo dõi
  `/JoyState`: khi chế độ joystick Yahboom đang bật, web bị suppress liên tục kể
  cả khi cần joystick đang ở vị trí giữa. Yahboom cũ không bị dừng, restart,
  remap hoặc thay cấu hình.
- Chế độ `twist_mux` vẫn có sẵn cho một lần chuyển đổi toàn bộ stack trong tương
  lai, nhưng không được bật cùng Yahboom cũ đang publish trực tiếp `/cmd_vel`.
- Giới hạn tốc độ được kẹp ở cả edge và bridge.

## Dịch vụ Compose

Bridge bổ sung nằm sau profile `ros2-control`:

- `ros-control-bridge`: bridge IPC, watchdog và physical-override guard.

Hai dịch vụ thay thế được tách riêng sang profile `ros2-managed-stack`:

- `micro-ros-agent`: `/dev/ttyUSB0`, 921600 baud.
- `yahboom-joystick`: launch Yahboom hiện có nhưng remap `cmd_vel` thành
  `/cmd_vel_joy`.

Không bật `ros2-managed-stack` trên Pi đang chạy hai container legacy. Profile
`ros2-control` không chứa và không tác động tới hai dịch vụ đó.

## Trình tự bật trên Pi sau này

1. Giữ `MOTION_BACKEND=simulator`, build và khởi động riêng
   `ros-control-bridge` trong profile `ros2-control`.
2. Xác nhận container Yahboom và micro-ROS cũ không đổi thời gian khởi động.
   `/cmd_vel` sẽ có hai publisher (`joy_ctrl` và `rovera_control_bridge`) nhưng
   bridge không phát Twist khi chưa có lệnh web.
3. Kiểm tra joystick; khi `/joy` có thao tác, bridge phải log việc suppress lệnh
   web và không xen Twist vào nguồn legacy.
4. Chỉ sau các kiểm tra đó mới đổi `MOTION_BACKEND=ros2` rồi recreate riêng
   `robot-simulator`.
5. Bắt đầu bằng giới hạn tốc độ thấp, đo độ trễ lệnh/dừng và chỉ tăng sau khi
   watchdog đã được xác nhận.

Các giá trị mặc định và giới hạn nằm trong `edge.env.example`. `ROS_DOMAIN_ID`
mặc định là 20 theo graph đã khảo sát trên xe.

## Camera và độ trễ điều khiển

Room LiveKit chính của robot tắt auto-subscribe và chỉ chủ động nhận audio từ
identity `user:*`. Vì camera encoded dùng một identity khác, Pi không còn tải
ngược chính luồng video mình vừa phát. Cấu hình mẫu dùng 1080p25 ở 6 Mbps để
chừa băng thông Wi-Fi cho gói điều khiển; pipeline camera vẫn dùng FPS V4L2 đã
thương lượng và hàng đợi chỉ giữ khung mới nhất ở đường raw.

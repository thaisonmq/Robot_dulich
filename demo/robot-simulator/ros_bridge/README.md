# Tích hợp điều khiển xe ROS 2

Phần này nối lệnh từ WebSocket của Rovera vào xe mà không để công việc camera
chặn lệnh điều khiển:

```text
Web -> robot-simulator -> Unix datagram (latest only) -> control_bridge
                                                     -> /cmd_vel
Joystick/Yahboom hiện có ----------------------------> /cmd_vel -> YB_Car_Node
                           \-> /joy -> physical-override guard
Obstacle detector -> /rovera/obstacle_stop ----------> safety interlock
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
- Chương trình chống vật cản phát `std_msgs/Bool` lên
  `/rovera/obstacle_stop`: `true` khóa và phát zero liên tục, `false` mở khóa.
  Lệnh velocity của web không thể tự xóa khóa này. Khi dùng `twist_mux`, zero
  đi qua `/cmd_vel_safety` với priority 255; web là 50 và joystick là 100.
- `ROS_OBSTACLE_WATCHDOG_MS=0` giữ tương thích khi chưa cài chương trình cảm
  biến. Khi đã tích hợp, đặt giá trị dương (khuyến nghị bắt đầu từ 500 ms) và
  publish cả `true` lẫn `false` định kỳ; bridge sẽ khóa ngay từ lúc khởi động
  và khóa lại nếu heartbeat quá hạn.
- Để chỉ khóa chiều gần vật cản, publish `std_msgs/UInt8` lên
  `/rovera/obstacle_directions`: bit 0 chặn tiến, bit 1 chặn lùi, bit 2 chặn
  quay trái và bit 3 chặn quay phải. Các thành phần vận tốc không hướng vào vật
  cản vẫn được giữ nguyên.

## Giao thức dừng do vật cản

Chương trình cảm biến không publish trực tiếp vào `/cmd_vel`. Nó publish ở
10--20 Hz:

```bash
# Có vật cản: giữ khóa cho tới khi vùng an toàn thực sự thông thoáng.
ros2 topic pub /rovera/obstacle_stop std_msgs/msg/Bool '{data: true}' -r 10

# Hết vật cản: tiếp tục heartbeat an toàn nếu watchdog đang bật.
ros2 topic pub /rovera/obstacle_stop std_msgs/msg/Bool '{data: false}' -r 10
```

Khóa theo hướng dùng bitmask:

```bash
# Cản phía trước (1): chặn tiến, vẫn cho lùi/quay.
ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 1}' -r 10

# Cản trước + trái (1 + 4 = 5): chặn tiến và quay trái.
ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 5}' -r 10

# Không có hướng bị chặn.
ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 0}' -r 10
```

Trong chế độ song song với Yahboom legacy, interlock luôn ưu tiên hơn lệnh web
vì nó nằm ngay trong `control_bridge`. Muốn khóa cả joystick với thứ tự ưu tiên
xác định, phải remap joystick sang `/cmd_vel_joy`, đặt
`ROS_USE_TWIST_MUX=true`, `ROS_WEB_CMD_VEL_TOPIC=/cmd_vel_web` và để
`twist_mux` là publisher duy nhất của `/cmd_vel`.

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


cản trước:

ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 \
  '{data: 1}' -r 10

cản sau:

ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 \
  '{data: 2}' -r 10

cản trái:

ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 \
  '{data: 4}' -r 10

cản phải:

ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 \
  '{data: 8}' -r 10

Có thể cộng các giá trị:
Trước + trái: 1 + 4 = 5.
Trước + phải: 1 + 8 = 9.
Tất cả hướng: 15.




Khi hết vật cản:

ros2 topic pub /rovera/obstacle_directions std_msgs/msg/UInt8 \
  '{data: 0}' -r 10
# Mapping, Nav2 và Motion Safety

> Tài liệu triển khai hiện hành: [Mapping và Navigation](MAPPING_AND_NAVIGATION.md), [Map Registry](MAP_REGISTRY.md) và [RViz2](RVIZ_MAPPING_GUIDE.md). Các mô tả cũ về raw scan/live-map trên Web hoặc yêu cầu đặt initial pose mặc định không còn áp dụng.

Tài liệu này mô tả kiến trúc production, thao tác vận hành và cổng an toàn cho tính năng tạo/phân phối map và điều hướng. Không dùng các lệnh `NavigateToPose`, `/cmd_vel*` hoặc test motor trong tài liệu này nếu chưa có xác nhận thử chuyển động vật lý.

## Kiến trúc và data flow

```text
Web Center ──HTTP/WebSocket──> Center Backend ──authenticated WS──> Robot Agent
                                                                    │ Unix JSON-RPC
                                                                    v
                                                           navigation-stack
                                                           TF/EKF/SLAM hoặc Nav2

/cmd_vel_joy ─┐
/cmd_vel_web ─┼─> twist_mux ─> velocity_smoother ─> motion-safety ─> /cmd_vel ─> MCU
/cmd_vel_nav ─┘                                      ^
                                      /scan + estop/cliff/bumper/range
```

- Center là map registry chính. PostgreSQL giữ metadata/lifecycle; bundle lớn nằm ở `MAP_STORAGE_DIR` (mặc định `/var/lib/rovera/map-storage`).
- Robot cache bundle đã verify trong volume `ROBOT_STATE_DIR`; tải qua HTTP,
  kiểm SHA-256, giải nén vào thư mục tạm rồi atomic rename. Fast path/restore
  không tin riêng marker: Edge re-hash archive gốc, đối chiếu metadata trong
  archive và kiểm lại schema, ảnh cùng checksum từng artifact trước khi load.
- Sau mỗi reconnect, restore map, retry upload và các lệnh có thể cấp motion
  authority bị chặn đến khi Edge đã lấy snapshot tombstone có thẩm quyền, ghi
  tombstone bền vững và deactivate/xóa mọi map bị thu hồi. Trạng thái hàng rào
  được công bố ở `health.map_registry`; Center preflight cũng fail-closed khi
  `ready=false`.
- Robot Agent tiếp tục sở hữu WebSocket, WebRTC và audio. ROS 2 chạy ở process/container khác và trao đổi với agent qua Unix socket, vì vậy Nav2 chết không kéo media xuống.
- `navigation-stack` chỉ chạy một mode: `MAPPING` dùng SLAM Toolbox `online_async`; `NAVIGATION` dùng Map Server, AMCL và Nav2.
- `motion-safety` chạy độc lập ở cả hai mode và phải là publisher duy nhất của `/cmd_vel` cuối.

Trong mode mapping, autosave định kỳ là một transaction local theo generation,
không phải một cặp posegraph ghi đè tại chỗ. Adapter chỉ bắt đầu khi safety
snapshot còn fresh và vận tốc command/odometry đều bằng zero; các service
pause/save/serialize/resume được single-flight với command operator. Một
generation chỉ được công bố qua `latest.json` sau khi `map.yaml`, `map.pgm`,
posegraph/data, terminal pose và SHA-256 đã validate, `fsync` và atomic rename.
Recovery bỏ qua mọi staging/generation chưa commit, kiểm lại toàn bộ checksum,
dùng terminal pose làm vùng tìm kiếm scan-to-map rồi giữ session ở `PAUSED` cho
tới khi operator chọn Resume.

Realtime chỉ gửi occupancy RLE đã giảm mẫu 1–2 Hz, scan giảm mẫu 5 Hz, pose/feedback 5 Hz và health 1–2 Hz. Center giữ snapshot mới nhất để reconnect; không replay queue patch cũ và không gửi raw costmap.

## Build và kiểm thử Center

Từ repository root:

```bash
docker compose build center-backend center-frontend
docker compose run --rm center-backend pytest -q
cd src/apps/center-frontend
npm test
npm run build
```

Khi triển khai migration:

```bash
docker compose up -d postgres redis livekit
docker compose run --rm center-backend alembic upgrade head
docker compose up -d --build center-backend center-frontend
```

Migration `0006_navigation_maps` bổ sung map version, mapping session, cache robot, POI/zones, mission và command receipt; map mẫu cũ vẫn dùng được ở simulator. Production navigation chỉ dùng version `ACTIVE`.

## Build và deploy Pi

### Chế độ tương thích với stack Yahboom có sẵn

Nếu robot đã có micro-ROS Agent/driver đang phục vụ chương trình khác, dự án
không được bật profile `hardware-core` hoặc `ros2-managed-stack`. Chỉ một Agent
được sở hữu `/dev/ttyUSB0`.

Agent legacy ổn định được quản lý riêng trong dự án:

```bash
docker compose -f compose.legacy-hardware.yml up -d --build
```

Trên Pi, graph điều khiển dùng `ROS_DOMAIN_ID=20`. Các container ROS dùng
`network_mode: host` và `ipc: host` để Fast DDS truyền shared memory xuyên
container có cùng UID; graph cũng được phép discovery từ máy ROS 2 khác trong
LAN. Yahboom và Agent cần chạy root cho phần cứng nên dùng UDPv4 trên các giao
diện LAN để giao tiếp với container ứng dụng UID 1000. Chỉ chạy một runtime
Yahboom/Agent trên domain này. Lệnh ROS CLI phải dùng cùng `ROS_DOMAIN_ID`.
Agent mặc định log `-v2`; `-v4` chỉ dùng tạm thời khi chẩn đoán. Giới hạn
virtual memory của Agent là 1,5 GiB: Fast DDS cần vùng địa chỉ lớn để tạo
thread/participant dù RSS thực tế chỉ vài chục MiB, nên không được hạ xuống
512 MiB.

Kernel Pi có thể không mount memory cgroup và bỏ qua `mem_limit`. Vì vậy
entrypoint Yahboom còn đặt giới hạn virtual memory từng process và dừng runtime
nếu tổng RSS vượt 900 MiB. Nếu guard dừng container, không restart-loop trước
khi kiểm tra log `Yahboom memory guard`.

Runtime managed dùng trần virtual memory 1,5 GiB mỗi process. Fast DDS Humble
có thể reserve hơn 1 GiB lúc tạo EKF/TF/robot state dù RSS thực còn thấp; trần
1 GiB sẽ làm `joint_state_publisher` lỗi `Resource temporarily unavailable`.
Container managed cho phép tối đa 512 PID/thread; mức 128 không đủ cho joy,
Fast DDS, EKF, TF và robot state chạy cùng lúc.

Hai Compose Agent dùng chung khóa `/var/lock/rovera-micro-ros/ttyUSB0.lock` và
kiểm tra PID đang giữ serial. Managed Agent còn yêu cầu xác nhận tường minh:

```bash
export ROVERA_MANAGED_HARDWARE_ACK=I_ACCEPT_EXCLUSIVE_SERIAL_OWNERSHIP
docker compose --profile hardware-core up micro-ros-agent
```

Không đặt biến xác nhận trên robot chạy legacy. Managed Agent mặc định
`restart: no`, nên reboot không thể âm thầm tạo Agent thứ hai. Launcher desktop
legacy kiểu `docker run -it --rm` phải được vô hiệu sau khi chuyển sang Compose.

Trong `demo/robot-simulator`:

```bash
cp .env.navigation.example .env.navigation
docker compose -f compose.navigation.yml --env-file .env.navigation --profile navigation build
docker compose build robot-simulator
```

### Hai đường khởi động không thể trộn lẫn

Để giữ nguyên Agent và joystick vendor trong lúc chỉ cần camera/SLAM read-only:

```bash
./scripts/start_pi_coexistence.sh
```

Preflight yêu cầu đúng một Agent đang giữ `/dev/ttyUSB0`, đúng một joystick
vendor và không có service managed cạnh tranh. Nó chỉ start `robot-simulator`
với `MOTION_BACKEND=disabled` cùng `mapping-stack`; mọi lệnh velocity từ Web
được ACK `rejected/MOTION_DISABLED`, không bị drop im lặng. Dừng riêng phần này:

```bash
./scripts/stop_pi_coexistence.sh
```

Để Web điều khiển xe thật, phải chuyển toàn bộ quyền `/cmd_vel` trong một giao
dịch có rollback. Chạy kiểm tra trước (không thay đổi gì):

```bash
./scripts/cutover_managed_motion.sh
```

Sau khi kê bánh và có người giám sát phần cứng:

```bash
export ROVERA_EXCLUSIVE_CMD_VEL_ACK=I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP
./scripts/cutover_managed_motion.sh --apply
```

Cutover giữ nguyên Agent legacy và các topic `/scan`, `/imu`, `/odom_raw`,
`/battery`; container Yahboom thay thế tiếp tục chạy bringup vendor để giữ
`/odom`, `/imu/data`, `/joint_states`, `/robot_description` và TF. Chỉ output
joystick được remap thành `/cmd_vel_joy`. Web dùng `/cmd_vel_web`, Nav2 dùng `/cmd_vel_nav`, và
`rovera_motion_safety` phải là publisher duy nhất của `/cmd_vel`. Image được
build trước khi joystick cũ dừng; hậu kiểm thất bại sẽ tự stop runtime mới và
restart joystick cũ. Script đồng thời đổi tên autostart vendor `uros.desktop`
theo cách khôi phục được, tránh reboot sinh lại publisher `/cmd_vel` trực tiếp.
Không dùng `docker compose up -d` với nhiều profile thay cho các script này và
không chạy thủ công `/home/pi/ros2_humble.sh` khi managed-motion đang hoạt động.

Khi đồng bộ sang Pi, khảo sát remote trước, sao lưu các file trùng tên và chỉ copy các mục sau:

```text
compose.navigation.yml
compose.legacy-hardware.yml
.env.navigation.example
micro-ros-agent-guard/
navigation-stack/
motion-safety/
simulator/client.py
simulator/config.py
simulator/map_cache.py
simulator/navigation_backends.py
compose.yaml
edge.env.example
```

Không copy `.env`, credential, state, map cache hay toàn bộ thư mục một cách mù quáng. Tạo `.env.navigation` trực tiếp trên robot, quyền `0600`. Build image không làm robot chạy. Chỉ start runtime sau khi xử lý xong mọi publisher trực tiếp vào `/cmd_vel` và hoàn tất cổng kiểm tra dưới đây.

## Cổng non-motion bắt buộc

Đầu tiên vô hiệu output motor hoặc kê bánh. Các lệnh sau chỉ đọc trạng thái:

```bash
export ROS_DOMAIN_ID=20
ros2 topic info /cmd_vel -v
ros2 topic hz /scan
ros2 topic hz /odom_raw
ros2 topic hz /imu
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_frame
docker stats --no-stream
vcgencmd measure_temp
```

Điều kiện để tiếp tục:

- `/cmd_vel` chỉ có publisher `rovera_motion_safety`; web/joy/Nav2 lần lượt chỉ publish `/cmd_vel_web`, `/cmd_vel_joy`, `/cmd_vel_nav`.
- `/scan` mới hơn 280 ms và frame `laser_frame`; TF không có publisher trùng.
- Adapter chỉ cho Start Mapping khi đồng thời có `/scan` fresh,
  `odom -> base_footprint` và `base_link -> laser_frame`. Thiếu một cổng sẽ trả
  `SCAN_STALE`, `ODOMETRY_UNAVAILABLE` hoặc `LIDAR_TF_UNAVAILABLE`; Center hạ
  capability mapping và khóa nút Start thay vì tạo phiên lỗi.
- `odom -> base_footprint` chỉ do EKF publish; vendor `/odom_raw` được normalize từ `odom_frame` sang `odom`.
- `map -> odom` chỉ do SLAM Toolbox ở mode mapping hoặc AMCL ở mode navigation publish.
- Nguồn hình học hiện tại là thân xe `0,30 × 0,20 × 0,15 m`, base ở tâm mặt đáy và LiDAR ở giữa mặt trên (`x=0`, `y=0`, `z=0,15 m`). Sensor normalizer chỉ loại endpoint nằm trong footprint vật lý 0,30 × 0,20 m; không còn envelope 0,40 × 0,36 m có thể che vật cản ở ngoài thân xe.
- Live corridor không được dùng phía thiếu dữ liệu để xóa một hard-margin
  violation ở phía đã quan sát. Vi phạm đầu tiên khóa velocity; sau chuỗi xác
  nhận, adapter giữ nguyên destination, ưu tiên một đoạn lùi thẳng chậm tới turn
  bay nếu rear safety + Saved Map swept-footprint cùng cho phép, rồi replan tới
  đích cũ. Nếu không chứng minh được relocation an toàn, robot giữ zero và chạy
  bounded periodic alternative replan; không lùi/quay mù.
- IMU đã được kiểm tra trục/yaw. Hiện orientation bị đánh dấu unavailable và chưa fusion yaw để tránh dùng covariance/axis sai.

Không chạy đồng thời MAPPING và NAVIGATION. Không start motion-safety cạnh stack cũ còn ghi thẳng `/cmd_vel`.

External obstacle interlock chạy trên domain 20 và nằm trong lớp safety cuối:

```bash
# Chặn tiến cho Web/joystick/Nav2
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 1}' -r 10

# Dừng publisher trên bằng Ctrl+C, sau đó mở lại hướng tiến
ROS_DOMAIN_ID=20 ros2 topic pub \
  /rovera/obstacle_directions std_msgs/msg/UInt8 '{data: 0}' -r 10
```

Không chạy đồng thời hai publisher giá trị `1` và `0`. Giá trị `0` chỉ mở khóa
hướng; LaserScan timeout, obstacle hình học, E-stop hoặc watchdog vẫn giữ zero.

## Tạo và activate map

1. Mở **Maps → Tạo map SLAM**, chọn robot online, nhập tên/site/tầng/ghi chú.
2. Bấm **Start Mapping**. Agent yêu cầu stack ở mode `MAPPING`; manual control đi qua motion-safety.
3. Theo dõi occupancy, pose/hướng, scan đỏ, trail, pin, kết nối và `/scan` health. Nút **DỪNG** gửi stop ngay. UI luôn ghi cliff sensor chưa khả dụng.
4. **Pause/Resume** tạm dừng/tiếp tục nhận scan của SLAM. **Save Draft** giữ session để làm tiếp; **Finish & Save** kết thúc version. Mất mạng không xóa session: bundle và marker retry ở volume robot, pose graph auto-save định kỳ.
5. Robot ghi map/pose graph/preview/metadata vào staging, checksum từng file,
   atomic rename và upload bundle. Center chỉ nhận flat regular-file archive,
   giới hạn cả kích thước nén/giải nén/member/tỷ lệ nén/pixel, parse YAML/JSON
   chặt, decode ảnh và bắt buộc `map.yaml`–image–metadata–artifact hash nhất
   quán trước khi lưu.
6. Mở chi tiết map, kiểm resolution/origin/dimensions/checksum rồi **Activate**. Lifecycle là `DRAFT → VALIDATING → ACTIVE → ARCHIVED`; không ghi đè version active.

Bundle có `map.yaml`, `map.pgm`, pose graph, `preview.png`, `metadata.json`; POI/keepout/speed zones được lưu riêng theo version trong registry.

## Robot khác tải map và điều hướng

1. Trong màn Control hiện tại, chọn version ở dropdown **Map ACTIVE**. Camera WebRTC vẫn giữ nguyên component lớn bên trái.
2. Center gửi map ID/version/checksum. Agent chỉ tải nếu cache không khớp, verify rồi gọi Map Server. UI hiển thị `LOADING_MAP` và sau đó `LOCALIZING`.
3. Khi chưa localized, click hoặc kéo trên map để chọn vị trí/hướng hiện tại, rồi **Xác nhận vị trí ban đầu**. Đây là `/initialpose`, không phải lệnh chuyển động.
4. Chọn POI hoặc click/kéo goal. Center gọi `ComputePathToPose`; Canvas hiển thị global path thật với resolution, origin yaw và đảo trục Y đúng chuẩn ROS.
5. **Bắt đầu** chỉ bật khi connected, map/version đúng, localized, Nav2 và safety healthy, scan fresh, không estop/collision, pin ≥15%, có lease và plan hợp lệ.
6. Start tạo `NavigateToPose`. Pause thực hiện cancel có lưu goal; Resume chỉ chạy khi người dùng bấm rõ. Cancel kết thúc mission. Manual input luôn hủy goal và không auto-resume.

Các state có thẩm quyền từ robot: `LOADING_MAP`, `LOCALIZING`, `READY`, `PLANNING`, `NAVIGATING`, `PAUSED`, `BLOCKED`, `ARRIVED`, `CANCELED`, `FAULT`. Mỗi command có `request_id`, expected state và ACK accepted/rejected; `CommandReceipt` bảo đảm retry idempotent.

## Kiểm tra Nav2 và safety

Ở chế độ motor vô hiệu hóa:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 action list | grep -E 'compute_path_to_pose|navigate_to_pose'
ros2 topic echo /safety/health --once
ros2 topic info /cmd_vel -v
```

Plan-only được phép sau khi TF/map/localization hợp lệ; không gửi NavigateToPose. Motion test cần một xác nhận riêng, khu vực trống, người cạnh hardware E-stop và tốc độ ban đầu tối đa 0,10 m/s.

Safety dùng khoảng cách từ từng điểm scan tới footprint chữ nhật, không dùng trung bình sector. Stop distance là `v*latency + v²/(2*braking_acceleration) + 0,10 m`; slow zone xa hơn 0,20 m. Scan timeout 280 ms, command timeout, empty scan, estop/cliff/bumper/range fault đều phát zero ngay. Sau khi sạch phải chờ 400 ms, rồi velocity smoother ramp lại.

## Giới hạn cảm biến và mở rộng

LiDAR 2D không thấy vật nằm ngoài mặt phẳng quét, vật quá thấp, dây mảnh hoặc mép cầu thang. Hệ thống hiện **không có chống rơi cầu thang**. Không được suy diễn `safety=HEALTHY` thành an toàn cầu thang.

Node đã có input `/safety/cliff`, `/safety/bumper`, `/safety/range`, directional bitmask và timeout. Khi bổ sung cảm biến, tạo driver riêng phát heartbeat/value chuẩn, thêm source/config timeout, viết test synthetic mất heartbeat/hướng rồi mới cho phép UI báo available. Depth/Range nên được chuyển thành điểm/zone cục bộ; hardware E-stop vẫn có ưu tiên cao nhất.

## Troubleshooting và rollback

- `SCAN_STALE`: kiểm `ROS_DOMAIN_ID=20`, QoS, timestamp, frame và topic `/scan`; output vẫn zero cho tới khi scan ổn định.
- `NOT_LOCALIZED`: kiểm TF `map→odom→base_footprint`, map/version và đặt lại initial pose.
- `MAP_LOAD_FAILED`/checksum: xóa riêng cache version lỗi sau khi đã sao lưu; agent sẽ tải lại vào temporary và verify. Không sửa bundle active tại chỗ.
- `NO_PATH`: xem footprint/costmap/inflation và goal có nằm trên ô free; không thay bằng route Manhattan.
- Adapter socket unavailable: kiểm volume `ROBOT_STATE_DIR` được mount giống nhau và healthcheck navigation; agent/WebRTC vẫn phải chạy.
- CPU/nhiệt cao: ghi `docker stats`, nhiệt, WebRTC bitrate trước/sau; không tự giảm video. Giảm tần số publish costmap/debug trước. Xóa/rotate log và archive map cũ để tránh đầy đĩa.
- Camera reconnect: xem riêng log agent/LiveKit. Navigation không được restart agent hoặc chạm pipeline media.

Rollback an toàn:

1. Không start/recreate runtime mới nếu kiểm tra publisher thất bại.
2. Với lỗi trong cutover, để script tự rollback. Nếu chỉ đang coexistence, chạy
   `scripts/stop_pi_coexistence.sh`; không stop Agent legacy.
3. Khôi phục các file agent/compose từ bản backup timestamp và recreate riêng service agent nếu cần.
4. Giữ `MOTION_BACKEND=simulator`, `NAVIGATION_BACKEND=simulator` cho tới khi stack ROS đã được xác nhận lại.
5. Không dùng `compose down -v`; lệnh đó có thể xóa state/map.

Mọi lần bật production cần ghi baseline và after: container CPU/RAM, nhiệt, disk, `/scan`/odom/IMU rate, controller deadline và tình trạng WebRTC. Việc build/deploy thành công không phải là quyền cho robot di chuyển.

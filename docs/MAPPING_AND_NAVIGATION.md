# Mapping và Navigation

Tài liệu này là nguồn vận hành chính cho subsystem bản đồ. Hai mode loại trừ nhau và camera/LiveKit luôn nằm trong màn **Control**.

## Kiến trúc

```text
Mapping:    Manual Control -> motion-safety -> SLAM Toolbox -> save local Pi -> Center Registry
Navigation: Saved Map -> map_server -> AMCL -> Nav2 -> motion-safety -> /cmd_vel
```

Pi chạy ROS 2 Humble, LiDAR, odometry, TF, một trong hai stack SLAM/Nav2, motion-safety và edge client. Pi không chạy RViz/X11/VNC. Laptop kỹ thuật chỉ tham gia DDS để subscribe và render.

Web không nhận raw `/scan`, live `/map` hoặc full costmap. Saved Map được tải một lần khi map/version thay đổi. Telemetry realtime chỉ gồm pose 5 Hz, trạng thái, path khi revision đổi và tối đa 600 lethal cell động. STOP/manual control vẫn đi trên control WebSocket riêng.

## Mapping

Trong màn Control chọn **Tạo bản đồ**. Camera và Manual Control không bị unmount.

1. Kiểm tra robot online, LiDAR, odometry, TF, SLAM và motion-safety.
2. **Start Mapping** hủy goal Nav2 đang chạy, gửi zero velocity rồi chuyển supervisor sang Mapping.
3. Điều khiển tay; xem `/map`, `/scan`, TF và `/odom` bằng RViz trên laptop.
4. **Stop** đưa session về `MAPPING_STOPPED_UNSAVED`; dữ liệu chưa bị xóa.
5. **Save** tạo và xác minh `map.yaml`, ảnh occupancy, `metadata.json`, preview và cặp pose-graph. Pi lưu local trước, rồi upload nền.
6. **Discard** bỏ session chưa lưu, không ảnh hưởng Saved Map cũ.

State chuẩn: `MAPPING_STARTING -> MAPPING_RUNNING -> MAPPING_STOPPED_UNSAVED -> MAPPING_SAVING -> FINISHED`; lỗi là `MAPPING_ERROR`.

Nếu Center chưa truy cập được, Save local vẫn thành công và registry Pi ghi `SYNC_PENDING`. Marker upload tồn tại qua restart và retry mỗi 10 giây. Center trả checksum; edge chỉ ghi `SYNCED` khi checksum trả về trùng bundle local.

**Continue Mapping** tải version có đủ `.posegraph` + `.data`, deserialize SLAM, tạo version mới và không ghi đè version cũ. Trong mode này không chạy autonomous navigation.

## Activate Saved Map và tự định vị

Tại **Bản đồ hành trình**, chọn map/version rồi bấm **Kích hoạt**. Edge tải bundle vào thư mục tạm, kiểm tra SHA-256 bundle, identity metadata và SHA-256 từng artifact, chặn path traversal, giải nén và atomic rename. Adapter đọc chính xác toàn bộ ảnh occupancy, gọi `map_server/load_map`, xác minh `/map`, rồi mới lưu active `(map_id, version)`.

Localization không hỏi tọa độ ban đầu theo mặc định:

1. `LOCALIZING_LAST_POSE`: thử pose gần nhất đúng map/version, có tuổi tối đa và covariance.
2. AMCL + LiDAR + TF xác minh; một pose publish đơn lẻ không đủ. Confidence kết hợp covariance và chuỗi pose ổn định.
3. Nếu pose cũ không hội tụ: `LOCALIZING_GLOBAL` qua `/reinitialize_global_localization`.
4. Khi cần thêm góc nhìn: `LOCALIZING_ROTATING` với tốc độ mặc định 20°/s, timeout 45 s, tối đa 360°. Velocity đi qua `/cmd_vel_nav -> twist_mux -> velocity smoother -> motion-safety`; E-stop, scan stale, obstacle/directional mask hoặc manual control dừng xoay ngay.
5. Chỉ `READY` mới cho chọn goal. Nếu thất bại, UI mới hiện **Thử lại** và **Chỉ vị trí robot gần đúng**; xác nhận fallback publish `/initialpose` với covariance rộng để AMCL refine.

Last Known Pose lưu 5 giây/lần gồm map/version, x/y/yaw, covariance và timestamp. Khi confidence tụt trong lúc đi, adapter cancel Nav2, phát zero qua safety chain, xóa path và global-localize lại; không tự tiếp tục goal cũ.

## Navigation trong Control

Camera luôn chiếm cột lớn. Mini-map bên phải render Saved Map với nearest-neighbor, footprint theo mét, heading, goal, `/plan` thật và lethal cells từ local costmap. **Expand** mở modal trong cùng Dashboard; LiveKit transport không bị remount.

Click map chỉ chọn goal/heading và gọi `ComputePathToPose`. Nút **Đi đến đây** mới gửi `NavigateToPose`. Edge kiểm tra map/version active, localization `READY`, bounds theo origin có rotation, unknown/occupied, clearance footprint và obstacle động. Frontend không tự tính hoặc smooth path; `/plan` mới thay hoàn toàn path cũ.

Manual Control gọi takeover: cancel Nav2 trước và không auto-resume. **Dừng điều hướng** cancel action và gửi stop. E-stop tiếp tục do motion-safety xử lý ở lớp cuối.

## State và health

Navigation dùng `MAP_LOADING`, `LOCALIZATION_INITIALIZING`, `LOCALIZING_LAST_POSE`, `LOCALIZING_GLOBAL`, `LOCALIZING_ROTATING`, `LOCALIZATION_FAILED`, `READY`, `PLANNING`, `NAVIGATING`, `BLOCKED`, `RECOVERY`, `LOCALIZATION_LOST`, `SUCCEEDED`, `CANCELED`, `FAILED`.

`GET /api/navigation/health/{robot_id}` trả mode, mapping health, navigation map/version/localization/Nav2 và Map Registry. `GET /api/maps/registry/health` tổng hợp local/sync/delete theo robot.

## Kiểm tra trên Pi khi triển khai

Pi đang tắt nên các bước hardware chưa thể chạy offline. Khi Pi bật:

```bash
cd demo/robot-simulator
docker compose -f compose.yaml -f compose.navigation.yml up -d --build
docker compose -f compose.yaml -f compose.navigation.yml ps
ROS_DOMAIN_ID=20 ros2 topic list
ROS_DOMAIN_ID=20 ros2 action list
```

Chạy lần lượt một phiên Mapping/Stop/Save, ngắt mạng Center để thấy `SYNC_PENDING`, kết nối lại để thấy `SYNCED`; sau đó Activate, quan sát auto localization, gửi goal, manual takeover, E-stop, xóa active map và kiểm tra `NO_ACTIVE_MAP`. Theo dõi CPU/RAM/process qua `docker stats` và `docker compose ... ps` qua ít nhất 10 lần chuyển mode.

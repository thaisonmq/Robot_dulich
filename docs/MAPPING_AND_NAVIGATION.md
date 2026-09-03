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

State chuẩn khi tạo mới: `MAPPING_STARTING -> MAPPING_RUNNING -> MAPPING_STOPPED_UNSAVED -> MAPPING_SAVING -> FINISHED`. Continue Mapping có thêm `MAPPING_LOCALIZING` trước `MAPPING_RUNNING`; lỗi là `MAPPING_ERROR`.

Nếu Center chưa truy cập được, Save local vẫn thành công và registry Pi ghi `SYNC_PENDING`. Marker upload tồn tại qua restart và retry mỗi 10 giây. Center trả checksum; edge chỉ ghi `SYNCED` khi checksum trả về trùng bundle local.

**Continue Mapping** tải version có đủ `.posegraph` + `.data`. Vì robot có thể đã được di chuyển sau phiên trước, người vận hành chỉ cần chọn vùng và hướng gần đúng trên đúng version map; đây là search hint, không phải pose được tin cậy. Center từ chối hint thiếu, không hữu hạn hoặc nằm ngoài map. Trước khi deserialize, adapter so scan mới với occupancy map trong bán kính 1,25 m quanh hint, tìm đủ 360° và từ chối khi vị trí/hướng thứ hai còn cạnh tranh. Pose tốt nhất mới được đưa vào `START_AT_GIVEN_POSE`; chỉ một probe scan đi vào SLAM, sau đó `/slam_toolbox/pose`, covariance và ba scan–map geometry check liên tiếp phải cùng đạt ngưỡng trước khi mở luồng mapping bình thường. Nếu không khớp/hết thời gian, adapter nạp lại pose-graph nguồn để bỏ probe scan và trả `MAPPING_ERROR`; version cũ luôn bất biến. Trong mode này không chạy autonomous navigation.

## Activate Saved Map và tự định vị

Tại **Bản đồ hành trình**, chọn map/version rồi bấm **Kích hoạt**. Edge tải bundle vào thư mục tạm, kiểm tra SHA-256 bundle, identity metadata và SHA-256 từng artifact, chặn path traversal, giải nén và atomic rename. Adapter đọc chính xác toàn bộ ảnh occupancy, gọi `map_server/load_map`, xác minh `/map`, rồi mới lưu active `(map_id, version)`.

Localization không hỏi tọa độ ban đầu theo mặc định:

1. `LOCALIZING_LAST_POSE`: dùng pose gần nhất đúng map/version làm gợi ý có covariance rộng, không coi là vị trí đã xác nhận vì robot có thể đã được di chuyển bằng tay.
2. AMCL + LiDAR + TF xác minh bằng dữ liệu mới; một pose publish đơn lẻ không đủ. Confidence kết hợp covariance và chuỗi pose ổn định, cùng một ngưỡng xác nhận cho cả pose cũ và global localization.
3. Pose navigation gần nhất chỉ được lưu sau 30 giây đủ bằng chứng. Khi khởi động
   lại, pose này được xác minh cục bộ với covariance/hướng đã kiểm chứng để có thể
   `READY` nhanh từ scan mới; nếu không khớp thì mới chuyển sang
   `LOCALIZING_GLOBAL` qua `/reinitialize_global_localization`.
4. Mặc định mọi lần map load, mở Control, mất định vị và nhập vị trí gần đúng đều định vị **thụ động**, chỉ dùng LiDAR/TF tại chỗ và không phát velocity. Khi đã phải chuyển sang global search, UI hiện ngay **Cho phép xoay để định vị** và **Chỉ vị trí robot gần đúng**, không bắt người vận hành chờ hết timeout. `LOCALIZING_ROTATING` chỉ được phép khi lệnh riêng truyền `allow_rotation=true`; phép kiểm quay dùng footprint thân xe 0,30 × 0,20 m cùng rotation margin riêng và vẫn chặn vật thể chạm sát cạnh thân. LiDAR ở giữa mặt trên; chỉ endpoint nằm bên trong chính footprint vật lý mới được loại như self-return trước AMCL/costmap, mọi tia bên ngoài vẫn được giữ. Velocity vẫn đi qua `/cmd_vel_nav -> twist_mux -> velocity smoother -> motion-safety`, và E-stop, scan stale, obstacle/directional mask hoặc manual control dừng xoay ngay.
5. Mỗi phiên Control mới yêu cầu robot xác minh lại pose hiện tại và chỉ `READY` mới cho chọn goal. Nếu thất bại, UI mới hiện **Thử lại** và **Chỉ vị trí robot gần đúng**; người dùng chỉ chọn vùng vị trí, không cần biết hướng robot. Điểm này là tâm vùng tìm kiếm `/initialpose` cục bộ với phương sai vị trí `0.36 m²` và phương sai hướng phủ đủ `360°`; LiDAR + AMCL tự tìm cả vị trí lẫn hướng. Pha này có tối đa 20 giây trước khi chuyển sang global localization để không xóa một particle cloud đang hội tụ ở mốc timeout ngắn của pose đã lưu. UI dùng ký hiệu vùng tìm kiếm, xóa ngay gợi ý sau khi gửi và chỉ hiện marker robot sau khi AMCL hội tụ.
6. Khi pose đã `READY`, nút **Đi đến đây** tái sử dụng particle cloud đang được LiDAR/AMCL cập nhật và chỉ tính lại đường từ pose mới nhất; không gọi global-localize lần hai. Global search chỉ chạy khi runtime thực sự chưa `READY`. Ngưỡng vào `READY` vẫn yêu cầu pose đứng yên, scan-map và confidence cao; sau khi quét đủ góc, state machine dừng thân xe, chờ hết quán tính rồi mới thu một cửa sổ mẫu tĩnh mới (`LOCALIZING_SETTLING`). Ngưỡng duy trì dùng hysteresis thấp hơn cùng AMCL/scan/TF/sensor-time còn mới để chuyển động tay hoặc bước lập đường không tự làm mất pose. Lệnh quét lặp trong lúc `LOCALIZING_GLOBAL`/`LOCALIZING_ROTATING`/`LOCALIZING_SETTLING` là idempotent và không reset tiến trình AMCL.

Last Known Pose lưu 5 giây/lần gồm map/version, x/y/yaw, covariance và timestamp. Khi confidence tụt trong lúc đi, adapter cancel Nav2, phát zero qua safety chain, xóa path và global-localize lại; không tự tiếp tục goal cũ.

## Navigation trong Control

Camera luôn chiếm cột lớn. Mini-map bên phải render Saved Map với nearest-neighbor, footprint theo mét, heading, goal, `/plan` thật và lethal cells từ local costmap. **Expand** mở modal trong cùng Dashboard; LiveKit transport không bị remount.

Click map chọn goal và gọi `ComputePathToPose`; click đơn tự đặt heading theo hướng từ robot tới goal, còn thao tác kéo mới yêu cầu heading cuối cụ thể. Điều này tránh đường vòng chỉ để kết thúc ở `yaw=0`. Nút **Đi đến đây** mới gửi `NavigateToPose`. Edge kiểm tra map/version active, localization `READY`, bounds theo origin có rotation, unknown/occupied, clearance footprint và obstacle động. Frontend không tự tính hoặc smooth path; State Lattice tạo path cost-aware đã làm mượt và `/plan` mới thay hoàn toàn path cũ.

Trong khi đi, Nav2 tính lại global path 1 Hz trên costmap động. Trường inflation rộng khiến planner ưu tiên hành lang thoáng và đi vòng vật cản nếu tồn tại lối an toàn. Vật cản tạm thời được chờ/lập đường lại và recovery theo chuỗi hữu hạn của Nav2; khi chuỗi này hết mà vẫn không có lối, action dừng an toàn, adapter xóa path/goal có thể resume và báo `BLOCKED` thay vì tự chạy lại vô hạn hoặc báo `FAILED` chung chung.

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

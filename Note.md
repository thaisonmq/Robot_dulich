# Hiện trạng chức năng ROVERA

> Rà soát theo mã nguồn hiện tại ngày 03/08/2026. Mục `[x]` là đã có luồng xử
> lý trong mã; các giới hạn của bản demo được ghi rõ, không xem là chức năng
> production hoàn chỉnh.

## Chức năng đã làm được

### 1. Tài khoản và phân quyền

- [x] Đăng ký tài khoản khách bằng username, email và mật khẩu.
- [x] Đăng nhập bằng username hoặc email; mật khẩu được băm PBKDF2.
- [x] Đăng nhập Google OAuth khi khai báo đủ cấu hình trong `.env`.
- [x] JWT access token, tự đưa người dùng về màn đăng nhập khi API trả `401`.
- [x] Xem và sửa họ tên, đổi mật khẩu cá nhân.
- [x] Ba vai trò `admin`, `operator`, `guest` và kiểm tra quyền ở cả API lẫn UI.
- [x] Admin tạo, phân quyền, khóa/mở và đặt lại mật khẩu cho operator/guest.
- [x] Operator chỉ được tạo và quản lý tài khoản guest.
- [x] Lưu audit cho các thao tác đăng nhập, đăng ký, sửa tài khoản và đổi mật
  khẩu; đã có API đọc lịch sử của một tài khoản.
- [x] Giao diện đa ngôn ngữ tĩnh: Việt, Anh, Trung, Hàn, Nhật, Thái, Pháp, Đức,
  Tây Ban Nha và Nga.

### 2. Quản lý robot

- [x] Danh sách robot có tìm kiếm, lọc trạng thái, phân trang và thống kê.
- [x] Thêm nhanh robot bằng IP/hostname, username và mật khẩu quản trị cục bộ.
- [x] Thêm robot bằng enrollment token một lần.
- [x] Sửa tên, khu vực, map, địa chỉ quản lý, username và mật khẩu robot.
- [x] Khóa/mở robot; chỉ cho xóa robot khi offline và không có phiên điều khiển.
- [x] Robot tự claim hồ sơ chờ, nhận device credential và lưu state riêng theo
  fingerprint của máy.
- [x] Device credential chỉ lưu SHA-256 ở Center; mật khẩu quản trị robot chỉ
  lưu PBKDF2 hash.
- [x] Robot đổi device credential lấy JWT ngắn hạn trước khi mở WebSocket và
  xin token LiveKit.
- [x] Theo dõi online/offline bằng WebSocket, heartbeat, `last_seen_at` và lưu
  lịch sử kết nối/disconnect trong PostgreSQL.
- [x] Edge agent tự kết nối lại Center, tải lại state sau khi restart và tự
  claim lại khi state không thuộc đúng thiết bị.

### 3. Phiên điều khiển và realtime

- [x] Mỗi robot chỉ có một phiên điều khiển active tại một thời điểm.
- [x] Khóa quyền điều khiển theo browser tab; tab khác không thể chiếm cùng
  session, tab cũ được phép reconnect bằng `client_id` của chính nó.
- [x] Lưu bản ghi phiên trong PostgreSQL, thời điểm bắt đầu/kết thúc, người kết
  thúc và lý do kết thúc.
- [x] Grace period reconnect cho browser hoặc robot; tự giải phóng phiên khi
  quá thời hạn hoặc browser tạo phiên nhưng không mở control channel.
- [x] Admin/operator xem danh sách phiên của guest, theo dõi ở chế độ chỉ xem
  và có thể cưỡng bức kết thúc phiên.
- [x] WebSocket riêng cho control và telemetry; kiểm tra session, sequence,
  TTL, loại message và kích thước message.
- [x] Điều khiển bằng phím mũi tên hoặc WASD, nút trên màn hình và touch/pointer.
- [x] Hỗ trợ tiến/lùi kết hợp quay, tăng tốc mềm 220 ms, phát lệnh 20 Hz và TTL
  300 ms.
- [x] Gửi STOP khi nhả phím, mất focus, ẩn tab, mất kết nối hoặc rời Dashboard;
  có nút dừng khẩn cấp phần mềm.
- [x] Edge loại lệnh trùng/hết hạn và có motion watchdog để không tiếp tục chạy
  khi mất lệnh mới.
- [x] Có motion simulator và ROS 2 control bridge phát `Twist`, ưu tiên joystick
  vật lý, giới hạn vận tốc và phát zero khi watchdog hết hạn.

### 4. Video, âm thanh và cấu hình thiết bị

- [x] LiveKit room riêng theo robot, token giới hạn theo vai trò publisher,
  controller, spectator và preview.
- [x] Xem video và nghe âm thanh từ robot; browser có thể bật microphone để
  đàm thoại hai chiều, tắt/bật loa.
- [x] Tự reconnect LiveKit, giữ frame tốt gần nhất khi video gián đoạn, theo dõi
  frame bị đứng và điều chỉnh buffer phát thích nghi.
- [x] Xem trước camera ở màn cấu hình bằng lease có heartbeat và thời hạn.
- [x] Liệt kê, quét và kiểm tra camera, microphone, loa thật trên máy edge; loại
  các nguồn không mở được hoặc không có tín hiệu.
- [x] Hỗ trợ test pattern, file, RTSP và USB camera.
- [x] Hỗ trợ audio im lặng, file, ALSA/PulseAudio/PipeWire và thiết bị Bluetooth
  HSP/HFP.
- [x] Chuyển camera đang phát ngay trong phiên điều khiển.
- [x] Cấu hình nguồn video/audio/loa từ Center, kiểm tra kết nối và kiểm tra từng
  nguồn media trước khi lưu.
- [x] Edge lưu cấu hình vào device state; không trả credential RTSP đầy đủ về
  trình duyệt.
- [x] Pipeline H.264 passthrough khi nguồn phù hợp; có software encoder và tự
  chọn VA-API/NVENC/RKMPP/V4L2M2M khi máy hỗ trợ.

### 5. Bản đồ và điều hướng bản demo

- [x] Hiển thị map mẫu, vị trí/hướng robot realtime và các điểm đến mẫu.
- [x] Xem trước tuyến, khoảng cách và thời gian dự kiến.
- [x] Gửi goal, hủy goal và nhận trạng thái `moving`, `arrived`, `cancelled`,
  `failed`.
- [x] Simulator có thể tự chạy qua các waypoint và cập nhật pose lên giao diện.

Lưu ý: tuyến hiện chỉ là đường Manhattan ba điểm trên map mẫu, chưa tránh vật
cản và chưa dùng Nav2. Edge đang chạy `MOTION_BACKEND=ros2` sẽ từ chối
`navigation.goal`; điều hướng robot thật chưa hoàn thành.

### 6. Docker và vận hành cơ bản

- [x] Docker Compose cho frontend, backend, PostgreSQL, Redis, LiveKit, coturn
  và simulator tùy chọn.
- [x] Backend tự chạy Alembic migration khi container khởi động.
- [x] Healthcheck, restart policy và volume giữ dữ liệu PostgreSQL/Redis.
- [x] Docker image edge chạy AMD64/ARM64, map camera/audio/DRM, giữ state ngoài
  container và giới hạn kích thước log.
- [x] Compose profile cho ROS 2 control bridge, micro-ROS agent và Yahboom
  joystick stack.

## Phần đã có khung nhưng chưa hoàn thiện

- [ ] Redis đã chạy trong Docker nhưng backend chưa sử dụng. `ConnectionHub`,
  robot presence, session lock, route, camera inventory và media lease vẫn nằm
  trong RAM của một backend process.
- [ ] Bảng `NavigationRoute`, `CommandLog` và `RobotEvent` đã có trong database
  nhưng chưa được ghi/đọc trong luồng ứng dụng.
- [ ] Map và destination có bảng database nhưng API vẫn trả hằng số
  `MAP-001`/`DEST-*` từ mã nguồn.
- [ ] Package chuyển đổi tọa độ map đã có và có unit test, nhưng `MapPanel` vẫn
  hard-code kích thước thế giới `16 x 10 m` thay vì dùng metadata của map.
- [ ] API lịch sử tài khoản đã có nhưng frontend chưa có màn hình hiển thị.
- [x] Coturn đã được nối vào LiveKit/browser bằng credential HMAC ngắn hạn,
  hỗ trợ TURN relay qua UDP và TCP.
- [ ] Giao diện hội thoại đã có lựa chọn ngôn ngữ nhưng dịch giọng nói realtime
  đang bị khóa bằng `translationEnabled = false`.
- [ ] Battery luôn là `78%`; RTT và packet loss của simulator là số ngẫu nhiên,
  chưa phải telemetry phần cứng/mạng thật.
- [ ] Profile simulator ở Compose gốc cần seed robot demo thủ công trên database
  mới; `SEED_DEMO_ROBOT` chưa được truyền vào container backend.
- [ ] Tài liệu có thiết kế `GamepadInputAdapter` nhưng mã frontend chưa triển
  khai Web Gamepad API.

## Chức năng cần làm thêm

### P0 — Cần trước khi vận hành robot thật/production

1. **Đưa realtime state và session lock sang Redis**
   - Dùng Redis lease/atomic lock cho robot presence, quyền điều khiển, media
     lease và timeout session.
   - Đồng bộ WebSocket/pub-sub giữa nhiều backend instance.
   - Khôi phục đúng session sau restart, không cho hai instance cùng cấp quyền
     điều khiển một robot.

2. **Hoàn thiện an toàn chuyển động trên robot thật**
   - Tích hợp physical emergency stop, bumper/LiDAR collision stop và giới hạn
     tốc độ theo trạng thái robot.
   - Kiểm thử fail-safe khi mất Center, mất Wi-Fi, process treo và ROS node
     restart.
   - Không tự gỡ software e-stop chỉ bằng một lệnh velocity mới nếu chưa có cơ
     chế re-arm rõ ràng.

3. **Tích hợp điều hướng ROS 2/Nav2 thật**
   - Chuyển goal/cancel sang Nav2 action server.
   - Trả planning/progress/result thật về Center.
   - Dùng occupancy grid, costmap, restricted zone và xử lý lỗi/timeout.

4. **Hoàn thiện bảo mật triển khai**
   - HTTPS/WSS qua reverse proxy, secret mạnh và xoay vòng secret/credential.
   - Rate limit và chống brute force cho login, Google exchange, robot claim,
     enrollment và token endpoints.
   - Thêm refresh/revoke/logout token; ngắt ngay WebSocket/LiveKit session khi
     tài khoản bị khóa hoặc đổi mật khẩu.
   - Không tự seed tài khoản demo ở môi trường production.

5. **Hoàn thiện TURN/NAT**
   - Đã cấp credential TURN ngắn hạn và khai báo coturn trong LiveKit/WebRTC.
   - Kiểm thử qua NAT, firewall, mạng di động và mạng doanh nghiệp.

6. **Sửa luồng Docker demo mới hoàn toàn**
   - Truyền `SEED_DEMO_ROBOT` vào backend hoặc tạo init job rõ ràng để
     `docker compose --profile demo up` chạy được trên volume mới mà không cần
     lệnh seed thủ công.

### P1 — Hoàn thiện sản phẩm

1. **Quản lý map và điểm đến**
   - CRUD/upload map, calibration resolution/origin, quản lý điểm đến và vùng
     cấm từ UI.
   - Dùng metadata map thay cho kích thước hard-code; lưu route vào database.

2. **Telemetry thật và cảnh báo**
   - Đọc pin, nhiệt độ, CPU, camera/audio, chất lượng Wi-Fi, RTT và packet loss
     thật từ edge/ROS.
   - Cảnh báo pin yếu, robot offline, media lỗi và watchdog kích hoạt.

3. **Audit và lịch sử vận hành**
   - Ghi command quan trọng, robot event, navigation result và toàn bộ vòng đời
     session vào các bảng đã có.
   - Thêm màn tra cứu/lọc/xuất báo cáo; đưa API lịch sử tài khoản lên frontend.

4. **Hoàn thiện tài khoản**
   - Xác minh email cho đăng ký mật khẩu.
   - Quên mật khẩu/đặt lại mật khẩu qua email.
   - Chính sách mật khẩu, khóa tạm sau nhiều lần đăng nhập sai và quản lý phiên
     đăng nhập đang hoạt động.

5. **Dịch hội thoại realtime**
   - Nhận dạng giọng nói, dịch, tổng hợp giọng nói và hiển thị transcript.
   - Cho phép bật/tắt theo từng chiều, xử lý độ trễ và quyền riêng tư.

6. **Gamepad web**
   - Cài `GamepadInputAdapter`, dead-zone, trục analog, nút emergency stop và
     cleanup khi gamepad mất kết nối/tab bị ẩn.

7. **Quan sát và khôi phục hệ thống**
   - Metrics, dashboard, tracing, structured log tập trung và cảnh báo lỗi.
   - Backup/restore PostgreSQL, chính sách retention và kiểm thử nâng cấp
     migration.

## Trạng thái kiểm thử khi rà soát

| Khu vực | Kết quả |
| --- | --- |
| Frontend Vitest | 9 files, 35 tests passed |
| Backend Pytest | 28 tests passed |
| Robot edge Pytest | 97 tests passed |
| TypeScript type-check | Passed |
| Playwright E2E | Có 2 kịch bản; chưa chạy trong lượt rà soát vì cần toàn bộ Docker stack |

Cần bổ sung test multi-instance/Redis lock, TURN/NAT, load test WebSocket,
Google OAuth thật, trình duyệt/mobile khác nhau và hardware-in-the-loop trên
Orange Pi/Yahboom trước khi đánh dấu production-ready.

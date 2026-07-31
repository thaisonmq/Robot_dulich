# ROVERA — Robot Telepresence Demo

Hệ thống trình diễn telepresence có center web trong [`src`](./src) và robot
simulator độc lập trong [`demo`](./demo). Simulator và robot thật dùng cùng
WebSocket/WebRTC contract; frontend không có nhánh logic dành riêng cho
simulator.

## Chạy bằng một lệnh

Yêu cầu Docker Engine + Docker Compose:

```bash
docker compose up -d
```

Mở <http://localhost:8080>.

- Email: `demo@rovera.local`
- Mật khẩu: `demo123`
- Admin cao nhất: `admin` / `admin123`
- OpenAPI: <http://localhost:8080/docs>
- Backend dành cho robot/LAN: `http://<CENTER_IP>:8888`
- LiveKit: `ws://localhost:7880`

Compose tự build lại source backend/frontend, chạy migration database và chờ
các dependency sẵn sàng. File `.env` là tùy chọn; khi không có file này hệ
thống dùng cấu hình mặc định dành cho môi trường demo.

Simulator không cần thiết để chạy backend/frontend. Khi muốn chạy thêm robot
mô phỏng, dùng:

```bash
docker compose --profile demo up -d
```

Robot `ROBOT-001` chuyển sang online sau khi được seed/enroll và container
simulator kết nối.
Dashboard vẫn điều khiển/map được nếu LiveKit hoặc nguồn media tạm lỗi. Khi
không có nguồn thật, màn video hiển thị rõ trạng thái chưa có tín hiệu.

## Tài khoản và phân quyền

Tài khoản được lưu trong PostgreSQL; mật khẩu chỉ lưu dưới dạng PBKDF2 hash.
Hệ thống có ba vai trò:

- `admin`: toàn quyền, tạo/khoá tài khoản và phân quyền nhân viên;
- `operator`: quản lý, cấu hình và vận hành robot;
- `guest`: vai trò mặc định khi tự đăng ký; được kết nối/điều khiển robot nhưng không xem hay sửa cấu hình kỹ thuật.
- `operator`: quản lý robot và chỉ quản lý các tài khoản `guest`; có thể xem cùng hoặc cưỡng bức kết thúc phiên điều khiển của khách.

Menu avatar mở trang hồ sơ cho mọi tài khoản; admin có thêm màn **Quản lý tài
khoản**. Tài khoản admin khởi tạo mặc định là `admin` / `admin123` và được đánh
dấu yêu cầu đổi mật khẩu. Có thể đổi credential khởi tạo bằng các biến
`BOOTSTRAP_ADMIN_*`.

Để bật đăng nhập/đăng ký Google, tạo OAuth 2.0 Web Client trong Google Cloud,
khai báo redirect URI `http://<CENTER_HOST>:8080/api/auth/google/callback`, rồi
đặt `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `GOOGLE_REDIRECT_URI` và
`FRONTEND_PUBLIC_URL` trong `.env`. Khi chưa cấu hình, nút Google vẫn hiển thị
nhưng ở trạng thái vô hiệu hoá.

## Cấu trúc

```text
src/
  apps/center-frontend/   React + TypeScript strict + Zustand + Query + LiveKit
  apps/center-backend/    FastAPI + gateway + sessions + maps/navigation
  packages/contracts/    Realtime contract v1.0
  packages/map-utils/    World/pixel/map conversion
  infrastructure/        PostgreSQL, Redis, LiveKit, coturn, Compose
demo/
  robot-simulator/       Process độc lập: media, command, motion, navigation
  e2e/                   Playwright
sample-data/             Map, destinations, route và thư mục media
docs/                    Architecture, protocol, API, media, ROS 2, tests
```

## Điều khiển

- `↑` tiến, `↓` lùi, `←` quay trái, `→` quay phải.
- Có thể giữ hai phím để đi chéo.
- `Space` dừng ngay.
- Nút màn hình hỗ trợ mouse, pen và touch.

Lệnh được gửi 20 Hz khi giữ và tăng tốc mềm trong 220 ms, TTL 300 ms. Keyup,
blur, tab hidden, unmount,
mất WebSocket và disconnect đều xóa trạng thái input. Watchdog simulator dừng
robot khi quá 400 ms không nhận lệnh mới.

## Dùng file, RTSP hoặc USB camera

File:

```env
SIMULATOR_MEDIA_SOURCE_TYPE=file
SIMULATOR_MEDIA_SOURCE=/media/museum-tour.mp4
SIMULATOR_AUDIO_SOURCE=/media/robot-audio.wav
```

Đặt file vào `sample-data/media`. RTSP:

```env
SIMULATOR_MEDIA_SOURCE_TYPE=rtsp
SIMULATOR_MEDIA_SOURCE=rtsp://user:password@camera/stream
```

USB camera:

```env
SIMULATOR_MEDIA_SOURCE_TYPE=camera
SIMULATOR_MEDIA_SOURCE=/dev/video0
SIMULATOR_CAMERA_FORMAT=mjpeg
SIMULATOR_CAMERA_WIDTH=1920
SIMULATOR_CAMERA_HEIGHT=1080
SIMULATOR_CAMERA_FPS=25
```

Chạy `./demo/robot-simulator/run.sh --list-cameras` để tìm thiết bị. Khi dùng
Docker, map thiết bị bằng `devices: ["/dev/video0:/dev/video0"]` trong service
`robot-simulator`. Mặc định FFmpeg scale/crop về Full HD 1920×1080, 25 fps;
LiveKit phát H.264 một lớp ở tối đa 8 Mbps. Pipeline raw dùng I420 và hàng đợi
nhỏ để giữ nhịp phát đều. Pipeline H.264 không bỏ frame nén trong hàng đợi,
còn trình duyệt dùng mục tiêu jitter 50 ms để tránh đứng/xước cả GOP khi một
frame tới trễ. Cả hai pipeline đều tự reconnect khi RTSP hoặc LiveKit gián
đoạn. Có thể hạ
`VIDEO_WIDTH`, `VIDEO_HEIGHT` và `VIDEO_BITRATE` trong `.env` nếu máy chạy
simulator có CPU hoặc băng thông hạn chế.

Frontend giữ khung hình tốt gần nhất khi track tạm gián đoạn và theo dõi tiến
độ frame để tự phục hồi subscription. Khi chạy LiveKit bằng Docker trên LAN,
đặt `LIVEKIT_NODE_IP` bằng IP LAN của máy chủ để ICE không quảng bá địa chỉ
bridge Docker.
Chi tiết ở [media-flow.md](docs/media-flow.md).

Microphone Bluetooth được quét qua socket PipeWire/PulseAudio của máy robot.
Với Docker Compose, socket mặc định là `/run/user/1000/pulse`; đặt
`PULSE_SOCKET_DIR` nếu desktop chạy bằng UID khác. Tai nghe phải đang kết nối,
không bị tắt tiếng và hỗ trợ profile HSP/HFP. Khi quét hoặc kiểm tra, robot tạm
chuyển từ A2DP sang HSP/HFP để thu tín hiệu thật rồi khôi phục profile ban đầu.
Chỉ thao tác **Lưu cấu hình** mới áp dụng và giữ profile của nguồn đã chọn.

## Cấu hình robot

Nhấn vào thẻ robot hoặc nút **Cấu hình** để mở trang cấu hình thiết bị. Trung
tâm gọi `configuration.get`/`configuration.update` trực tiếp qua WebSocket của
simulator; trung tâm không giữ một bản cấu hình giả. Khi simulator tắt, màn
hình báo robot ngoại tuyến. Khi lưu, simulator áp dụng nguồn video, profile
chất lượng, RTSP transport và nối lại media. Credential RTSP được simulator
giữ lại nhưng không bao giờ trả về trình duyệt.

Màn **Danh sách robot** là device registry của Center:

- thêm robot chỉ bằng IP/hostname, tài khoản và mật khẩu quản trị có sẵn trên
  thiết bị; robot có thể đang tắt;
- tìm kiếm, lọc trạng thái, phân trang, sửa metadata, vô hiệu hoá hoặc xoá
  robot offline;
- cho phép sửa nóng IP/hostname, tài khoản hoặc mật khẩu khi robot online mà
  không cắt phiên WSS/WebRTC hiện tại; credential mới dùng cho lần claim sau;
- tự cập nhật online/offline theo kết nối WSS, không dựa vào cấu hình `.env`;
- chỉ lưu PBKDF2 hash của mật khẩu quản trị và SHA-256 của device credential
  trong database. Credential rõ chỉ được trả cho edge agent khi claim.

Khi robot online, trang cấu hình đọc trực tiếp danh sách camera/microphone trên
máy edge sau khi người vận hành bấm **Quét**. Camera và microphone phần cứng
được chọn từ danh sách thay vì nhập tay `/dev/video0` hoặc
`plughw:CARD=Camera,DEV=0`; trang cũng cho phép thử WSS, thử từng nguồn media và
xem trước luồng camera. Quét chỉ giữ camera trả về được frame và microphone có
kết nối/tín hiệu; endpoint rút cáp, bị tắt hoặc chỉ trả digital silence không
được đưa vào danh sách chọn.

## Chạy từng service bằng `run.sh`

Mỗi service chạy trong một terminal riêng. Backend và simulator dùng chung
`.venv` ở thư mục gốc. Script tự tạo `.venv` và cài dependency ở lần chạy đầu;
các lần sau dùng lại ngay. Khi `requirements.txt` hoặc `package-lock.json`
thay đổi, script sẽ tự cập nhật dependency.

```bash
# Terminal 1 — backend
./src/apps/center-backend/run.sh
```

```bash
# Terminal 2 — frontend
./src/apps/center-frontend/run.sh
```

```bash
# Terminal 3 — robot simulator
./demo/robot-simulator/run.sh
```

Frontend mở tại <http://localhost:5173>. Backend/frontend đọc `.env` ở thư mục
gốc. Simulator ưu tiên `.env` nằm cạnh `demo/robot-simulator/run.sh`, sau đó
mới dùng `.env` ở thư mục gốc. Có thể đổi host/port local bằng
`CENTER_BACKEND_HOST`, `CENTER_BACKEND_PORT`, `CENTER_FRONTEND_HOST` và
`CENTER_FRONTEND_PORT`.

Simulator tiếp tục thử kết nối lại Center và media theo backoff mà không làm
gián đoạn watchdog an toàn.

## Chạy simulator trên máy khác

`run.sh` mặc định dùng container `rovera/robot-edge:1.1.0`, trong đó Python,
FFmpeg và toàn bộ thư viện đã được cố định. Máy edge chỉ cần Docker Engine;
không dùng Python, pip hoặc FFmpeg cài trên host. Camera USB/ALSA, device state
và network host được map tự động. Nếu Docker chưa sẵn sàng, script dừng với
thông báo rõ ràng để không vô tình dùng dependency của host. Chỉ khi cần chế
độ cũ mới chọn rõ bằng `ROVERA_RUNTIME=native`.

Chép thư mục `demo/robot-simulator` sang máy edge và giữ `.env` riêng trên từng
robot. Không cần cài hoặc nạp image thủ công. Compose tự build image từ
Dockerfile khi image chưa tồn tại; các lần chạy sau tái sử dụng image/cache:

```bash
cd demo/robot-simulator
docker compose up -d
docker compose ps
docker compose logs -f
docker compose down
```

`docker compose down` chỉ xoá container/network; credential và cấu hình camera
được giữ trong `ROBOT_STATE_DIR`, mặc định thư mục `./state` cạnh
`compose.yaml`; đường dẫn này không thay đổi khi chạy qua `sudo`. Docker tự
chọn base image
`python:3.12.11-slim-bookworm` đúng kiến trúc AMD64/ARM64 của máy.

Không copy thư mục `state/` hoặc file `device.json` sang robot khác. Mỗi file
state chứa credential riêng và được gắn với fingerprint của máy đã claim.
Nếu lỡ copy, phiên bản agent hiện tại tự bỏ qua state không đúng máy, claim lại
theo `ROBOT_MANAGEMENT_ADDRESS` rồi ghi `device.json` mới. Nếu `device.json`
bị copy nhầm thành thư mục, agent đổi tên nó thành `device.json.invalid-*` để
giữ dữ liệu cũ trước khi tạo file state hợp lệ.

Center và LiveKit chạy trên máy chủ, còn simulator/Orange Pi chỉ cần kết nối
outbound. Tài khoản quản trị robot được tạo sẵn khi cài image edge. Trên Center,
chọn **Danh sách robot → Thêm robot** rồi nhập đúng IP/hostname, tài khoản và
mật khẩu đó. Trên máy robot, cấu hình dùng chung Center và danh tính cục bộ:

```env
CENTER_API_URL=https://center.example.com
CENTER_ROBOT_WS_URL=wss://center.example.com/ws/robot/connect
ROBOT_MANAGEMENT_ADDRESS=192.168.1.20
ROBOT_USERNAME=operator
ROBOT_PASSWORD=<mat-khau-quan-tri-cuc-bo>
ROBOT_STATE_FILE=~/.config/rovera/device.json
```

Sau đó chạy:

```bash
cp demo/robot-simulator/edge.env.example demo/robot-simulator/.env
chmod 600 demo/robot-simulator/.env
./demo/robot-simulator/run.sh --check-config
./demo/robot-simulator/run.sh
```

Nếu chạy bằng systemd, dùng
`ROBOT_ENV_FILE=/etc/rovera/robot.env` và
`ROBOT_STATE_FILE=/var/lib/rovera/device.json`; unit service tự tạo thư mục
state đúng quyền.

Lần đầu edge agent tự claim hồ sơ đang chờ bằng địa chỉ/tài khoản/mật khẩu,
nhận định danh và credential riêng rồi lưu file state với quyền `0600`. Các lần
khởi động tiếp theo agent dùng state đã lưu, không cần sửa `.env`. Token ghép
nối một lần vẫn được giữ như phương án nâng cao. Robot đổi
credential dài hạn lấy JWT 15 phút qua HTTPS, dùng JWT để mở WSS và xin token
LiveKit publish 30 phút chỉ có quyền vào phòng
`robot-{robot_id}`. Máy robot không giữ `JWT_SECRET`, `LIVEKIT_API_SECRET` hay
credential người dùng. Với mạng LAN không có DNS/TLS, có thể dùng IP và
`http/ws` để thử nghiệm; triển khai thật phải đặt reverse proxy TLS và dùng
`https/wss`.

Không cần khai báo nguồn camera/microphone trên máy edge trước khi chạy. Agent
khởi động bằng test pattern, kết nối Center, rồi operator chọn USB camera, RTSP
hoặc microphone trong màn **Cấu hình**. Lựa chọn được lưu cùng device state và
được khôi phục sau khi reboot. Thiếu `/dev/video0` chỉ tạo cảnh báo, không chặn
robot online.

## Dữ liệu và migration

Compose dùng PostgreSQL; local mặc định SQLite. Migration đầu tiên:

```bash
cd src/apps/center-backend
alembic upgrade head
```

Schema gồm users, robots, connections, control sessions, maps, destinations,
routes, important command logs và robot events. Joystick stream không được ghi
PostgreSQL.

## Các phần mock có chủ đích

- Route preview dùng đường Manhattan hợp lệ trong map mẫu, không phải Nav2.
- Pin/network health simulator dùng giá trị mô phỏng.
- Nguồn `test` phát moving test pattern và silent audio; file/RTSP/camera dùng
  FFmpeg thật.
- Persistence adapter demo giữ presence/session realtime trong memory. Schema
  PostgreSQL đầy đủ; production multi-instance nên chuyển lock/presence sang
  Redis lease.
- Authentication người dùng có một tài khoản demo từ environment. Robot dùng
  registry trong database, credential claim, hash mật khẩu/secret riêng và JWT
  ngắn hạn. Production nên bổ sung audit chi tiết, rate limit endpoint claim
  và KMS/HSM cho các bí mật cấp hệ thống.

Không phần mock nào thay đổi interface của frontend hoặc robot gateway.

## Tài liệu

- [Kiến trúc + sequence diagrams](docs/architecture.md)
- [REST API](docs/api.md)
- [WebSocket protocol](docs/websocket-protocol.md)
- [Media/WebRTC](docs/media-flow.md)
- [Database](docs/database-schema.md)
- [Orange Pi 5 + ROS 2](docs/integration-robot.md)
- [Gamepad](docs/gamepad.md)
- [Kiểm thử](docs/testing.md)

## Bảo mật trước production

Đổi toàn bộ secret trong `.env`, dùng HTTPS/WSS, giới hạn CORS, không expose
PostgreSQL/Redis, cấu hình TURN public, lưu robot credential trong device
secret, rate-limit gateway/control và chuyển session lock sang Redis atomic
lease. Không log token, mật khẩu hoặc RTSP URL có credential.

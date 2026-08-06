# ROVERA — Robot Telepresence

ROVERA là hệ thống điều khiển robot từ xa qua trình duyệt, gồm giao diện quản
lý, API trung tâm, kết nối thời gian thực và phần mềm chạy trên robot.

Hướng dẫn mapping, phân phối map, Nav2 và motion safety: [docs/navigation-mapping.md](./docs/navigation-mapping.md).

## Các thành phần

- `center-frontend`: giao diện web để quản lý, xem camera và điều khiển robot.
- `center-backend`: REST API, WebSocket, xác thực và quản lý phiên điều khiển.
- `postgres`: lưu tài khoản, robot, bản đồ và cấu hình.
- `redis`: hỗ trợ trạng thái và kết nối thời gian thực.
- `livekit` + `coturn`: truyền video, âm thanh qua WebRTC.
- `robot-simulator`: robot mô phỏng/edge agent, chỉ chạy khi cần.

Mã nguồn chính nằm trong [`src`](./src), phần robot trong
[`demo/robot-simulator`](./demo/robot-simulator), dữ liệu mẫu trong
[`sample-data`](./sample-data).

## Yêu cầu

- Docker Engine
- Docker Compose v2 (`docker compose`)

Kiểm tra Docker trước khi chạy:

```bash
docker --version
docker compose version
```

## Cấu hình `.env` cho Center

Tại thư mục gốc của dự án:

```bash
cp .env.example .env
```

Mở `.env` và chỉnh các nhóm biến sau.

### Bắt buộc nên đổi

| Biến | Mục đích |
| --- | --- |
| `POSTGRES_PASSWORD` | Mật khẩu PostgreSQL. Dùng mật khẩu mạnh khi triển khai thật. |
| `JWT_SECRET` | Khóa ký access token, tối thiểu 32 ký tự ngẫu nhiên. |
| `LIVEKIT_API_KEY`, `LIVEKIT_API_SECRET` | Cặp khóa LiveKit; secret phải có tối thiểu 32 ký tự ngẫu nhiên. |
| `TURN_SECRET` | Shared secret tối thiểu 32 ký tự; LiveKit dùng để cấp credential coturn ngắn hạn. |
| `BOOTSTRAP_ADMIN_USERNAME` | Tên đăng nhập quản trị được tạo ở lần khởi tạo database đầu tiên. |
| `BOOTSTRAP_ADMIN_PASSWORD` | Mật khẩu quản trị ban đầu. |
| `BOOTSTRAP_ADMIN_EMAIL` | Email của tài khoản quản trị. |
| `DEMO_PASSWORD` | Mật khẩu tài khoản operator mẫu; không giữ giá trị mặc định khi máy có thể được truy cập từ mạng. |

`BOOTSTRAP_ADMIN_*` chỉ tạo tài khoản khi database chưa có admin. Thay đổi các
biến này sau đó không đổi thông tin của tài khoản đã tồn tại.

### Khi truy cập từ máy khác trong LAN

Thay `192.168.1.10` bằng IP của máy chạy Docker:

```env
FRONTEND_PUBLIC_URL=https://192.168.1.10
LIVEKIT_PUBLIC_URL=wss://192.168.1.10
LIVEKIT_ROBOT_URL=wss://192.168.1.10
LIVEKIT_NODE_IP=192.168.1.10
TURN_HOST=192.168.1.10
TURN_EXTERNAL_IP=192.168.1.10
TLS_HOSTNAME=192.168.1.10
```

Nếu chỉ dùng trên chính máy chạy Docker (`localhost`), có thể giữ giá trị mặc
định trong `.env.example`.

### Tùy chọn

- `FRONTEND_PORT`, `FRONTEND_HTTPS_PORT`, `BACKEND_PORT`: đổi cổng web và API.
- `LIVEKIT_HTTP_PORT`, `LIVEKIT_TCP_PORT`, `LIVEKIT_UDP_START`,
  `LIVEKIT_UDP_END`: đổi cổng LiveKit khi bị trùng cổng.
- `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `GOOGLE_REDIRECT_URI`: bật đăng
  nhập Google. Redirect URI phải có dạng
  `https://<CENTER_HOST>/api/auth/google/callback` và trùng cấu hình trên
  Google Cloud.
- `SIMULATOR_MEDIA_SOURCE_TYPE`, `SIMULATOR_MEDIA_SOURCE`,
  `SIMULATOR_AUDIO_SOURCE`: cấu hình nguồn media cho simulator.

Ví dụ nguồn media của simulator:

```env
# Test pattern, không cần camera
SIMULATOR_MEDIA_SOURCE_TYPE=test

# Hoặc camera RTSP
# SIMULATOR_MEDIA_SOURCE_TYPE=rtsp
# SIMULATOR_MEDIA_SOURCE=rtsp://user:password@CAMERA_IP:554/live

# Hoặc file đặt trong sample-data/media
# SIMULATOR_MEDIA_SOURCE_TYPE=file
# SIMULATOR_MEDIA_SOURCE=/media/video.mp4
```

## Chạy Center bằng Docker

Khởi tạo và build toàn bộ dịch vụ:

```bash
docker compose up -d --build
```

Các địa chỉ mặc định:

- Giao diện: <https://localhost>
- OpenAPI: <https://localhost/docs>
- Robot API/WebSocket: `https://<CENTER_HOST>` / `wss://<CENTER_HOST>/ws/robot/connect`
- LiveKit signaling: `wss://<CENTER_HOST>` (được proxy tại `/rtc`)
- TURN relay: `turn:<TURN_HOST>:3478` qua UDP/TCP

Cổng HTTP `8080` chỉ chuyển hướng sang HTTPS. Cổng backend `8888` và signaling
LiveKit `7880` chỉ bind loopback để chẩn đoán tại máy chủ.

Ở lần chạy đầu, container frontend tự tạo CA nội bộ và server certificate tại
`src/infrastructure/tls`. Import `ca.crt` vào trust store của trình duyệt/robot
trong LAN. Khi triển khai Internet, thay `server.crt` và
`server.key` bằng chứng thư của CA tin cậy rồi recreate `center-frontend`.
Khi dùng robot Docker, copy `ca.crt` sang đúng đường dẫn
`CENTER_TLS_CA_FILE` trên máy robot; launcher sẽ mount CA cho cả API và LiveKit.

Xem trạng thái và log:

```bash
docker compose ps
docker compose logs -f --tail=200
```

Sau khi sửa `.env` hoặc mã nguồn, áp dụng lại cấu hình:

```bash
docker compose up -d --build --force-recreate
```

## Chạy robot/edge trên máy riêng bằng Docker

Trên máy robot, vào thư mục `demo/robot-simulator` và tạo file cấu hình riêng:

```bash
cd demo/robot-simulator
cp edge.env.example .env
chmod 600 .env
```

Cần sửa tối thiểu các biến sau trong `.env`:

```env
CENTER_API_URL=https://192.168.1.10
CENTER_ROBOT_WS_URL=wss://192.168.1.10/ws/robot/connect
CENTER_TLS_CA_FILE=/etc/rovera/rovera-ca.crt
ROBOT_MANAGEMENT_ADDRESS=192.168.1.20
ROBOT_USERNAME=operator
ROBOT_PASSWORD=<mat-khau-quan-tri-cua-robot>
```

- `192.168.1.10`: IP máy chạy Center.
- `192.168.1.20`: IP/hostname quản lý của robot.
- `ROBOT_USERNAME` và `ROBOT_PASSWORD`: tài khoản cục bộ dùng khi thêm robot
  trong màn **Danh sách robot** của Center.
- Kiểm tra và sửa `ROBOT_UID`, `ROBOT_GID`, `VIDEO_GID`, `AUDIO_GID`,
  `RENDER_GID` nếu UID/GID trên máy robot khác giá trị mẫu.
- Giữ `MOTION_BACKEND=simulator` cho đến khi đã cấu hình và kiểm tra ROS 2.

Build và chạy edge agent:

```bash
docker compose up -d --build
docker compose ps
docker compose logs -f robot-simulator
```

Cấu hình camera, microphone, loa và RTSP được chọn trên giao diện Center sau
khi robot online. Trạng thái robot được giữ trong `./state` và không mất khi
container được tạo lại.

## Dừng hoặc xóa dữ liệu

Dừng các container, vẫn giữ dữ liệu PostgreSQL và Redis:

```bash
docker compose down
```

Xóa cả container và toàn bộ volume dữ liệu:

```bash
docker compose down -v
```

Lệnh có `-v` sẽ xóa tài khoản, robot và dữ liệu đã lưu; chỉ dùng khi muốn khởi
tạo lại hệ thống từ đầu.

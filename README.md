# ROVERA — Robot Telepresence

ROVERA là hệ thống điều khiển robot từ xa qua trình duyệt, gồm giao diện quản
lý, API trung tâm, kết nối thời gian thực và phần mềm chạy trên robot.

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
| `BOOTSTRAP_ADMIN_USERNAME` | Tên đăng nhập quản trị được tạo ở lần khởi tạo database đầu tiên. |
| `BOOTSTRAP_ADMIN_PASSWORD` | Mật khẩu quản trị ban đầu. |
| `BOOTSTRAP_ADMIN_EMAIL` | Email của tài khoản quản trị. |
| `DEMO_PASSWORD` | Mật khẩu tài khoản operator mẫu; không giữ giá trị mặc định khi máy có thể được truy cập từ mạng. |

`BOOTSTRAP_ADMIN_*` chỉ tạo tài khoản khi database chưa có admin. Thay đổi các
biến này sau đó không đổi thông tin của tài khoản đã tồn tại.

### Khi truy cập từ máy khác trong LAN

Thay `192.168.1.10` bằng IP của máy chạy Docker:

```env
FRONTEND_PUBLIC_URL=http://192.168.1.10:8080
LIVEKIT_PUBLIC_URL=ws://192.168.1.10:7880
LIVEKIT_ROBOT_URL=ws://192.168.1.10:7880
LIVEKIT_NODE_IP=192.168.1.10
```

Nếu chỉ dùng trên chính máy chạy Docker (`localhost`), có thể giữ giá trị mặc
định trong `.env.example`.

### Tùy chọn

- `FRONTEND_PORT`, `BACKEND_PORT`: đổi cổng web và API.
- `LIVEKIT_HTTP_PORT`, `LIVEKIT_TCP_PORT`, `LIVEKIT_UDP_START`,
  `LIVEKIT_UDP_END`: đổi cổng LiveKit khi bị trùng cổng.
- `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `GOOGLE_REDIRECT_URI`: bật đăng
  nhập Google. Redirect URI phải có dạng
  `http://<CENTER_IP>:8080/api/auth/google/callback` và trùng cấu hình trên
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

- Giao diện: <http://localhost:8080>
- OpenAPI: <http://localhost:8080/docs>
- Backend cho robot trong LAN: `http://<CENTER_IP>:8888`
- LiveKit: `ws://<CENTER_IP>:7880`

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
CENTER_API_URL=http://192.168.1.10:8888
CENTER_ROBOT_WS_URL=ws://192.168.1.10:8888/ws/robot/connect
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

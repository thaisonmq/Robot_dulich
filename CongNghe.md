# Công nghệ sử dụng trong hệ thống ROVERA

## 1. Phạm vi tài liệu

Tài liệu này mô tả các công nghệ **thực sự xuất hiện trong mã và cấu hình hiện
tại** của ROVERA, vai trò của từng công nghệ trong bài toán robot du lịch từ xa,
và cách chúng đang được áp dụng trong dự án. Nội dung được đối chiếu từ:

- mã giao diện tại [`src/apps/center-frontend`](./src/apps/center-frontend);
- mã dịch vụ trung tâm tại
  [`src/apps/center-backend`](./src/apps/center-backend);
- hạ tầng tại [`src/infrastructure`](./src/infrastructure);
- edge agent, mô phỏng và tích hợp ROS 2 tại
  [`demo/robot-simulator`](./demo/robot-simulator);
- hợp đồng dữ liệu dùng chung tại [`src/packages`](./src/packages);
- cấu hình triển khai trong [`docker-compose.yml`](./docker-compose.yml) và các
  tệp Compose của robot.

Tài liệu dùng ba cách diễn đạt để tránh nhầm lẫn:

- **Đang dùng**: mã hiện tại gọi trực tiếp công nghệ hoặc dịch vụ đó.
- **Tùy chọn/robot thật**: đã có mã tích hợp nhưng chỉ chạy khi bật profile hoặc
  biến môi trường tương ứng.
- **Đã khai báo nhưng chưa dùng trực tiếp**: có dependency/cấu hình, nhưng mã
  ứng dụng hiện chưa import hoặc chưa dùng nó trong luồng chính.

## 2. Bài toán hệ thống đang giải quyết

ROVERA là một nền tảng **telepresence cho du lịch**. Một người dùng có thể truy
cập robot qua trình duyệt để quan sát địa điểm bằng camera, nghe/nói hai chiều,
điều khiển robot, chọn điểm đến trên bản đồ và theo dõi trạng thái robot. Người
vận hành còn có thể quản lý tài khoản, robot, camera, bản đồ và giám sát phiên
của khách.

Bài toán này không chỉ là một trang web thông thường. Hệ thống phải xử lý đồng
thời bốn loại luồng có đặc tính rất khác nhau:

1. Dữ liệu quản trị như tài khoản, robot, bản đồ và lịch sử phiên cần tính nhất
   quán và lưu bền vững.
2. Lệnh chuyển động cần độ trễ thấp, có thời hạn ngắn và không được phát lại khi
   kết nối chập chờn.
3. Video và âm thanh cần truyền thời gian thực, thích nghi với mạng và đi xuyên
   NAT/firewall.
4. Mapping, định vị và navigation phải phối hợp với LiDAR, odometry, TF và lớp
   an toàn chuyển động chạy cục bộ trên robot.

Vì vậy mã hiện tại tách hệ thống thành giao diện web, Center backend, hạ tầng
media thời gian thực và edge runtime trên robot.

## 3. Kiến trúc tổng thể

```text
Trình duyệt
  ├─ React SPA ── HTTPS/REST ───────────────┐
  ├─ Control/Telemetry ── WSS ─────────────┤
  └─ Camera + âm thanh ── WebRTC ───────┐  │
                                        │  │
Nginx/TLS                               │  │
  ├─ /api, /ws  ───────────────> FastAPI Center
  └─ /rtc       ───────────────> LiveKit SFU ── Redis
                                      │
                                    coturn
                                      │
Robot Edge Python <── HTTPS/WSS/WebRTC┘
  ├─ FFmpeg/GStreamer + V4L2/RTSP + PulseAudio/ALSA
  ├─ motion/navigation simulator (mặc định an toàn)
  └─ Unix socket JSON <──> ROS 2 Humble
                           ├─ SLAM Toolbox
                           ├─ Nav2 + AMCL + Map Server
                           ├─ robot_localization + TF2
                           └─ twist_mux + velocity smoother + motion safety

FastAPI Center ── SQLAlchemy/psycopg ── PostgreSQL
               └─ map bundle files ──── Docker volume map_storage
```

Ý nghĩa của việc tách luồng:

- REST chịu trách nhiệm cho thao tác có tính giao dịch như đăng nhập, tạo robot,
  tạo phiên, cấu hình và quản lý bản đồ.
- WebSocket chuyển lệnh điều khiển và telemetry vì các bản tin này cần hai chiều
  và độ trễ thấp.
- WebRTC/LiveKit đảm nhiệm riêng media; video không đi qua FastAPI WebSocket.
- ROS 2 và các cảm biến nằm cục bộ trên robot, nên robot vẫn có thể dừng an toàn
  mà không phụ thuộc vào độ trễ Internet.

## 4. Tổng quan công nghệ theo lớp

| Lớp | Công nghệ chính | Cách áp dụng hiện tại |
| --- | --- | --- |
| Giao diện | React 19, TypeScript 5.8, Vite 7 | SPA quản lý, điều khiển, camera, mapping và navigation |
| Trạng thái frontend | TanStack Query, Zustand, Web Storage | Server state, runtime state và khôi phục phiên trên tab |
| Đồ họa | HTML Canvas 2D, CSS, Lucide React | Render saved map, robot, đường đi, vật cản và giao diện responsive |
| Backend | Python 3.12, FastAPI, Uvicorn, Pydantic | REST API, OpenAPI, WebSocket gateway, validation và cấu hình |
| Dữ liệu | PostgreSQL 16, SQLAlchemy 2, psycopg 3, Alembic | Tài khoản, robot, phiên, map registry, mission và migration |
| Điều khiển realtime | WebSocket + JSON envelope | Browser ↔ Center ↔ robot, tách control và telemetry |
| Media | WebRTC, LiveKit, coturn, Redis | Camera và âm thanh hai chiều, SFU và TURN relay |
| Robot edge | Python asyncio, websockets, HTTPX, Pydantic | Xác thực robot, reconnect, telemetry, media và điều phối backend |
| Xử lý media edge | FFmpeg, GStreamer, V4L2, ALSA/PulseAudio | Camera USB/RTSP/test, microphone, loa và hardware acceleration |
| Robotics | ROS 2 Humble, rclpy, Fast DDS, micro-ROS | Giao tiếp node/topic/action/service và nối tới phần cứng |
| Mapping | SLAM Toolbox, OccupancyGrid, YAML/PGM, Pillow | Tạo map, pose graph, preview, kiểm tra goal và version hóa |
| Navigation | Nav2, AMCL, TF2, EKF | Định vị, lập đường, bám đường và recovery |
| An toàn | twist_mux, Nav2 Velocity Smoother, node safety riêng | Phân xử nguồn lệnh, giới hạn gia tốc, watchdog và E-stop |
| Triển khai | Docker, Docker Compose, Nginx, TLS/OpenSSL | Đóng gói Center và robot, profile hóa phần cứng/ROS |
| Kiểm thử | Vitest, Testing Library, jsdom, pytest, Playwright | Unit, async/integration và luồng trình duyệt E2E |

## 5. Công nghệ frontend

### 5.1. React 19

**React** là nền tảng giao diện chính. Ứng dụng được khởi tạo bằng
`createRoot` trong [`main.tsx`](./src/apps/center-frontend/src/main.tsx), sau đó
ghép các provider cho query, i18n và trạng thái ứng dụng.

Trong bài toán ROVERA, React được dùng để:

- hiển thị danh sách và trạng thái online/offline của robot;
- tạo hoặc kết thúc phiên điều khiển;
- giữ màn camera, bảng điều khiển và mini-map đồng bộ với telemetry;
- quản lý tài khoản, phân quyền, cấu hình camera/microphone/loa;
- chạy quy trình tạo bản đồ, kích hoạt map, tự định vị và gửi goal Nav2.

Dashboard cố ý giữ media transport tồn tại khi mở rộng bản đồ hoặc đổi panel,
tránh unmount camera làm gián đoạn buổi tham quan. Phần triển khai chính nằm ở
[`DashboardPage.tsx`](./src/apps/center-frontend/src/pages/DashboardPage.tsx).

### 5.2. TypeScript 5.8

Frontend được viết bằng **TypeScript** để mô hình hóa rõ `Robot`, `Session`,
`Pose`, `Health`, map, mission và message envelope. Điều này đặc biệt cần thiết
cho robot từ xa vì một nhầm lẫn giữa `map_id`, `version`, `session_id` hoặc loại
message có thể tạo ra lệnh sai robot/sai phiên.

Các type frontend nằm trong
[`types/index.ts`](./src/apps/center-frontend/src/types/index.ts); hợp đồng realtime
dùng chung nằm ở [`src/packages/contracts/index.ts`](./src/packages/contracts/index.ts).
Build production chạy `tsc -b` trước Vite nên lỗi kiểu dữ liệu có thể chặn build.

### 5.3. Vite 7

**Vite** đảm nhiệm dev server, hot reload và build bundle production. Cấu hình
[`vite.config.ts`](./src/apps/center-frontend/vite.config.ts) proxy `/api`,
`/health` và `/ws` tới Center backend khi phát triển cục bộ. Khi production,
bundle Vite được copy vào image Nginx.

### 5.4. TanStack Query 5

**TanStack React Query** quản lý server state: lấy danh sách robot/map/user,
cache kết quả, biểu diễn loading/error, chạy mutation và invalidate/refetch sau
khi cập nhật.

Ví dụ hiện tại:

- `RobotListPage` query robot và active session;
- `RobotConfigurationPage` query cấu hình, map và media source;
- `MapManagementPage` query map registry;
- `DashboardPage` query saved map, destination, camera và video profile;
- `MappingControlPanel` dùng mutation cho start/stop/save/discard mapping.

React Query không thay WebSocket: dữ liệu CRUD đi qua query, còn pose/health và
lệnh điều khiển đi qua kênh realtime riêng.

### 5.5. Zustand 5

**Zustand** giữ runtime state dùng chung như user hiện tại, robot được chọn,
session, pose, health, trạng thái media/control/navigation và route. Store nằm ở
[`appStore.ts`](./src/apps/center-frontend/src/state/appStore.ts).

Session đang điều khiển được ghi thêm vào `sessionStorage`, giúp cùng một tab có
thể khôi phục ngữ cảnh sau refresh. Token và tùy chọn ngôn ngữ/sidebar dùng
`localStorage` qua mã frontend. Đây là state phía trình duyệt; quyền điều khiển
thật vẫn được backend kiểm tra theo JWT và session lease.

### 5.6. Router tự xây dựng bằng History API

Dự án **không dùng React Router**. Tệp
[`router.ts`](./src/apps/center-frontend/src/router.ts) dùng `history.pushState`,
`popstate` và `useSyncExternalStore` của React. Cách này đủ cho tập route hiện
tại như login, robots, control, maps và account, đồng thời giảm một dependency.

Nginx dùng `try_files ... /index.html` để truy cập trực tiếp hoặc refresh route
SPA vẫn hoạt động.

### 5.7. LiveKit Client và WebRTC trên trình duyệt

[`MediaTransport.ts`](./src/apps/center-frontend/src/transports/MediaTransport.ts)
dùng trực tiếp **`livekit-client`** để kết nối một LiveKit room, subscribe camera
và audio của robot, publish microphone người dùng và xử lý reconnect/token mới.
Lớp `AdaptiveVideoBuffer` theo dõi thống kê nhận video để điều chỉnh buffer khi
mạng không ổn định.

Dependency `@livekit/components-react` có trong `package.json`, nhưng mã
`src` hiện **không import trực tiếp** package này. UI media hiện dùng
`livekit-client` và DOM media API tự viết.

### 5.8. WebSocket API gốc của trình duyệt

Frontend không dùng thư viện WebSocket bên ngoài. Ba transport hiện có:

- [`ControlTransport.ts`](./src/apps/center-frontend/src/transports/ControlTransport.ts):
  velocity, STOP, PTZ, ACK, lock theo `client_id` và reconnect;
- [`TelemetryTransport.ts`](./src/apps/center-frontend/src/transports/TelemetryTransport.ts):
  pose, health, navigation status/visualization;
- [`MappingTransport.ts`](./src/apps/center-frontend/src/transports/MappingTransport.ts):
  trạng thái mapping thời gian thực.

Control và telemetry được tách để lưu lượng hiển thị không chặn lệnh chuyển
động. Lệnh có UUID, sequence, timestamp và TTL. Khi mất kết nối, input đang giữ
được xóa và robot có watchdog để dừng thay vì phát lại lệnh cũ.

### 5.9. HTML Canvas 2D và bộ chuyển đổi tọa độ map

Saved map được render bằng **Canvas 2D** trong
[`MapPanel.tsx`](./src/apps/center-frontend/src/components/MapPanel.tsx), không
dùng Leaflet hay Google Maps. Canvas vẽ:

- ảnh occupancy map với `imageSmoothingEnabled = false`;
- footprint và hướng của robot;
- goal được chọn;
- global path thật do Nav2 trả về;
- các lethal cell/vật cản động từ local costmap.

[`src/packages/map-utils/index.ts`](./src/packages/map-utils/index.ts) chuyển
đổi hai chiều giữa tọa độ thế giới ROS theo mét và pixel, có xử lý cả rotation
của `origin`. Đây là chi tiết quan trọng vì click đúng trên ảnh nhưng đổi sai hệ
tọa độ có thể gửi robot tới vị trí khác.

### 5.10. CSS, Lucide React và i18n tự xây dựng

- CSS thuần trong `styles.css`, `map-workspace.css` và
  `operations-shell.css` tạo layout vận hành; dự án không dùng Tailwind hoặc
  component framework.
- **Lucide React** cung cấp icon cho menu, nút trạng thái, bản đồ và cảnh báo.
- I18n dùng React Context và các tệp JSON/TypeScript nội bộ tại
  [`src/i18n`](./src/apps/center-frontend/src/i18n). Provider tự chọn ngôn ngữ
  trình duyệt, lưu tùy chọn theo tài khoản và đổi `dir=rtl` cho ngôn ngữ viết
  từ phải sang trái. Dự án không dùng `react-i18next`.

## 6. Backend và API Center

### 6.1. Python 3.12

Backend và robot edge đều dùng **Python 3.12** trong Docker image. Python phù
hợp với dự án vì có hệ sinh thái web bất đồng bộ tốt, đồng thời ROS 2 có `rclpy`
để viết adapter/navigation và safety node cùng ngôn ngữ.

### 6.2. FastAPI 0.116 và Uvicorn 0.35

**FastAPI** cung cấp REST API, dependency injection, validation, upload file,
WebSocket endpoint, static file và OpenAPI. **Uvicorn** chạy ứng dụng theo chuẩn
ASGI. Điểm vào là [`app/main.py`](./src/apps/center-backend/app/main.py).

FastAPI đang giải quyết:

- đăng ký/đăng nhập và Google OAuth;
- RBAC cho `admin`, `operator`, `guest`;
- quản lý robot, enrollment/claim và cấu hình thiết bị;
- tạo, giám sát và kết thúc control session;
- map registry, upload/download bundle, mapping lifecycle;
- compute path, navigation mission và health;
- gateway WebSocket cho browser và robot.

`python-multipart` được cài để FastAPI xử lý `File`, `Form` và `UploadFile` của
bundle bản đồ, dù mã không cần import package này trực tiếp.

### 6.3. Pydantic 2 và pydantic-settings

**Pydantic** kiểm tra request/response model và message payload. Các schema
chính nằm trong [`schemas/messages.py`](./src/apps/center-backend/app/schemas/messages.py).
**pydantic-settings** ánh xạ biến môi trường thành đối tượng `Settings` tại
[`core/config.py`](./src/apps/center-backend/app/core/config.py), gồm database,
JWT, LiveKit, CORS, timeout, map storage và Google OAuth.

Robot edge cũng dùng Pydantic Settings cho địa chỉ Center, TLS, media profile,
watchdog và lựa chọn motion/navigation backend.

### 6.4. SQLAlchemy 2, psycopg 3 và PostgreSQL 16

**SQLAlchemy ORM** định nghĩa entity và transaction; **psycopg 3** là driver
kết nối; **PostgreSQL 16** là cơ sở dữ liệu production trong Compose.

Mã hiện lưu các nhóm dữ liệu sau trong PostgreSQL:

- user, OAuth identity, login code và audit log;
- robot registry, credential hash và lịch sử kết nối;
- control session;
- map, map version, mapping session, cache trạng thái trên từng robot và
  tombstone/ACK xóa;
- POI, destination, keepout zone và speed zone;
- route, navigation mission;
- command receipt để chống xử lý trùng và log/event.

Model nằm trong
[`models/entities.py`](./src/apps/center-backend/app/models/entities.py). Mặc
định trong code có URL SQLite để chạy đơn giản, và test backend dùng SQLite tạm;
nhưng Compose production truyền URL PostgreSQL. Vì vậy SQLite là fallback/test,
không phải database chính khi triển khai hệ thống đầy đủ.

### 6.5. Alembic 1.16

**Alembic** quản lý thay đổi schema có version tại
[`migrations/versions`](./src/apps/center-backend/migrations/versions). Container
backend chạy `alembic upgrade head` trước Uvicorn, giúp các bảng RBAC, robot
registry, supervision session và map/navigation mới được nâng cấp theo thứ tự.

### 6.6. AsyncIO và runtime hub trong bộ nhớ

Center dùng **`asyncio`** cho WebSocket, heartbeat monitor, command request/ACK,
timeout và fan-out telemetry. [`services/hub.py`](./src/apps/center-backend/app/services/hub.py)
giữ socket robot, socket người dùng, session runtime, lease media và pending
request trong bộ nhớ tiến trình.

Đây là trạng thái hiện tại cần hiểu đúng: PostgreSQL lưu bản ghi bền vững, nhưng
socket và điều phối realtime vẫn là **in-memory hub**. Do đó cấu hình hiện phù
hợp một process Uvicorn/Center; chưa thể tự động scale ngang nhiều backend
replica chỉ bằng cách tăng container.

### 6.7. HTTPX

Backend dùng **HTTPX** cho luồng Google OAuth/OIDC: gọi token endpoint và lấy
thông tin tài khoản. Robot edge dùng `httpx.AsyncClient` nhiều hơn để enroll,
claim, đổi credential lấy JWT, lấy LiveKit token và đồng bộ map bundle với
Center qua HTTPS.

### 6.8. Lưu trữ map trên filesystem

Metadata map/version nằm trong PostgreSQL, còn bundle nhị phân được
[`MapBundleStore`](./src/apps/center-backend/app/services/map_storage.py) lưu vào
thư mục `MAP_STORAGE_DIR`, gắn với Docker volume `map_storage`.

Code kiểm tra giới hạn kích thước, SHA-256, danh sách artifact, đường dẫn nguy
hiểm và identity trong metadata trước khi chấp nhận bundle. Hiện dự án chưa dùng
S3/object storage; khi chạy nhiều Center replica, phần filesystem này cũng cần
được thay bằng kho dùng chung hoặc shared volume.

## 7. Xác thực, phân quyền và bảo mật

### 7.1. JWT với PyJWT

**PyJWT** tạo access token người dùng và robot. Code phân biệt claim
`type=access` với `type=robot`, dùng `sub` làm user/robot identity và kiểm tra
`exp`. Thuật toán mặc định hiện là **HS256**, khóa ký lấy từ `JWT_SECRET`.

Robot không đưa credential dài hạn vào query WebSocket. Edge đổi credential lấy
robot JWT ngắn hạn qua HTTPS, rồi dùng Bearer token để mở WSS và xin media token.

### 7.2. PBKDF2-HMAC-SHA256 và hashing credential

[`core/security.py`](./src/apps/center-backend/app/core/security.py) dùng thư
viện chuẩn Python để hash mật khẩu bằng **PBKDF2-HMAC-SHA256**, salt ngẫu nhiên
và 600.000 vòng; so sánh bằng `hmac.compare_digest` để giảm rò rỉ timing.

- Mật khẩu user và mật khẩu quản lý robot được lưu dưới dạng PBKDF2 hash.
- Device credential và enrollment token ngẫu nhiên chỉ được Center lưu
  SHA-256; credential rõ được trả cho robot ở thời điểm cấp phát.
- Edge lưu identity/credential cục bộ với quyền file `0600` và gắn fingerprint
  máy để hạn chế dùng nhầm state bị copy sang thiết bị khác.

### 7.3. RBAC và khóa phiên

Ba role hiện có là `admin`, `operator`, `guest`. Backend là nơi quyết định quyền,
không tin vào việc frontend ẩn nút. Session runtime chỉ cho một controller; tab
đầu tiên claim session bằng `client_id`, tab khác bị đóng với mã `4009`. Admin
và operator có thể giám sát phiên guest bằng LiveKit token chỉ-subscribe.

### 7.4. Google OAuth 2.0/OpenID Connect

Google login là tích hợp tùy chọn, bật khi có `GOOGLE_CLIENT_ID` và
`GOOGLE_CLIENT_SECRET`. Backend dùng state/code ngắn hạn, liên kết
`AuthIdentity`, rồi trả login code một lần cho callback frontend. Nếu không có
cấu hình Google, đăng nhập username/email và password vẫn hoạt động.

### 7.5. TLS, Nginx và CORS

**Nginx 1.27** phục vụ SPA và làm reverse proxy cho FastAPI, WebSocket và
LiveKit signaling. Cấu hình tại
[`nginx.conf`](./src/apps/center-frontend/nginx.conf) bật TLS 1.2/1.3, redirect
HTTP sang HTTPS, chuyển header WebSocket Upgrade và đặt timeout dài hơn cho
bundle SLAM.

Script entrypoint dùng **OpenSSL** tạo CA/server certificate nội bộ cho môi
trường LAN. Robot có thể mount CA này để HTTPX, WebSocket và LiveKit SDK xác minh
WSS. FastAPI còn cấu hình CORS từ biến môi trường. Khi public Internet cần thay
certificate tự ký bằng certificate của CA tin cậy.

## 8. Giao tiếp realtime và hợp đồng dữ liệu

### 8.1. REST và WebSocket có trách nhiệm khác nhau

REST được dùng cho thao tác có kết quả rõ ràng và cần lưu database. WebSocket
dùng cho control/telemetry liên tục. Center có các kênh chính:

- `/ws/robot/connect`: kết nối dài hạn từ edge agent;
- `/ws/user/control/{robot_id}`: lệnh điều khiển của tab đang giữ lease;
- `/ws/user/telemetry/{robot_id}`: pose, health, ACK và navigation visualization;
- WebSocket mapping tương ứng cho tiến trình tạo map.

### 8.2. JSON envelope và JSON Schema Draft 2020-12

Mọi message realtime dùng envelope version `1.0` gồm:

- `message_id` UUID để nhận diện/deduplicate;
- `message_type` để route lệnh;
- `robot_id` và `session_id` để tránh gửi nhầm;
- `sequence` để phát hiện message cũ hoặc đảo thứ tự;
- `timestamp` và `ttl_ms` để từ chối lệnh hết hạn;
- `payload` chứa dữ liệu riêng của loại message.

Schema máy đọc được nằm ở
[`message.schema.json`](./src/packages/contracts/message.schema.json), theo
**JSON Schema Draft 2020-12**. TypeScript contract song song nằm ở
[`src/packages/contracts/index.ts`](./src/packages/contracts/index.ts).

### 8.3. ACK, idempotency, timeout và fail-safe

Lệnh quản trị mapping/navigation có `request_id` và `expected_state`. Backend
lưu `CommandReceipt`; nếu client retry cùng request, hệ thống trả kết quả trước
thay vì chạy lại một thao tác nguy hiểm. State machine chặn chuyển trạng thái
không hợp lệ.

Lệnh joystick thì không queue/retry. Khi WSS hỏng, browser không phát lại vận
tốc cũ; robot dừng bằng watchdog. Đây là lựa chọn đúng cho điều khiển từ xa:
đối với chuyển động, bỏ một command an toàn hơn chạy trễ một command cũ.

## 9. Media thời gian thực

### 9.1. WebRTC

**WebRTC** là nền tảng truyền camera và âm thanh độ trễ thấp. Nó giải quyết mã
hóa media, jitter, thống kê chất lượng mạng, UDP/TCP fallback và NAT traversal.
Trong ROVERA, media không nhúng vào JSON/WebSocket nên không làm nghẽn kênh
STOP hoặc telemetry.

### 9.2. LiveKit Server và SDK

**LiveKit Server v1.12.0** chạy như SFU. Mỗi robot dùng room
`robot-{robot_id}`. Backend dùng `livekit-api` để ký token có grant khác nhau:

- controller có thể publish microphone và subscribe camera/audio;
- spectator chỉ subscribe;
- robot `main` publish/subscribe cho audio và control media;
- robot `video` dùng identity riêng để camera publisher không thay thế kết nối
  main.

Browser dùng `livekit-client`; edge dùng **LiveKit Python SDK** và một publisher
GStreamer tối ưu cho video H.264.

### 9.3. coturn

**coturn 4.6.3** là TURN server. Nếu browser và robot không tạo được đường media
trực tiếp do NAT/firewall, LiveKit cấp credential HMAC ngắn hạn để hai bên relay
qua coturn bằng UDP hoặc TCP. Đây là thành phần cần thiết khi robot nằm ở một
mạng du lịch còn người dùng ở mạng khác.

### 9.4. Redis

**Redis 7.4** đang được cấu hình làm backend trạng thái/phân phối của LiveKit và
có persistent volume riêng. Mặc dù `REDIS_URL` cũng được truyền vào Center và
có field trong `Settings`, mã FastAPI hiện **chưa tạo Redis client và chưa dùng
Redis cho session, cache, lease hay message bus**.

Vì vậy không nên mô tả Redis hiện tại là nơi lưu control session của backend.
Session realtime vẫn nằm trong `hub` của process FastAPI.

## 10. Robot edge và xử lý thiết bị

### 10.1. Python asyncio, websockets và HTTPX

[`simulator/client.py`](./demo/robot-simulator/simulator/client.py) là edge
gateway. Nó dùng `asyncio` chạy song song receive loop, simulation, telemetry,
heartbeat, media lease, map sync và navigation runtime. Package `websockets`
mở kết nối WSS dài hạn; HTTPX xử lý API HTTPS.

Khi mạng mất, edge reconnect bằng exponential backoff có jitter, đồng thời dừng
motion và media lease. Message ID đã xử lý được giữ trong tập/deque để hạn chế
xử lý lặp trong một phiên chạy.

### 10.2. Motion và navigation backend có thể thay thế

Edge định nghĩa abstraction cho backend:

- `MOTION_BACKEND=simulator` là mặc định an toàn, chỉ cập nhật pose mô phỏng;
- backend ROS 2 chuyển velocity qua Unix domain socket tới ROS bridge;
- `NAVIGATION_BACKEND=simulator` tạo đường thẳng/mô phỏng di chuyển;
- `NAVIGATION_BACKEND=ros2` gọi adapter Nav2/SLAM thật.

Điều này cho phép chạy demo và test toàn bộ Center mà không cần robot vật lý,
nhưng vẫn có đường tích hợp phần cứng khi triển khai.

### 10.3. FFmpeg

Image edge cài **FFmpeg** để đọc và chuyển đổi nhiều loại nguồn:

- test pattern;
- file video;
- camera V4L2 `/dev/video*`;
- RTSP camera IP;
- audio từ ALSA/PulseAudio.

Mã xây pipeline theo source/profile và có nhánh hardware acceleration. Image
generic x86_64 cài VA-API driver; build `MEDIA_PLATFORM=rk3588` có thể dùng
Rockchip MPP và bản FFmpeg Rockchip. Raspberry Pi 5 vẫn dùng nhánh generic,
không bị nhận nhầm là RK3588 chỉ vì cùng kiến trúc ARM64.

### 10.4. GStreamer và Go publisher

Dockerfile edge build `livekit/gstreamer-publisher` từ commit được pin, áp patch
cục bộ và biên dịch bằng **Go 1.26 + CGO**. Binary runtime nhận pipeline
**GStreamer 1.0** và publish video H.264 vào LiveKit bằng identity `video`.

Go ở đây là công nghệ build cho publisher chuyên dụng, không phải ngôn ngữ của
business logic edge. GStreamer được chọn cho nhánh video tối ưu vì có pipeline
plugin, V4L2/RTSP và hardware encoder; Python LiveKit connection vẫn đảm nhiệm
audio và subscription phía robot.

### 10.5. V4L2, ALSA, PulseAudio/PipeWire

- **V4L2** phát hiện camera USB, format, độ phân giải và FPS; `v4l2-ctl` hỗ trợ
  probe và PTZ UVC.
- **ALSA** là lớp thiết bị âm thanh Linux.
- `pactl`/PulseAudio compatibility socket cho phép container nhìn thấy nguồn
  PipeWire/PulseAudio của user, gồm microphone Bluetooth HSP/HFP.
- Edge quét, probe rồi chỉ trả các nguồn hoạt động cho Center; nguồn bị phát
  hiện nhưng lỗi được đưa vào danh sách rejected để UI báo rõ.

Compose mount `/dev`, thêm video/audio/render group và device cgroup rule để
edge truy cập thiết bị mà vẫn chạy bằng UID/GID cấu hình thay vì mặc định root.

### 10.6. RTSP, ONVIF và WS-Discovery

Camera IP dùng **RTSP** làm nguồn video. Edge hỗ trợ chọn TCP/UDP transport và
không trả credential RTSP rõ về Center. **ONVIF/WS-Discovery** được dùng để quét
camera cùng LAN, đọc media profile, codec/FPS/PTZ và điều khiển ContinuousMove,
Stop hoặc zoom khi thiết bị hỗ trợ.

Đối với camera USB, PTZ đi qua V4L2/UVC; đối với camera mạng, PTZ đi qua ONVIF.
[`camera_ptz.py`](./demo/robot-simulator/simulator/camera_ptz.py) chọn cơ chế dựa
trên source và capability.

### 10.7. Tini và systemd

Container edge dùng **tini** làm PID 1 để chuyển signal và thu gom process con
FFmpeg/GStreamer đúng cách. Repo còn có unit **systemd** để tự khởi động edge và
mode supervisor trên robot Linux; đây là lớp vận hành host bên ngoài Docker.

## 11. ROS 2 và tích hợp phần cứng robot

### 11.1. ROS 2 Humble và rclpy

Các container navigation, motion safety và bridge dùng image
**ROS 2 Humble trên Ubuntu Jammy**. Node riêng được viết bằng **rclpy** và đóng
gói `ament_python`, build bằng **colcon**.

ROS 2 cung cấp:

- topic cho `/scan`, odometry, TF và velocity;
- service cho load/save map và global localization;
- action dài hạn như `ComputePathToPose` và `NavigateToPose`;
- discovery giữa node cảm biến, navigation, safety và chassis.

### 11.2. Fast DDS và `ROS_DOMAIN_ID`

`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` chọn **eProsima Fast DDS** làm middleware
DDS. `ROS_DOMAIN_ID` cô lập robot khỏi ROS graph không liên quan trên cùng mạng.
[`micro_ros_fastdds.xml`](./demo/robot-simulator/micro_ros_fastdds.xml) cấu hình
transport/discovery dùng chung giữa container.

### 11.3. micro-ROS và nền tảng Yahboom

Repo có profile **micro-ROS Agent** để nối thiết bị vi điều khiển qua serial
`/dev/ttyUSB0` ở 921600 baud vào ROS 2 graph. Các profile/guard khóa ownership
thiết bị để tránh hai Agent cùng chiếm serial.

Bridge tương thích stack Yahboom có thể xuất `geometry_msgs/Twist` và phối hợp
joystick cũ. Các mode `ros2-control`, `managed-motion`, `hardware-core` và
`ros2-managed-stack` đều là **opt-in**. Chỉ bật profile không tự làm robot chạy
nếu `MOTION_BACKEND` vẫn giữ giá trị `simulator`.

### 11.4. Unix domain socket giữa edge và ROS

Edge agent không import ROS 2 trực tiếp trong container media. Nó gửi JSON qua
**Unix domain socket** tới motion bridge và navigation adapter. Lợi ích:

- tách dependency Python 3.12 của edge khỏi Python/ROS của Humble;
- tách crash media khỏi ROS graph;
- không cấp Docker socket cho application container;
- cho phép host supervisor chuyển mode Mapping/Navigation an toàn.

## 12. Mapping và quản lý bản đồ

### 12.1. SLAM Toolbox

Trong mode Mapping, **SLAM Toolbox `online_async`** nhận LiDAR, odometry và TF để
tạo occupancy map cùng pose graph. Mapping và Navigation loại trừ nhau: lúc tạo
map không chạy autonomous navigation; khi start mapping, goal Nav2 cũ phải bị
hủy và velocity được đưa về zero.

State machine hiện gồm các trạng thái như `MAPPING_STARTING`,
`MAPPING_RUNNING`, `MAPPING_STOPPED_UNSAVED`, `MAPPING_SAVING`, `FINISHED` và
`MAPPING_ERROR`. Người vận hành lái tay để robot khám phá không gian du lịch rồi
stop/save/discard.

### 12.2. Định dạng map ROS: YAML, PGM/PNG và OccupancyGrid

Một map version gồm tối thiểu:

- `map.yaml`: resolution, origin, threshold và đường dẫn ảnh;
- `map.pgm` hoặc ảnh occupancy tương ứng;
- `metadata.json`: identity, kích thước, checksum và danh sách artifact;
- `preview.png` cho web;
- `posegraph.posegraph` và `posegraph.data` khi cần continue mapping.

**PyYAML** đọc metadata ROS, **Pillow** đọc ảnh occupancy và tạo/kiểm tra preview.
`SavedOccupancyMap` giữ grid đầy đủ để kiểm tra bounds, unknown/occupied và
clearance của goal; không dùng ảnh đã downsample để quyết định an toàn.

### 12.3. Version hóa, checksum và đồng bộ offline

Bundle map được đóng gói `tar.gz`; Center và edge dùng **SHA-256** cho toàn bundle
và từng artifact. Khi tải xuống, edge kiểm tra checksum, map/version trong
metadata, path traversal rồi giải nén vào staging và `os.replace`/atomic rename.

Khi Center mất mạng, robot vẫn có thể save map cục bộ, đánh dấu `SYNC_PENDING`
và retry nền. Chỉ khi checksum Center trả lại trùng bundle local mới chuyển
`SYNCED`. Xóa map dùng tombstone và ACK từ từng robot để không làm map cũ sống
lại sau reconnect.

### 12.4. RViz2

**RViz2** chỉ là công cụ kỹ thuật chạy trên laptop Ubuntu để xem `/scan`, `/map`,
TF, odometry, AMCL, path và costmap qua DDS. Pi không chạy desktop/X11/VNC.
Các cấu hình và script nằm tại [`config/rviz`](./config/rviz) và [`scripts`](./scripts).
Giao diện người dùng cuối vẫn dùng Canvas và telemetry đã giới hạn, không stream
raw ROS topic nặng lên browser.

## 13. Localization và Nav2

### 13.1. robot_localization và TF2

**robot_localization EKF** hợp nhất nguồn odometry đã chuẩn hóa và publish
`odom -> base_footprint`. **robot_state_publisher** cùng **TF2** tạo chuỗi frame:

```text
map -> odom -> base_footprint -> base_link -> laser_frame / imu_frame
```

Sensor normalizer đổi frame/topic vendor về quy ước ROVERA. Khi dùng runtime
Yahboom có sẵn, launch file tránh tạo hai authority EKF/TF và chỉ thêm transform
LiDAR còn thiếu.

### 13.2. AMCL và Map Server

Trong mode Navigation, **Nav2 Map Server** nạp đúng `map.yaml`; adapter chỉ ghi
active map sau khi service load thành công và `/map` được xác minh. **AMCL** dùng
saved occupancy map, LiDAR và TF để ước lượng pose robot.

Luồng tự định vị hiện có nhiều mức:

1. thử last known pose đúng map/version;
2. kiểm tra covariance và chuỗi pose ổn định;
3. nếu không hội tụ, gọi global localization;
4. có thể xoay robot qua safety chain để lấy thêm góc quét;
5. chỉ khi state `READY` mới nhận goal;
6. người dùng chỉ đặt approximate initial pose sau khi tự định vị thất bại.

### 13.3. Nav2

Image navigation cài và launch các thành phần Nav2:

- `nav2_planner` với NavFn để lập global path;
- `nav2_controller` với Regulated Pure Pursuit để bám đường;
- `nav2_bt_navigator` chạy behavior tree;
- `nav2_behaviors` cho recovery;
- `nav2_waypoint_follower`;
- `nav2_lifecycle_manager` quản lý vòng đời node;
- costmap và `nav2_velocity_smoother` hỗ trợ chuyển động ổn định.

Frontend click map mới chỉ chọn goal và gọi `ComputePathToPose`. Nút **Đi đến
đây** mới gọi `NavigateToPose`. Adapter kiểm tra active map/version, localization,
bounds, occupancy, footprint clearance và obstacle trước khi gửi action. Manual
control sẽ cancel navigation và hệ thống không tự resume goal cũ.

## 14. An toàn chuyển động

### 14.1. twist_mux

**twist_mux** phân xử nhiều nguồn velocity. Cấu hình ưu tiên joystick, web và
Nav2 để chỉ nguồn hợp lệ được đưa xuống chuỗi điều khiển. Khi hệ thống chạy lớp
safety đầy đủ, node safety là producer cuối duy nhất của `/cmd_vel` chassis.

### 14.2. Nav2 Velocity Smoother

**Velocity Smoother** giới hạn vận tốc/gia tốc để lệnh đổi không quá giật, phù
hợp robot chở camera trong khu du lịch. STOP khẩn cấp, watchdog và interlock an
toàn không chờ smoothing mà phát zero ngay.

### 14.3. Motion safety node riêng

Package [`motion-safety`](./demo/robot-simulator/motion-safety) tính khoảng dừng
từ vận tốc, latency, gia tốc phanh, footprint và clearance. Nó có thể xử lý:

- E-stop;
- bumper, cliff và range input;
- command watchdog và scan watchdog;
- directional obstacle mask;
- vùng STOP/SLOW từ LiDAR;
- hysteresis sau khi vật cản biến mất.

Hai lưu ý đúng với cấu hình hiện tại:

1. `lidar_obstacle_avoidance_enabled` trong `safety.yaml` đang là **`false`**,
   nên lớp hình học tránh vật cản bằng `/scan` chưa bật mặc định. E-stop,
   watchdog và các topic interlock khác vẫn là cơ chế độc lập.
2. Tài liệu runtime xác nhận Pi hiện chưa có cliff sensor; không được coi hệ
   thống hiện tại là đã chống rơi cầu thang.

Do liên quan chuyển động thật, profile ROS và hardware backend cần field test
với người đứng cạnh E-stop trước khi cho phép motor.

## 15. Docker, Compose và vận hành

### 15.1. Docker và multi-stage build

Mỗi lớp có image riêng:

- backend: `python:3.12-slim`;
- frontend build: `node:22-alpine`, runtime: `nginx:1.27-alpine`;
- edge: multi-stage Go/GStreamer + `python:3.12-slim-bookworm`;
- navigation/safety/bridge: `ros:humble-ros-base-jammy`;
- database/cache/media dùng image được pin trong Compose.

Multi-stage build giúp browser chỉ nhận static bundle, không mang Node toolchain
vào production; edge chỉ copy binary Go đã biên dịch vào runtime.

### 15.2. Docker Compose

Center Compose tạo network bridge `rovera`, healthcheck, dependency order và ba
volume bền vững: PostgreSQL, Redis, map storage. Các service chính là Postgres,
Redis, LiveKit, coturn, FastAPI và Nginx frontend; simulator Center chỉ chạy khi
bật profile `demo`.

Robot dùng `network_mode: host` vì ROS 2 DDS, camera IP, PulseAudio và thiết bị
local cần nhìn network/runtime host. Các profile tách biệt simulator, ROS bridge,
managed hardware, mapping/navigation và safety để tránh tự động chiếm motor hay
serial khi người vận hành chưa chủ động bật.

### 15.3. Healthcheck, restart và log rotation

Compose dùng healthcheck cho Postgres, Redis, LiveKit, FastAPI, Nginx, motion
socket và navigation adapter. `restart: unless-stopped` giữ service chạy sau
reboot; log edge/ROS dùng `json-file` với `max-size` và `max-file` để không lấp
đầy ổ đĩa robot.

## 16. Kiểm thử

### 16.1. Frontend: Vitest, Testing Library và jsdom

**Vitest 3**, **React Testing Library**, `jest-dom` và **jsdom** kiểm thử component,
query/mutation behavior và code không phụ thuộc browser thật. Các test hiện bao
phủ input, quyền, API client, i18n, map viewport, mapping UI, operations shell,
control transport và adaptive video buffer.

### 16.2. Backend/edge: pytest và pytest-asyncio

**pytest 8** và **pytest-asyncio** kiểm thử API, protocol, account, seed, mapping,
navigation, media, PTZ, map registry, motion driver và safety core. Backend test
dùng SQLite tạm; integration test có thể mở WebSocket robot/user để kiểm tra
route command và telemetry.

Pillow và PyYAML nằm trong requirements dev của edge để chạy test map/navigation
thuần Python; image navigation ROS cài package hệ thống tương ứng cho runtime.

### 16.3. Playwright

**Playwright** chạy E2E trên browser theo luồng login → chọn robot → điều khiển →
STOP → xem route/navigation → disconnect. Cấu hình giữ trace, screenshot và
video khi test lỗi.

Repo hiện có lệnh test qua `Makefile` và tài liệu chạy tay. Không thấy workflow
CI được khai báo trong cây mã hiện tại, vì vậy không nên khẳng định test tự động
chạy trên mỗi commit nếu chưa bổ sung pipeline bên ngoài.

## 17. Các luồng nghiệp vụ và công nghệ tham gia

### 17.1. Đăng nhập và mở phiên tham quan

1. React gửi login bằng Fetch/REST tới FastAPI.
2. FastAPI kiểm tra PBKDF2 hash trong PostgreSQL và cấp JWT bằng PyJWT.
3. React Query lấy danh sách robot; Zustand giữ robot đang chọn.
4. FastAPI tạo control session, ghi PostgreSQL và khóa runtime trong hub.
5. Backend trả hai WSS URL cùng LiveKit room/token đúng quyền.

### 17.2. Điều khiển robot

1. Keyboard/on-screen control được chuẩn hóa thành `linear_x`, `angular_z`.
2. Frontend gửi JSON envelope qua control WebSocket.
3. FastAPI kiểm tra JWT, owner session, `client_id`, sequence, timestamp và TTL.
4. Center forward lệnh qua robot WSS, không queue/retry velocity.
5. Edge chọn simulator hoặc Unix socket ROS backend.
6. ROS bridge/twist_mux/velocity smoother/safety chuyển lệnh tới chassis.
7. ACK, pose và health quay về telemetry WebSocket.

### 17.3. Camera và đàm thoại

1. Edge lấy LiveKit token bằng robot JWT.
2. FFmpeg/GStreamer đọc camera USB, RTSP hoặc test source.
3. Robot publish camera/audio vào LiveKit room.
4. Browser subscribe bằng `livekit-client`; microphone operator publish ngược
   về room.
5. WebRTC đi trực tiếp khi có thể, dùng coturn khi NAT/firewall bắt buộc relay.

### 17.4. Tạo và đồng bộ bản đồ

1. React Query mutation gọi FastAPI tạo mapping session có `request_id`.
2. Center gửi command idempotent qua robot WSS.
3. Mode supervisor bật ROS 2 Mapping; SLAM Toolbox tạo occupancy map/pose graph.
4. Adapter dùng PyYAML/Pillow tạo metadata và preview, tính SHA-256 rồi atomic
   save local.
5. Edge dùng HTTPX upload `tar.gz`; FastAPI kiểm tra và lưu vào map volume,
   SQLAlchemy ghi version vào PostgreSQL.
6. Mất Center thì marker `SYNC_PENDING` tồn tại và edge retry sau reconnect.

### 17.5. Tự định vị và đi tới điểm tham quan

1. Browser chọn saved map/version; Center yêu cầu edge tải và activate.
2. Edge xác minh bundle, adapter gọi Nav2 Map Server.
3. AMCL + LiDAR + TF thử last pose rồi global localization/rotation nếu cần.
4. Canvas hiển thị pose và map; click được đổi pixel ↔ tọa độ ROS theo mét.
5. `ComputePathToPose` trả path thật để người dùng xem trước.
6. Sau xác nhận, `NavigateToPose` điều khiển robot qua Nav2 và safety chain.
7. Manual takeover, STOP, E-stop hoặc mất localization đều hủy/dừng chuyển động.

## 18. Hiện trạng cần lưu ý khi đọc kiến trúc

- Production Center dùng PostgreSQL; SQLite chỉ là fallback và database test.
- Redis đang phục vụ LiveKit, chưa thay thế in-memory hub của FastAPI.
- Map binary nằm trên filesystem volume, không nằm trong PostgreSQL và chưa dùng
  object storage.
- Frontend dùng router và i18n tự viết; không có React Router/i18next.
- Map trên web dùng Canvas 2D; chưa dùng một GIS framework như Leaflet.
- `@livekit/components-react` đã khai báo nhưng chưa được import trong mã
  frontend hiện tại.
- Motion và navigation mặc định của edge là simulator; ROS 2/hardware là opt-in.
- Mapping và Navigation ROS chạy loại trừ nhau để không có hai nguồn authority.
- Lớp LiDAR obstacle avoidance của motion-safety đang tắt trong cấu hình mặc
  định; cliff sensor cũng chưa hiện diện.
- WebSocket/session runtime hiện thiết kế cho một Center process, chưa phải kiến
  trúc multi-replica phân tán.
- Repo có test unit/integration/E2E nhưng không thấy pipeline CI trong mã.

Các điểm trên không phủ nhận công nghệ đã tích hợp; chúng xác định chính xác
mức độ **đang áp dụng** để khi triển khai hoặc mở rộng không giả định một khả
năng mà runtime hiện chưa bật.

# REST API

Interactive OpenAPI is available at `/docs`.

- `POST /api/auth/login`
- `POST /api/robot-auth/claim` — edge agent nhận hồ sơ đang chờ bằng địa chỉ,
  tài khoản và mật khẩu robot
- `POST /api/robot-auth/enroll` — đổi enrollment token một lần lấy device
  credential; phương án provisioning nâng cao
- `POST /api/robot-auth/token` — đổi device credential lấy robot JWT ngắn hạn
- `POST /api/robot-auth/media-token?purpose=main|video` — robot JWT lấy token
  đúng room; `video` dùng identity camera riêng để không thay thế kết nối
  audio/subscription `main`
- `GET /api/robots?page=&page_size=&search=&status=` — danh sách phân trang và
  tổng hợp online/offline/pending
- `POST /api/robots` — tạo hồ sơ offline và enrollment token một lần
- `POST /api/robots/quick-add` — luồng mặc định cho operator: IP/hostname,
  tài khoản và mật khẩu
- `GET /api/robots/{robot_id}`
- `PATCH /api/robots/{robot_id}` — sửa metadata hoặc cho phép kết nối
- `DELETE /api/robots/{robot_id}` — chỉ xoá robot đang offline
- `POST /api/robots/{robot_id}/enrollment` — thu hồi credential cũ và tạo liên
  kết ghép nối mới; robot phải offline
- `GET /api/robots/{robot_id}/configuration`
- `PATCH /api/robots/{robot_id}/configuration`
- `GET /api/robots/{robot_id}/media-sources?media_kind=video|audio|speaker` —
  yêu cầu edge agent quét camera V4L2, microphone hoặc loa ALSA/
  PipeWire/PulseAudio ngay trên máy robot. Kết quả chỉ chứa thiết bị đã probe
  thành công; endpoint phát hiện nhưng không hoạt động nằm trong
  `rejected_*_sources`.
- `POST /api/robots/{robot_id}/diagnostics/connection`
- `POST /api/robots/{robot_id}/diagnostics/media`
- `POST /api/robots/{robot_id}/preview-token`
- `POST /api/robots/{robot_id}/connect`
- `POST /api/robots/{robot_id}/disconnect`
- `POST /api/sessions`
- `GET /api/sessions/{session_id}`
- `DELETE /api/sessions/{session_id}`
- `GET /api/maps/{map_id}`
- `GET /api/maps/{map_id}/destinations`
- `POST /api/navigation/preview`
- `POST /api/navigation/goal`
- `POST /api/navigation/cancel`
- `GET /health`

Demo credentials: `demo@rovera.local` / `demo123`.

Các endpoint quản lý robot yêu cầu user JWT. `quick-add` lưu PBKDF2 hash, không
lưu mật khẩu rõ. Endpoint claim chỉ trả device credential cho edge có đủ ba
thông tin khớp; Center sau đó chỉ giữ SHA-256 của credential.

# WebSocket protocol v1.0

All realtime messages use the envelope in
[`message.schema.json`](../src/packages/contracts/message.schema.json).
Maximum accepted message size is 64 KiB.

## Robot gateway

`/ws/robot/connect?robot_id=ROBOT-001`

Header bắt buộc:

```http
Authorization: Bearer <short-lived-robot-jwt>
```

Robot lấy JWT qua `POST /api/robot-auth/token`. Credential dài hạn không nằm
trong URL/query string và chỉ được gửi qua HTTPS. `sub` và `type=robot` trong
JWT phải khớp `robot_id`; token người dùng không thể dùng cho robot gateway.

Robot sends `robot.heartbeat`, `robot.pose`, `robot.health`,
`navigation.status`, and `command.ack`. Center sends `control.velocity`,
`control.stop`, `control.estop.reset`, `camera.ptz`, `navigation.goal`, and
`navigation.cancel`.

## User control

`/ws/user/control/{robot_id}?session_id=...&token=...&client_id=...`

The backend validates the JWT, session ownership, monotonic sequence, timestamp,
TTL and live robot connection before forwarding. Joystick messages are never
queued or retried. `control.stop` is forwarded immediately.

`control.stop` và `control.estop.reset` là hai safety transition đặc biệt:
Center correlate `request_id` và chỉ trả ACK cuối về control socket sau ACK của
Edge. Edge trả `completed` khi odometry mới nằm trong ngưỡng zero liên tục theo
dwell cấu hình; hết timeout trả `unknown`, không suy diễn rằng robot đã đứng
yên. `control.estop.reset` là cách duy nhất nhả software latch; một velocity mới
không tự reset E-stop.

`camera.ptz` chỉ được chuyển từ tab đang giữ quyền điều khiển. Payload dùng
`operation=move|zoom|stop`, hai trục `pan`/`tilt` hoặc `zoom` trong khoảng
`-1..1`, và `speed=slow|medium|fast`. Edge agent tự chọn V4L2/UVC cho camera
USB hoặc ONVIF ContinuousMove/Stop cho nguồn RTSP, dựa trên capability đã dò.

Màn cấu hình gửi `media.onvif.scan` khi người vận hành bấm **Quét ONVIF**.
Edge dùng WS-Discovery trong cùng mạng LAN, đọc toàn bộ media profile rồi trả
`media.onvif.devices` với codec, độ phân giải, FPS, PTZ và RTSP path tương ứng.
Credential của nguồn RTSP hiện tại chỉ được dùng cho đúng hostname đó, không
được thử trên camera khác. Thiết bị cần xác thực trả `auth_required=true`; UI
gửi lại `media.onvif.scan` với `target_host`, `username`, `password` để đọc riêng
thiết bị đó. RTSP URL gửi về Center luôn bỏ user/password; edge chỉ giữ credential
đã xác thực và ghép vào nguồn tương ứng khi người vận hành chọn/lưu profile.
Camera đã được WS-Discovery phát hiện nhưng khóa Media vẫn được trả trong danh
sách với `auth_required=true`. Với Dahua/Hikvision, `suggested_profiles` cung cấp
path RTSP chuẩn của hãng để hiển thị trước; đây là gợi ý, còn codec/FPS/PTZ chính
xác chỉ được xác nhận sau lần đăng nhập riêng cho camera đó.

`client_id` is generated once per page process and is not persisted. The first
tab to claim a session remains its controller; another tab is closed with code
`4009`. A reconnect from the same tab may replace its stale socket. After a
successful claim, the backend sends `control.ready` with the accepted
`client_id`; clients wait for this message before enabling controls.

## User telemetry

`/ws/user/telemetry/{robot_id}?session_id=...&token=...`

The browser receives telemetry and acknowledgements for the selected robot only.
Out-of-order pose messages are ignored client-side by sequence number.

## Reconnect

Clients use exponential backoff with jitter capped at 15 seconds. A reconnect
creates new WebSocket objects and never replays messages. The browser clears
held inputs before reconnecting; the robot watchdog stops motion after 400 ms.

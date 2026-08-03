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
`control.stop`, `navigation.goal`, and `navigation.cancel`.

## User control

`/ws/user/control/{robot_id}?session_id=...&token=...&client_id=...`

The backend validates the JWT, session ownership, monotonic sequence, timestamp,
TTL and live robot connection before forwarding. Joystick messages are never
queued or retried. `control.stop` is forwarded immediately.

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

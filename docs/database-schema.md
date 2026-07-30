# Database schema

SQLAlchemy migrations create:

- `users`: identity, password hash, role, active flag.
- `robots`: stable public `robot_id`, map/site, status, availability,
  capabilities, last heartbeat, trạng thái enabled/enrolled, device
  fingerprint, địa chỉ/tài khoản quản trị, PBKDF2 hash mật khẩu, hash device
  credential và hash/expiry của enrollment token nâng cao.
- `robot_connections`: gateway connection audit.
- `control_sessions`: exclusive controller lock and lifecycle.
- `maps`, `destinations`: map metadata and server-authorized goals.
- `navigation_routes`: generated route snapshots.
- `command_logs`: only important commands, never joystick stream.
- `robot_events`: connection, safety and fault events.

The local demo defaults to SQLite for zero-setup. Docker Compose sets
`DATABASE_URL` to PostgreSQL. Realtime presence and the single-controller lock
are represented by replaceable service interfaces; production should use Redis
leases for multi-instance backend deployment.

Mật khẩu quản trị robot và device credential rõ không được lưu trong Center.
Luồng mặc định lưu PBKDF2 hash của mật khẩu để edge tự claim khi online, sau đó
chỉ SHA-256 của credential ngẫu nhiên được giữ lại. Luồng token nâng cao lưu
hash token cho đến khi dùng. Cấu hình RTSP/camera/micro thuộc edge device,
tránh đưa URL có user/password vào database hoặc browser.

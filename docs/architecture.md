# Kiến trúc Robot Telepresence

## Mục tiêu

Hệ thống giữ cùng một ranh giới tích hợp cho robot mô phỏng và robot thật. Trình
duyệt chỉ biết `robot_id`; mọi lệnh đều đi qua Session Manager và Command
Router. Robot luôn chủ động mở kết nối outbound đến Center Backend.

```mermaid
flowchart LR
  Browser[React Web App] -->|REST + JWT| API[FastAPI Center]
  Browser <-->|Control / Telemetry WS| API
  Browser <-->|WebRTC| LK[LiveKit]
  API --> Registry[Robot Registry]
  API --> Sessions[Session Manager]
  API --> Nav[Map / Navigation]
  API --> PG[(PostgreSQL)]
  API --> Redis[(Redis)]
  Simulator[Robot Simulator] -->|Outbound Robot WS| API
  Simulator <-->|WebRTC| LK
  RealRobot[Orange Pi 5 + ROS 2] -. same protocol .-> API
  RealRobot -. same media room .-> LK
```

## Ranh giới module

- `src/apps/center-frontend`: User Web Application.
- `src/apps/center-backend`: registry, session, command/telemetry gateway, maps,
  navigation và token media.
- `src/packages/contracts`: JSON Schema và TypeScript types độc lập transport.
- `src/packages/map-utils`: phép đổi tọa độ world/pixel/Leaflet.
- `demo/robot-simulator`: process độc lập; không được import vào center.
- `src/infrastructure`: Docker Compose và cấu hình dịch vụ.

`ConnectionHub` trong bản demo là adapter presence/session realtime in-memory.
Các repository SQLAlchemy và cấu hình Redis đã được tách để thay adapter mà
không đổi API hoặc simulator protocol.

## Luồng robot kết nối

```mermaid
sequenceDiagram
  participant U as Operator
  participant R as Robot/Simulator
  participant C as Center Backend
  participant D as Database
  U->>C: POST /api/robots/quick-add (IP + user + password)
  C->>D: robot offline + PBKDF2 password hash
  C-->>U: hồ sơ đang chờ robot
  R->>C: HTTPS address + user + password + fingerprint
  C->>D: kiểm tra hash, lưu hash device credential
  C-->>R: robot_id + credential
  R->>C: HTTPS robot_id + device credential
  C->>D: kiểm tra hash + enabled
  C-->>R: short-lived robot JWT
  R->>C: WSS /ws/robot/connect + Bearer robot JWT
  C->>D: status=online + connection audit
  C->>R: gateway.welcome
  loop heartbeat
    R->>C: robot.heartbeat
    R->>C: robot.pose / robot.health
    C-->>U: telemetry broadcast
  end
  C-->>R: control.velocity / navigation.goal
  R-->>C: command.ack
```

## Luồng cấu hình robot

```mermaid
sequenceDiagram
  participant U as Browser
  participant C as Center Backend
  participant R as Robot/Simulator
  U->>C: GET/PATCH /api/robots/{id}/configuration
  C->>R: configuration.get / configuration.update
  R->>R: đọc hoặc áp dụng cấu hình + nối lại media
  R-->>C: configuration.state
  C-->>U: cấu hình đã ẩn credential
```

Center chỉ chuyển tiếp yêu cầu và chờ phản hồi có `request_id`; cấu hình thuộc
robot/simulator. Robot ngoại tuyến trả HTTP 409, còn yêu cầu quá hạn trả HTTP
504.

## Luồng người dùng và WebRTC

```mermaid
sequenceDiagram
  participant U as Browser
  participant C as Center Backend
  participant L as LiveKit
  participant R as Robot
  R->>C: robot JWT → scoped publisher token
  C-->>R: LiveKit URL + room-scoped token
  R->>L: join room robot-{robot_id}
  U->>C: POST /api/sessions {robot_id}
  C-->>U: session_id + scoped media token + WS URLs
  U->>C: Control WS + Telemetry WS
  U->>L: join room robot-{robot_id}
  R->>L: publish camera + microphone
  L-->>U: subscribed video/audio
  U->>L: publish microphone after user consent
  L-->>R: subscribed user audio
```

## Luồng teleoperation

```mermaid
sequenceDiagram
  participant I as InputManager
  participant C as Center
  participant R as Robot
  I->>C: control.velocity seq=N ttl=300ms
  C->>C: validate session/sequence/TTL/rate
  C->>R: forward immediately
  R->>R: deduplicate + watchdog
  R-->>C: command.ack accepted
  C-->>I: command.ack
  Note over I,R: keyup/blur/hidden/disconnect => STOP and clear state
```

## Luồng navigation

```mermaid
sequenceDiagram
  participant U as Browser
  participant C as Center
  participant R as Robot
  U->>C: POST /api/navigation/preview
  C-->>U: validated route points + distance
  U->>C: POST /api/navigation/goal
  C->>R: navigation.goal with route
  R-->>C: accepted/planning/moving
  loop route
    R-->>C: robot.pose
    C-->>U: robot.pose
  end
  R-->>C: navigation.status arrived
```

## State machines

| Domain | States |
| --- | --- |
| Robot connection | `idle → selecting → connecting → connected → reconnecting → disconnecting`; terminal/exception: `offline`, `error` |
| Media | `idle → requesting_token → connecting → connected`; exception: `reconnecting`, `no_video`, `no_audio`, `permission_denied`, `failed` |
| Control | `disabled → ready → active → stopping`; exception: `expired`, `session_lost`, `robot_offline` |
| Navigation | `idle → previewing → route_ready → sending_goal → moving → arrived`; exception: `cancelled`, `failed` |

## Lộ trình triển khai

1. Infrastructure, persistence schema, backend skeleton.
2. Robot Registry, gateway, heartbeat/presence.
3. Session lock, command/telemetry WebSocket, simulator motion.
4. LiveKit token/media adapters.
5. Safe input manager và teleoperation UI.
6. Map, destination, preview và navigation simulator.
7. Tests, recovery, observability và ROS 2 integration.

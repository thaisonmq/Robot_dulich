#!/usr/bin/env python3
"""Generate the implementation checklist for the Robot Telepresence project."""

from __future__ import annotations

import json
from pathlib import Path

import xlsxwriter


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "KeHoach_DauViec_Robot_Telepresence.xlsx"


def compact_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2)


def task(
    task_id: str,
    package: str,
    module: str,
    work: str,
    dod: str,
    current: str,
    gap: str,
    priority: str,
    order: str,
    dependencies: str,
    contract: str,
    evidence: str,
    team: str,
    size: str,
    source: str,
    done: bool = False,
    note: str = "",
) -> dict[str, object]:
    return {
        "done": done,
        "id": task_id,
        "package": package,
        "module": module,
        "work": work,
        "dod": dod,
        "current": current,
        "gap": gap,
        "priority": priority,
        "order": order,
        "dependencies": dependencies,
        "contract": contract,
        "evidence": evidence,
        "team": team,
        "size": size,
        "source": source,
        "note": note,
    }


TASKS = [
    task("ARCH-01", "WP1 - Kiến trúc", "Dùng chung", "Chốt ranh giới 4 vùng và sơ đồ triển khai production", "Có sơ đồ logical/deployment, cổng mạng, trust boundary và owner từng service được duyệt.", "Đã có kiến trúc logical và sequence cho demo.", "Bổ sung deployment topology production, HA, reverse proxy TLS, vùng mạng robot/center/admin.", "Must", "GĐ1", "Hạ tầng, Security", "HTTPS/WSS/WebRTC/TURN/Tailscale", "docs/architecture.md; docs/Module_HeThong.docx", "Kiến trúc", "M", "Module hệ thống I-II"),
    task("ARCH-02", "WP1 - Kiến trúc", "Contracts", "Chuẩn hóa envelope realtime v1.0", "Mọi message có message_id, schema_version, robot_id, session_id, sequence, timestamp, ttl_ms; contract test chạy qua CI.", "Đã có JSON Schema và Pydantic/TypeScript/Python model tương ứng.", "Mở rộng message types production và chốt quy tắc tương thích ngược.", "Must", "GĐ1", "R02, C05, U02", "src/packages/contracts/message.schema.json", "src/packages/contracts/message.schema.json; app/schemas/messages.py; simulator/messages.py", "Backend + Edge", "S", "Module hệ thống IX", True),
    task("ARCH-03", "WP1 - Kiến trúc", "Contracts", "Thống nhất tên trường vận tốc giữa tài liệu và mã", "Chỉ còn một contract chính thức; adapter cũ có version/migration rõ.", "Mã hiện tại dùng linear_x/angular_z; tài liệu mockup dùng vx/wz.", "Chốt canonical payload, cập nhật schema, docs, simulator, frontend và test.", "Must", "GĐ1", "ARCH-02, R03", "control.velocity payload", "src/apps/center-frontend/src/transports/ControlTransport.ts; docs/websocket-protocol.md", "Backend + Edge + Frontend", "S", "Hai tài liệu + mã hiện tại", False, "Điểm lệch contract cần xử lý trước khi nối ROS 2."),
    task("ARCH-04", "WP1 - Kiến trúc", "Contracts", "Chuẩn hóa error_code và correlation_id", "REST/WS trả mã AUTH_*, COMMAND_*, ROBOT_*, NAV_*, SAFETY_*, MEDIA_*, AI_* ổn định; log truy được theo correlation_id.", "Hiện chủ yếu dùng HTTP detail tiếng Việt và trạng thái ACK đơn giản.", "Tạo error schema, middleware correlation, mapping lỗi và contract tests.", "Must", "GĐ1", "C01, C05, C12", "ErrorResponse; command.ack", "src/apps/center-backend/app/api", "Backend", "M", "Module hệ thống IX.4"),
    task("ARCH-05", "WP1 - Kiến trúc", "Dev workflow", "Thiết lập CI cho backend, frontend, simulator và E2E", "PR chạy lint/typecheck/unit/integration; E2E chạy trên Compose; lưu artifact và coverage.", "Có test và lệnh chạy nhưng chưa có pipeline CI trong repo.", "Tạo workflow CI, cache dependency, quality gate và báo cáo coverage.", "Must", "GĐ1", "Test suite hiện có", "pytest; vitest; playwright; docker compose", "docs/testing.md; demo/e2e", "DevOps", "M", "Module hệ thống XI"),
    task("ARCH-06", "WP1 - Kiến trúc", "Dữ liệu mẫu", "Hoàn thiện bộ simulator/mock cho mọi tích hợp chưa có", "Có MCU/base, ROS 2/Nav2, translation, Vision/RAG, safety, BMS mock với lỗi/timeout cấu hình được.", "Đã có robot simulator cho motion/navigation/media; thiếu các service AI và phần cứng.", "Bổ sung mock server/nodes và fixture cho happy path + fault injection.", "Must", "GĐ1-GĐ6", "ARCH-02", "Các payload tại sheet Tích hợp & dữ liệu", "demo/robot-simulator; sample-data", "QA + các nhóm module", "L", "Module hệ thống X.2"),
    task("ARCH-07", "WP1 - Kiến trúc", "Security", "Quản lý secret và cấu hình theo môi trường", "Không có secret mặc định yếu ở production; secret rotate được; file edge 0600; có checklist bàn giao.", "Đã tách .env, edge state 0600 và không đưa secret server xuống robot.", "Bổ sung vault/KMS, rotation, production validation và secret scan CI.", "Must", "GĐ3", "C02, R02", "JWT/LIVEKIT/device credential/TURN credential", ".env.example; simulator/client.py", "Security + DevOps", "M", "Module hệ thống R02/C02"),
    task("R01-01", "WP2 - Robot & Safety", "R01", "Chuẩn hóa systemd/bringup trên Orange Pi 5", "Các service khởi động đúng thứ tự sau reboot/mất điện; Restart/watchdog/startup timeout được kiểm thử.", "Có unit systemd cho edge simulator container.", "Tách unit cho robot-agent, ROS 2, media, audio, display, safety; dependency và safe boot.", "Must", "GĐ1", "Orange Pi image", "systemd unit + health readiness", "demo/robot-simulator/deploy/rovera-robot.service", "Edge/DevOps", "M", "Module hệ thống R01"),
    task("R01-02", "WP2 - Robot & Safety", "R01", "Health tài nguyên và thiết bị", "Gửi CPU/RAM/nhiệt/disk/uptime và health camera/LiDAR/MCU/network; READY bị chặn khi safety dependency lỗi.", "robot.health hiện có pin/mạng/camera/audio/navigation mô phỏng.", "Đọc metrics Linux và diagnostic ROS 2 thật; định nghĩa ngưỡng degraded/fault.", "Must", "GĐ1-GĐ2", "R06, R07, R09, R13", "robot.health", "demo/robot-simulator/simulator/client.py; app/services/hub.py", "Edge", "M", "Module hệ thống R01"),
    task("R01-03", "WP2 - Robot & Safety", "R01", "Log rotation và giới hạn tài nguyên", "Disk không đầy sau soak test; media/AI bị giới hạn CPU/RAM/IO và không làm trễ safety.", "Docker restart đã có; chưa có policy resource/log production.", "Cấu hình journald/logrotate/cgroup, quota và cảnh báo dung lượng.", "Must", "GĐ1", "C12", "system metrics + service events", "src/infrastructure/docker-compose.yml", "Edge/DevOps", "S", "Module hệ thống R01"),
    task("R02-01", "WP2 - Robot & Safety", "R02", "Credential riêng, claim/enroll và JWT ngắn hạn", "Mỗi robot có credential riêng, state 0600, JWT 15 phút; robot khác không dùng chéo credential.", "Đã có quick-add, claim/enroll, hash credential và robot JWT.", "Bổ sung rotate/revoke production, audit và secure element/TPM nếu có.", "Must", "GĐ3", "C02, C03", "/api/robot-auth/claim|token|enroll", "app/api/robot_auth.py; simulator/client.py; tests/test_integration.py", "Backend + Edge", "M", "Module hệ thống R02/C03", True),
    task("R02-02", "WP2 - Robot & Safety", "R02", "Outbound WSS, heartbeat và reconnect không replay", "Mất mạng không block ROS; reconnect backoff+jitter ≤15s; không phát lại joystick cũ.", "Simulator đã thực hiện WSS outbound, heartbeat, backoff và dừng motion khi ngắt.", "Port sang Robot Agent Orange Pi thật và test chuyển mạng Wi-Fi/4G/5G.", "Must", "GĐ3", "C05, R03, R07", "robot.heartbeat; gateway.welcome", "demo/robot-simulator/simulator/client.py", "Edge", "M", "Module hệ thống R02", True, "Đã hoàn thành ở simulator, chưa phải edge ROS 2 production."),
    task("R02-03", "WP2 - Robot & Safety", "R02", "Idempotency và validation command tại robot", "Loại message_id trùng; kiểm schema/version/session/sequence/timestamp/TTL trước khi gọi ROS.", "Simulator có model, expired(), processed_ids và sequence.", "Thêm giới hạn bộ nhớ/TTL cho dedupe, capability validation và error_code.", "Must", "GĐ3", "ARCH-02, R03", "command envelope", "simulator/messages.py; simulator/client.py", "Edge", "M", "Module hệ thống R02"),
    task("R02-04", "WP2 - Robot & Safety", "R02", "Telemetry buffer có giới hạn khi offline", "Chỉ buffer telemetry quan trọng với quota; joystick không queue; flush theo sampling sau reconnect.", "Không thấy hàng đợi telemetry offline; dữ liệu mất khi ngắt là hành vi demo.", "Thiết kế ring buffer, retention và backpressure không ảnh hưởng control.", "Should", "GĐ3", "C06", "robot.health/pose/event", "demo/robot-simulator/simulator/client.py", "Edge", "M", "Module hệ thống R02"),
    task("R03-01", "WP2 - Robot & Safety", "R03", "Tạo ROS 2 Command Gateway", "Node nhận command đã xác thực và publish topic/service/action; không có đường network xuống motor trực tiếp.", "Chưa có ROS 2 workspace; simulator xử lý lệnh nội bộ.", "Tạo package, lifecycle node, command registry và launch file.", "Must", "GĐ1-GĐ3", "R02, R04, R05, R07", "control.velocity/stop; navigation.goal/cancel", "docs/integration-robot.md", "ROS 2", "L", "Module hệ thống R03"),
    task("R03-02", "WP2 - Robot & Safety", "R03", "Map teleop sang geometry_msgs/Twist qua twist_mux", "linear/angular được clamp; teleop/autonomy arbitration đúng priority; STOP ra zero ngay.", "Chỉ có MotionSimulator.", "Viết adapter ROS 2, QoS, rate/deadband/acceleration limit và unit test mock node.", "Must", "GĐ1", "R03-01, R04, R07", "/teleop/cmd_vel; geometry_msgs/Twist", "demo/robot-simulator/simulator/motion.py", "ROS 2", "M", "Module hệ thống R03-R04"),
    task("R03-03", "WP2 - Robot & Safety", "R03", "Map mission sang Nav2 action và lifecycle status", "ACK tách khỏi accepted/running/completed/failed/cancelled; timeout/cancel có mã lỗi ổn định.", "Navigation simulator trả trạng thái cơ bản.", "Tích hợp NavigateToPose/waypoint action, feedback, cancel, recovery và error mapping.", "Must", "GĐ2", "R05, C05", "navigation.goal/status/cancel", "demo/robot-simulator/simulator/navigation.py", "ROS 2", "L", "Module hệ thống R03/R05"),
    task("R04-01", "WP2 - Robot & Safety", "R04", "Input joystick/keyboard 10-20 Hz, dead-man và smoothing", "Giữ phím gửi 20 Hz; nhả/blur/hidden/unmount dừng; ramp không gây giật.", "Frontend đã có input manager, 20 Hz, ramp 220 ms và test keyboard.", "Đo trên robot thật, thêm joystick/gamepad theo phạm vi và hiệu chỉnh limit.", "Must", "GĐ3-GĐ5", "R03-02", "control.velocity ttl=300ms", "useTeleoperation.ts; utils/input.ts; tests/input.test.ts", "Frontend + ROS 2", "M", "Module hệ thống R04", True, "Hoàn thành phía web/simulator."),
    task("R04-02", "WP2 - Robot & Safety", "R04", "Watchdog dừng khi mất lệnh", "Robot phát zero/hardware stop trong ≤500 ms khi không có command/heartbeat.", "Simulator watchdog 400 ms và server gửi stop khi WS/session đóng.", "Triển khai watchdog độc lập trong ROS 2 + MCU và đo thời gian thực.", "Must", "GĐ1", "R07, R08", "control.stop; watchdog heartbeat", "simulator/motion.py; api/websockets.py", "ROS 2 + Firmware", "M", "AT-03"),
    task("R04-03", "WP2 - Robot & Safety", "R04", "Fault test packet loss/delay/reorder/duplicate/reconnect", "Có báo cáo đo latency, stop time; không vượt speed limit và không replay.", "Có unit test TTL/sequence và reconnect logic; chưa có network fault suite.", "Dùng tc/netem hoặc proxy fault, tự động hóa test và lưu metric.", "Must", "GĐ3", "C05, R07", "sequence/timestamp/ttl_ms", "tests/test_protocol.py; docs/testing.md", "QA + Edge", "M", "Module hệ thống R04/X"),
    task("R05-01", "WP2 - Robot & Safety", "R05", "Dựng bản đồ thật, localization và Nav2", "Robot đi đến waypoint với sai số đã chốt, có map/version và pose ổn định.", "Map SVG/JSON mẫu và route Manhattan; chưa có Nav2.", "Khảo sát site, SLAM, AMCL/localization, costmap, planner/controller/BT.", "Must", "GĐ2", "R06, C11", "map package; NavigateToPose", "sample-data/maps/map-001.json; app/services/maps.py", "ROS 2 + Field", "XL", "Module hệ thống R05"),
    task("R05-02", "WP2 - Robot & Safety", "R05", "Waypoint, no-go, speed zone và validation", "Không nhận goal ngoài vùng; robot/center dùng cùng map version; zone giới hạn tốc độ có hiệu lực.", "Có destination whitelist mẫu nhưng chưa có polygon/no-go/version handshake.", "Mở rộng schema map/location/zone, publish package và validate hai phía.", "Must", "GĐ2", "C11, R07", "location_id/map_version/zones", "sample-data/maps/map-001.json", "Backend + ROS 2", "L", "Module hệ thống R05/C11"),
    task("R05-03", "WP2 - Robot & Safety", "R05", "Pause/resume/cancel/recovery mission", "Cancel phản hồi nhanh; stuck/blocked có recovery giới hạn; trạng thái phản ánh robot thật.", "Có goal/cancel và simulator status; chưa có pause/resume/recovery.", "Bổ sung protocol và Nav2 behavior tree/recovery policy.", "Must", "GĐ2", "R03-03, C05", "navigation.pause/resume/status", "app/api/navigation.py; simulator/navigation.py", "ROS 2 + Backend", "L", "Module hệ thống R05"),
    task("R06-01", "WP2 - Robot & Safety", "R06", "Tích hợp driver encoder/IMU/LiDAR/cliff/depth", "Topic đúng timestamp/frame; sensor timeout/outlier được diagnostic và đưa sang safety.", "Không có driver/ROS 2 sensor trong repo.", "Chọn thiết bị, viết/tích hợp driver, QoS và diagnostic aggregator.", "Must", "GĐ2", "R08, R07", "/scan /imu /odom diagnostics", "Chưa có", "ROS 2 + Hardware", "XL", "Module hệ thống R06"),
    task("R06-02", "WP2 - Robot & Safety", "R06", "Calibration, TF tree và sensor fusion", "IMU/wheel separation/radius được hiệu chỉnh; TF hợp lệ; localization không trôi quá ngưỡng.", "Chưa có.", "Cấu hình robot_localization, clock sync, calibration procedure và regression bag.", "Must", "GĐ2", "R06-01", "tf/odom/map/base_link", "Chưa có", "ROS 2", "L", "Module hệ thống R06"),
    task("R07-01", "WP2 - Robot & Safety", "R07", "Thiết kế Safety priority matrix", "E-stop > collision/cliff > watchdog > teleop > navigation; mọi trạng thái và transition được test.", "Có stop khi disconnect/watchdog ở simulator, chưa có Safety Manager.", "Viết safety state machine, reason code và tài liệu hazard/priority.", "Must", "GĐ1", "R04, R05, R06, R08, R13", "safety.state/event", "simulator/motion.py; docs/Module_HeThong.docx", "Safety + ROS 2", "L", "Module hệ thống R07"),
    task("R07-02", "WP2 - Robot & Safety", "R07", "Collision Monitor và vùng STOP/SLOW", "Vật cản/cliff gây slow/stop trong giới hạn; ứng dụng không bypass được.", "Chưa có.", "Cấu hình Nav2 Collision Monitor, sensor zones và safe_cmd_vel output.", "Must", "GĐ1-GĐ2", "R06, R07-01", "cmd_vel_in → safe_cmd_vel", "Chưa có", "Safety + ROS 2", "L", "Module hệ thống R07"),
    task("R07-03", "WP2 - Robot & Safety", "R07", "So sánh commanded/actual velocity và phát fault", "Kẹt bánh/motor runaway/encoder stale được phát hiện, stop và log nguyên nhân.", "Chưa có.", "Tạo monitor, threshold theo tốc độ, debounce và fault injection.", "Must", "GĐ1", "R08, R06", "motor feedback + safety event", "Chưa có", "Safety + Firmware", "M", "Module hệ thống R07"),
    task("R07-04", "WP2 - Robot & Safety", "R07", "E-stop vật lý và UI end-to-end", "E-stop vật lý cắt công suất độc lập Linux; UI stop/physical stop hiển thị và audit; reset có điều kiện.", "UI chỉ có ngắt kết nối/STOP theo input; chưa có E-stop thật hoặc safety event.", "Thiết kế mạch, firmware input, ROS state, API/event và nút UI cố định.", "Must", "GĐ1-GĐ5", "R08, C12, U02", "control.stop; safety.estop", "DashboardPage.tsx", "Hardware + Firmware + Full stack", "XL", "AT-04"),
    task("R08-01", "WP2 - Robot & Safety", "R08", "Thiết kế protocol CAN/UART Orange Pi-MCU", "Frame có version, sequence, CRC, ACK, timeout 200-500 ms; fuzz/duplicate/CRC test đạt.", "Chưa có firmware/protocol.", "Chốt transport, frame layout, units, endian, error codes và simulator.", "Must", "GĐ1", "R03, R07", "SET_VELOCITY/STOP/STATUS/FAULT", "Sheet Tích hợp & dữ liệu", "Firmware + ROS 2", "L", "Module hệ thống R08"),
    task("R08-02", "WP2 - Robot & Safety", "R08", "Firmware motor PID/encoder/brake/watchdog", "Rút cáp/treo Orange Pi robot dừng; CRC sai/lệnh cũ không chạy; motor feedback ổn định.", "Chưa có.", "Lập state machine, PID, current/speed limit, hardware watchdog và HIL test.", "Must", "GĐ1", "R08-01, phần cứng", "MCU status telemetry", "Chưa có", "Firmware", "XL", "Module hệ thống R08"),
    task("R08-03", "WP2 - Robot & Safety", "R08", "Bootloader/update có recovery", "Update lỗi/mất điện không brick MCU; có rollback hoặc recovery procedure.", "Chưa có.", "Chọn bootloader, ký image/checksum và test failure mode.", "Should", "GĐ7", "C13", "firmware manifest", "Chưa có", "Firmware", "L", "Module hệ thống R08/C13"),
    task("R09-01", "WP3 - WebRTC", "R09", "Capture file/RTSP/USB và publish LiveKit", "Nguồn media chọn được trên edge; reconnect khi nguồn lỗi; browser nhận video.", "Đã có FFmpeg adapters file/RTSP/V4L2, UI cấu hình/preview và LiveKit publish.", "Xác minh trên Orange Pi/camera thật; bổ sung CSI nếu dùng.", "Must", "GĐ4", "C07, U02", "LiveKit track camera", "simulator/media.py; RobotConfigurationPage.tsx", "Media/Edge", "M", "Module hệ thống R09", True),
    task("R09-02", "WP3 - WebRTC", "R09", "Hardware encode H.264 Rockchip MPP/GStreamer", "1080p/25fps không dùng software encode quá tải; CPU/nhiệt đạt ngưỡng trong soak test.", "Đã có đường H.264 depay/parse trực tiếp không decode/re-encode; nguồn khác tự probe RKMPP/VA-API/NVENC/V4L2M2M trước software. Chưa benchmark CPU/nhiệt trên Orange Pi thật.", "Chạy soak test 1080p/25fps trên Orange Pi 5, xác nhận h264_rkmpp và ghi CPU/GPU/nhiệt.", "Must", "GĐ4", "Orange Pi camera driver", "H.264/WebRTC", "simulator/media.py; Dockerfile", "Media/Edge", "M", "Module hệ thống R09"),
    task("R09-03", "WP3 - WebRTC", "R09", "Frozen-frame detection, adaptive bitrate và reconnect", "Camera treo được phát hiện; bitrate giảm khi loss; media tự phục hồi khi chuyển mạng.", "Edge có frame timeout/reconnect; browser có frame watchdog/recovery; bitrate cấu hình tĩnh.", "Nối WebRTC stats/packet loss vào adaptation và robot.health.", "Must", "GĐ4", "C06, C07", "WebRTC stats; robot.health", "simulator/media.py; MediaTransport.ts", "Media + Frontend", "M", "Module hệ thống R09"),
    task("R09-04", "WP3 - WebRTC", "R09", "Snapshot/PTZ và lưu ảnh theo session", "PTZ có limit/ACK; snapshot gắn session/time, object key, signed URL và retention.", "Chưa có message type/API/storage; camera config chỉ chọn nguồn.", "Thiết kế camera.ptz/photo.capture, object storage, photos API và UI.", "Should", "GĐ4-GĐ5", "C05, C09, U02", "camera.ptz; photo.capture; /sessions/{id}/photos", "Chưa có", "Full stack + Media", "L", "Mockup AT-09"),
    task("R09-05", "WP3 - WebRTC", "R09", "Soak test video 72 giờ qua Wi-Fi/4G/5G/NAT", "Không leak/restart ngoài policy; latency/bitrate/reconnect/temperature có báo cáo.", "Có unit media, chưa có soak/field report.", "Tạo test harness, dashboard metric và kịch bản nhiều nhà mạng.", "Must", "GĐ4", "C12", "WebRTC stats", "docs/testing.md", "QA + Media", "L", "Module hệ thống R09/X"),
    task("R10-01", "WP5 - Audio DSP", "R10", "Chọn mic array/audio interface/loa và bố trí cơ khí", "Có BOM, playback reference AEC, clock ổn định, không clipping ở khoảng cách thiết kế.", "Simulator hỗ trợ audio source/silent track; chưa có phần cứng audio.", "Đánh giá 4-6 mic, sound card, amp/loa, giảm rung/gió và đo delay.", "Must", "GĐ4", "Hardware", "PCM capture/playback + AEC reference", "Robot_Telepresence...docx VI", "Audio + Hardware", "L", "Mockup VI"),
    task("R10-02", "WP5 - Audio DSP", "R10", "Pipeline HPF/beamforming/AEC/NS/AGC/VAD", "Không echo loop, WER đạt mục tiêu 4 môi trường; audio DSP 20-60 ms điển hình.", "LiveKit audio hai chiều cơ bản; không có DSP/VAD.", "Tích hợp WebRTC APM/RNNoise/vendor DSP, tuning và metrics.", "Must", "GĐ4", "R10-01, C08", "PCM 16/48kHz, VAD events", "simulator/media.py", "Audio/Edge", "XL", "Mockup VI"),
    task("R10-03", "WP5 - Audio DSP", "R10", "Direct/full-duplex/PTT/conversation mode", "Chế độ rõ trên UI; barge-in hoạt động; có PTT fallback khi ồn/mạng xấu.", "Browser có bật/tắt mic và loa; chưa có mode/PTT/barge-in.", "Bổ sung state machine, signaling mode, giảm tốc/dừng conversation mode.", "Must", "GĐ4-GĐ6", "R07, C08, U04", "audio.mode; vad.start/end; tts.cancel", "DashboardPage.tsx", "Audio + Frontend", "L", "Mockup VI.3"),
    task("R11-01", "WP4 - Web UI", "R11", "Xây robot display kiosk", "Tự khởi động/phục hồi; hiển thị video khách lớn, phụ đề 1-2 dòng, trạng thái camera/mic/privacy.", "Không có app robot-display.", "Tạo ứng dụng kiosk, local health, session states và mất mạng/end session UX.", "Must", "GĐ5", "R01, R10, C08", "session.state; translation.final", "Chưa có", "Frontend Edge", "L", "Module hệ thống R11/Mockup Hình 6"),
    task("R12-01", "WP6 - AI", "R12", "Benchmark AI nhẹ trên RK3588S/RKNN", "Model detection/VAD chạy process riêng, có FPS/CPU/NPU/nhiệt; tự hạ cấp khi quá tải.", "Chưa có.", "Chọn model, convert RKNN, resource limit và overload test với Nav2/Safety.", "Could", "GĐ6", "R09, R07", "edge.ai.detection/health", "Chưa có", "Edge AI", "L", "Module hệ thống R12"),
    task("R13-01", "WP2 - Robot & Safety", "R13", "Tích hợp BMS và power telemetry", "Đọc SOC/voltage/current/temp/charging/fault; mất dữ liệu kích hoạt policy.", "Battery hiện là giá trị mô phỏng.", "Chốt BMS protocol, driver, calibration và robot.health mapping.", "Must", "GĐ1-GĐ2", "R07, C06", "battery telemetry", "app/services/hub.py; sample-data/robots/robots.json", "Hardware + ROS 2", "L", "Module hệ thống R13"),
    task("R13-02", "WP2 - Robot & Safety", "R13", "Ngưỡng warn/critical/shutdown và power integrity", "Orange Pi không reset khi motor/modem tải; không nhận tour khi pin thiếu; shutdown an toàn.", "Chưa có policy thật.", "Thiết kế DC-DC, đo sụt áp, state machine nguồn và test tải xấu nhất.", "Must", "GĐ1-GĐ2", "R07, R05", "power.state/event", "Chưa có", "Hardware + Safety", "XL", "Module hệ thống R13"),
    task("C01-01", "WP1 - Kiến trúc", "C01", "Reverse proxy/API Gateway TLS production", "Chỉ gateway public; HTTPS/WSS, security headers, request size, timeout và health route đúng.", "FastAPI/CORS và nginx frontend; chưa có reverse proxy TLS production.", "Thêm gateway config, certificate automation và environment hardening.", "Must", "GĐ3", "DevOps", "HTTPS/WSS", "src/apps/center-frontend/nginx.conf; app/main.py", "Backend + DevOps", "M", "Module hệ thống C01"),
    task("C01-02", "WP1 - Kiến trúc", "C01", "Rate limit/WAF/basic protection", "Rate limit theo user/robot/IP; claim/login/WS handshake chống brute force; có audit.", "Chưa có rate limit/WAF.", "Tích hợp Redis limiter, reverse proxy rule và test abuse.", "Must", "GĐ3", "C02, Redis", "429/ErrorResponse", "app/api/auth.py; app/api/robot_auth.py", "Security + Backend", "M", "Module hệ thống C01"),
    task("C02-01", "WP7 - Vận hành", "C02", "Auth người dùng production, role/permission matrix", "Guest/operator/support/admin có quyền tách biệt; viewer không gửi command.", "Login demo, JWT và cột role đã có; endpoint chưa kiểm role chi tiết.", "CRUD user, password hash production, RBAC dependency và permission tests.", "Must", "GĐ3-GĐ7", "C01, C04", "JWT claims role/permissions", "app/api/auth.py; models/entities.py", "Backend + Security", "L", "Module hệ thống C02"),
    task("C02-02", "WP7 - Vận hành", "C02", "Refresh/revoke/MFA cho tài khoản nhạy cảm", "Token thu hồi không dùng lại; MFA cho admin/kỹ thuật; session browser an toàn.", "Chỉ access token; không refresh/revoke/MFA.", "Thiết kế refresh rotation, revocation store và MFA flow.", "Should", "GĐ7", "Redis/KMS", "auth token lifecycle", "app/core/security.py", "Backend + Security", "L", "Module hệ thống C02"),
    task("C03-01", "WP1 - Kiến trúc", "C03", "Robot registry CRUD, claim và trạng thái online/offline", "Thêm/sửa/xóa offline, filter/paging, credential claim, connection audit hoạt động.", "Đã có API/UI/test đầy đủ cho demo.", "Bổ sung RBAC/audit chi tiết và concurrency production.", "Must", "GĐ3", "C02", "robots API + robot-auth", "app/api/robots.py; RobotListPage.tsx; tests/test_integration.py", "Backend + Frontend", "M", "Module hệ thống C03", True),
    task("C03-02", "WP7 - Vận hành", "C03", "Versioned configuration rollout và maintenance lock", "Config có version/audit/rollback; robot fault/maintenance không được cấp session.", "Config thuộc edge và update qua request/response, chưa version/audit/rollout.", "Thêm config schema/version, desired/reported state, validation và lock readiness.", "Must", "GĐ7", "R02, C04, C12", "configuration.get/update/state", "app/api/robots.py; simulator/client.py", "Backend + Edge", "L", "Module hệ thống C03"),
    task("C04-01", "WP1 - Kiến trúc", "C04", "Phiên độc quyền và lifecycle cơ bản", "Một robot chỉ có một controller; end session gửi STOP và giải phóng robot.", "Hub in-memory đã khóa một controller; create/get/delete session và media lease.", "Đưa state/lease sang Redis/DB, phục hồi khi nhiều instance hoặc restart.", "Must", "GĐ3", "Redis, C05", "POST/GET/DELETE /api/sessions", "app/api/sessions.py; app/services/hub.py", "Backend", "L", "Module hệ thống C04"),
    task("C04-02", "WP1 - Kiến trúc", "C04", "Control lease heartbeat/grace/takeover", "Mất heartbeat thu hồi lease và robot dừng; support takeover có audit/thông báo.", "Control WS disconnect đóng session; chưa có control.heartbeat/lease/takeover riêng.", "Thêm lease API/event, timeout monitor, role check, takeover workflow.", "Must", "GĐ3-GĐ7", "C02, C05, U02", "control.heartbeat; session.state; control-lease API", "app/api/websockets.py", "Backend + Frontend", "L", "Mockup API/AT-11"),
    task("C05-01", "WP1 - Kiến trúc", "C05", "Validate session/sequence/timestamp/TTL và route đúng robot", "Lệnh sai/hết hạn/reorder bị từ chối; joystick không queue/retry.", "Đã có WebSocket validation, selected robot routing và ACK.", "Thêm rate limit theo loại, schema payload chặt và error codes.", "Must", "GĐ3", "ARCH-02, C04", "control.velocity/stop; command.ack", "app/api/websockets.py; tests/test_integration.py", "Backend", "M", "Module hệ thống C05", True),
    task("C05-02", "WP1 - Kiến trúc", "C05", "Business command lifecycle/idempotency/ACK timeout", "Mission/PTZ/config có accepted→running→terminal; retry có giới hạn; dedupe store TTL.", "Navigation có accepted/status nhưng chưa lifecycle store/ACK timeout/retry policy.", "Tạo command registry, persistence, timeout worker và metrics.", "Must", "GĐ3", "R02, R03", "command.ack/status/error", "app/api/navigation.py; app/services/hub.py", "Backend", "L", "Module hệ thống C05"),
    task("C05-03", "WP7 - Vận hành", "C05", "Audit important command, không lưu joystick stream", "STOP/E-stop/takeover/config/mission có actor/message_id/reason; log chống sửa.", "Có bảng command_logs nhưng chưa thấy ghi log vào command path.", "Tích hợp repository/audit middleware và retention.", "Must", "GĐ7", "C12", "command audit record", "models/entities.py", "Backend", "M", "Module hệ thống C05/C12"),
    task("C06-01", "WP7 - Vận hành", "C06", "Realtime telemetry không chặn command", "Pose/health/status phát đúng robot, out-of-order bị bỏ, sampling có giới hạn.", "Đã broadcast in-memory và frontend bỏ pose out-of-order.", "Thêm throttling/backpressure, schema per field và multi-instance pub/sub.", "Must", "GĐ3-GĐ7", "Redis, R02", "robot.pose/health/navigation.status", "app/api/websockets.py; TelemetryTransport.ts", "Backend", "M", "Module hệ thống C06"),
    task("C06-02", "WP7 - Vận hành", "C06", "Lịch sử telemetry, alert và dashboard API", "Tra được sự cố theo robot/time; retention/downsample; cảnh báo pin/mạng/nhiệt/sensor.", "Chỉ giữ state mới nhất in-memory; bảng robot_events chưa được dùng đầy đủ.", "Chọn TSDB/PostgreSQL policy, ingest worker, query API và alert rules.", "Must", "GĐ7", "C12", "telemetry series + events", "models/entities.py; app/services/hub.py", "Backend + Observability", "L", "Module hệ thống C06"),
    task("C07-01", "WP3 - WebRTC", "C07", "LiveKit/TURN và token room-scoped ngắn hạn", "Robot/user chỉ vào đúng room; secret không xuống client; control vẫn chạy khi media lỗi.", "Compose có LiveKit 1.12, coturn, token robot/user scoped và media tách control.", "Production TLS/NAT/CGNAT test, TURN credential ngắn hạn và HA.", "Must", "GĐ4", "C01, R09", "LiveKit token/room robot-{id}", "src/infrastructure; app/services/media.py; docs/media-flow.md", "Media + DevOps", "L", "Module hệ thống C07", True),
    task("C07-02", "WP3 - WebRTC", "C07", "Media reconnect/adaptive quality và ICE metrics", "Reconnect ≥95% cho gián đoạn <30s; lưu RTT/jitter/loss/bitrate/ICE route.", "Browser/edge reconnect; chưa ingest đầy đủ WebRTC stats.", "Thu stats, policy ưu tiên audio/control, test chuyển mạng.", "Must", "GĐ4", "C06, R09", "WebRTC getStats/quality events", "MediaTransport.ts; simulator/media.py", "Media + Observability", "L", "AT-05/AT-10"),
    task("C08-01", "WP6 - Translation", "C08", "Chọn provider/model và streaming ASR→MT→TTS", "Partial/final, dịch theo cụm, TTS chunk đầu ≤2s điển hình; hủy khi barge-in.", "Chưa có translation service/message types.", "PoC 2-3 provider, đo latency/cost/WER và xây orchestrator adapter.", "Must", "GĐ6", "R10, C07, U04", "translation.partial/final; tts.audio/cancel", "Sheet Tích hợp & dữ liệu", "Speech AI", "XL", "Module hệ thống C08/Mockup VI"),
    task("C08-02", "WP6 - Translation", "C08", "Glossary, confidence và fallback", "Tên địa danh/món ăn/số tiền đúng; TTS lỗi vẫn có subtitle; có text-only/PTT fallback.", "Chưa có.", "Thiết kế glossary version/admin, confidence threshold và cached system phrases.", "Must", "GĐ6", "C10, U04", "glossary_id/version; translation result", "Chưa có", "Speech AI + Backend", "L", "Module hệ thống C08"),
    task("C09-01", "WP6 - AI", "C09", "Vision snapshot/OCR/VLM bất đồng bộ", "Chỉ gửi snapshot theo yêu cầu; response có answer/confidence/context; timeout và privacy policy.", "Chưa có API/message/storage.", "Tạo request job, upload signed URL, provider adapter, size limit và retention.", "Could", "GĐ6", "R09-04, C10, C11", "vision.request/result", "Sheet Tích hợp & dữ liệu", "AI + Backend", "XL", "Module hệ thống C09"),
    task("C10-01", "WP6 - AI", "C10", "Tourism RAG có nguồn nội bộ và evaluation", "Câu trả lời bám tài liệu, có source_id/version; dataset đánh giá và guardrail không tạo pose/cmd_vel.", "Chưa có.", "Chuẩn hóa corpus/metadata, ingest/retrieval/LLM, eval và admin workflow.", "Could", "GĐ6", "C09, C11", "rag.query/result/citations", "Sheet Tích hợp & dữ liệu", "AI/Data", "XL", "Module hệ thống C10"),
    task("C11-01", "WP4 - Web UI", "C11", "Map/destination API và route preview mẫu", "Frontend tải map/destination và preview route đúng coordinate transform.", "Đã có API, sample map/destination, utility và test; route là Manhattan mock.", "Giữ làm simulator contract; bổ sung DB repository/version và Nav2 route thật.", "Must", "GĐ2-GĐ5", "R05", "GET maps/destinations; navigation.preview", "sample-data/maps; app/api/maps.py; map-utils", "Backend + Frontend", "M", "Module hệ thống C11", True, "Chỉ đánh dấu hoàn thành cho dữ liệu/preview demo."),
    task("C11-02", "WP4 - Web UI", "C11", "Tour/version/rollback/no-go production", "Map package versioned, không đổi giữa mission; tour/POI/zone publish và rollback được.", "Chưa có tour/no-go/version rollout.", "Mở rộng DB/API, admin validation và sync desired/reported map version.", "Must", "GĐ2-GĐ7", "R05, C03", "map manifest/location/tour/zones", "models/entities.py; sample-data/maps", "Backend + ROS 2", "XL", "Module hệ thống C11"),
    task("C12-01", "WP7 - Vận hành", "C12", "Metrics/log/tracing chuẩn hóa", "Có robot_id/site_id/session_id/message_id/correlation_id; dashboard latency/error/resource.", "Có /health và log cơ bản; chưa Prometheus/OTel/Loki.", "Instrument API/WS/media/edge, centralized logging và retention.", "Must", "GĐ7", "C01, C05, C06", "OpenTelemetry/Prometheus metrics", "app/main.py", "Observability", "L", "Module hệ thống C12"),
    task("C12-02", "WP7 - Vận hành", "C12", "Alert, incident timeline và runbook", "Alert severity/dedup/silence; không spam; timeline truy được nguyên nhân và action.", "Chưa có.", "Định nghĩa rules cho offline/pin/temp/sensor/media/control; runbook và escalation.", "Must", "GĐ7", "C06, R07", "robot.alert/incident", "Chưa có", "Operations", "L", "Module hệ thống C12"),
    task("C13-01", "WP7 - Vận hành", "C13", "OTA signed manifest/canary/rollback", "Version từng module, checksum/signature, resume, bandwidth limit; mất điện/mạng không brick.", "Chưa có OTA service/agent.", "Thiết kế manifest, artifact store, rollout state, health gate và rollback.", "Should", "GĐ7", "R01, R02, R08", "ota.manifest/status", "Sheet Tích hợp & dữ liệu", "DevOps + Edge", "XL", "Module hệ thống C13"),
    task("U01-01", "WP4 - Web UI", "U01", "Login, robot list, filter/paging và bắt đầu session", "Chưa login không vào control; robot online/available mới kết nối; lỗi rõ ràng.", "Đã có login demo, list/filter/page, editor/config và create session.", "Bổ sung RBAC, site selector, privacy/consent và production pre-check.", "Must", "GĐ5", "C02, C03, C04", "auth/robots/sessions APIs", "LoginPage.tsx; RobotListPage.tsx; RobotEditorPage.tsx", "Frontend", "M", "Module hệ thống U01", True),
    task("U01-02", "WP4 - Web UI", "U01", "Pre-check trước kết nối", "Kiểm browser/mic/network/robot ready/pin/safety; chỉ bật Kết nối khi bắt buộc đạt.", "Trang cấu hình có diagnostics; luồng chọn robot chưa có pre-check đầy đủ.", "Tạo checklist preflight và API readiness tổng hợp.", "Must", "GĐ5", "C03, C06, C07", "robot readiness + media diagnostics", "RobotConfigurationPage.tsx; RobotListPage.tsx", "Frontend + Backend", "L", "Mockup Hình 4"),
    task("U02-01", "WP4 - Web UI", "U02", "Dashboard video/joystick/pin/mạng/mic/loa", "Video là chính; thao tác touch/keyboard; trạng thái lệnh và media rõ; responsive phạm vi MVP.", "Đã có dashboard, video, control pad, pin/RTT, mic/loa và map.", "Thử usability trên tablet/mobile nhỏ; thêm session timer/packet loss/control lease.", "Must", "GĐ5", "C05, C06, C07", "control + telemetry + LiveKit", "DashboardPage.tsx; ControlPad.tsx", "Frontend", "M", "Module hệ thống U02", True),
    task("U02-02", "WP4 - Web UI", "U02", "Nút dừng khẩn luôn thấy và khóa UI khi mất session", "STOP cố định, cách xa nút thường; loss/expiry/takeover khóa control ngay và thông báo.", "Space/keyup/disconnect gửi stop; chưa có nút E-stop nổi bật/takeover/session expiry UX.", "Thêm emergency control, session.state listener và accessibility confirmation.", "Must", "GĐ5", "R07-04, C04-02", "control.stop; session.state; safety.state", "DashboardPage.tsx; useTeleoperation.ts", "Frontend", "L", "Module hệ thống U02/AT-04"),
    task("U02-03", "WP4 - Web UI", "U02", "PTZ, chụp ảnh, fullscreen/PiP và quality selector", "Control có ACK/limit; ảnh theo session; trạng thái quyền camera rõ.", "Chưa có.", "Bổ sung UI sau khi R09-04/C05 contract hoàn tất.", "Should", "GĐ5", "R09-04", "camera.ptz/photo.capture", "Chưa có", "Frontend", "L", "Mockup Hình 5"),
    task("U03-01", "WP4 - Web UI", "U03", "Map, marker robot, POI, route và mission state", "Chỉ chọn POI hợp lệ; progress/cancel phản ánh robot; coordinate đúng.", "Đã có MapPanel, destination, route preview và navigation state với simulator.", "Bổ sung no-go/speed zones, confirm hành động và trạng thái Nav2 đầy đủ.", "Must", "GĐ5", "C11, R05", "map/destination/navigation events", "MapPanel.tsx; map-utils; tests/map-utils.test.ts", "Frontend", "M", "Module hệ thống U03", True, "Hoàn thành ở mức simulator."),
    task("U04-01", "WP6 - Translation", "U04", "UI direct/translated, PTT/full-duplex và phụ đề hai chiều", "Phân biệt speaker/source-target; trạng thái nghe-dịch-phát; subtitle 1-2 dòng trên robot.", "Chỉ có mic/loa trực tiếp; chưa translation UI.", "Thiết kế state machine, controls, captions và consent/privacy.", "Must", "GĐ6", "C08, R10, R11", "translation.partial/final; audio.mode", "DashboardPage.tsx", "Frontend + Speech", "XL", "Module hệ thống U04"),
    task("U05-01", "WP6 - AI", "U05", "UX hỏi đáp text/voice/vision", "Có timeout/cancel/confidence/source; phân biệt AI answer và command; không tự điều khiển.", "Chưa có.", "Xây UI sau contract C09/C10; intent validation và confirmation riêng.", "Could", "GĐ6", "C09, C10, C08", "vision/rag query/result", "Chưa có", "Frontend + AI", "L", "Module hệ thống U05"),
    task("OPS-01", "WP7 - Vận hành", "Admin", "Fleet dashboard, phiên đang chạy, alert và takeover", "Operator xem trạng thái nhiều robot, phiên/alert; support takeover đúng quyền và có audit.", "Robot list có tổng hợp cơ bản; chưa admin dashboard/takeover.", "Tạo admin API/UI, RBAC, realtime updates và incident link.", "Should", "GĐ7", "C02, C04, C06, C12", "fleet/session/alert APIs", "RobotListPage.tsx", "Full stack", "XL", "Mockup WP7/AT-11"),
    task("OPS-02", "WP7 - Vận hành", "Privacy", "Consent và retention audio/video/transcript/photo", "Ghi hình mặc định tắt; consent rõ; xóa theo hạn; signed URL ngắn hạn; audit truy cập.", "Chưa có recording/transcript/photo nên chưa có policy thực thi.", "Thiết kế data classification, consent records, retention jobs và UI.", "Must", "GĐ6-GĐ7", "C08, C09, R09-04", "session consent + media metadata", "models/entities.py", "Security + Backend", "L", "Mockup VIII/IX"),
    task("TEST-01", "WP8 - Field test", "QA", "Giữ và mở rộng unit/integration/E2E hiện có", "Test login→session→control→telemetry→route; registry; media config; watchdog chạy ổn định trong CI.", "Đã có pytest/vitest/Playwright cho các luồng demo chính.", "Thêm coverage cho lỗi, schema mới, RBAC, Redis lease và robot thật adapter.", "Must", "Liên tục", "ARCH-05", "test fixtures", "src/apps/center-backend/tests; demo/robot-simulator/tests; demo/e2e", "QA + Dev", "M", "docs/testing.md", True),
    task("TEST-02", "WP8 - Field test", "QA/Safety", "Safety fault injection và HIL", "Test mất heartbeat, LiDAR/encoder, E-stop, CRC, rút cáp, CPU overload; thời gian dừng có bằng chứng.", "Chỉ có watchdog simulator và TTL/session tests.", "Dựng HIL rig, timestamp camera/log và test tự động lặp.", "Must", "GĐ1-GĐ3", "R06-R08", "safety events + MCU frames", "docs/testing.md", "QA + Safety", "XL", "AT-02/03/04"),
    task("TEST-03", "WP8 - Field test", "QA/Network", "NAT/CGNAT và 4G/5G của ít nhất hai nhà mạng", "Kết nối ≤10s; control ≤200ms điển hình; media qua TURN; reconnect đạt mục tiêu.", "Chưa có biên bản field test.", "Chuẩn hóa kịch bản, clock sync, collect ICE/RTT/jitter/loss và report.", "Must", "GĐ4", "C07, R09", "WebRTC/WS metrics", "docs/media-flow.md", "QA + Network", "L", "AT-01/05/10"),
    task("TEST-04", "WP8 - Field test", "QA/Audio", "Audio/dịch ở 4 môi trường và double-talk", "Đo WER, VAD, clipping, first partial/stable/TTS; không loop khi loa đang phát.", "Chưa có audio DSP/translation.", "Chuẩn bị corpus/glossary/người nói/thiết bị đo và acceptance thresholds.", "Must", "GĐ5-GĐ6", "R10, C08", "audio/translation metrics", "Robot_Telepresence...docx VI/IX", "QA + Audio", "XL", "AT-06/07/08"),
    task("TEST-05", "WP8 - Field test", "QA", "Soak test tối thiểu 72 giờ", "Không đầy disk/leak/treo; robot tự phục hồi; Nav2/Safety deadline không bị media/AI ảnh hưởng.", "Chưa có kết quả soak test.", "Tạo workload, fault schedule, dashboards và tiêu chí pass/fail.", "Must", "Trước nghiệm thu", "R01, C12", "metrics/logs/alerts", "Module hệ thống X", "QA + Operations", "L", "Checklist field test"),
    task("TEST-06", "WP8 - Field test", "QA/Security", "Security, privacy và browser matrix", "Chrome/Edge/Safari theo phạm vi; token/ACL/TURN/recording/consent đúng; pentest finding đóng.", "Có auth tests cơ bản; chưa browser matrix/pentest.", "Threat model, OWASP/API/WS test, dependency scan và privacy verification.", "Must", "Trước nghiệm thu", "C01, C02, OPS-02", "security test evidence", "tests/test_integration.py", "QA + Security", "L", "Mockup IX/Phụ lục B"),
]


CURRENT_STATE = [
    ("R01", "Orange Pi system management", "Một phần", 35, "Có Docker/systemd edge unit và restart; health endpoint trung tâm.", "Thiếu bringup ROS 2, health thiết bị/tài nguyên, log rotation, safe boot.", "demo/robot-simulator/deploy/rovera-robot.service"),
    ("R02", "Robot Agent", "Một phần", 65, "Simulator có claim/enroll, JWT, outbound WSS, heartbeat, reconnect, dedupe, config/media.", "Cần port sang Orange Pi/ROS 2, telemetry buffer, secure production và capability/version.", "demo/robot-simulator/simulator/client.py"),
    ("R03", "ROS 2 Command Gateway", "Chưa có", 10, "Contract và hướng tích hợp đã mô tả.", "Không có ROS 2 workspace/node/topic/action adapter.", "docs/integration-robot.md"),
    ("R04", "Teleoperation", "Một phần", 70, "Web input 20 Hz, ramp, TTL, STOP trên keyup/blur/disconnect; simulator watchdog 400 ms.", "Chưa nối twist_mux/Safety/MCU và chưa đo mạng/robot thật.", "useTeleoperation.ts; simulator/motion.py"),
    ("R05", "Navigation", "Mock", 25, "Map/destination/preview/goal/cancel và simulator motion theo route.", "Route Manhattan; thiếu SLAM/AMCL/Nav2/no-go/recovery/pause.", "app/api/navigation.py; simulator/navigation.py"),
    ("R06", "Sensors & localization", "Chưa có", 0, "Không có implementation.", "Thiếu toàn bộ driver, calibration, TF, fusion và diagnostics.", "Chưa có"),
    ("R07", "Safety Manager", "Mock", 20, "Có watchdog motion và STOP khi WS/session đóng.", "Thiếu E-stop/collision/cliff/priority matrix/commanded-vs-actual/reason codes.", "simulator/motion.py; app/api/websockets.py"),
    ("R08", "MCU chassis", "Chưa có", 0, "Không có firmware/protocol.", "Thiếu CAN/UART CRC, PID, encoder, brake, watchdog, bootloader.", "Chưa có"),
    ("R09", "Camera/video", "Một phần", 60, "LiveKit publish; test/file/RTSP/USB; reconnect; UI config/preview.", "Thiếu CSI/MPP/GStreamer proof, adaptive bitrate, PTZ/snapshot, 72h field test.", "simulator/media.py; RobotConfigurationPage.tsx"),
    ("R10", "Two-way audio", "Một phần", 25, "LiveKit audio track, browser mic/speaker; edge consume user audio.", "Thiếu phần cứng, AEC/NS/AGC/VAD, modes, barge-in và noise tests.", "simulator/media.py; DashboardPage.tsx"),
    ("R11", "Robot display", "Chưa có", 0, "Không có ứng dụng kiosk.", "Thiếu toàn bộ display UI/state/privacy/subtitle/autostart.", "Chưa có"),
    ("R12", "Edge AI", "Chưa có", 0, "Không có.", "Thiếu RKNN benchmark/process/resource degradation.", "Chưa có"),
    ("R13", "Battery/power", "Mock", 5, "Có battery_percent mô phỏng.", "Thiếu BMS, nguồn, thresholds, charge history/return-to-dock policy.", "app/services/hub.py"),
    ("C01", "API Gateway", "Một phần", 35, "FastAPI, CORS, health; nginx phục vụ frontend.", "Thiếu reverse proxy TLS production, rate limit, WAF, correlation/tracing.", "app/main.py; nginx.conf"),
    ("C02", "Auth/User", "Một phần", 35, "Login demo, JWT, user model có role.", "Thiếu user management, RBAC enforcement, refresh/revoke/MFA/policy.", "app/api/auth.py; app/core/security.py"),
    ("C03", "Robot Management", "Một phần", 75, "Registry CRUD, paging/filter, quick-add, claim/enroll, config/diagnostics, audit connection.", "Thiếu config version/rollout/maintenance readiness/audit chi tiết.", "app/api/robots.py; RobotListPage.tsx"),
    ("C04", "Session Management", "Một phần", 50, "Exclusive in-memory session, media lease, create/get/delete và STOP khi end.", "Thiếu Redis lease multi-instance, control heartbeat/grace/takeover/audit lifecycle.", "app/api/sessions.py; app/services/hub.py"),
    ("C05", "Command Service", "Một phần", 55, "WS validate session/TTL/sequence/type, route robot, ACK; navigation goal/cancel.", "Thiếu schema registry payload, business lifecycle/idempotency store/retry/metrics/audit.", "app/api/websockets.py; app/api/navigation.py"),
    ("C06", "Telemetry Service", "Một phần", 30, "Realtime pose/health/status broadcast và latest state.", "Thiếu history/downsample/retention/dashboard API/alerts/multi-instance pubsub.", "app/api/websockets.py; app/services/hub.py"),
    ("C07", "Media Server", "Một phần", 70, "LiveKit/coturn Compose, room-scoped user/robot token, reconnect.", "Thiếu production TLS/TURN credential/HA/CGNAT evidence/stats adaptation.", "src/infrastructure; app/services/media.py"),
    ("C08", "Translation", "Chưa có", 0, "Không có service/contract.", "Thiếu ASR/MT/TTS streaming, glossary, latency metrics/fallback.", "Chưa có"),
    ("C09", "Vision AI", "Chưa có", 0, "Không có.", "Thiếu snapshot job/API/storage/privacy/provider adapter.", "Chưa có"),
    ("C10", "Tourism RAG", "Chưa có", 0, "Không có.", "Thiếu corpus/metadata/retrieval/LLM/eval/admin/guardrail.", "Chưa có"),
    ("C11", "Map/Location/Tour", "Mock", 45, "Map/destination API, sample map, coordinate utility và route preview.", "Thiếu map version, tour, zone/no-go, rollback, Nav2 integration.", "sample-data/maps; app/api/maps.py"),
    ("C12", "Monitoring/Alert", "Chưa có", 15, "Có /health, stdout log và connection DB record.", "Thiếu metrics/tracing/log aggregation/alert/incident/audit/runbook.", "app/main.py; models/entities.py"),
    ("C13", "OTA", "Chưa có", 0, "Không có.", "Thiếu artifact/manifest/signature/canary/resume/rollback.", "Chưa có"),
    ("U01", "Login/select robot", "Một phần", 65, "Login, robot list/filter/paging, CRUD/config và create session.", "Thiếu pre-check, site/consent/RBAC/low-battery readiness.", "LoginPage.tsx; RobotListPage.tsx"),
    ("U02", "Control UI", "Một phần", 60, "Video, joystick, pin/RTT, mic/loa, command state và disconnect.", "Thiếu visible E-stop, packet loss/session timer/lease/takeover/PTZ/photo/mobile validation.", "DashboardPage.tsx"),
    ("U03", "Map UI", "Mock", 45, "MapPanel, POI, pose, route, goal/cancel và state.", "Thiếu no-go/zones/confirm/Nav2 progress/real map scale field test.", "MapPanel.tsx"),
    ("U04", "Communication/translation UI", "Chưa có", 15, "Có mic/loa direct talk cơ bản.", "Thiếu PTT/mode/subtitles/speaker labels/translation fallback/privacy.", "DashboardPage.tsx"),
    ("U05", "AI Q&A UI", "Chưa có", 0, "Không có.", "Thiếu toàn bộ text/voice/vision Q&A UX.", "Chưa có"),
]


INTEGRATIONS = [
    ("INT-01", "Đăng nhập người dùng", "Web → C02", "REST", "Đã có", "POST /api/auth/login", compact_json({"email": "demo@rovera.local", "password": "demo123"}), "access_token; user{id,email,role}", "Đổi demo auth sang RBAC production; refresh/revoke/MFA.", "app/api/auth.py"),
    ("INT-02", "Đăng ký/claim robot", "Edge ↔ C03", "HTTPS", "Đã có", "POST /api/robot-auth/claim", compact_json({"management_address": "192.168.1.20", "username": "operator", "password": "<local-password>", "device_fingerprint": "orange-pi-5:machine-id"}), "robot_id; credential (chỉ trả một lần)", "Rate limit, audit và TLS production.", "app/api/robot_auth.py"),
    ("INT-03", "Robot JWT", "Edge → C02/C03", "HTTPS", "Đã có", "POST /api/robot-auth/token", compact_json({"robot_id": "ROBOT-001", "credential": "<device-credential>"}), "Bearer robot JWT 15 phút", "Rotate/revoke và secure element nếu có.", "app/api/robot_auth.py"),
    ("INT-04", "Gateway robot", "Edge ↔ C05/C06", "WSS", "Đã có", "/ws/robot/connect?robot_id=ROBOT-001", compact_json({"message_id": "7b8e5c52-0d74-4f8e-b97e-4f7ff8aa1001", "schema_version": "1.0", "message_type": "robot.heartbeat", "robot_id": "ROBOT-001", "session_id": "", "sequence": 12, "timestamp": "2026-07-29T09:00:00+07:00", "ttl_ms": 0, "payload": {}}), "gateway.welcome + command; robot sends telemetry/ACK", "TLS, capability/version negotiation và multi-instance.", "docs/websocket-protocol.md"),
    ("INT-05", "Điều khiển vận tốc", "Web → C05 → R02/R03", "WSS", "Một phần", "control.velocity", compact_json({"message_id": "7b8e5c52-0d74-4f8e-b97e-4f7ff8aa1002", "schema_version": "1.0", "message_type": "control.velocity", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 1842, "timestamp": "2026-07-29T09:20:15.240+07:00", "ttl_ms": 300, "payload": {"linear_x": 0.25, "angular_z": -0.30}}), "command.ack accepted/expired/rejected/robot_offline", "Chốt linear_x/angular_z hay vx/wz; nối ROS 2 twist_mux và Safety.", "ControlTransport.ts; api/websockets.py"),
    ("INT-06", "Control heartbeat/lease", "Web ↔ C04 → R07", "WSS/REST", "Chưa có", "control.heartbeat; /control-lease", compact_json({"message_type": "control.heartbeat", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 1843, "timestamp": "2026-07-29T09:20:15.340+07:00", "ttl_ms": 500, "payload": {"lease_id": "lease-001"}}), "session.state + safety stop khi timeout", "Thêm message schema, lease Redis và UI state.", "Tài liệu mockup VIII"),
    ("INT-07", "ROS 2 teleop", "R03 → R04/R07", "ROS 2 topic", "Chưa có", "/teleop/cmd_vel", "linear:\n  x: 0.25\nangular:\n  z: -0.30", "safe_cmd_vel sau twist_mux/collision", "Tạo node gateway, QoS và local watchdog.", "docs/integration-robot.md"),
    ("INT-08", "Orange Pi ↔ MCU", "R03/R07 ↔ R08", "CAN/UART", "Chưa có", "SET_VELOCITY / STATUS / STOP", "SOF=0xAA55 | ver=1 | type=0x01 | seq=1842 | vx_mm_s=250 | wz_mrad_s=-300 | ttl_ms=300 | CRC16", "ACK + encoder/current/voltage/e-stop/fault", "Chốt binary frame, unit/endian/CRC/timeout và tạo MCU simulator.", "Module hệ thống R08"),
    ("INT-09", "Telemetry pose", "R06/R05 → R02 → C06/U03", "ROS 2 + WSS", "Mock", "robot.pose", compact_json({"message_type": "robot.pose", "robot_id": "ROBOT-001", "session_id": "", "sequence": 901, "timestamp": "2026-07-29T09:20:16+07:00", "ttl_ms": 0, "payload": {"map_id": "MAP-001", "map_version": "1.0.0", "x": 5.5, "y": 6.0, "yaw": 0.0, "linear_velocity": 0.0, "angular_velocity": 0.0}}), "Frontend marker + history/sampling", "Map TF/odom thật, map_version và timestamp sync.", "app/services/hub.py"),
    ("INT-10", "Health/BMS", "R01/R06/R13 → C06/C12", "WSS", "Mock", "robot.health", compact_json({"message_type": "robot.health", "robot_id": "ROBOT-001", "sequence": 902, "timestamp": "2026-07-29T09:20:16+07:00", "ttl_ms": 0, "payload": {"battery": {"soc_percent": 78, "voltage_v": 25.4, "current_a": -1.8, "temperature_c": 36.2, "charging": False}, "system": {"cpu_percent": 42, "ram_percent": 55, "temperature_c": 61}, "camera": "online", "lidar": "online", "mcu": "online", "safety": "ready"}}), "Dashboard + alerts + readiness", "Mở rộng schema và nối nguồn thật; hiện payload phẳng/mô phỏng.", "simulator/client.py"),
    ("INT-11", "Navigation goal", "U03/C11 → C05 → R03/R05", "REST + WSS + Nav2", "Mock", "navigation.goal", compact_json({"message_type": "navigation.goal", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 1850, "timestamp": "2026-07-29T09:21:00+07:00", "ttl_ms": 5000, "payload": {"location_id": "DEST-001", "map_id": "MAP-001", "map_version": "1.0.0"}}), "navigation.status accepted/planning/moving/blocked/arrived/failed", "Current code sends route_id + points; production nên gửi location/map version rồi Nav2 plan.", "app/api/navigation.py"),
    ("INT-12", "Map/location/zone", "C11 ↔ R05/U03", "REST/package", "Một phần", "GET /api/maps/{id}; destinations", compact_json({"map_id": "MAP-001", "version": "1.0.0", "resolution_m_per_pixel": 0.05, "origin": {"x": 0, "y": 0, "yaw": 0}, "locations": [{"location_id": "DEST-001", "name": "Cổng chính", "x": 2.5, "y": 7.0, "yaw": 1.57}], "zones": [{"zone_id": "NO-GO-01", "type": "no_go", "polygon": [[1, 1], [2, 1], [2, 2], [1, 2]]}]}), "Validated map package + checksum", "Thêm version/checksum/zones/tours/rollback.", "sample-data/maps/map-001.json"),
    ("INT-13", "Safety event", "R07/R08 → C06/C12/U02", "WSS", "Chưa có", "safety.event", compact_json({"message_type": "safety.event", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 910, "timestamp": "2026-07-29T09:22:01+07:00", "ttl_ms": 0, "payload": {"state": "stopped", "reason_code": "SAFETY_ESTOP_ACTIVE", "source": "physical_estop", "latched": True, "commanded_vx": 0.25, "actual_vx": 0.0}}), "UI lock + incident/audit", "Thêm schema, Safety Manager, DB event và UI.", "Module hệ thống R07"),
    ("INT-14", "LiveKit media", "R09/R10 ↔ C07 ↔ Web", "WebRTC", "Đã có", "room robot-ROBOT-001", compact_json({"room": "robot-ROBOT-001", "robot_identity": "robot:ROBOT-001", "user_identity": "user:user-001:session:sess-001", "tracks": ["camera", "robot-microphone", "user-microphone"]}), "H.264/Opus tracks + ICE recovery", "Production TLS/TURN/CGNAT; MPP/AEC/adaptation/stats.", "docs/media-flow.md"),
    ("INT-15", "Translation streaming", "R10/Web ↔ C08 ↔ R11", "WebRTC data/WS", "Chưa có", "translation.partial/final", compact_json({"message_type": "translation.final", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 120, "timestamp": "2026-07-29T09:23:00+07:00", "ttl_ms": 10000, "payload": {"utterance_id": "utt-001", "speaker": "local", "source_language": "vi-VN", "target_language": "en-US", "source_text": "Đây là Văn Miếu.", "translated_text": "This is the Temple of Literature.", "confidence": 0.94, "glossary_version": "tourism-vi-en@3"}}), "Subtitle + TTS audio chunks", "Thêm contract types, provider adapter, glossary, barge-in/fallback.", "Robot_Telepresence...docx VI/VIII"),
    ("INT-16", "Vision snapshot", "U05/R09 → C09", "REST async", "Chưa có", "POST /api/v1/vision/requests", compact_json({"robot_id": "ROBOT-001", "session_id": "sess-001", "question": "Trước mắt là gì?", "snapshot_object_key": "sessions/sess-001/20260729T092400Z.jpg", "context": {"map_id": "MAP-001", "location_id": "DEST-001", "pose": {"x": 2.5, "y": 7.0, "yaw": 1.57}}}), "job_id rồi vision.result {answer,confidence,objects,source_ids}", "Cần snapshot/photo storage, privacy/retention và provider.", "Module hệ thống C09"),
    ("INT-17", "Tourism RAG", "C09/U05 ↔ C10", "REST/internal", "Chưa có", "POST /api/v1/rag/query", compact_json({"query": "Văn Miếu được xây dựng khi nào?", "language": "vi", "site_id": "SITE-HANOI-01", "location_id": "DEST-001", "knowledge_version": "2026.07"}), "answer; confidence; citations[{source_id,title,version}]", "Cần corpus, metadata, evaluation, guardrail và admin workflow.", "Module hệ thống C10"),
    ("INT-18", "Robot display", "C04/C08/C12 → R11", "WSS/WebRTC", "Chưa có", "session.state + translation.final", compact_json({"message_type": "session.state", "robot_id": "ROBOT-001", "session_id": "sess-001", "sequence": 20, "timestamp": "2026-07-29T09:25:00+07:00", "ttl_ms": 0, "payload": {"state": "active", "camera_active": True, "microphone_active": True, "privacy_notice": True}}), "Kiosk UI/avatar/video/subtitle", "Thêm message types và robot-display app.", "Module hệ thống R11"),
    ("INT-19", "Monitoring/alert", "Tất cả → C12", "OTLP/metrics/log", "Chưa có", "OTel resource + robot.alert", compact_json({"severity": "critical", "alert_code": "ROBOT_HEARTBEAT_LOST", "robot_id": "ROBOT-001", "site_id": "SITE-HANOI-01", "session_id": "sess-001", "message_id": "7b8e5c52-0d74-4f8e-b97e-4f7ff8aa1999", "occurred_at": "2026-07-29T09:26:00+07:00", "detail": {"last_seen_ms": 620}}), "Incident/timeline/runbook link", "Instrument, retention, dedup/silence và dashboard.", "Module hệ thống C12"),
    ("INT-20", "OTA", "C13 ↔ R01/R02/R08", "HTTPS", "Chưa có", "ota.manifest/status", compact_json({"release_id": "robot-edge-1.2.0", "target": "robot-agent", "version": "1.2.0", "url": "https://updates.example.com/robot-agent-1.2.0.tar.zst", "sha256": "<64-hex>", "signature": "<base64>", "min_battery_percent": 50, "rollout": {"channel": "canary", "percent": 10}, "rollback_version": "1.1.0"}), "download/install/health/rollback status", "Artifact signing, resume, health gate và failure tests.", "Module hệ thống C13"),
    ("INT-21", "Support takeover", "Admin ↔ C02/C04/C05 ↔ U02", "REST + WSS", "Chưa có", "POST /api/v1/sessions/{id}/control-lease", compact_json({"action": "takeover", "reason": "Khách yêu cầu hỗ trợ", "requested_by": "support-001", "expected_lease_version": 7}), "session.state/control.revoked + audit", "RBAC, optimistic version, notification và STOP handover.", "Mockup AT-11"),
    ("INT-22", "Ảnh theo phiên", "U02/R09 ↔ Storage", "REST/object", "Chưa có", "GET /api/v1/sessions/{id}/photos", compact_json({"photo_id": "photo-001", "session_id": "sess-001", "robot_id": "ROBOT-001", "captured_at": "2026-07-29T09:27:00+07:00", "object_key": "sessions/sess-001/photo-001.jpg", "thumbnail_url": "https://signed.example/...", "expires_at": "2026-07-29T09:32:00+07:00"}), "Signed URL ngắn hạn + retention", "Cần photo.capture, storage, consent và API.", "Mockup VIII"),
]


ACCEPTANCE = [
    ("AT-01", "Kết nối robot qua mạng khác NAT", "Video/audio lên trong ≤10 giây qua TURN khi cần.", "C07, R09, U02", "Chưa chạy", "Log ICE route; timestamp click→first frame/audio.", "Must"),
    ("AT-02", "Điều khiển tiến/lùi/quay", "Robot phản hồi đúng; nhả điều khiển dừng; không vượt speed/acceleration limit.", "R03, R04, R07, R08", "Chưa chạy robot thật", "Encoder + command timestamp + video.", "Must"),
    ("AT-03", "Mất heartbeat", "Robot dừng trong ≤500 ms.", "R02, R04, R07, R08", "Simulator có watchdog; chưa HIL", "Ngắt mạng có chủ đích, đo commanded/actual velocity.", "Must"),
    ("AT-04", "Dừng khẩn UI và vật lý", "Robot dừng, UI khóa, event có reason/timestamp; vật lý độc lập Linux.", "R07, R08, U02, C12", "Chưa có", "HIL + video + MCU log + audit DB.", "Must"),
    ("AT-05", "Mạng yếu", "Video tự giảm chất lượng; audio/control vẫn ưu tiên.", "C07, R09, R10", "Một phần reconnect", "tc/netem + WebRTC stats.", "Must"),
    ("AT-06", "AEC double-talk", "Không vòng lặp; nhận được lời gần mic khi loa đang phát.", "R10", "Chưa có", "File âm chuẩn + mic/loa robot thật + WER/ERLE.", "Must"),
    ("AT-07", "Dịch khách → địa phương", "Có phụ đề và tiếng dịch đầu tiên trong mục tiêu ≤2 giây điển hình.", "C08, U04, R11", "Chưa có", "Speech timestamps và glossary dataset.", "Must"),
    ("AT-08", "Dịch địa phương → khách", "Khách nghe bản dịch, có transcript nguồn/đích.", "C08, U04", "Chưa có", "4 môi trường âm thanh + confidence.", "Must"),
    ("AT-09", "PTZ và chụp ảnh", "Camera phản hồi; ảnh lưu đúng session và quyền tải.", "R09, U02", "Chưa có", "ACK PTZ + object metadata + access audit.", "Should"),
    ("AT-10", "Reconnect", "Khôi phục media; robot không tự chạy lại lệnh trước đó.", "R02, C07, U02", "Logic demo có", "Ngắt 1/5/15/30 giây, lặp ≥100 lần.", "Must"),
    ("AT-11", "Takeover hỗ trợ", "Nhân viên giành quyền; khách mất quyền và được thông báo; có audit.", "C02, C04, U02", "Chưa có", "Concurrency test hai client.", "Should"),
    ("AT-12", "Kết thúc phiên", "Robot dừng, quyền khóa, log/thống kê lưu.", "C04, C05, R07", "Một phần", "REST/WS/robot/event DB evidence.", "Must"),
    ("FT-01", "Camera/encoder/nhiệt 2 giờ", "Không treo; hardware encode; nhiệt trong giới hạn thiết kế.", "R01, R09", "Chưa chạy", "Dashboard nhiệt/CPU/frame progress.", "Must"),
    ("FT-02", "Wi-Fi + 4G/5G hai nhà mạng", "Control/media/telemetry đạt ngưỡng hoặc hạ cấp an toàn.", "C07, R02, R09", "Chưa chạy", "Bảng RTT/jitter/loss/reconnect/TURN.", "Must"),
    ("FT-03", "Browser matrix", "Chrome/Edge/Safari theo phạm vi; quyền camera/mic và codec đúng.", "U01, U02, U04", "Chưa chạy", "Playwright/manual matrix.", "Must"),
    ("FT-04", "Pin và mất nguồn/reboot", "Không reset do motor/modem; reboot tự lên; critical shutdown đúng policy.", "R01, R13", "Chưa chạy", "Power logger + system boot logs.", "Must"),
    ("FT-05", "Soak 72 giờ", "Không leak/disk full/deadlock; service tự phục hồi; safety deadline ổn.", "Toàn hệ thống", "Chưa chạy", "Metrics/log/incident summary.", "Must"),
]


DEPENDENCIES = [
    ("Gate 1", "Khung gầm & an toàn", "R08 MCU + E-stop + watchdog; R03 base driver; R07 Safety", "Mất lệnh/Orange Pi robot tự dừng; E-stop độc lập.", "Không cho robot thật chạy remote trước khi đạt."),
    ("Gate 2", "Cảm biến & Nav2", "R06 sensor/TF/fusion; R05 Nav2; collision/no-go", "Đi waypoint và tránh vật cản an toàn tại khu thử.", "Dùng simulator song song nhưng không xem là nghiệm thu robot thật."),
    ("Gate 3", "Kết nối trung tâm", "R02, C01-C06, contracts, credential/TLS", "Remote control ổn định; mất mạng dừng; không replay.", "Mã hiện tại gần nhất với gate này nhưng còn production hardening."),
    ("Gate 4", "Video & audio", "R09, R10, C07, TURN", "Hai chiều ổn định qua mạng thật; AEC/NS đạt.", "AEC và 4G/5G cần thử sớm."),
    ("Gate 5", "Giao diện người dùng", "U01-U04, R11", "Người dùng hoàn thành tour cơ bản không cần UI kỹ thuật.", "Nút STOP và trạng thái quyền luôn nhìn thấy."),
    ("Gate 6", "Dịch & AI", "C08-C10, U04-U05, R12", "Fallback rõ; AI không chạm trực tiếp motor/pose.", "Chỉ tích hợp sau khi audio input đủ sạch."),
    ("Gate 7", "Vận hành nhiều robot", "C12-C13, Admin, audit, OTA", "Quản lý fleet, rollback và truy vết sự cố.", "Cần trước field rollout quy mô."),
]


def add_title(ws, title: str, subtitle: str, last_col: int, formats: dict) -> None:
    ws.merge_range(0, 0, 0, last_col, title, formats["title"])
    ws.merge_range(1, 0, 1, last_col, subtitle, formats["subtitle"])
    ws.set_row(0, 30)
    ws.set_row(1, 36)


def main() -> None:
    workbook = xlsxwriter.Workbook(OUTPUT)
    workbook.set_properties(
        {
            "title": "Kế hoạch đầu việc Robot Telepresence",
            "subject": "Đối chiếu hai tài liệu Word với mã nguồn hiện tại",
            "author": "MQ ICT Solutions / Codex",
            "comments": "Generated from repository evidence on 2026-07-29",
        }
    )

    colors = {
        "navy": "#10203D",
        "blue": "#1759D6",
        "cyan": "#DCEBFF",
        "green": "#C6EFCE",
        "green_text": "#176B3A",
        "amber": "#FFF2CC",
        "amber_text": "#8A5A00",
        "red": "#FCE8E6",
        "red_text": "#A12622",
        "gray": "#EEF1F5",
        "mid_gray": "#C8D0DB",
        "white": "#FFFFFF",
        "text": "#162238",
    }
    base = {"font_name": "Aptos", "font_size": 10, "font_color": colors["text"]}
    formats = {
        "title": workbook.add_format({**base, "bold": True, "font_size": 18, "font_color": colors["white"], "bg_color": colors["navy"], "align": "left", "valign": "vcenter"}),
        "subtitle": workbook.add_format({**base, "font_size": 10, "font_color": "#DDE7F7", "bg_color": colors["navy"], "align": "left", "valign": "vcenter", "text_wrap": True}),
        "header": workbook.add_format({**base, "bold": True, "font_color": colors["white"], "bg_color": colors["blue"], "border": 1, "border_color": "#0F49B4", "align": "center", "valign": "vcenter", "text_wrap": True}),
        "cell": workbook.add_format({**base, "border": 1, "border_color": "#D8DEE8", "valign": "top", "text_wrap": True}),
        "cell_center": workbook.add_format({**base, "border": 1, "border_color": "#D8DEE8", "align": "center", "valign": "vcenter", "text_wrap": True}),
        "checkbox": workbook.add_format({**base, "border": 1, "border_color": "#D8DEE8", "align": "center", "valign": "vcenter", "font_size": 12}),
        "done_row": workbook.add_format({"bg_color": colors["green"], "font_color": colors["green_text"]}),
        "percent": workbook.add_format({**base, "border": 1, "border_color": "#D8DEE8", "num_format": "0%", "align": "center", "valign": "vcenter"}),
        "big_number": workbook.add_format({**base, "bold": True, "font_size": 20, "font_color": colors["navy"], "bg_color": colors["white"], "border": 1, "border_color": colors["mid_gray"], "align": "center", "valign": "vcenter"}),
        "metric_label": workbook.add_format({**base, "bold": True, "font_color": colors["blue"], "bg_color": colors["cyan"], "border": 1, "border_color": colors["mid_gray"], "align": "center", "valign": "vcenter", "text_wrap": True}),
        "section": workbook.add_format({**base, "bold": True, "font_size": 12, "font_color": colors["white"], "bg_color": colors["navy"], "align": "left", "valign": "vcenter"}),
        "note": workbook.add_format({**base, "bg_color": "#F8FAFD", "border": 1, "border_color": colors["mid_gray"], "text_wrap": True, "valign": "top"}),
        "json": workbook.add_format({"font_name": "Consolas", "font_size": 9, "font_color": colors["text"], "border": 1, "border_color": "#D8DEE8", "valign": "top", "text_wrap": True}),
    }

    # Backlog first so overview formulas can reference a stable range.
    ws = workbook.add_worksheet("Backlog")
    headers = [
        "Hoàn thành", "ID", "Gói công việc", "Module", "Đầu việc", "Kết quả / Definition of Done",
        "Hiện trạng mã", "Khoảng trống / Cần làm", "Ưu tiên", "Trình tự", "Phụ thuộc",
        "Dữ liệu / Hợp đồng tích hợp", "Bằng chứng / Điểm vào hiện có", "Nhóm phụ trách",
        "Độ lớn", "Nguồn yêu cầu", "Ghi chú",
    ]
    add_title(
        ws,
        "BACKLOG HOÀN THÀNH DỰ ÁN ROBOT TELEPRESENCE",
        "Bấm ô checkbox ở cột A để đánh dấu hoàn thành; toàn bộ dòng sẽ chuyển xanh lá. "
        "Các ô đã đánh dấu ban đầu là phần đã được xác minh trong mã hiện tại ở mức demo/simulator.",
        len(headers) - 1,
        formats,
    )
    header_row = 3
    for col, header in enumerate(headers):
        ws.write(header_row, col, header, formats["header"])
    for row_offset, item in enumerate(TASKS, start=header_row + 1):
        ws.insert_checkbox(row_offset, 0, bool(item["done"]), formats["checkbox"])
        values = [
            item["id"], item["package"], item["module"], item["work"], item["dod"], item["current"],
            item["gap"], item["priority"], item["order"], item["dependencies"], item["contract"],
            item["evidence"], item["team"], item["size"], item["source"], item["note"],
        ]
        for col, value in enumerate(values, start=1):
            fmt = formats["cell_center"] if col in {1, 3, 8, 9, 13, 14} else formats["cell"]
            ws.write(row_offset, col, value, fmt)
        ws.set_row(row_offset, 64)
    first_data = header_row + 1
    last_data = header_row + len(TASKS)
    ws.conditional_format(first_data, 0, last_data, len(headers) - 1, {
        "type": "formula", "criteria": f"=$A{first_data + 1}=TRUE", "format": formats["done_row"]
    })
    ws.conditional_format(first_data, 8, last_data, 8, {"type": "text", "criteria": "containing", "value": "Must", "format": workbook.add_format({"bg_color": colors["red"], "font_color": colors["red_text"], "bold": True})})
    ws.conditional_format(first_data, 8, last_data, 8, {"type": "text", "criteria": "containing", "value": "Should", "format": workbook.add_format({"bg_color": colors["amber"], "font_color": colors["amber_text"], "bold": True})})
    ws.autofilter(header_row, 0, last_data, len(headers) - 1)
    ws.freeze_panes(first_data, 4)
    widths = [12, 11, 19, 12, 34, 38, 31, 38, 10, 12, 23, 31, 33, 20, 9, 22, 25]
    for col, width in enumerate(widths):
        ws.set_column(col, col, width)
    ws.set_row(header_row, 42)
    ws.hide_gridlines(2)

    # Overview.
    ws = workbook.add_worksheet("Tổng quan")
    ws.activate()
    add_title(
        ws,
        "KẾ HOẠCH TRIỂN KHAI — TỔNG QUAN",
        "Nguồn: docs/Module_HeThong.docx, docs/Robot_Telepresence_GiaiDoan1_GiaiPhap_TinhNang_Luong_Mockup.docx "
        "và khảo sát mã nguồn tại ngày 29/07/2026.",
        7,
        formats,
    )
    ws.set_column("A:A", 3)
    ws.set_column("B:B", 27)
    ws.set_column("C:H", 18)
    ws.write("B4", "Tổng đầu việc", formats["metric_label"])
    ws.write_formula("B5", f"=COUNTA(Backlog!$B$5:$B${last_data + 1})", formats["big_number"])
    ws.write("C4", "Đã hoàn thành", formats["metric_label"])
    ws.write_formula("C5", f'=COUNTIF(Backlog!$A$5:$A${last_data + 1},TRUE)', formats["big_number"])
    ws.write("D4", "Chưa hoàn thành", formats["metric_label"])
    ws.write_formula("D5", "=B5-C5", formats["big_number"])
    ws.write("E4", "Tiến độ", formats["metric_label"])
    ws.write_formula("E5", '=IFERROR(C5/B5,0)', workbook.add_format({**base, "bold": True, "font_size": 20, "font_color": colors["green_text"], "bg_color": colors["white"], "border": 1, "border_color": colors["mid_gray"], "align": "center", "valign": "vcenter", "num_format": "0%"}))
    ws.write("F4", "Must còn lại", formats["metric_label"])
    ws.write_formula("F5", f'=COUNTIFS(Backlog!$I$5:$I${last_data + 1},"Must",Backlog!$A$5:$A${last_data + 1},FALSE)', formats["big_number"])
    ws.write("G4", "Module chưa có", formats["metric_label"])
    ws.write_formula("G5", '=COUNTIF(\'Hiện trạng mã\'!$C$5:$C$36,"Chưa có")', formats["big_number"])
    ws.write("H4", "Điểm tích hợp", formats["metric_label"])
    ws.write_formula("H5", f"=COUNTA('Tích hợp & dữ liệu'!$A$5:$A${len(INTEGRATIONS) + 4})", formats["big_number"])
    ws.set_row(4, 48)

    ws.merge_range("B7:H7", "Cách sử dụng file", formats["section"])
    instructions = [
        "1. Sheet Backlog là danh sách đầu việc chính. Bấm checkbox cột A; dòng hoàn thành tự chuyển xanh.",
        "2. Lọc theo Gói công việc, Module, Ưu tiên, Trình tự hoặc Nhóm phụ trách để lập sprint.",
        "3. Hiện trạng mã cho biết phần nào đã có, đang mock/một phần hoặc chưa có, kèm bằng chứng file.",
        "4. Tích hợp & dữ liệu cung cấp endpoint/event/topic và payload mẫu để nối vào mã hiện tại.",
        "5. Nghiệm thu là checklist riêng cho AT-01…AT-12 và field test. Không xem simulator là bằng chứng robot thật.",
        "6. Thứ tự triển khai bắt buộc: MCU/Safety → Sensor/Nav2 → Center → Media → UI → Translation/AI → Operations.",
        "Tương thích checkbox: Excel Microsoft 365/Excel 2024 hiển thị checkbox trong ô. Ứng dụng cũ có thể hiển thị TRUE/FALSE; đặt TRUE vẫn tô xanh dòng.",
    ]
    for idx, line in enumerate(instructions, start=7):
        ws.merge_range(idx, 1, idx, 7, line, formats["note"])
        ws.set_row(idx, 29 if idx < 13 else 42)

    ws.merge_range("B16:H16", "Kết luận khảo sát mã hiện tại", formats["section"])
    summary = [
        ("Đã có nền tốt", "Center + simulator: auth cơ bản, robot registry/credential, session lock, WSS control/telemetry, LiveKit/TURN, media file/RTSP/USB, joystick an toàn, map/navigation mock và test."),
        ("Chưa phải robot production", "Không có firmware MCU, ROS 2 gateway, sensor/TF/Nav2, E-stop/collision safety, BMS/power. Đây là critical path trước khi điều khiển robot thật."),
        ("Media/audio", "Video đã có pipeline demo tốt nhưng chưa chứng minh Rockchip MPP/CSI/adaptive bitrate/72h. Audio mới ở mức truyền hai chiều, chưa AEC/NS/AGC/VAD."),
        ("Dịch/AI", "Chưa có Translation, Vision AI, Tourism RAG và UI tương ứng. Sheet tích hợp cung cấp payload mẫu để phát triển độc lập."),
        ("Vận hành production", "Thiếu RBAC đầy đủ, Redis lease/multi-instance, telemetry history, metrics/log/alert, audit command, OTA, retention/consent và field test."),
        ("Kết quả kiểm tra", "Backend 13/13, simulator 17/17 và frontend 8/8 test đều đạt; frontend production build đạt. Build có cảnh báo MediaTransport chunk 533 kB cần tối ưu sau."),
    ]
    for idx, (label, text_value) in enumerate(summary, start=16):
        ws.write(idx, 1, label, formats["metric_label"])
        ws.merge_range(idx, 2, idx, 7, text_value, formats["note"])
        ws.set_row(idx, 48)
    ws.hide_gridlines(2)
    ws.freeze_panes(3, 1)

    # Current-state audit.
    ws = workbook.add_worksheet("Hiện trạng mã")
    state_headers = ["Module", "Tên", "Đánh giá", "Mức bao phủ ước tính", "Đã có trong mã", "Còn thiếu", "Bằng chứng"]
    add_title(ws, "HIỆN TRẠNG MÃ NGUỒN THEO MODULE", "Mức bao phủ là đánh giá kỹ thuật tương đối để ưu tiên, không phải phần trăm nghiệm thu hợp đồng.", len(state_headers) - 1, formats)
    for col, header in enumerate(state_headers):
        ws.write(3, col, header, formats["header"])
    for row, values in enumerate(CURRENT_STATE, start=4):
        for col, value in enumerate(values):
            if col == 3:
                ws.write_number(row, col, value / 100, formats["percent"])
            else:
                ws.write(row, col, value, formats["cell_center"] if col in {0, 2} else formats["cell"])
        ws.set_row(row, 57)
    state_last = 3 + len(CURRENT_STATE)
    ws.conditional_format(4, 2, state_last, 2, {"type": "text", "criteria": "containing", "value": "Chưa có", "format": workbook.add_format({"bg_color": colors["red"], "font_color": colors["red_text"], "bold": True})})
    ws.conditional_format(4, 2, state_last, 2, {"type": "text", "criteria": "containing", "value": "Mock", "format": workbook.add_format({"bg_color": colors["amber"], "font_color": colors["amber_text"], "bold": True})})
    ws.conditional_format(4, 2, state_last, 2, {"type": "text", "criteria": "containing", "value": "Một phần", "format": workbook.add_format({"bg_color": colors["cyan"], "font_color": colors["blue"], "bold": True})})
    ws.conditional_format(4, 3, state_last, 3, {"type": "data_bar", "bar_color": "#4F81BD"})
    ws.autofilter(3, 0, state_last, len(state_headers) - 1)
    ws.freeze_panes(4, 2)
    for col, width in enumerate([10, 26, 14, 18, 46, 50, 42]):
        ws.set_column(col, col, width)
    ws.set_row(3, 42)
    ws.hide_gridlines(2)

    # Integration data.
    ws = workbook.add_worksheet("Tích hợp & dữ liệu")
    int_headers = ["ID", "Luồng tích hợp", "Nguồn → Đích", "Kênh", "Hiện trạng", "Endpoint / Event / Topic", "Dữ liệu mẫu", "Đầu ra mong đợi", "Cần làm để tích hợp", "Điểm vào / Nguồn"]
    add_title(ws, "MA TRẬN TÍCH HỢP VÀ DỮ LIỆU MẪU", "Payload mẫu bám contract hiện tại khi có; các dòng chưa có là contract đề xuất cần chốt/version trước khi code.", len(int_headers) - 1, formats)
    for col, header in enumerate(int_headers):
        ws.write(3, col, header, formats["header"])
    for row, values in enumerate(INTEGRATIONS, start=4):
        for col, value in enumerate(values):
            fmt = formats["json"] if col == 6 else (formats["cell_center"] if col in {0, 3, 4} else formats["cell"])
            ws.write(row, col, value, fmt)
        ws.set_row(row, 135)
    int_last = 3 + len(INTEGRATIONS)
    ws.conditional_format(4, 4, int_last, 4, {"type": "text", "criteria": "containing", "value": "Đã có", "format": workbook.add_format({"bg_color": colors["green"], "font_color": colors["green_text"], "bold": True})})
    ws.conditional_format(4, 4, int_last, 4, {"type": "text", "criteria": "containing", "value": "Một phần", "format": workbook.add_format({"bg_color": colors["amber"], "font_color": colors["amber_text"], "bold": True})})
    ws.conditional_format(4, 4, int_last, 4, {"type": "text", "criteria": "containing", "value": "Mock", "format": workbook.add_format({"bg_color": colors["amber"], "font_color": colors["amber_text"], "bold": True})})
    ws.conditional_format(4, 4, int_last, 4, {"type": "text", "criteria": "containing", "value": "Chưa có", "format": workbook.add_format({"bg_color": colors["red"], "font_color": colors["red_text"], "bold": True})})
    ws.autofilter(3, 0, int_last, len(int_headers) - 1)
    ws.freeze_panes(4, 6)
    for col, width in enumerate([10, 24, 23, 13, 13, 29, 55, 31, 42, 35]):
        ws.set_column(col, col, width)
    ws.set_row(3, 42)
    ws.hide_gridlines(2)

    # Acceptance checklist.
    ws = workbook.add_worksheet("Nghiệm thu")
    at_headers = ["Đạt", "Mã", "Kịch bản", "Điều kiện đạt", "Module liên quan", "Hiện trạng", "Bằng chứng cần lưu", "Ưu tiên", "Người kiểm", "Ngày kiểm", "Ghi chú"]
    add_title(ws, "CHECKLIST NGHIỆM THU VÀ FIELD TEST", "Bấm checkbox cột A khi kịch bản đã chạy trên đúng môi trường và có bằng chứng. Dòng đạt sẽ chuyển xanh.", len(at_headers) - 1, formats)
    for col, header in enumerate(at_headers):
        ws.write(3, col, header, formats["header"])
    for row, values in enumerate(ACCEPTANCE, start=4):
        ws.insert_checkbox(row, 0, False, formats["checkbox"])
        for col, value in enumerate(values, start=1):
            ws.write(row, col, value, formats["cell_center"] if col in {1, 4, 5, 7} else formats["cell"])
        ws.write_blank(row, 8, None, formats["cell"])
        ws.write_blank(row, 9, None, formats["cell_center"])
        ws.write_blank(row, 10, None, formats["cell"])
        ws.set_row(row, 65)
    at_last = 3 + len(ACCEPTANCE)
    ws.conditional_format(4, 0, at_last, len(at_headers) - 1, {"type": "formula", "criteria": "=$A5=TRUE", "format": formats["done_row"]})
    ws.autofilter(3, 0, at_last, len(at_headers) - 1)
    ws.freeze_panes(4, 3)
    for col, width in enumerate([9, 10, 27, 44, 24, 22, 38, 10, 18, 14, 26]):
        ws.set_column(col, col, width)
    ws.set_row(3, 42)
    ws.hide_gridlines(2)

    # Dependency gates.
    ws = workbook.add_worksheet("Phụ thuộc")
    dep_headers = ["Cổng", "Giai đoạn", "Phạm vi bắt buộc", "Điều kiện qua cổng", "Lưu ý"]
    add_title(ws, "CỔNG PHỤ THUỘC VÀ THỨ TỰ TRIỂN KHAI", "Không đảo thứ tự làm AI/UI trước khi đường điều khiển an toàn được kiểm chứng trên robot thật.", len(dep_headers) - 1, formats)
    for col, header in enumerate(dep_headers):
        ws.write(3, col, header, formats["header"])
    for row, values in enumerate(DEPENDENCIES, start=4):
        for col, value in enumerate(values):
            ws.write(row, col, value, formats["cell_center"] if col == 0 else formats["cell"])
        ws.set_row(row, 75)
    ws.set_column(0, 0, 12)
    ws.set_column(1, 1, 25)
    ws.set_column(2, 4, 48)
    ws.set_row(3, 42)
    ws.freeze_panes(4, 1)
    ws.hide_gridlines(2)

    # Source traceability.
    ws = workbook.add_worksheet("Nguồn đối chiếu")
    source_headers = ["Nhóm", "Tệp", "Phạm vi đã dùng", "Ghi chú"]
    add_title(ws, "NGUỒN ĐỐI CHIẾU", "Danh sách tài liệu và tệp mã chính dùng để đánh giá hiện trạng và lập backlog.", len(source_headers) - 1, formats)
    sources = [
        ("Yêu cầu", "docs/Module_HeThong.docx", "Module R01-R13, C01-C13, U01-U05; contract; thứ tự; DoD.", "Tài liệu thiết kế module hệ thống."),
        ("Yêu cầu", "docs/Robot_Telepresence_GiaiDoan1_GiaiPhap_TinhNang_Luong_Mockup.docx", "MVP, API/sự kiện, audio/dịch, UI mockup, AT-01…AT-12, field checklist.", "Tài liệu giải pháp và mockup giai đoạn 1."),
        ("Kiến trúc mã", "README.md; docs/architecture.md", "Cấu trúc repo, sequence, phần mock có chủ đích.", "Nguồn chính để phân biệt demo và production."),
        ("API/Contract", "docs/api.md; docs/websocket-protocol.md; src/packages/contracts/message.schema.json", "REST/WS hiện có và envelope.", "Phát hiện lệch payload vận tốc với tài liệu mockup."),
        ("Backend", "src/apps/center-backend/app", "Auth, registry, sessions, navigation, WSS hub, DB models, LiveKit token.", "Đã kiểm route và service chính."),
        ("Frontend", "src/apps/center-frontend/src", "Login, robot CRUD/config, dashboard, joystick, map, media recovery.", "Đã kiểm page/component/transport/test."),
        ("Edge simulator", "demo/robot-simulator", "Credential, WSS, motion/nav simulation, media file/RTSP/USB, device state.", "Không được xem là ROS 2/MCU robot thật."),
        ("Hạ tầng", "docker-compose.yml; src/infrastructure", "PostgreSQL, Redis, LiveKit, Coturn, app containers.", "Redis hiện chưa được dùng cho session/presence lease."),
        ("Dữ liệu mẫu", "sample-data/maps; sample-data/routes; sample-data/robots", "Map/destination/route/robot demo.", "Cần mở rộng zone/tour/version và fixtures AI/hardware."),
        ("Kiểm thử", "docs/testing.md; */tests; demo/e2e", "Unit/integration/E2E hiện có.", "Chưa có HIL/network/audio/soak/security field evidence."),
    ]
    for col, header in enumerate(source_headers):
        ws.write(3, col, header, formats["header"])
    for row, values in enumerate(sources, start=4):
        for col, value in enumerate(values):
            ws.write(row, col, value, formats["cell"])
        ws.set_row(row, 60)
    ws.set_column(0, 0, 17)
    ws.set_column(1, 1, 63)
    ws.set_column(2, 3, 62)
    ws.set_row(3, 42)
    ws.freeze_panes(4, 1)
    ws.hide_gridlines(2)

    workbook.close()
    print(OUTPUT)


if __name__ == "__main__":
    main()

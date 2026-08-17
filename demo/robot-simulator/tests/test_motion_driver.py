import errno
import json

import pytest

from simulator.client import RobotConnectionClient
from simulator.config import SimulatorConfig
from simulator.control_protocol import (
    LatestMotionSlot,
    MOTION_PROTOCOL_VERSION,
    OBSTACLE_FRONT,
    OBSTACLE_LEFT,
    OBSTACLE_REAR,
    OBSTACLE_RIGHT,
    MotionDatagram,
    SafetyInterlock,
    decode_motion_datagram,
    encode_motion_datagram,
    joy_input_active,
)
from simulator.messages import make_message
from simulator.motion import MotionSimulator
from simulator.motion_driver import (
    DisabledMotionDriver,
    MotionDisabledError,
    UnixMotionDriver,
    build_motion_driver,
)


class FakeDatagramSocket:
    def __init__(self) -> None:
        self.blocking = True
        self.sent: list[tuple[bytes, str]] = []
        self.closed = False
        self.error: OSError | None = None

    def setblocking(self, blocking: bool) -> None:
        self.blocking = blocking

    def sendto(self, payload: bytes, path: str) -> int:
        if self.error:
            raise self.error
        self.sent.append((payload, path))
        return len(payload)

    def close(self) -> None:
        self.closed = True


def test_disabled_motion_driver_is_explicitly_fail_closed() -> None:
    config = SimulatorConfig(motion_backend="disabled")
    driver = build_motion_driver(config, MotionSimulator(config))

    assert isinstance(driver, DisabledMotionDriver)
    with pytest.raises(MotionDisabledError, match="legacy /cmd_vel"):
        driver.set_velocity(0.1, 0.0)
    assert driver.watchdog() is False
    driver.stop("safe_noop")
    driver.close()


def test_motion_protocol_rejects_expired_and_non_finite_commands() -> None:
    valid = MotionDatagram(
        protocol_version=MOTION_PROTOCOL_VERSION,
        boot_id="boot-1",
        sequence=1,
        message_type="velocity",
        sent_monotonic_ns=1_000_000_000,
        ttl_ms=250,
        linear_x=0.2,
        angular_z=0.3,
    )

    with pytest.raises(ValueError, match="expired"):
        decode_motion_datagram(
            encode_motion_datagram(valid), now_ns=1_300_000_001
        )

    invalid = json.loads(encode_motion_datagram(valid))
    invalid["linear_x"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        decode_motion_datagram(
            json.dumps(invalid).encode(), now_ns=1_100_000_000
        )


def test_latest_motion_slot_keeps_only_newest_command() -> None:
    slot = LatestMotionSlot()

    def command(
        sequence: int,
        message_type: str = "velocity",
        boot_id: str = "boot-1",
    ) -> MotionDatagram:
        return MotionDatagram(
            protocol_version=MOTION_PROTOCOL_VERSION,
            boot_id=boot_id,
            sequence=sequence,
            message_type=message_type,  # type: ignore[arg-type]
            sent_monotonic_ns=1_000_000_000,
            ttl_ms=250,
            linear_x=0.2,
        )

    assert slot.stage(command(1)) is True
    assert slot.stage(command(2, "stop")) is True
    assert slot.stage(command(1)) is False

    newest = slot.take()
    assert newest is not None
    assert newest.sequence == 2
    assert newest.message_type == "stop"
    assert slot.take() is None

    restarted = command(0, boot_id="boot-2")
    assert slot.stage(restarted) is True


def test_legacy_joystick_override_ignores_resting_noise() -> None:
    assert joy_input_active(
        [0.02, -0.08, 0.0, 0.0, 0.0, 1.0],
        [0, 0],
        deadzone=0.12,
        axis_indices=(1, 2),
    ) is False
    assert joy_input_active(
        [0.0, -0.5], [0, 0], deadzone=0.12, axis_indices=(1, 2)
    ) is True
    assert joy_input_active(
        [0.0, 0.0], [0, 1], deadzone=0.12, axis_indices=(1, 2)
    ) is True


def test_safety_interlock_obstacle_has_priority_over_clear_heartbeat() -> None:
    interlock = SafetyInterlock(watchdog_ms=500)

    assert interlock.locked(now=10.0)
    assert interlock.reason(now=10.0) == "watchdog"

    interlock.update(False, now=10.0)
    assert not interlock.locked(now=10.4)

    interlock.update(True, now=10.4)
    assert interlock.locked(now=10.41)
    assert interlock.reason(now=10.41) == "obstacle"

    interlock.update(False, now=10.5)
    assert not interlock.locked(now=10.9)
    assert interlock.locked(now=11.001)
    assert interlock.reason(now=11.001) == "watchdog"


def test_safety_interlock_can_run_without_heartbeat_watchdog() -> None:
    interlock = SafetyInterlock()

    assert not interlock.locked(now=20.0)
    interlock.update(True, now=20.0)
    assert interlock.locked(now=200.0)
    interlock.update(False, now=200.0)
    assert not interlock.locked(now=500.0)


def test_safety_interlock_filters_only_blocked_directions() -> None:
    interlock = SafetyInterlock()
    interlock.update_directions(OBSTACLE_FRONT | OBSTACLE_LEFT, now=10.0)

    assert interlock.filter_velocity(0.3, 0.4, now=10.1) == (0.0, 0.0)
    assert interlock.filter_velocity(-0.2, -0.3, now=10.1) == (-0.2, -0.3)
    assert interlock.filter_velocity(-0.2, 0.4, now=10.1) == (-0.2, 0.0)
    assert interlock.filter_velocity(0.3, -0.3, now=10.1) == (0.0, -0.3)

    interlock.update_directions(OBSTACLE_REAR | OBSTACLE_RIGHT, now=10.2)
    assert interlock.filter_velocity(0.3, 0.4, now=10.3) == (0.3, 0.4)
    assert interlock.filter_velocity(-0.2, -0.3, now=10.3) == (0.0, 0.0)


def test_safety_interlock_rejects_unknown_direction_bits() -> None:
    interlock = SafetyInterlock()

    with pytest.raises(ValueError, match="direction mask"):
        interlock.update_directions(16, now=10.0)


def test_unix_motion_driver_clamps_and_never_blocks() -> None:
    transport = FakeDatagramSocket()
    now = [10.0]
    config = SimulatorConfig(
        motion_backend="ros2",
        motion_socket_path="/tmp/does-not-open-a-real-socket",
        ros_max_forward_speed=0.33,
        ros_max_reverse_speed=0.25,
        ros_max_angular_speed=0.8,
    )
    driver = UnixMotionDriver(
        config,
        transport=transport,  # type: ignore[arg-type]
        clock=lambda: now[0],
    )

    driver.set_velocity(2.0, -4.0)

    assert transport.blocking is False
    command = decode_motion_datagram(
        transport.sent[-1][0], now_ns=int(now[0] * 1_000_000_000)
    )
    assert command.message_type == "velocity"
    assert command.linear_x == 0.33
    assert command.angular_z == -0.8

    transport.error = BlockingIOError(errno.EAGAIN, "full")
    driver.set_velocity(0.1, 0.1)


def test_unix_motion_watchdog_sends_repeated_zero_stop() -> None:
    transport = FakeDatagramSocket()
    now = [20.0]
    config = SimulatorConfig(
        motion_backend="ros2",
        motion_socket_path="/tmp/motion.sock",
        motion_watchdog_ms=250,
    )
    driver = UnixMotionDriver(
        config,
        transport=transport,  # type: ignore[arg-type]
        clock=lambda: now[0],
    )
    driver.set_velocity(0.2, 0.0)
    now[0] += 0.251

    assert driver.watchdog(now[0]) is True
    stops = [
        decode_motion_datagram(payload, now_ns=int(now[0] * 1_000_000_000))
        for payload, _path in transport.sent[1:]
    ]
    assert len(stops) == 3
    assert all(command.message_type == "stop" for command in stops)
    assert all(command.linear_x == 0 for command in stops)
    assert stops[-1].reason == "edge_watchdog"


class FakeMotionDriver:
    def __init__(self) -> None:
        self.velocities: list[tuple[float, float]] = []
        self.stops: list[str] = []

    def set_velocity(self, linear_x: float, angular_z: float) -> None:
        self.velocities.append((linear_x, angular_z))

    def stop(self, reason: str = "") -> None:
        self.stops.append(reason)

    def watchdog(self, _now: float | None = None) -> bool:
        return False

    def close(self) -> None:
        return None


class FakeGatewaySocket:
    def __init__(self, messages: list[dict]) -> None:
        self.messages = messages
        self.sent: list[dict] = []

    def __aiter__(self):
        return self

    async def __anext__(self) -> str:
        if not self.messages:
            raise StopAsyncIteration
        return json.dumps(self.messages.pop(0))

    async def send(self, payload: str) -> None:
        self.sent.append(json.loads(payload))


@pytest.mark.asyncio
async def test_disabled_motion_backend_rejects_web_velocity(tmp_path) -> None:
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="disabled",
            navigation_backend="ros2",
            robot_state_file=str(tmp_path / "missing-device.json"),
            map_cache_dir=str(tmp_path / "maps"),
        )
    )
    socket = FakeGatewaySocket(
        [
            make_message(
                "control.velocity",
                "ROBOT-001",
                1,
                {"linear_x": 0.1, "angular_z": 0.0},
                "session-1",
                300,
            )
        ]
    )

    await client._receive_loop(socket)

    assert socket.sent[-1]["payload"]["status"] == "rejected"
    assert socket.sent[-1]["payload"]["error_code"] == "MOTION_DISABLED"


@pytest.mark.asyncio
async def test_ros2_backends_route_velocity_and_stop_without_simulator_fallback(
    monkeypatch,
    tmp_path,
) -> None:
    driver = FakeMotionDriver()
    monkeypatch.setattr(
        "simulator.client.build_motion_driver", lambda _config, _simulator: driver
    )
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="ros2",
            navigation_backend="ros2",
            robot_state_file=str(tmp_path / "missing-device.json"),
        )
    )
    socket = FakeGatewaySocket(
        [
            make_message(
                "control.velocity",
                "ROBOT-001",
                1,
                {"linear_x": 0.2, "angular_z": -0.4},
                "session-1",
                300,
            ),
            make_message(
                "control.stop",
                "ROBOT-001",
                2,
                {"reason": "input_released"},
                "session-1",
                300,
            ),
        ]
    )

    await client._receive_loop(socket)

    assert driver.velocities == [(0.2, -0.4)]
    assert driver.stops == ["input_released"]
    acknowledgements = [
        message
        for message in socket.sent
        if message["message_type"] == "command.ack"
    ]
    assert [message["payload"]["status"] for message in acknowledgements] == [
        "accepted",
        "completed",
    ]
    assert client.navigation.status == "idle"


@pytest.mark.asyncio
async def test_ros2_backend_samples_control_dispatch_latency(
    monkeypatch,
    tmp_path,
    caplog,
) -> None:
    driver = FakeMotionDriver()
    monkeypatch.setattr(
        "simulator.client.build_motion_driver", lambda _config, _simulator: driver
    )
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="ros2",
            navigation_backend="ros2",
            robot_state_file=str(tmp_path / "missing-device.json"),
        )
    )
    socket = FakeGatewaySocket(
        [
            make_message(
                "control.velocity",
                "ROBOT-001",
                1,
                {"linear_x": 0.2, "angular_z": 0.0},
                "session-1",
                300,
            )
        ]
    )

    with caplog.at_level("INFO", logger="simulator.gateway"):
        await client._receive_loop(socket)

    assert "control latency browser_to_edge_ms=" in caplog.text
    assert "edge_dispatch_ms=" in caplog.text


@pytest.mark.asyncio
async def test_ros2_backend_rejects_non_finite_velocity(
    monkeypatch,
    tmp_path,
) -> None:
    driver = FakeMotionDriver()
    monkeypatch.setattr(
        "simulator.client.build_motion_driver", lambda _config, _simulator: driver
    )
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="ros2",
            navigation_backend="ros2",
            robot_state_file=str(tmp_path / "missing-device.json"),
        )
    )
    socket = FakeGatewaySocket(
        [
            make_message(
                "control.velocity",
                "ROBOT-001",
                1,
                {"linear_x": float("nan"), "angular_z": 0.0},
                "session-1",
                300,
            )
        ]
    )

    await client._receive_loop(socket)

    assert driver.velocities == []
    assert driver.stops == ["invalid_velocity"]
    assert socket.sent[-1]["payload"]["status"] == "rejected"

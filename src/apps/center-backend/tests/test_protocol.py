import asyncio
from datetime import datetime, timedelta, timezone
from uuid import uuid4

import pytest

from app.api import robots as robot_api
from app.api import websockets as websocket_api
from app.api.websockets import runtime_capabilities_from_health
from app.schemas.messages import RealtimeMessage
from app.services.hub import ConnectionHub


def message(timestamp: datetime, ttl_ms: int = 300) -> RealtimeMessage:
    return RealtimeMessage(
        message_id=uuid4(),
        message_type="control.velocity",
        robot_id="ROBOT-001",
        session_id=str(uuid4()),
        sequence=1,
        timestamp=timestamp,
        ttl_ms=ttl_ms,
        payload={"linear_x": 0.4, "angular_z": 0},
    )


def test_ttl_validation() -> None:
    assert message(datetime.now(timezone.utc)).expired() is False
    assert message(datetime.now(timezone.utc) - timedelta(seconds=1)).expired() is True


def test_real_mapping_capability_requires_scan_odometry_and_lidar_tf() -> None:
    base = {
        "motion_backend": "ros2",
        "navigation_backend": "ros2",
        "nav2": "READY",
        "scan_fresh": True,
        "odometry_ready": True,
        "lidar_tf_ready": True,
    }
    ready = runtime_capabilities_from_health(base)
    assert ready["mapping"] is True
    assert ready["mapping_blockers"] == []

    missing_scan = runtime_capabilities_from_health({**base, "scan_fresh": False})
    assert missing_scan["mapping"] is False
    assert missing_scan["mapping_blockers"] == ["SCAN_STALE"]

    missing_tf = runtime_capabilities_from_health(
        {**base, "odometry_ready": False, "lidar_tf_ready": False}
    )
    assert missing_tf["mapping"] is False
    assert missing_tf["mapping_blockers"] == [
        "ODOMETRY_UNAVAILABLE",
        "LIDAR_TF_UNAVAILABLE",
    ]


def test_idle_runtime_can_start_mapping_without_running_nav2_or_slam() -> None:
    capabilities = runtime_capabilities_from_health({
        "motion_backend": "ros2",
        "navigation_backend": "ros2",
        "mode": "IDLE",
        "nav2": "STOPPED",
        "scan_fresh": False,
        "odometry_ready": False,
        "lidar_tf_ready": False,
    })

    assert capabilities["mapping"] is True
    assert capabilities["mapping_blockers"] == []
    assert capabilities["navigation"] is False


def test_mapping_does_not_require_nav2_runtime() -> None:
    capabilities = runtime_capabilities_from_health({
        "motion_backend": "ros2",
        "navigation_backend": "ros2",
        "mode": "MAPPING",
        "nav2": "MAPPING",
        "scan_fresh": True,
        "odometry_ready": True,
        "lidar_tf_ready": True,
    })

    assert capabilities["mapping"] is True
    assert capabilities["navigation"] is False


def test_simulator_capability_remains_self_contained() -> None:
    capabilities = runtime_capabilities_from_health(
        {"motion_backend": "simulator", "navigation_backend": "simulator"}
    )
    assert capabilities["mapping"] is True
    assert capabilities["source"] == "simulator"


@pytest.mark.asyncio
async def test_closing_an_obsolete_websocket_is_idempotent() -> None:
    class AlreadyClosedSocket:
        async def close(self, **_kwargs: object) -> None:
            raise RuntimeError(
                "Unexpected ASGI message 'websocket.close', after sending "
                "'websocket.close'"
            )

    await websocket_api.ws_error(
        AlreadyClosedSocket(), 4001, "replaced connection"  # type: ignore[arg-type]
    )


@pytest.mark.asyncio
async def test_session_lock_allows_only_one_controller() -> None:
    hub = ConnectionHub()
    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    first = await hub.create_session("ROBOT-001", "user-a", 60)
    assert first.status == "active"
    with pytest.raises(RuntimeError, match="robot_busy"):
        await hub.create_session("ROBOT-001", "user-b", 60)
    assert await hub.close_session(first.session_id, "user-a")
    second = await hub.create_session("ROBOT-001", "user-b", 60)
    assert second.user_id == "user-b"


@pytest.mark.asyncio
async def test_session_without_absolute_timeout_stays_active() -> None:
    hub = ConnectionHub()
    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    session = await hub.create_session("ROBOT-001", "user-a", 0)
    session.started_at = datetime.now(timezone.utc) - timedelta(days=3)

    assert hub.get_session(session.session_id) is session
    assert session.expires_at is None
    assert session.status == "active"


@pytest.mark.asyncio
async def test_session_without_control_channel_releases_robot_lock() -> None:
    hub = ConnectionHub()
    sent: list[dict] = []

    class Socket:
        async def send_json(self, data: dict) -> None:
            sent.append(data)

    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    hub.robot_sockets["ROBOT-001"] = Socket()  # type: ignore[assignment]
    abandoned = await hub.create_session("ROBOT-001", "user-a", 60)
    abandoned.started_at = datetime.now(timezone.utc) - timedelta(seconds=16)

    closed = await hub.expire_unconnected_sessions(15)

    assert [session.session_id for session in closed] == [abandoned.session_id]
    assert abandoned.status == "ended"
    assert abandoned.end_reason == "control_connect_timeout"
    assert "ROBOT-001" not in hub.robot_session
    assert robot.availability == "available"
    assert [item["message_type"] for item in sent[-3:]] == [
        "navigation.cancel", "control.stop", "media.stop",
    ]
    assert sent[-1]["message_type"] == "media.stop"

    replacement = await hub.create_session("ROBOT-001", "user-b", 60)
    replacement.control_connected = True
    replacement.started_at = datetime.now(timezone.utc) - timedelta(seconds=16)
    assert await hub.expire_unconnected_sessions(15) == []
    assert hub.robot_session["ROBOT-001"] == replacement.session_id


@pytest.mark.asyncio
async def test_robot_disconnect_uses_reconnect_grace_before_ending_session() -> None:
    hub = ConnectionHub()
    notifications: list[dict] = []

    class RobotSocket:
        async def send_json(self, _data: dict) -> None:
            pass

    class UserSocket:
        async def send_json(self, data: dict) -> None:
            notifications.append(data)

        async def close(self, **_kwargs: object) -> None:
            pass

    robot_socket = RobotSocket()
    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    hub.robot_sockets["ROBOT-001"] = robot_socket  # type: ignore[assignment]
    robot.status = "online"
    session = await hub.create_session("ROBOT-001", "user-a", 60)
    hub.session_sockets[session.session_id] = {UserSocket()}  # type: ignore[arg-type]

    await hub.unregister_robot("ROBOT-001", robot_socket)  # type: ignore[arg-type]

    assert session.status == "active"
    assert session.robot_disconnected_at is not None
    assert hub.robot_session["ROBOT-001"] == session.session_id
    assert robot.availability == "offline"
    assert notifications == []

    replacement_socket = RobotSocket()
    await hub.register_robot(
        "ROBOT-001", replacement_socket  # type: ignore[arg-type]
    )
    assert session.robot_disconnected_at is None
    assert session.status == "active"

    await hub.unregister_robot(
        "ROBOT-001", replacement_socket  # type: ignore[arg-type]
    )
    assert session.robot_disconnected_at is not None
    session.robot_disconnected_at -= timedelta(seconds=301)
    ended = await hub.expire_disconnected_sessions(300)

    assert ended == [session]
    assert session.status == "ended"
    assert session.end_reason == "robot_reconnect_timeout"
    assert "ROBOT-001" not in hub.robot_session
    assert notifications[0]["message_type"] == "session.ended"
    assert notifications[0]["payload"]["reason"] == "robot_reconnect_timeout"


@pytest.mark.asyncio
async def test_control_disconnect_reconnects_within_five_minute_grace() -> None:
    hub = ConnectionHub()
    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    session = await hub.create_session("ROBOT-001", "user-a", 0)
    session.control_ever_connected = True
    session.control_last_seen_at = datetime.now(timezone.utc)
    session.control_disconnected_at = datetime.now(timezone.utc)

    assert await hub.expire_disconnected_sessions(300) == []
    session.control_connected = True
    session.control_disconnected_at = None
    assert hub.get_session(session.session_id) is session

    session.control_connected = False
    session.control_disconnected_at = datetime.now(timezone.utc) - timedelta(seconds=301)
    ended = await hub.expire_disconnected_sessions(300)
    assert ended == [session]
    assert session.end_reason == "control_reconnect_timeout"


@pytest.mark.asyncio
async def test_missing_control_heartbeat_ends_session_after_five_minutes() -> None:
    hub = ConnectionHub()
    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    session = await hub.create_session("ROBOT-001", "user-a", 0)
    session.control_ever_connected = True
    session.control_connected = True
    session.control_last_seen_at = datetime.now(timezone.utc) - timedelta(seconds=301)

    ended = await hub.expire_disconnected_sessions(300)
    assert ended == [session]
    assert session.end_reason == "control_reconnect_timeout"


@pytest.mark.asyncio
async def test_robot_routing_targets_only_selected_socket() -> None:
    hub = ConnectionHub()
    sent: list[dict] = []

    class Socket:
        async def send_json(self, data: dict) -> None:
            sent.append(data)

    hub.robot_sockets["ROBOT-001"] = Socket()  # type: ignore[assignment]
    command = {"robot_id": "ROBOT-001", "message_type": "control.stop"}
    assert await hub.forward_to_robot("ROBOT-001", command)
    assert sent == [command]
    assert not await hub.forward_to_robot("ROBOT-002", command)


@pytest.mark.asyncio
async def test_media_lease_starts_renews_and_stops_with_session() -> None:
    hub = ConnectionHub()
    sent: list[dict] = []

    class Socket:
        async def send_json(self, data: dict) -> None:
            sent.append(data)

    robot = hub.sync_registry_robot(
        "ROBOT-001", "Test robot", "Test site", "MAP-001", enrolled=True
    )
    robot.status = "online"
    hub.robot_sockets["ROBOT-001"] = Socket()  # type: ignore[assignment]
    session = await hub.create_session("ROBOT-001", "user-a", 60)

    assert await hub.start_session_media(session, 30)
    assert sent[-1]["message_type"] == "media.start"
    assert sent[-1]["payload"]["lease_id"] == f"session:{session.session_id}"

    session.control_connected = True
    session.media_renewed_at = datetime.now(timezone.utc) - timedelta(seconds=11)
    await hub.renew_session_media_leases(30, 10)
    assert sent[-1]["message_type"] == "media.start"

    assert await hub.close_session(session.session_id, "user-a")
    assert sent[-1]["message_type"] == "media.stop"


@pytest.mark.asyncio
async def test_robot_request_waits_for_matching_simulator_response() -> None:
    hub = ConnectionHub()
    sent: list[dict] = []

    class Socket:
        async def send_json(self, data: dict) -> None:
            sent.append(data)

    hub.robot_sockets["ROBOT-001"] = Socket()  # type: ignore[assignment]
    request = asyncio.create_task(
        hub.request_robot("ROBOT-001", "configuration.get", {}, timeout_seconds=31)
    )
    await asyncio.sleep(0)
    assert sent[0]["message_type"] == "configuration.get"
    assert sent[0]["ttl_ms"] == 30_000
    request_id = sent[0]["payload"]["request_id"]
    assert not hub.resolve_robot_request(
        "ROBOT-002", request_id, {"request_id": request_id, "ok": True}
    )
    assert hub.resolve_robot_request(
        "ROBOT-001",
        request_id,
        {"request_id": request_id, "ok": True, "video_profile": "full_hd"},
    )
    assert (await request)["video_profile"] == "full_hd"


@pytest.mark.asyncio
async def test_robot_request_fails_when_simulator_is_offline() -> None:
    hub = ConnectionHub()
    with pytest.raises(ConnectionError, match="robot_offline"):
        await hub.request_robot("ROBOT-001", "configuration.get", {})


@pytest.mark.asyncio
async def test_robot_request_times_out_when_simulator_does_not_respond() -> None:
    hub = ConnectionHub()

    class Socket:
        async def send_json(self, _data: dict) -> None:
            pass

    hub.robot_sockets["ROBOT-001"] = Socket()  # type: ignore[assignment]
    with pytest.raises(asyncio.TimeoutError):
        await hub.request_robot(
            "ROBOT-001", "configuration.get", {}, timeout_seconds=0.01
        )


@pytest.mark.asyncio
async def test_media_source_scan_forwards_requested_kind(monkeypatch) -> None:
    forwarded: list[tuple[str, str, dict]] = []

    async def fake_configuration_request(
        robot_id: str,
        message_type: str,
        payload: dict,
        **_: object,
    ) -> dict:
        forwarded.append((robot_id, message_type, payload))
        return {
            "ok": True,
            "media_kind": "audio",
            "video_sources": [],
            "audio_sources": [],
        }

    monkeypatch.setattr(
        robot_api, "configuration_from_simulator", fake_configuration_request
    )

    response = await robot_api.get_robot_media_sources(
        "ROBOT-229", "audio", "user-1"
    )

    assert response["media_kind"] == "audio"
    assert forwarded == [
        ("ROBOT-229", "media.sources.get", {"media_kind": "audio"})
    ]


@pytest.mark.asyncio
async def test_speaker_scan_forwards_output_kind(monkeypatch) -> None:
    forwarded: list[tuple[str, str, dict]] = []

    async def fake_configuration_request(
        robot_id: str,
        message_type: str,
        payload: dict,
        **_: object,
    ) -> dict:
        forwarded.append((robot_id, message_type, payload))
        return {
            "ok": True,
            "media_kind": "speaker",
            "speaker_sources": [],
        }

    monkeypatch.setattr(
        robot_api, "configuration_from_simulator", fake_configuration_request
    )

    response = await robot_api.get_robot_media_sources(
        "ROBOT-229", "speaker", "user-1"
    )

    assert response["media_kind"] == "speaker"
    assert forwarded == [
        ("ROBOT-229", "media.sources.get", {"media_kind": "speaker"})
    ]

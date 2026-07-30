import asyncio
from datetime import datetime, timedelta, timezone
from uuid import uuid4

import pytest

from app.api import robots as robot_api
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
        hub.request_robot("ROBOT-001", "configuration.get", {}, timeout_seconds=1)
    )
    await asyncio.sleep(0)
    assert sent[0]["message_type"] == "configuration.get"
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

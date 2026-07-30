import hashlib
import asyncio
import json
import time

import pytest

from simulator.client import RobotConnectionClient
from simulator.config import SimulatorConfig


@pytest.mark.asyncio
async def test_camera_starts_only_while_media_lease_is_active() -> None:
    client = RobotConnectionClient(SimulatorConfig(media_enabled=True))

    class FakeMedia:
        connected = False
        connect_count = 0
        disconnect_count = 0

        async def connect(self) -> None:
            self.connect_count += 1
            self.connected = True

        async def disconnect(self) -> None:
            self.disconnect_count += 1
            self.connected = False

    media = FakeMedia()
    client.media = media  # type: ignore[assignment]
    task = asyncio.create_task(client._media_loop())
    await asyncio.sleep(0.03)
    assert media.connect_count == 0

    client._start_media_lease({"lease_id": "session:test", "ttl_seconds": 30})
    await asyncio.sleep(0.03)
    assert media.connect_count == 1
    assert media.connected

    client.media_leases["session:test"] = time.monotonic() - 1
    client.media_lease_changed.set()
    await asyncio.sleep(0.03)
    assert not media.connected

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


def test_configuration_is_owned_and_applied_by_simulator() -> None:
    client = RobotConnectionClient(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://operator:secret@camera.local:554/old",
        )
    )

    client._apply_configuration(
        {
            "device_ip": "192.168.1.40",
            "video_source_type": "rtsp",
            "video_source": "rtsp://camera.local:8554/main",
            "video_profile": "balanced",
            "rtsp_transport": "udp",
            "camera_label": "Camera sảnh",
        }
    )

    assert client.config.device_ip == "192.168.1.40"
    assert client.config.video_width == 1280
    assert client.config.video_height == 720
    assert client.config.video_bitrate == 2_500_000
    assert client.config.rtsp_transport == "udp"
    assert client.config.simulator_media_source == (
        "rtsp://operator:secret@camera.local:8554/main"
    )
    assert client._public_video_source() == "rtsp://camera.local:8554/main"


def test_enrolled_device_identity_is_loaded_from_protected_state(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    first = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-EDGE-01",
            robot_credential="edge-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )
    first._save_device_state()

    second = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_state_file=str(state_file),
        )
    )

    assert second.config.robot_id == "ROBOT-EDGE-01"
    assert second.config.robot_credential == "edge-secret-at-least-sixteen"
    assert state_file.stat().st_mode & 0o777 == 0o600
    assert json.loads(state_file.read_text())["device_fingerprint"]


def test_device_state_copied_to_another_machine_is_ignored(
    tmp_path, monkeypatch
) -> None:
    state_file = tmp_path / "device.json"
    monkeypatch.setattr(
        RobotConnectionClient,
        "_device_fingerprint",
        lambda self: "robot-host-a:machine-id-a",
    )
    source = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-HOST-A",
            robot_credential="host-a-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )
    source._save_device_state()

    monkeypatch.setattr(
        RobotConnectionClient,
        "_device_fingerprint",
        lambda self: "robot-host-b:machine-id-b",
    )
    copied = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_management_address="192.168.6.229",
            robot_state_file=str(state_file),
        )
    )

    assert copied.config.robot_id == "UNASSIGNED"
    assert copied.config.robot_credential == ""
    assert not copied._state_loaded


def test_legacy_state_for_another_management_address_is_ignored(
    tmp_path,
) -> None:
    state_file = tmp_path / "device.json"
    state_file.write_text(
        json.dumps(
            {
                "robot_id": "ROBOT-HOST-A",
                "credential": "host-a-secret-at-least-sixteen",
                "configuration": {
                    "device_ip": "192.168.6.145",
                },
            }
        )
    )

    copied = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_management_address="192.168.6.229",
            robot_state_file=str(state_file),
        )
    )

    assert copied.config.robot_id == "UNASSIGNED"
    assert copied.config.robot_credential == ""
    assert not copied._state_loaded


def test_removing_loaded_state_clears_in_memory_identity(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    source = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-HOST-A",
            robot_credential="host-a-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )
    source._save_device_state()
    loaded = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_management_address="192.168.6.145",
            robot_state_file=str(state_file),
        )
    )
    state_file.unlink()

    assert loaded._reset_identity_if_state_removed()
    assert loaded.config.robot_id == "UNASSIGNED"
    assert loaded.config.robot_credential == ""
    assert not loaded._state_loaded


def test_non_file_state_target_is_preserved_and_replaced(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    state_file.mkdir()
    (state_file / "copied-file").write_text("preserve me")
    client = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-RECOVERED",
            robot_credential="recovered-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )

    client._save_device_state()

    backups = list(tmp_path.glob("device.json.invalid-*"))
    assert state_file.is_file()
    assert len(backups) == 1
    assert (backups[0] / "copied-file").read_text() == "preserve me"


def test_center_media_configuration_is_persisted_in_device_state(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    first = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-CONFIG-01",
            robot_credential="edge-secret-at-least-sixteen",
            robot_state_file=str(state_file),
            simulator_media_source_type="test",
        )
    )
    first._apply_configuration(
        {
            "device_ip": "192.168.6.229",
            "video_source_type": "camera",
            "video_source": "/dev/video2",
            "video_profile": "balanced",
            "rtsp_transport": "tcp",
            "camera_label": "Camera USB phía trước",
            "audio_source_type": "device",
            "audio_source": "default",
            "microphone_label": "Microphone USB",
        }
    )
    first._save_device_state()

    restarted = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_state_file=str(state_file),
            simulator_media_source_type="test",
        )
    )

    assert restarted.config.robot_id == "ROBOT-CONFIG-01"
    assert restarted.config.simulator_media_source_type == "camera"
    assert restarted.config.simulator_media_source == "/dev/video2"
    assert restarted.config.video_profile == "balanced"
    assert restarted.config.simulator_audio_source_type == "device"
    assert restarted.config.simulator_audio_source == "default"


def test_device_microphone_requires_a_selected_alsa_source() -> None:
    client = RobotConnectionClient(SimulatorConfig())

    with pytest.raises(ValueError, match="chọn nguồn microphone"):
        client._apply_configuration(
            {
                "device_ip": "192.168.6.229",
                "video_source_type": "test",
                "video_source": "generated://test-pattern",
                "video_profile": "balanced",
                "rtsp_transport": "tcp",
                "camera_label": "Camera kiểm thử",
                "audio_source_type": "device",
                "audio_source": "",
                "microphone_label": "Microphone USB",
            }
        )


def test_saving_bluetooth_microphone_applies_hsp_profile(monkeypatch) -> None:
    prepared_sources: list[str] = []
    monkeypatch.setattr(
        "simulator.client.prepare_audio_source",
        lambda source: prepared_sources.append(source) or None,
    )
    client = RobotConnectionClient(SimulatorConfig())

    client._apply_configuration(
        {
            "device_ip": "192.168.6.145",
            "video_source_type": "test",
            "video_source": "generated://test-pattern",
            "video_profile": "balanced",
            "rtsp_transport": "tcp",
            "camera_label": "Camera kiểm thử",
            "audio_source_type": "device",
            "audio_source": (
                "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
            ),
            "microphone_label": "A11ULTIMATE",
        }
    )

    assert prepared_sources == [
        "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
    ]


@pytest.mark.asyncio
async def test_silent_audio_is_not_reported_as_working_microphone() -> None:
    client = RobotConnectionClient(SimulatorConfig())

    result = await client._probe_media(
        {
            "media_kind": "audio",
            "configuration": {"audio_source_type": "silent"},
        }
    )

    assert result["ok"] is False
    assert result["detail"] == "Chưa chọn microphone để kiểm tra"


@pytest.mark.asyncio
async def test_device_audio_probe_requires_a_real_signal(monkeypatch) -> None:
    client = RobotConnectionClient(SimulatorConfig())
    monkeypatch.setattr(
        "simulator.client.probe_audio_source",
        lambda source: (False, "Không thu được tín hiệu microphone (-91.0 dB)"),
    )

    result = await client._probe_media(
        {
            "media_kind": "audio",
            "configuration": {
                "audio_source_type": "device",
                "audio_source": "plughw:CARD=Camera,DEV=0",
            },
        }
    )

    assert result["ok"] is False
    assert result["detail"] == "Không thu được tín hiệu microphone (-91.0 dB)"


@pytest.mark.asyncio
async def test_video_probe_identifies_an_active_rtsp_stream(monkeypatch) -> None:
    client = RobotConnectionClient(SimulatorConfig())

    class Process:
        returncode = 0

        async def communicate(self) -> tuple[bytes, bytes]:
            return b"", b""

    async def create_process(*args, **kwargs):
        return Process()

    monkeypatch.setattr(asyncio, "create_subprocess_exec", create_process)

    result = await client._probe_media(
        {
            "media_kind": "video",
            "configuration": {
                "video_source_type": "rtsp",
                "video_source": "rtsp://camera.local/live",
                "rtsp_transport": "tcp",
            },
        }
    )

    assert result["ok"] is True
    assert result["detail"] == "Luồng RTSP đã trả về hình ảnh"


def test_new_enrollment_token_takes_priority_over_existing_device_state(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    previous = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-OLD",
            robot_credential="old-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )
    previous._used_enrollment_token_hash = hashlib.sha256(
        b"old-enrollment-token"
    ).hexdigest()
    previous._save_device_state()

    replacement = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="environment-secret-at-least-sixteen",
            robot_enrollment_token="new-enrollment-token-at-least-thirty-two-characters",
            robot_state_file=str(state_file),
        )
    )

    assert replacement.config.robot_id == "UNASSIGNED"
    assert replacement.config.robot_enrollment_token.startswith("new-enrollment")


def test_used_enrollment_token_reuses_saved_device_state(tmp_path) -> None:
    state_file = tmp_path / "device.json"
    token = "one-time-enrollment-token-at-least-thirty-two-characters"
    enrolled = RobotConnectionClient(
        SimulatorConfig(
            robot_id="ROBOT-EDGE-02",
            robot_credential="edge-secret-at-least-sixteen",
            robot_state_file=str(state_file),
        )
    )
    enrolled._used_enrollment_token_hash = hashlib.sha256(token.encode()).hexdigest()
    enrolled._save_device_state()

    restarted = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_enrollment_token=token,
            robot_state_file=str(state_file),
        )
    )

    assert restarted.config.robot_id == "ROBOT-EDGE-02"
    assert restarted.config.robot_credential == "edge-secret-at-least-sixteen"
    assert restarted.config.robot_enrollment_token == ""


@pytest.mark.asyncio
async def test_edge_claims_pending_robot_with_local_credentials(
    tmp_path, monkeypatch
) -> None:
    requests: list[tuple[str, dict]] = []

    class Response:
        status_code = 200

        @staticmethod
        def raise_for_status() -> None:
            return None

        @staticmethod
        def json() -> dict:
            return {
                "robot_id": "ROBOT-CLAIMED",
                "credential": "claimed-secret-at-least-sixteen",
            }

    class Client:
        async def __aenter__(self):
            return self

        async def __aexit__(self, *_args):
            return None

        async def post(self, path: str, json: dict):
            requests.append((path, json))
            return Response()

    monkeypatch.setattr(
        "simulator.client.httpx.AsyncClient",
        lambda **_kwargs: Client(),
    )
    state_file = tmp_path / "claimed-device.json"
    edge = RobotConnectionClient(
        SimulatorConfig(
            robot_id="UNASSIGNED",
            robot_credential="",
            robot_management_address="192.168.50.27",
            robot_username="robot-operator",
            robot_password="local-device-password",
            robot_state_file=str(state_file),
        )
    )

    await edge._claim()

    assert requests[0][0] == "/api/robot-auth/claim"
    assert requests[0][1]["management_address"] == "192.168.50.27"
    assert edge.config.robot_id == "ROBOT-CLAIMED"
    assert edge.config.robot_credential == "claimed-secret-at-least-sixteen"
    assert state_file.stat().st_mode & 0o777 == 0o600

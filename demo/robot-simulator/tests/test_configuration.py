import asyncio
import hashlib
import json
import time
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest
from simulator.client import (
    RobotConnectionClient,
    localization_pose_safe_to_persist,
    mapping_autosave_recovery,
)
from simulator.config import SimulatorConfig
from simulator.map_cache import MapCacheError
from simulator.media import MediaPublisher, SourceVideoProbe
from simulator.messages import make_message
from simulator.navigation_backends import NavigationBackendError


def test_pose_persistence_requires_sustained_independent_localization_evidence() -> None:
    state = {
        "localized": True,
        "localization_state": "READY",
        "localization_verification_version": 2,
        "localization_diagnostics": {
            "scan_map_score": 0.55,
            "scan_map_threshold": 0.35,
            "ready_evidence_hold_ms": 30_001,
            "pose_stability": {"passed": True},
            "sensor_time": {"clock_state": "SYNCED"},
        },
    }

    assert localization_pose_safe_to_persist(state)
    state["localization_verification_version"] = 1
    assert not localization_pose_safe_to_persist(state)
    state["localization_verification_version"] = 2
    state["localization_diagnostics"]["ready_evidence_hold_ms"] = 29_999
    assert not localization_pose_safe_to_persist(state)
    state["localization_diagnostics"]["ready_evidence_hold_ms"] = 30_001
    state["localization_diagnostics"]["scan_map_score"] = 0.34
    assert not localization_pose_safe_to_persist(state)


def _write_autosave_generation(
    autosave: Path,
    map_id: str,
    version: int,
    generation: str,
    *,
    publish: bool = True,
) -> Path:
    root = autosave / map_id
    destination = root / "generations" / generation
    destination.mkdir(parents=True)
    artifacts = {
        "map.yaml": b"image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        "map.pgm": b"P5\n1 1\n255\n\xff",
        "posegraph.posegraph": b"posegraph",
        "posegraph.data": b"scan-data",
    }
    for filename, content in artifacts.items():
        (destination / filename).write_bytes(content)
    manifest = {
        "schema_version": 1,
        "map_id": map_id,
        "version": version,
        "generation": generation,
        "terminal_pose": {"x": 1.25, "y": -0.5, "yaw": 0.75},
        "files": {
            filename: hashlib.sha256(content).hexdigest()
            for filename, content in artifacts.items()
        },
    }
    (destination / "manifest.json").write_text(json.dumps(manifest))
    if publish:
        (root / "latest.json").write_text(json.dumps({"generation": generation}))
    return destination / "posegraph"


def test_mapping_autosave_recovery_requires_a_committed_verified_generation(
    tmp_path,
) -> None:
    cache = tmp_path / "maps" / "cache"
    autosave = tmp_path / "maps" / ".autosave"
    autosave.mkdir(parents=True)

    with pytest.raises(MapCacheError, match="complete committed mapping autosave"):
        mapping_autosave_recovery(str(cache), "MAP-RECOVERY", 1)

    basename = _write_autosave_generation(
        autosave, "MAP-RECOVERY", 1, "generation-1"
    )
    recovery = mapping_autosave_recovery(str(cache), "MAP-RECOVERY", 1)
    assert recovery.posegraph_path == basename
    assert recovery.initial_pose == {"x": 1.25, "y": -0.5, "yaw": 0.75}
    assert recovery.generation == "generation-1"

    # An interrupted next write is not visible until latest.json is replaced.
    _write_autosave_generation(
        autosave, "MAP-RECOVERY", 1, "generation-2", publish=False
    )
    assert mapping_autosave_recovery(
        str(cache), "MAP-RECOVERY", 1
    ).generation == "generation-1"

    basename.with_suffix(".data").write_bytes(b"corrupt")
    with pytest.raises(MapCacheError, match="complete committed mapping autosave"):
        mapping_autosave_recovery(str(cache), "MAP-RECOVERY", 1)

    with pytest.raises(MapCacheError, match="invalid map identity"):
        mapping_autosave_recovery(str(cache), "../escape", 1)


@pytest.mark.asyncio
async def test_mapping_recovery_dispatches_local_autosave_as_mapping_start(tmp_path) -> None:
    cache = tmp_path / "maps" / "cache"
    autosave = tmp_path / "maps" / ".autosave"
    autosave.mkdir(parents=True)
    basename = _write_autosave_generation(
        autosave, "MAP-RECOVERY", 1, "generation-1"
    )
    client = RobotConnectionClient(SimulatorConfig(
        navigation_backend="ros2",
        map_cache_dir=str(cache),
        media_enabled=False,
    ))
    dispatched: list[dict] = []

    class FakeNavigationBackend:
        def state(self) -> dict:
            return {"state": "IDLE", "mode": "MAPPING"}

        async def execute(self, command: str, payload: dict) -> dict:
            dispatched.append({"command": command, "payload": payload})
            return {
                "status": "completed",
                "current_state": "PAUSED" if command == "mapping.pause" else "MAPPING_RUNNING",
            }

    client.navigation_backend = FakeNavigationBackend()  # type: ignore[assignment]
    client.map_registry_ready.set()

    result = await client._execute_navigation_command(
        "mapping.recover",
        {
            "map_id": "MAP-RECOVERY",
            "version": 1,
            "expected_state": "IDLE",
        },
    )

    assert result["current_state"] == "PAUSED"
    assert result["recovered_from_autosave"] is True
    assert [item["command"] for item in dispatched] == ["mapping.start", "mapping.pause"]
    assert dispatched[0]["payload"]["posegraph_path"] == str(basename)
    assert dispatched[0]["payload"]["initial_pose"] == {
        "x": 1.25,
        "y": -0.5,
        "yaw": 0.75,
    }
    assert dispatched[0]["payload"]["autosave_generation"] == "generation-1"


@pytest.mark.asyncio
async def test_map_resync_queues_the_exact_local_bundle(tmp_path) -> None:
    cache = tmp_path / "maps" / "cache"
    bundle = cache / "created" / "MAP-RESYNC" / "v2" / "map-bundle.tar.gz"
    bundle.parent.mkdir(parents=True)
    bundle.write_bytes(b"complete-local-bundle")
    client = RobotConnectionClient(SimulatorConfig(
        navigation_backend="ros2",
        map_cache_dir=str(cache),
        media_enabled=False,
    ))

    class FakeNavigationBackend:
        def state(self) -> dict:
            return {"state": "IDLE"}

    def close_background(operation: Any) -> None:
        operation.close()

    client.navigation_backend = FakeNavigationBackend()  # type: ignore[assignment]
    client._spawn_background = close_background  # type: ignore[method-assign]
    client.map_registry_ready.set()

    result = await client._execute_navigation_command(
        "map.resync", {"map_id": "MAP-RESYNC", "version": 2}
    )

    assert result["sync_status"] == "SYNC_PENDING"
    marker = bundle.with_name(".upload-pending.json")
    assert json.loads(marker.read_text()) == {
        "map_id": "MAP-RESYNC",
        "version": 2,
        "robot_id": client.config.robot_id,
        "bundle_path": str(bundle),
    }
    assert not list(bundle.parent.glob("..upload-pending.json.*.tmp"))

    bundle.unlink()
    with pytest.raises(NavigationBackendError, match="bundle local") as failure:
        await client._execute_navigation_command(
            "map.resync", {"map_id": "MAP-RESYNC", "version": 2}
        )
    assert failure.value.code == "LOCAL_MAP_MISSING"


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


@pytest.mark.asyncio
async def test_live_camera_picker_returns_only_probed_working_sources(
    monkeypatch,
) -> None:
    active = [
        {"type": "camera", "value": "/dev/video0", "label": "USB Camera"}
    ]
    rejected = [
        {
            "type": "camera",
            "value": "/dev/video1",
            "label": "USB metadata",
            "reason": "Không có frame",
        }
    ]
    monkeypatch.setattr(
        "simulator.client.discover_video_sources", lambda: (active, rejected)
    )
    client = RobotConnectionClient(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
        )
    )

    class Socket:
        sent: list[str] = []

        async def send(self, message: str) -> None:
            self.sent.append(message)

    socket = Socket()
    await client._camera_sources(socket, "request-camera-list")
    message = json.loads(socket.sent[0])

    assert message["payload"]["video_sources"] == active
    assert message["payload"]["selected_source"] == "/dev/video0"


@pytest.mark.asyncio
async def test_live_camera_picker_returns_selected_leased_camera_without_probe(
    monkeypatch,
) -> None:
    active = [
        {"type": "camera", "value": "/dev/video0", "label": "USB Camera"}
    ]
    calls: list[set[str] | None] = []

    def discover(
        known_active_sources: set[str] | None = None,
    ) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
        calls.append(known_active_sources)
        return active, []

    monkeypatch.setattr("simulator.client.discover_video_sources", discover)
    client = RobotConnectionClient(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
        )
    )
    client._start_media_lease({"lease_id": "session:test", "ttl_seconds": 30})

    class Socket:
        sent: list[str] = []

        async def send(self, message: str) -> None:
            self.sent.append(message)

    await client._camera_sources(Socket(), "request-camera-list")

    assert calls == []


@pytest.mark.asyncio
async def test_slow_camera_inventory_does_not_block_command_receive_loop(
    monkeypatch,
) -> None:
    client = RobotConnectionClient(SimulatorConfig())
    camera_started = asyncio.Event()
    release_camera = asyncio.Event()
    ping_handled = asyncio.Event()

    async def slow_camera_sources(_socket, _request_id: str) -> None:
        camera_started.set()
        await release_camera.wait()

    async def diagnostics_result(
        _socket, _request_id: str, _kind: str, _result: dict
    ) -> None:
        ping_handled.set()

    monkeypatch.setattr(client, "_camera_sources", slow_camera_sources)
    monkeypatch.setattr(client, "_diagnostics_result", diagnostics_result)

    class Socket:
        def __aiter__(self):
            async def messages():
                yield json.dumps(make_message(
                    "media.cameras.get",
                    client.config.robot_id,
                    1,
                    {"request_id": "camera-request"},
                ))
                yield json.dumps(make_message(
                    "diagnostics.ping",
                    client.config.robot_id,
                    2,
                    {"request_id": "ping-request"},
                ))

            return messages()

    await client._receive_loop(Socket())
    await asyncio.wait_for(camera_started.wait(), timeout=0.2)
    await asyncio.wait_for(ping_handled.wait(), timeout=0.2)
    assert not release_camera.is_set()

    release_camera.set()
    await asyncio.gather(*tuple(client._background_tasks))


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
    assert client.config.video_bitrate == 2_000_000
    assert client.config.rtsp_transport == "udp"
    assert client.config.simulator_media_source == (
        "rtsp://operator:secret@camera.local:8554/main"
    )
    assert client._public_video_source() == "rtsp://camera.local:8554/main"


def test_configuration_does_not_copy_rtsp_credentials_to_another_camera() -> None:
    client = RobotConnectionClient(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://operator:secret@192.168.6.142/live",
        )
    )

    client._apply_configuration(
        {
            "device_ip": "192.168.6.145",
            "video_source_type": "rtsp",
            "video_source": "rtsp://192.168.6.128/main",
            "video_profile": "full_hd",
            "rtsp_transport": "tcp",
            "camera_label": "Camera 128",
        }
    )

    assert client.config.simulator_media_source == "rtsp://192.168.6.128/main"


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
            "audio_output_type": "device",
            "audio_output": "plughw:CARD=Speaker,DEV=0",
            "speaker_label": "Loa USB",
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
    assert restarted.config.simulator_audio_output_type == "device"
    assert restarted.config.simulator_audio_output == "plughw:CARD=Speaker,DEV=0"
    assert restarted.config.speaker_label == "Loa USB"


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
async def test_speaker_probe_plays_an_audible_tone(monkeypatch) -> None:
    client = RobotConnectionClient(SimulatorConfig())
    calls: list[tuple[str, bool]] = []

    def probe(output: str, *, audible: bool = False) -> tuple[bool, str]:
        calls.append((output, audible))
        return True, "Đã phát âm báo kiểm tra qua loa"

    monkeypatch.setattr("simulator.client.probe_audio_output", probe)

    result = await client._probe_media(
        {
            "media_kind": "speaker",
            "configuration": {
                "audio_output_type": "device",
                "audio_output": "plughw:CARD=Speaker,DEV=0",
            },
        }
    )

    assert result["ok"] is True
    assert calls == [("plughw:CARD=Speaker,DEV=0", True)]


@pytest.mark.asyncio
async def test_video_probe_identifies_an_active_rtsp_stream(monkeypatch) -> None:
    client = RobotConnectionClient(SimulatorConfig())
    monkeypatch.setattr(
        MediaPublisher,
        "_probe_video_source",
        lambda _publisher: SourceVideoProbe(
            codec="h264",
            width=1920,
            height=1080,
            fps=Fraction(25, 1),
            measured_fps=Fraction(25, 1),
            packet_count=63,
            pixel_format="yuv420p",
            profile="Main",
            passthrough_safe=True,
        ),
    )

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
    assert result["detail"] == "Nguồn video đã hoàn tất kiểm tra realtime"
    assert result["fps_measured"] == "25"
    assert result["route"] == "passthrough"
    assert result["encoder"] == "copy"


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

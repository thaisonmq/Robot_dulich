from pathlib import Path
from subprocess import CompletedProcess

from simulator import media_devices


ARECORD_OUTPUT = """\
**** List of CAPTURE Hardware Devices ****
card 2: Camera [USB Camera], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 3: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
"""

PULSE_OUTPUT = """\
Auto-detected sources for pulse:
  alsa_output.pci-0000_00_1f.3.analog-stereo.monitor [Monitor of Built-in Audio]
* bluez_input.41_42_FF_68_01_59.headset-head-unit [A11ULTIMATE] (none)
  bluez_output.41_42_FF_68_01_59.headset-head-unit.monitor [Monitor of A11ULTIMATE]
"""

BLUETOOTH_CARD_OUTPUT = """\
Card #4430
    Name: bluez_card.41_42_FF_68_01_59
    Properties:
        device.description = "A11ULTIMATE"
    Profiles:
        a2dp-sink: High Fidelity Playback (sinks: 1, sources: 0, priority: 16, available: yes)
        headset-head-unit-msbc: Headset Head Unit (sinks: 1, sources: 1, priority: 3, available: yes)
"""


def test_arecord_hardware_is_returned_as_plughw_source() -> None:
    assert media_devices.parse_arecord_devices(ARECORD_OUTPUT) == [
        {
            "type": "device",
            "value": "plughw:CARD=Camera,DEV=0",
            "label": "USB Camera · USB Audio",
        },
        {
            "type": "device",
            "value": "plughw:CARD=Device,DEV=0",
            "label": "USB PnP Sound Device · USB Audio",
        },
    ]


def test_pulse_scan_keeps_capture_source_but_excludes_monitors() -> None:
    assert media_devices.parse_pulse_sources(PULSE_OUTPUT) == [
        {
            "type": "pulse",
            "value": (
                "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
            ),
            "label": "A11ULTIMATE",
        }
    ]


def test_bluetooth_card_is_candidate_while_profile_is_a2dp() -> None:
    assert media_devices.parse_bluetooth_cards(BLUETOOTH_CARD_OUTPUT) == [
        {
            "type": "pulse",
            "value": (
                "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
            ),
            "label": "A11ULTIMATE · Bluetooth HSP/HFP",
        }
    ]


def test_audio_candidates_include_hardware_but_not_virtual_default(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(args[0], 0, ARECORD_OUTPUT, ""),
    )

    sources = media_devices.discover_audio_candidates()

    assert [source["value"] for source in sources] == [
        "plughw:CARD=Camera,DEV=0",
        "plughw:CARD=Device,DEV=0",
    ]


def test_audio_scan_falls_back_to_proc_asound(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: (_ for _ in ()).throw(FileNotFoundError()),
    )
    (tmp_path / "pcm").write_text(
        "02-00: USB Audio : USB Audio : playback 1 : capture 1\n"
    )
    card_path = tmp_path / "card2"
    card_path.mkdir()
    (card_path / "id").write_text("Camera\n")

    sources = media_devices.discover_audio_candidates(tmp_path)

    assert sources[0] == {
        "type": "device",
        "value": "plughw:CARD=Camera,DEV=0",
        "label": "USB Audio",
    }


def test_video_scan_uses_v4l2_names_and_natural_device_order(
    tmp_path: Path,
) -> None:
    device_root = tmp_path / "dev"
    sysfs_root = tmp_path / "sys"
    device_root.mkdir()
    for device_name in ("video10", "video2", "video0"):
        (device_root / device_name).touch()
        name_path = sysfs_root / device_name
        name_path.mkdir(parents=True)
        (name_path / "name").write_text(f"Camera {device_name}\n")

    sources = media_devices.discover_video_candidates(device_root, sysfs_root)

    assert [source["value"] for source in sources] == [
        str(device_root / "video0"),
        str(device_root / "video2"),
        str(device_root / "video10"),
    ]
    assert sources[0]["label"] == "Camera video0"


def test_audio_probe_rejects_disconnected_analog_microphone(monkeypatch) -> None:
    mixer_output = """\
numid=25,iface=CARD,name='Headphone Mic Jack'
  : values=off
numid=24,iface=CARD,name='Headset Mic Phantom Jack'
  : values=on
"""
    monkeypatch.setattr(media_devices, "_alsa_card_index", lambda card_id: 0)
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(args[0], 0, mixer_output, ""),
    )

    ok, detail = media_devices.probe_audio_source(
        "plughw:CARD=PCH,DEV=0"
    )

    assert not ok
    assert "Không phát hiện microphone cắm" in detail


def test_card_index_falls_back_to_arecord_inside_container(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(args[0], 0, ARECORD_OUTPUT, ""),
    )

    assert media_devices._alsa_card_index("Camera", tmp_path) == 2


def test_audio_probe_rejects_digital_silence(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices, "_analog_input_unavailable", lambda source: None
    )
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(
            args[0], 0, "", "max_volume: -91.0 dB"
        ),
    )

    ok, detail = media_devices.probe_audio_source(
        "plughw:CARD=Camera,DEV=0"
    )

    assert not ok
    assert "-91.0 dB" in detail


def test_audio_probe_accepts_real_signal(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices, "_analog_input_unavailable", lambda source: None
    )
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(
            args[0], 0, "", "max_volume: -37.5 dB"
        ),
    )

    ok, detail = media_devices.probe_audio_source(
        "plughw:CARD=Camera,DEV=0"
    )

    assert ok
    assert "-37.5 dB" in detail


def test_audio_probe_uses_pulse_for_bluetooth_source(monkeypatch) -> None:
    commands: list[list[str]] = []

    monkeypatch.setattr(
        media_devices, "prepare_audio_source", lambda source: None
    )
    monkeypatch.setattr(
        media_devices, "_pulse_input_unavailable", lambda source: None
    )
    monkeypatch.setattr(
        media_devices,
        "_active_bluetooth_profile",
        lambda card_name: "a2dp-sink-sbc",
    )
    restored_profiles: list[tuple[str, str]] = []
    monkeypatch.setattr(
        media_devices,
        "_set_bluetooth_profile",
        lambda card_name, profile: (
            restored_profiles.append((card_name, profile)) or None
        ),
    )

    def run(command, **kwargs):
        commands.append(command)
        return CompletedProcess(command, 0, "", "max_volume: -18.0 dB")

    monkeypatch.setattr(media_devices.subprocess, "run", run)

    ok, detail = media_devices.probe_audio_source(
        "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
    )

    assert ok
    assert "-18.0 dB" in detail
    assert commands[0][commands[0].index("-f") + 1] == "pulse"
    assert commands[0][commands[0].index("-i") + 1] == (
        "bluez_input.41_42_FF_68_01_59.headset-head-unit"
    )
    assert restored_profiles == [
        ("bluez_card.41_42_FF_68_01_59", "a2dp-sink-sbc")
    ]


def test_audio_probe_rejects_muted_pulse_microphone(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices, "prepare_audio_source", lambda source: None
    )
    monkeypatch.setattr(
        media_devices,
        "_active_bluetooth_profile",
        lambda card_name: "a2dp-sink-sbc",
    )
    monkeypatch.setattr(
        media_devices,
        "_set_bluetooth_profile",
        lambda card_name, profile: None,
    )
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: CompletedProcess(
            args[0], 0, "Mute: yes\n", ""
        ),
    )

    ok, detail = media_devices.probe_audio_source(
        "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
    )

    assert not ok
    assert "tắt tiếng" in detail
def test_scan_separates_active_and_rejected_sources() -> None:
    candidates = [
        {"type": "camera", "value": "/dev/video0", "label": "Camera 0"},
        {"type": "camera", "value": "/dev/video1", "label": "Camera 1"},
    ]

    active, rejected = media_devices._active_sources(
        candidates,
        lambda source: (
            (True, "Đã có frame")
            if source.endswith("0")
            else (False, "Không có frame")
        ),
    )

    assert [source["value"] for source in active] == ["/dev/video0"]
    assert rejected == [
        {
            "type": "camera",
            "value": "/dev/video1",
            "label": "Camera 1",
            "reason": "Không có frame",
        }
    ]

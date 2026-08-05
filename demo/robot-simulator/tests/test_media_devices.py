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

PULSE_SINK_OUTPUT = """\
Auto-detected sinks for pulse:
* alsa_output.usb-Logitech_Speaker-00.analog-stereo [Logitech USB Speaker] (none)
  bluez_output.41_42_FF_68_01_59.a2dp-sink [A11ULTIMATE] (none)
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

V4L2_MODES_OUTPUT = """\
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture
    [0]: 'MJPG' (Motion-JPEG, compressed)
        Size: Discrete 1280x720
            Interval: Discrete 0.017s (60.000 fps)
        Size: Discrete 1920x1080
            Interval: Discrete 0.033s (30.000 fps)
    [1]: 'YUYV' (YUYV 4:2:2)
        Size: Discrete 1280x720
            Interval: Discrete 0.111s (9.000 fps)
        Size: Discrete 640x480
            Interval: Discrete 0.033s (30.000 fps)
        Size: Discrete 800x600
            Interval: Discrete 0.050s (20.000 fps)
"""


def test_v4l2_scan_parses_real_discrete_capture_modes() -> None:
    modes = media_devices.parse_v4l2_modes(V4L2_MODES_OUTPUT)

    assert modes == [
        {"format": "mjpeg", "fourcc": "MJPG", "width": 1280, "height": 720, "fps": 60.0},
        {"format": "mjpeg", "fourcc": "MJPG", "width": 1920, "height": 1080, "fps": 30.0},
        {"format": "yuyv422", "fourcc": "YUYV", "width": 1280, "height": 720, "fps": 9.0},
        {"format": "yuyv422", "fourcc": "YUYV", "width": 640, "height": 480, "fps": 30.0},
        {"format": "yuyv422", "fourcc": "YUYV", "width": 800, "height": 600, "fps": 20.0},
    ]


def test_camera_mode_selection_uses_supported_mode_nearest_stream_profile() -> None:
    selected = media_devices.select_v4l2_mode(
        media_devices.parse_v4l2_modes(V4L2_MODES_OUTPUT),
        854,
        480,
        20,
    )

    assert selected == {
        "format": "yuyv422",
        "fourcc": "YUYV",
        "width": 800,
        "height": 600,
        "fps": 20.0,
    }


def test_camera_mode_selection_honours_supported_format_override() -> None:
    selected = media_devices.select_v4l2_mode(
        media_devices.parse_v4l2_modes(V4L2_MODES_OUTPUT),
        1280,
        720,
        20,
        "mjpeg",
    )

    assert selected is not None
    assert selected["format"] == "mjpeg"


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


def test_pulse_scan_returns_playback_sinks() -> None:
    assert media_devices.parse_pulse_sinks(PULSE_SINK_OUTPUT) == [
        {
            "type": "pulse",
            "value": "pulse:alsa_output.usb-Logitech_Speaker-00.analog-stereo",
            "label": "Logitech USB Speaker",
        },
        {
            "type": "pulse",
            "value": "pulse:bluez_output.41_42_FF_68_01_59.a2dp-sink",
            "label": "A11ULTIMATE",
        },
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


def test_speaker_scan_falls_back_to_proc_playback(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda *args, **kwargs: (_ for _ in ()).throw(FileNotFoundError()),
    )
    (tmp_path / "pcm").write_text(
        "02-00: USB Audio : USB Speaker : playback 1 : capture 0\n"
    )
    card_path = tmp_path / "card2"
    card_path.mkdir()
    (card_path / "id").write_text("Speaker\n")

    sources = media_devices.discover_speaker_candidates(tmp_path)

    assert sources == [
        {
            "type": "device",
            "value": "plughw:CARD=Speaker,DEV=0",
            "label": "USB Speaker",
        }
    ]


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


def test_video_probe_keeps_camera_already_used_by_live_stream(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices.subprocess,
        "run",
        lambda command, **_kwargs: CompletedProcess(
            command,
            1,
            "",
            "/dev/video0: Device or resource busy\n",
        ),
    )

    ok, detail = media_devices.probe_video_source("/dev/video0")

    assert ok
    assert "luồng trực tiếp" in detail


def test_video_scan_does_not_reopen_camera_reserved_by_live_view(
    monkeypatch,
) -> None:
    candidates = [
        {"type": "camera", "value": "/dev/video0", "label": "Camera live"},
        {"type": "camera", "value": "/dev/video2", "label": "Camera phụ"},
    ]
    probed: list[str] = []
    monkeypatch.setattr(
        media_devices, "discover_video_candidates", lambda: candidates
    )

    def probe(source: str) -> tuple[bool, str]:
        probed.append(source)
        return source == "/dev/video2", "Đã có frame"

    monkeypatch.setattr(media_devices, "probe_video_source", probe)

    active, rejected = media_devices.discover_video_sources({"/dev/video0"})

    assert [source["value"] for source in active] == [
        "/dev/video0",
        "/dev/video2",
    ]
    assert rejected == []
    assert probed == ["/dev/video2"]


def test_video_scan_payload_exposes_only_working_cameras(monkeypatch) -> None:
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
        media_devices, "discover_video_sources", lambda: (active, rejected)
    )

    result = media_devices.discover_media_sources("video")

    assert result["video_sources"] == active
    assert result["rejected_video_sources"] == []


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


def test_bluetooth_speaker_survives_profile_name_change(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices,
        "_pulse_sink_names",
        lambda: {
            "bluez_output.41_42_FF_68_01_59.headset-head-unit",
        },
    )

    output_format, output = media_devices.resolve_audio_output(
        "pulse:bluez_output.41_42_FF_68_01_59.a2dp-sink"
    )

    assert output_format == "pulse"
    assert output == "bluez_output.41_42_FF_68_01_59.headset-head-unit"


def test_pulse_output_fallback_has_conversation_sized_buffer(monkeypatch) -> None:
    monkeypatch.setattr(
        media_devices,
        "_pulse_sink_names",
        lambda: {"alsa_output.usb-speaker"},
    )

    arguments = media_devices.audio_output_args(
        "pulse:alsa_output.usb-speaker"
    )

    assert arguments == [
        "-f",
        "pulse",
        "-buffer_duration",
        "60",
        "-prebuf",
        "0",
        "-minreq",
        "960",
        "alsa_output.usb-speaker",
    ]


def test_speaker_probe_plays_short_test_tone(monkeypatch) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(
        media_devices, "_pulse_output_unavailable", lambda output: None
    )
    monkeypatch.setattr(
        media_devices,
        "audio_output_args",
        lambda output: ["-f", "alsa", output],
    )

    def run(command, **kwargs):
        commands.append(command)
        return CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(media_devices.subprocess, "run", run)

    ok, detail = media_devices.probe_audio_output(
        "plughw:CARD=Speaker,DEV=0", audible=True
    )

    assert ok
    assert "âm báo" in detail
    assert "sine=frequency=880:sample_rate=48000" in commands[0]
    assert commands[0][-3:] == ["-f", "alsa", "plughw:CARD=Speaker,DEV=0"]


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

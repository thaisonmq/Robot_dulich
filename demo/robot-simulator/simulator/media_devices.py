from concurrent.futures import ThreadPoolExecutor
import re
import subprocess
import time
from pathlib import Path
from typing import Any, Callable


_ARECORD_DEVICE = re.compile(
    r"^card\s+(?P<card_index>\d+):\s*"
    r"(?P<card_id>[^\s]+)\s+\[(?P<card_name>.*?)\],\s*"
    r"device\s+(?P<device_index>\d+):\s*"
    r"(?P<device_id>.*?)(?:\s+\[(?P<device_name>.*?)\])?\s*$",
    re.IGNORECASE,
)
_PROC_PCM = re.compile(
    r"^(?P<card_index>\d+)-(?P<device_index>\d+):\s*"
    r"(?P<device_id>.*?)\s*:\s*(?P<device_name>.*?)\s*:\s*(?P<capabilities>.*)$"
)
_ALSA_DEVICE = re.compile(
    r"^plughw:CARD=(?P<card_id>[^,]+),DEV=(?P<device_index>\d+)$"
)
_MAX_VOLUME = re.compile(r"max_volume:\s*(?P<value>-?inf|-?\d+(?:\.\d+)?)\s*dB")
_AUDIO_SIGNAL_FLOOR_DB = -85.0
_PULSE_PREFIX = "pulse:"
_PULSE_PLAYBACK_BUFFER_MS = 60
_PULSE_PLAYBACK_MIN_REQUEST_BYTES = 960
_BLUETOOTH_SOURCE = re.compile(
    r"^bluez_input\.(?P<device>.+)\.headset-head-unit$"
)
_BLUETOOTH_SINK = re.compile(
    r"^bluez_output\.(?P<device>.+)\.(?P<profile>.+)$"
)
_BLUETOOTH_PROFILES = (
    "headset-head-unit-msbc",
    "headset-head-unit",
    "headset-head-unit-cvsd",
)


def _unique_sources(sources: list[dict[str, str]]) -> list[dict[str, str]]:
    unique: list[dict[str, str]] = []
    seen: set[str] = set()
    for source in sources:
        if source["value"] in seen:
            continue
        seen.add(source["value"])
        unique.append(source)
    return unique


def _device_sort_key(path: Path) -> tuple[str, int]:
    match = re.search(r"(\d+)$", path.name)
    return (path.name.rstrip("0123456789"), int(match.group(1)) if match else -1)


def discover_video_candidates(
    device_root: Path = Path("/dev"),
    sysfs_root: Path = Path("/sys/class/video4linux"),
) -> list[dict[str, str]]:
    sources: list[dict[str, str]] = []
    devices = sorted(
        (
            path
            for path in device_root.glob("video*")
            if re.fullmatch(r"video\d+", path.name)
        ),
        key=_device_sort_key,
    )
    for device in devices:
        label = ""
        try:
            label = (sysfs_root / device.name / "name").read_text().strip()
        except OSError:
            pass
        sources.append(
            {
                "type": "camera",
                "value": str(device),
                "label": label or f"Camera {device.name}",
            }
        )
    return sources


def parse_arecord_devices(output: str) -> list[dict[str, str]]:
    sources: list[dict[str, str]] = []
    for raw_line in output.splitlines():
        match = _ARECORD_DEVICE.match(raw_line.strip())
        if not match:
            continue
        card_id = match.group("card_id").strip()
        device_index = int(match.group("device_index"))
        value = f"plughw:CARD={card_id},DEV={device_index}"
        card_name = match.group("card_name").strip()
        device_name = (match.group("device_name") or match.group("device_id")).strip()
        label_parts = [part for part in (card_name, device_name) if part]
        sources.append(
            {
                "type": "device",
                "value": value,
                "label": " · ".join(label_parts) or f"ALSA {card_id}",
            }
        )
    return _unique_sources(sources)


def parse_aplay_devices(output: str) -> list[dict[str, str]]:
    """Parse ALSA playback hardware using the same layout as ``arecord -l``."""
    return parse_arecord_devices(output)


def parse_pulse_sources(output: str) -> list[dict[str, str]]:
    """Parse capture sources exposed by PulseAudio or PipeWire's Pulse server."""
    sources: list[dict[str, str]] = []
    for raw_line in output.splitlines():
        match = re.match(
            r"^\s*\*?\s*(?P<name>\S+)\s+\[(?P<label>.+)]"
            r"(?:\s+\([^)]*\))?\s*$",
            raw_line,
        )
        if not match:
            continue
        name = match.group("name").strip()
        label = match.group("label").strip()
        if name.endswith(".monitor") or label.lower().startswith("monitor of "):
            continue
        sources.append(
            {
                "type": "pulse",
                "value": f"{_PULSE_PREFIX}{name}",
                "label": label,
            }
        )
    return _unique_sources(sources)


def parse_pulse_sinks(output: str) -> list[dict[str, str]]:
    """Parse playback sinks exposed by PulseAudio or PipeWire's Pulse server."""
    sources: list[dict[str, str]] = []
    for raw_line in output.splitlines():
        match = re.match(
            r"^\s*\*?\s*(?P<name>\S+)\s+\[(?P<label>.+)]"
            r"(?:\s+\([^)]*\))?\s*$",
            raw_line,
        )
        if not match:
            continue
        name = match.group("name").strip()
        label = match.group("label").strip()
        sources.append(
            {
                "type": "pulse",
                "value": f"{_PULSE_PREFIX}{name}",
                "label": label,
            }
        )
    return _unique_sources(sources)


def parse_bluetooth_cards(output: str) -> list[dict[str, str]]:
    """Return possible Bluetooth microphones, including cards currently on A2DP."""
    sources: list[dict[str, str]] = []
    for card_block in re.split(r"(?=^Card #)", output, flags=re.MULTILINE):
        name_match = re.search(
            r"^\s*Name:\s*(bluez_card\.(?P<device>\S+))\s*$",
            card_block,
            re.MULTILINE,
        )
        if not name_match:
            continue
        has_capture_profile = any(
            re.search(
                rf"^\s*{re.escape(profile)}:.*sources:\s*[1-9]\d*.*"
                r"available:\s*yes",
                card_block,
                re.MULTILINE | re.IGNORECASE,
            )
            for profile in _BLUETOOTH_PROFILES
        )
        if not has_capture_profile:
            continue
        description_match = re.search(
            r'^\s*device\.description\s*=\s*"(?P<label>.*?)"\s*$',
            card_block,
            re.MULTILINE,
        )
        device = name_match.group("device")
        label = (
            description_match.group("label").strip()
            if description_match
            else device.replace("_", ":")
        )
        sources.append(
            {
                "type": "pulse",
                "value": (
                    f"{_PULSE_PREFIX}bluez_input.{device}.headset-head-unit"
                ),
                "label": f"{label} · Bluetooth HSP/HFP",
            }
        )
    return _unique_sources(sources)


def discover_pulse_sources() -> list[dict[str, str]]:
    sources: list[dict[str, str]] = []
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-sources", "pulse"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
        sources.extend(parse_pulse_sources(result.stdout))
    except (OSError, subprocess.TimeoutExpired):
        pass

    # A Bluetooth card on A2DP has no capture source yet. Keep it as a
    # candidate so the probe can temporarily select its HSP/HFP profile.
    try:
        result = subprocess.run(
            ["pactl", "list", "cards"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
        if result.returncode == 0:
            sources.extend(parse_bluetooth_cards(result.stdout))
    except (OSError, subprocess.TimeoutExpired):
        pass
    return _unique_sources(sources)


def discover_pulse_sinks() -> list[dict[str, str]]:
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-sinks", "pulse"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return parse_pulse_sinks(result.stdout)


def _discover_audio_devices_from_proc(
    proc_asound_root: Path,
    capability: str,
) -> list[dict[str, str]]:
    try:
        pcm_lines = (proc_asound_root / "pcm").read_text().splitlines()
    except OSError:
        return []

    sources: list[dict[str, str]] = []
    for line in pcm_lines:
        match = _PROC_PCM.match(line.strip())
        if not match or capability not in match.group("capabilities").lower():
            continue
        card_index = int(match.group("card_index"))
        device_index = int(match.group("device_index"))
        try:
            card_id = (
                proc_asound_root / f"card{card_index}" / "id"
            ).read_text().strip()
        except OSError:
            card_id = str(card_index)
        value = f"plughw:CARD={card_id},DEV={device_index}"
        device_name = match.group("device_name").strip()
        sources.append(
            {
                "type": "device",
                "value": value,
                "label": device_name or f"ALSA {card_id}",
            }
        )
    return _unique_sources(sources)


def _discover_audio_sources_from_proc(
    proc_asound_root: Path,
) -> list[dict[str, str]]:
    return _discover_audio_devices_from_proc(proc_asound_root, "capture")


def _discover_speaker_sources_from_proc(
    proc_asound_root: Path,
) -> list[dict[str, str]]:
    return _discover_audio_devices_from_proc(proc_asound_root, "playback")


def discover_audio_candidates(
    proc_asound_root: Path = Path("/proc/asound"),
) -> list[dict[str, str]]:
    sources: list[dict[str, str]] = []
    try:
        result = subprocess.run(
            ["arecord", "-l"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
        sources = parse_arecord_devices(result.stdout)
    except (OSError, subprocess.TimeoutExpired):
        pass

    if not sources:
        sources = _discover_audio_sources_from_proc(proc_asound_root)
    sources.extend(discover_pulse_sources())
    return _unique_sources(sources)


def discover_speaker_candidates(
    proc_asound_root: Path = Path("/proc/asound"),
) -> list[dict[str, str]]:
    sources: list[dict[str, str]] = []
    try:
        result = subprocess.run(
            ["aplay", "-l"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
        sources = parse_aplay_devices(result.stdout)
    except (OSError, subprocess.TimeoutExpired):
        pass

    if not sources:
        sources = _discover_speaker_sources_from_proc(proc_asound_root)
    # Prefer the Pulse/PipeWire entry in the UI when both it and the raw ALSA
    # device exist. Keeping both still allows headless edge images without a
    # user audio server to select the hardware path directly.
    return _unique_sources([*discover_pulse_sinks(), *sources])


def _last_error(stderr: str, source: str) -> str:
    lines = [line.strip() for line in stderr.splitlines() if line.strip()]
    detail = lines[-1] if lines else "Thiết bị không phản hồi"
    return detail.replace(source, "<thiết bị>")


def probe_video_source(source: str) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            [
                "ffmpeg",
                "-hide_banner",
                "-nostdin",
                "-loglevel",
                "error",
                "-f",
                "v4l2",
                "-i",
                source,
                "-frames:v",
                "1",
                "-an",
                "-f",
                "null",
                "-",
            ],
            capture_output=True,
            check=False,
            text=True,
            timeout=4,
        )
    except subprocess.TimeoutExpired:
        return False, "Camera không trả về frame trong thời gian kiểm tra"
    except OSError:
        return False, "Không chạy được FFmpeg để kiểm tra camera"
    if result.returncode != 0:
        # The selected camera is normally held open by the live FFmpeg process.
        # Treat an exclusive-open failure as proof that this is a real capture
        # device, otherwise refreshing the list would hide the camera in use.
        if "device or resource busy" in result.stderr.lower():
            return True, "Camera đang được luồng trực tiếp sử dụng"
        return False, _last_error(result.stderr, source)
    return True, "Camera đã trả về frame hình ảnh"


def _alsa_card_index(
    card_id: str, proc_asound_root: Path = Path("/proc/asound")
) -> int | None:
    if card_id.isdigit():
        return int(card_id)
    for id_path in proc_asound_root.glob("card*/id"):
        try:
            if id_path.read_text().strip() != card_id:
                continue
            return int(id_path.parent.name.removeprefix("card"))
        except (OSError, ValueError):
            continue
    try:
        result = subprocess.run(
            ["arecord", "-l"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    for raw_line in result.stdout.splitlines():
        match = _ARECORD_DEVICE.match(raw_line.strip())
        if match and match.group("card_id").strip() == card_id:
            return int(match.group("card_index"))
    return None


def _analog_input_unavailable(source: str) -> str | None:
    match = _ALSA_DEVICE.match(source)
    if not match:
        return None
    card_index = _alsa_card_index(match.group("card_id"))
    if card_index is None:
        return None
    try:
        result = subprocess.run(
            ["amixer", "-c", str(card_index), "contents"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None

    microphone_jacks: list[bool] = []
    has_internal_microphone = False
    for block in result.stdout.split("numid=")[1:]:
        name_match = re.search(r"name='([^']+)'", block)
        value_match = re.search(r"^\s*:\s*values=(\w+)", block, re.MULTILINE)
        if not name_match:
            continue
        name = name_match.group(1).lower()
        if "internal mic" in name:
            has_internal_microphone = True
        if "mic jack" in name and "phantom" not in name and value_match:
            microphone_jacks.append(value_match.group(1).lower() == "on")
    if microphone_jacks and not any(microphone_jacks) and not has_internal_microphone:
        return "Không phát hiện microphone cắm vào cổng analog"
    return None


def _pulse_source_names() -> set[str]:
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-sources", "pulse"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()
    return {
        source["value"].removeprefix(_PULSE_PREFIX)
        for source in parse_pulse_sources(result.stdout)
    }


def _pulse_sink_names() -> set[str]:
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-sinks", "pulse"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()
    return {
        source["value"].removeprefix(_PULSE_PREFIX)
        for source in parse_pulse_sinks(result.stdout)
    }


def _bluetooth_card_name(source: str) -> str | None:
    if not source.startswith(_PULSE_PREFIX):
        return None
    match = _BLUETOOTH_SOURCE.match(source.removeprefix(_PULSE_PREFIX))
    if not match:
        return None
    return f"bluez_card.{match.group('device')}"


def _bluetooth_sink_device(output: str) -> str | None:
    if not output.startswith(_PULSE_PREFIX):
        return None
    match = _BLUETOOTH_SINK.match(output.removeprefix(_PULSE_PREFIX))
    return match.group("device") if match else None


def resolve_audio_output(output: str) -> tuple[str, str]:
    """Resolve a configured output to an FFmpeg muxer and current device name.

    Bluetooth changes the suffix of its Pulse sink when the headset switches
    between A2DP and HSP/HFP. Match the stable device part so a saved speaker
    remains usable after enabling that headset's microphone.
    """
    if not output.startswith(_PULSE_PREFIX):
        return "alsa", output
    requested_sink = output.removeprefix(_PULSE_PREFIX)
    available_sinks = _pulse_sink_names()
    if requested_sink in available_sinks:
        return "pulse", requested_sink
    bluetooth_device = _bluetooth_sink_device(output)
    if bluetooth_device:
        prefix = f"bluez_output.{bluetooth_device}."
        replacement = next(
            (sink for sink in sorted(available_sinks) if sink.startswith(prefix)),
            None,
        )
        if replacement:
            return "pulse", replacement
    raise ValueError("Loa PipeWire/PulseAudio không còn khả dụng")


def audio_output_args(output: str) -> list[str]:
    output_format, resolved_output = resolve_audio_output(output)
    if output_format == "pulse":
        # FFmpeg's Pulse output defaults to a roughly two-second buffer. Keep
        # the fallback and the speaker diagnostic conversational instead. The
        # native runtime path uses pacat/aplay with the same latency goal.
        return [
            "-f",
            output_format,
            "-buffer_duration",
            str(_PULSE_PLAYBACK_BUFFER_MS),
            "-prebuf",
            "0",
            "-minreq",
            str(_PULSE_PLAYBACK_MIN_REQUEST_BYTES),
            resolved_output,
        ]
    return ["-f", output_format, resolved_output]


def prepare_audio_output(output: str) -> str | None:
    """Check that a logical speaker output can resolve to a current sink."""
    if not output:
        return "Hãy chọn loa trên robot"
    if not output.startswith(_PULSE_PREFIX):
        return None
    try:
        resolve_audio_output(output)
    except ValueError as exc:
        return str(exc)
    return None


def _active_bluetooth_profile(card_name: str) -> str | None:
    try:
        result = subprocess.run(
            ["pactl", "list", "cards"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    for card_block in re.split(r"(?=^Card #)", result.stdout, flags=re.MULTILINE):
        name_match = re.search(
            r"^\s*Name:\s*(?P<name>\S+)\s*$",
            card_block,
            re.MULTILINE,
        )
        if not name_match or name_match.group("name") != card_name:
            continue
        profile_match = re.search(
            r"^\s*Active Profile:\s*(?P<profile>\S+)\s*$",
            card_block,
            re.MULTILINE,
        )
        return profile_match.group("profile") if profile_match else None
    return None


def _set_bluetooth_profile(card_name: str, profile: str) -> str | None:
    try:
        result = subprocess.run(
            ["pactl", "set-card-profile", card_name, profile],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except subprocess.TimeoutExpired:
        return "Quá thời gian thay đổi profile Bluetooth"
    except OSError:
        return "Container robot thiếu pactl để thay đổi profile Bluetooth"
    if result.returncode != 0:
        return _last_error(result.stderr, card_name)
    return None


def prepare_audio_source(source: str) -> str | None:
    """Make a logical audio source available before FFmpeg opens it."""
    if not source.startswith(_PULSE_PREFIX):
        return None
    pulse_source = source.removeprefix(_PULSE_PREFIX)
    if pulse_source in _pulse_source_names():
        return None

    card_name = _bluetooth_card_name(source)
    if not card_name:
        return "Nguồn PipeWire/PulseAudio không còn khả dụng"

    last_error = ""
    for profile in _BLUETOOTH_PROFILES:
        profile_error = _set_bluetooth_profile(card_name, profile)
        if profile_error:
            last_error = profile_error
            continue
        for _ in range(5):
            time.sleep(0.1)
            if pulse_source in _pulse_source_names():
                return None
    return last_error or "Tai nghe Bluetooth chưa kết nối ở chế độ HSP/HFP"


def _pulse_input_unavailable(source: str) -> str | None:
    if not source.startswith(_PULSE_PREFIX):
        return None
    pulse_source = source.removeprefix(_PULSE_PREFIX)
    try:
        result = subprocess.run(
            ["pactl", "get-source-mute", pulse_source],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode == 0 and re.search(
        r"\b(?:yes|true)\b", result.stdout, re.IGNORECASE
    ):
        return "Microphone đang bị tắt tiếng (mute) trên máy robot"
    return None


def _pulse_output_unavailable(output: str) -> str | None:
    if not output.startswith(_PULSE_PREFIX):
        return None
    try:
        _, pulse_sink = resolve_audio_output(output)
    except ValueError as exc:
        return str(exc)
    try:
        result = subprocess.run(
            ["pactl", "get-sink-mute", pulse_sink],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode == 0 and re.search(
        r"\b(?:yes|true)\b", result.stdout, re.IGNORECASE
    ):
        return "Loa đang bị tắt tiếng (mute) trên máy robot"
    return None


def _probe_audio_source(source: str) -> tuple[bool, str]:
    unavailable_reason = _analog_input_unavailable(source)
    if unavailable_reason:
        return False, unavailable_reason
    unavailable_reason = prepare_audio_source(source)
    if unavailable_reason:
        return False, unavailable_reason
    unavailable_reason = _pulse_input_unavailable(source)
    if unavailable_reason:
        return False, unavailable_reason
    input_format = "pulse" if source.startswith(_PULSE_PREFIX) else "alsa"
    input_source = source.removeprefix(_PULSE_PREFIX)
    try:
        result = subprocess.run(
            [
                "ffmpeg",
                "-hide_banner",
                "-nostdin",
                "-loglevel",
                "info",
                "-f",
                input_format,
                "-i",
                input_source,
                "-t",
                "1.25",
                "-af",
                "volumedetect",
                "-vn",
                "-f",
                "null",
                "-",
            ],
            capture_output=True,
            check=False,
            text=True,
            timeout=4,
        )
    except subprocess.TimeoutExpired:
        return False, "Microphone không trả về dữ liệu trong thời gian kiểm tra"
    except OSError:
        return False, "Không chạy được FFmpeg để kiểm tra microphone"
    if result.returncode != 0:
        return False, _last_error(result.stderr, input_source)

    volume_match = _MAX_VOLUME.search(result.stderr)
    if not volume_match or volume_match.group("value") == "-inf":
        return False, "Microphone chỉ trả về digital silence"
    maximum_db = float(volume_match.group("value"))
    if maximum_db <= _AUDIO_SIGNAL_FLOOR_DB:
        return False, f"Không thu được tín hiệu microphone ({maximum_db:.1f} dB)"
    return True, f"Đã thu được tín hiệu microphone ({maximum_db:.1f} dB)"


def probe_audio_source(
    source: str,
    preserve_system_configuration: bool = True,
) -> tuple[bool, str]:
    """Probe a microphone without leaving the host on another audio profile."""
    card_name = _bluetooth_card_name(source)
    original_profile: str | None = None
    if preserve_system_configuration and card_name:
        original_profile = _active_bluetooth_profile(card_name)
        if not original_profile:
            return (
                False,
                "Không đọc được profile Bluetooth hiện tại; đã giữ nguyên cấu hình",
            )

    result: tuple[bool, str] = (False, "Không kiểm tra được microphone")
    try:
        result = _probe_audio_source(source)
    finally:
        if preserve_system_configuration and card_name and original_profile:
            restore_error = _set_bluetooth_profile(card_name, original_profile)
            if restore_error:
                result = (
                    False,
                    f"Không khôi phục được profile Bluetooth: {restore_error}",
                )
    return result


def probe_audio_output(
    output: str,
    *,
    audible: bool = False,
) -> tuple[bool, str]:
    """Open a speaker sink and optionally play a short, low-volume test tone."""
    unavailable_reason = _pulse_output_unavailable(output)
    if unavailable_reason:
        return False, unavailable_reason
    try:
        output_args = audio_output_args(output)
    except ValueError as exc:
        return False, str(exc)
    generator = (
        "sine=frequency=880:sample_rate=48000"
        if audible
        else "anullsrc=r=48000:cl=mono"
    )
    command = [
        "ffmpeg",
        "-hide_banner",
        "-nostdin",
        "-loglevel",
        "error",
        "-f",
        "lavfi",
        "-i",
        generator,
        "-t",
        "0.45" if audible else "0.2",
        "-ac",
        "1",
    ]
    if audible:
        command.extend(["-filter:a", "volume=0.18"])
    command.extend(output_args)
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
    except subprocess.TimeoutExpired:
        return False, "Loa không nhận dữ liệu trong thời gian kiểm tra"
    except OSError:
        return False, "Không chạy được FFmpeg để kiểm tra loa"
    if result.returncode != 0:
        return False, _last_error(result.stderr, output)
    if audible:
        return True, "Đã phát âm báo kiểm tra qua loa"
    return True, "Loa đã nhận luồng âm thanh kiểm tra"


def _active_sources(
    candidates: list[dict[str, str]],
    probe: Callable[[str], tuple[bool, str]],
    *,
    max_workers: int = 4,
) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    if not candidates:
        return [], []
    worker_count = min(max_workers, len(candidates))
    with ThreadPoolExecutor(max_workers=worker_count) as executor:
        results = list(executor.map(lambda item: probe(item["value"]), candidates))
    active: list[dict[str, str]] = []
    rejected: list[dict[str, str]] = []
    for source, (is_active, detail) in zip(candidates, results):
        if is_active:
            active.append(source)
        else:
            rejected.append({**source, "reason": detail})
    return active, rejected


def discover_video_sources() -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    # Probe sequentially. Multiple /dev/videoN nodes can belong to one physical
    # UVC camera and opening them concurrently can make a valid node look busy.
    return _active_sources(
        discover_video_candidates(), probe_video_source, max_workers=1
    )


def discover_audio_sources() -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    return _active_sources(discover_audio_candidates(), probe_audio_source)


def discover_speaker_sources() -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    # Raw ALSA and Pulse entries can refer to the same physical device. Probe
    # them sequentially so the direct ALSA open does not race the sound server.
    return _active_sources(
        discover_speaker_candidates(), probe_audio_output, max_workers=1
    )


def discover_media_sources(media_kind: str = "all") -> dict[str, Any]:
    video_sources: list[dict[str, str]] = []
    rejected_video_sources: list[dict[str, str]] = []
    audio_sources: list[dict[str, str]] = []
    rejected_audio_sources: list[dict[str, str]] = []
    speaker_sources: list[dict[str, str]] = []
    rejected_speaker_sources: list[dict[str, str]] = []
    if media_kind in {"all", "video"}:
        video_sources, _rejected_video_sources = discover_video_sources()
        # Camera pickers must contain only usable sources. Raspberry Pi exposes
        # many codec, ISP and metadata nodes under /dev/video*; returning them as
        # rejected entries still makes some Center views render them as choices.
        rejected_video_sources = []
    if media_kind in {"all", "audio"}:
        audio_sources, rejected_audio_sources = discover_audio_sources()
    if media_kind in {"all", "speaker"}:
        speaker_sources, rejected_speaker_sources = discover_speaker_sources()
    return {
        "video_sources": video_sources,
        "audio_sources": audio_sources,
        "speaker_sources": speaker_sources,
        "rejected_video_sources": rejected_video_sources,
        "rejected_audio_sources": rejected_audio_sources,
        "rejected_speaker_sources": rejected_speaker_sources,
    }

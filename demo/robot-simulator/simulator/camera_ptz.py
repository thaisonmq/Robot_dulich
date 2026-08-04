from __future__ import annotations

import base64
import asyncio
import hashlib
import os
import re
import socket
import subprocess
import time
from uuid import uuid4
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any
from urllib.parse import quote, unquote, urlsplit, urlunsplit
from xml.etree import ElementTree
from xml.sax.saxutils import escape

import httpx


_V4L2_CONTROL = re.compile(
    r"^\s*(?P<name>[a-zA-Z0-9_]+)\s+0x[0-9a-fA-F]+\s+"
    r"\([^)]+\)\s*:\s*(?P<details>.*)$"
)
_V4L2_VALUE = re.compile(r"\b(min|max|step|default|value)=(-?\d+)\b")
_SPEEDS = {"slow": 0.25, "medium": 0.55, "fast": 1.0}
_PAN_CONTROLS = ("pan_speed", "pan_relative", "pan_absolute")
_TILT_CONTROLS = ("tilt_speed", "tilt_relative", "tilt_absolute")
_ZOOM_CONTROLS = ("zoom_continuous", "zoom_relative", "zoom_absolute")


@dataclass(frozen=True, slots=True)
class V4L2Control:
    name: str
    minimum: int
    maximum: int
    step: int
    value: int


@dataclass(frozen=True, slots=True)
class OnvifTarget:
    media_url: str
    ptz_url: str
    profile_token: str
    username: str
    password: str
    pan: bool = True
    tilt: bool = True
    zoom: bool = True


def parse_v4l2_controls(output: str) -> dict[str, V4L2Control]:
    controls: dict[str, V4L2Control] = {}
    for line in output.splitlines():
        match = _V4L2_CONTROL.match(line)
        if not match:
            continue
        values = {
            name: int(value)
            for name, value in _V4L2_VALUE.findall(match.group("details"))
        }
        controls[match.group("name")] = V4L2Control(
            name=match.group("name"),
            minimum=values.get("min", 0),
            maximum=values.get("max", 0),
            step=max(1, values.get("step", 1)),
            value=values.get("value", values.get("default", 0)),
        )
    return controls


def _read_v4l2_controls(device: str) -> dict[str, V4L2Control]:
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-ctrls"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {}
    if result.returncode != 0:
        return {}
    return parse_v4l2_controls(result.stdout)


def usb_ptz_capabilities(device: str) -> dict[str, Any]:
    controls = _read_v4l2_controls(device)
    pan = any(name in controls for name in _PAN_CONTROLS)
    tilt = any(name in controls for name in _TILT_CONTROLS)
    zoom = any(name in controls for name in _ZOOM_CONTROLS)
    return {
        "supported": pan or tilt or zoom,
        "pan": pan,
        "tilt": tilt,
        "zoom": zoom,
        "transport": "uvc",
    }


def _speed_value(value: object) -> float:
    return _SPEEDS.get(str(value), _SPEEDS["medium"])


def _directional_value(
    control: V4L2Control,
    direction: float,
    speed: float,
) -> int:
    if not direction:
        return 0
    limit = control.maximum if direction > 0 else abs(control.minimum)
    raw_magnitude = max(control.step, round(limit * speed))
    magnitude = max(control.step, round(raw_magnitude / control.step) * control.step)
    value = magnitude if direction > 0 else -magnitude
    return max(control.minimum, min(control.maximum, value))


def _absolute_step(
    control: V4L2Control,
    direction: float,
    speed: float,
) -> int:
    span = max(control.step, control.maximum - control.minimum)
    raw_increment = max(control.step, round(span * (0.025 + speed * 0.05)))
    increment = max(
        control.step,
        round(raw_increment / control.step) * control.step,
    )
    value = control.value + (increment if direction > 0 else -increment)
    return max(control.minimum, min(control.maximum, value))


def _axis_setting(
    controls: dict[str, V4L2Control],
    names: tuple[str, ...],
    direction: float,
    speed: float,
) -> tuple[str, int] | None:
    for name in names:
        control = controls.get(name)
        if control is None:
            continue
        if name.endswith("_absolute"):
            return name, _absolute_step(control, direction, speed)
        return name, _directional_value(control, direction, speed)
    return None


def set_usb_ptz(device: str, payload: dict[str, Any]) -> bool:
    controls = _read_v4l2_controls(device)
    if not controls:
        return False
    operation = str(payload.get("operation", ""))
    speed = _speed_value(payload.get("speed", "medium"))
    settings: list[tuple[str, int]] = []

    if operation == "move":
        try:
            pan = max(-1.0, min(1.0, float(payload.get("pan", 0))))
            tilt = max(-1.0, min(1.0, float(payload.get("tilt", 0))))
        except (TypeError, ValueError):
            return False
        if pan:
            setting = _axis_setting(controls, _PAN_CONTROLS, pan, speed)
            if setting:
                settings.append(setting)
        if tilt:
            setting = _axis_setting(controls, _TILT_CONTROLS, tilt, speed)
            if setting:
                settings.append(setting)
    elif operation == "zoom":
        try:
            zoom = max(-1.0, min(1.0, float(payload.get("zoom", 0))))
        except (TypeError, ValueError):
            return False
        if zoom:
            setting = _axis_setting(controls, _ZOOM_CONTROLS, zoom, speed)
            if setting:
                settings.append(setting)
    elif operation == "stop":
        for name in ("pan_speed", "tilt_speed", "zoom_continuous"):
            control = controls.get(name)
            if control and control.minimum <= 0 <= control.maximum:
                settings.append((name, 0))
        # Relative and absolute UVC commands complete on their own.
        if not settings:
            return any(
                name in controls
                for name in (*_PAN_CONTROLS, *_TILT_CONTROLS, *_ZOOM_CONTROLS)
            )
    else:
        return False

    if not settings:
        return False
    values = ",".join(f"{name}={value}" for name, value in settings)
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, f"--set-ctrl={values}"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_onvif_capability_urls(xml: str) -> tuple[str, str]:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return "", ""
    urls: dict[str, str] = {}
    for element in root.iter():
        name = _local_name(element.tag)
        if name not in {"Media", "PTZ"}:
            continue
        for child in element.iter():
            if _local_name(child.tag) == "XAddr" and child.text:
                urls[name] = child.text.strip()
                break
    return urls.get("Media", ""), urls.get("PTZ", "")


def parse_onvif_profile(xml: str) -> tuple[str, str]:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return "", ""
    for profile in root.iter():
        if _local_name(profile.tag) not in {"Profiles", "Profile"}:
            continue
        profile_token = str(profile.attrib.get("token", "")).strip()
        for child in profile.iter():
            if _local_name(child.tag) != "PTZConfiguration":
                continue
            configuration_token = str(child.attrib.get("token", "")).strip()
            if profile_token:
                return profile_token, configuration_token
    return "", ""


def parse_onvif_ptz_spaces(xml: str) -> tuple[bool, bool, bool]:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return True, True, True
    names = {_local_name(element.tag) for element in root.iter()}
    pan_tilt = any("PanTilt" in name for name in names)
    zoom = any("Zoom" in name for name in names)
    if not pan_tilt and not zoom:
        return True, True, True
    return pan_tilt, pan_tilt, zoom


def parse_onvif_profiles(xml: str) -> list[dict[str, Any]]:
    """Return every Media1 profile and its useful video metadata."""
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return []
    profiles: list[dict[str, Any]] = []
    for profile in root.iter():
        if _local_name(profile.tag) not in {"Profiles", "Profile"}:
            continue
        token = str(profile.attrib.get("token", "")).strip()
        if not token:
            continue
        name = token
        encoder = ""
        width = height = fps = bitrate = 0
        ptz = False
        for child in profile:
            child_name = _local_name(child.tag)
            if child_name == "Name" and child.text:
                name = child.text.strip() or token
            elif child_name == "PTZConfiguration":
                ptz = True
            elif child_name == "VideoEncoderConfiguration":
                for value in child.iter():
                    value_name = _local_name(value.tag)
                    text = (value.text or "").strip()
                    try:
                        number = int(float(text)) if text else 0
                    except ValueError:
                        number = 0
                    if value_name == "Encoding":
                        encoder = text
                    elif value_name == "Width":
                        width = number
                    elif value_name == "Height":
                        height = number
                    elif value_name == "FrameRateLimit":
                        fps = number
                    elif value_name == "BitrateLimit":
                        bitrate = number
        profiles.append(
            {
                "token": token,
                "name": name,
                "encoding": encoder,
                "width": width,
                "height": height,
                "fps": fps,
                "bitrate_kbps": bitrate,
                "ptz": ptz,
            }
        )
    return profiles


def parse_onvif_stream_uri(xml: str) -> str:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return ""
    for element in root.iter():
        if _local_name(element.tag) == "Uri" and element.text:
            return element.text.strip()
    return ""


def parse_ws_discovery_response(xml: str) -> list[dict[str, Any]]:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return []
    matches: list[dict[str, Any]] = []
    for match in root.iter():
        if _local_name(match.tag) != "ProbeMatch":
            continue
        xaddrs: list[str] = []
        scopes: list[str] = []
        endpoint = ""
        for child in match.iter():
            name = _local_name(child.tag)
            text = (child.text or "").strip()
            if name == "XAddrs" and text:
                xaddrs.extend(text.split())
            elif name == "Scopes" and text:
                scopes.extend(text.split())
            elif name == "Address" and text and not endpoint:
                endpoint = text
        if xaddrs:
            matches.append(
                {"xaddrs": xaddrs, "scopes": scopes, "endpoint": endpoint}
            )
    return matches


def _scope_label(scopes: list[str], fallback: str) -> str:
    for prefix in ("name", "hardware", "location"):
        marker = f"onvif://www.onvif.org/{prefix}/"
        for scope in scopes:
            if scope.startswith(marker):
                label = unquote(scope[len(marker):]).replace("_", " ").strip()
                if label:
                    return label
    return fallback


def suggested_rtsp_profiles(
    host: str,
    name: str,
    scopes: list[str],
) -> list[dict[str, Any]]:
    """Return vendor-standard paths while authenticated ONVIF Media is locked."""
    identity = unquote(" ".join((name, *scopes))).casefold()
    hostname = f"[{host}]" if ":" in host else host
    candidates: list[tuple[str, str]] = []
    if "dahua" in identity:
        candidates = [
            ("Main stream", "/cam/realmonitor?channel=1&subtype=0"),
            ("Sub stream", "/cam/realmonitor?channel=1&subtype=1"),
        ]
    elif "hikvision" in identity or "ds-2" in identity:
        candidates = [
            ("Main stream", "/Streaming/Channels/101"),
            ("Sub stream", "/Streaming/Channels/102"),
        ]
    return [
        {
            "token": f"suggested-{index}",
            "name": profile_name,
            "encoding": "H264",
            "width": 0,
            "height": 0,
            "fps": 0,
            "bitrate_kbps": 0,
            "ptz": False,
            "rtsp_url": f"rtsp://{hostname}:554{path}",
            "path": path,
        }
        for index, (profile_name, path) in enumerate(candidates, start=1)
    ]


def _public_uri(uri: str) -> str:
    parsed = urlsplit(uri)
    if not parsed.hostname:
        return uri
    hostname = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
    port = f":{parsed.port}" if parsed.port else ""
    return urlunsplit(
        (parsed.scheme, f"{hostname}{port}", parsed.path, parsed.query, parsed.fragment)
    )


def _ws_discover_onvif(local_ip: str, timeout: float = 1.8) -> list[dict[str, Any]]:
    message_id = f"uuid:{uuid4()}"
    probe = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<e:Envelope xmlns:e="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:w="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
        'xmlns:d="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
        'xmlns:dn="http://www.onvif.org/ver10/network/wsdl">'
        '<e:Header><w:MessageID>' + message_id + '</w:MessageID>'
        '<w:To e:mustUnderstand="true">urn:schemas-xmlsoap-org:ws:2005:04:'
        'discovery</w:To><w:Action e:mustUnderstand="true">http://schemas.'
        'xmlsoap.org/ws/2005/04/discovery/Probe</w:Action></e:Header>'
        '<e:Body><d:Probe><d:Types>dn:NetworkVideoTransmitter</d:Types>'
        '</d:Probe></e:Body></e:Envelope>'
    ).encode()
    discovered: dict[str, dict[str, Any]] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        if local_ip:
            try:
                packed_ip = socket.inet_aton(local_ip)
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, packed_ip)
                sock.bind((local_ip, 0))
            except OSError:
                sock.bind(("", 0))
        else:
            sock.bind(("", 0))
        sock.settimeout(0.25)
        # Repeating the multicast probe covers cameras that miss the first UDP
        # packet while their network stack is waking up.
        sock.sendto(probe, ("239.255.255.250", 3702))
        started = time.monotonic()
        repeated = False
        while time.monotonic() - started < timeout:
            if not repeated and time.monotonic() - started >= 0.45:
                sock.sendto(probe, ("239.255.255.250", 3702))
                repeated = True
            try:
                payload, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            for match in parse_ws_discovery_response(
                payload.decode("utf-8", errors="replace")
            ):
                xaddr = next(
                    (
                        item for item in match["xaddrs"]
                        if urlsplit(item).scheme in {"http", "https"}
                    ),
                    match["xaddrs"][0],
                )
                host = urlsplit(xaddr).hostname or address[0]
                current = discovered.get(host)
                if current is None or len(match["scopes"]) > len(current["scopes"]):
                    discovered[host] = {
                        "host": host,
                        "xaddr": xaddr,
                        "scopes": match["scopes"],
                    }
    finally:
        sock.close()
    return list(discovered.values())


def _normalize_xaddr(url: str, source_host: str) -> str:
    parsed = urlsplit(url)
    if not parsed.hostname or parsed.hostname in {"0.0.0.0", "127.0.0.1", "localhost"}:
        hostname = f"[{source_host}]" if ":" in source_host else source_host
        port = f":{parsed.port}" if parsed.port else ""
        return urlunsplit(
            (parsed.scheme or "http", f"{hostname}{port}", parsed.path, parsed.query, "")
        )
    return url


def _ws_security(username: str, password: str) -> str:
    if not username:
        return ""
    nonce = os.urandom(16)
    created = (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )
    digest = base64.b64encode(
        hashlib.sha1(nonce + created.encode() + password.encode()).digest()
    ).decode()
    encoded_nonce = base64.b64encode(nonce).decode()
    return (
        '<wsse:Security s:mustUnderstand="1">'
        '<wsse:UsernameToken>'
        f"<wsse:Username>{escape(username)}</wsse:Username>"
        '<wsse:Password Type="http://docs.oasis-open.org/wss/2004/01/'
        f'oasis-200401-wss-username-token-profile-1.0#PasswordDigest">{digest}</wsse:Password>'
        '<wsse:Nonce EncodingType="http://docs.oasis-open.org/wss/2004/01/'
        f'oasis-200401-wss-soap-message-security-1.0#Base64Binary">{encoded_nonce}</wsse:Nonce>'
        f'<wsu:Created>{created}</wsu:Created>'
        '</wsse:UsernameToken>'
        '</wsse:Security>'
    )


def _soap_envelope(body: str, username: str, password: str) -> str:
    return (
        '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
        'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
        'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
        'xmlns:tt="http://www.onvif.org/ver10/schema" '
        'xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/'
        'oasis-200401-wss-wssecurity-secext-1.0.xsd" '
        'xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/'
        'oasis-200401-wss-wssecurity-utility-1.0.xsd">'
        f"<s:Header>{_ws_security(username, password)}</s:Header>"
        f"<s:Body>{body}</s:Body>"
        "</s:Envelope>"
    )


async def _soap_request(
    client: httpx.AsyncClient,
    url: str,
    action: str,
    body: str,
    username: str,
    password: str,
    *,
    timeout: float = 2.0,
) -> str:
    auth = httpx.DigestAuth(username, password) if username else None
    response = await client.post(
        url,
        content=_soap_envelope(body, username, password),
        auth=auth,
        timeout=timeout,
        headers={
            "Content-Type": (
                f'application/soap+xml; charset=utf-8; action="{action}"'
            )
        },
    )
    response.raise_for_status()
    return response.text


def _soap_fault_text(xml: str) -> str:
    """Return the useful text from a SOAP fault without exposing the payload."""
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError:
        return ""
    values: list[str] = []
    for element in root.iter():
        if _local_name(element.tag) in {"Text", "Reason", "Subcode", "Value"}:
            value = " ".join((element.text or "").split())
            if value and value not in values:
                values.append(value)
    return " · ".join(values[:2])


def _is_onvif_auth_error(response: httpx.Response, username: str) -> bool:
    if response.status_code in {401, 403}:
        return True
    body = response.text.casefold()
    auth_markers = (
        "notauthorized",
        "not authorized",
        "unauthorized",
        "authentication",
        "invalidsecurity",
        "wsse",
        "password",
        "credential",
    )
    # Dahua and Hikvision devices commonly wrap missing WS-Security credentials
    # in a generic HTTP 400 SOAP fault instead of returning HTTP 401.
    return any(marker in body for marker in auth_markers) or (
        response.status_code == 400 and not username
    )


class CameraPtzController:
    def __init__(self) -> None:
        self._onvif_cache: dict[str, tuple[float, OnvifTarget | None]] = {}
        self._onvif_credentials: dict[str, tuple[str, str]] = {}
        # Reusing one connection pool removes a TCP handshake and digest-auth
        # round trip from every press/release PTZ event.
        self._http = httpx.AsyncClient(verify=False)
        self._command_lock = asyncio.Lock()

    async def close(self) -> None:
        await self._http.aclose()

    async def scan_onvif(
        self,
        local_ip: str,
        credential_source: str,
        target_host: str = "",
        username: str = "",
        password: str = "",
    ) -> list[dict[str, Any]]:
        """Discover ONVIF devices and resolve every advertised RTSP profile."""
        discovered = await asyncio.to_thread(_ws_discover_onvif, local_ip)
        configured = urlsplit(credential_source)
        configured_host = configured.hostname or ""
        requested_host = target_host.strip().casefold()
        fallback_host = requested_host or configured_host
        if fallback_host and not any(
            str(item.get("host", "")).casefold() == fallback_host.casefold()
            for item in discovered
        ):
            hostname = f"[{fallback_host}]" if ":" in fallback_host else fallback_host
            discovered.append(
                {
                    "host": fallback_host,
                    "xaddr": f"http://{hostname}/onvif/device_service",
                    "scopes": [],
                }
            )
        if requested_host:
            discovered = [
                item
                for item in discovered
                if str(item.get("host", "")).casefold() == requested_host
            ]
        configured_username = unquote(configured.username or "")
        configured_password = unquote(configured.password or "")
        semaphore = asyncio.Semaphore(6)

        async def inspect(item: dict[str, Any]) -> dict[str, Any]:
            host = str(item.get("host", ""))
            if requested_host and host.casefold() == requested_host:
                device_username, device_password = username, password
            elif configured_host and host.casefold() == configured_host.casefold():
                device_username, device_password = (
                    configured_username,
                    configured_password,
                )
            else:
                device_username, device_password = self._onvif_credentials.get(
                    host.casefold(), ("", "")
                )
            async with semaphore:
                result = await self._inspect_onvif_device(
                    item, device_username, device_password
                )
            if result.get("profiles") and device_username:
                self._onvif_credentials[host.casefold()] = (
                    device_username,
                    device_password,
                )
            return result

        results = await asyncio.gather(
            *(inspect(item) for item in discovered),
            return_exceptions=True,
        )
        devices: list[dict[str, Any]] = []
        for item, result in zip(discovered, results, strict=True):
            if isinstance(result, Exception):
                host = str(item.get("host", ""))
                scopes = list(item.get("scopes") or [])
                name = _scope_label(scopes, host)
                devices.append(
                    {
                        "host": host,
                        "name": name,
                        "xaddr": _public_uri(str(item.get("xaddr", ""))),
                        "profiles": [],
                        "suggested_profiles": suggested_rtsp_profiles(
                            host, name, scopes
                        ),
                        "error": "Không đọc được profile ONVIF; kiểm tra tài khoản camera",
                        "auth_required": False,
                    }
                )
            else:
                devices.append(result)
        return sorted(devices, key=lambda item: (item["name"], item["host"]))

    async def _inspect_onvif_device(
        self,
        discovered: dict[str, Any],
        username: str,
        password: str,
    ) -> dict[str, Any]:
        host = str(discovered.get("host", ""))
        scopes = list(discovered.get("scopes") or [])
        xaddr = _normalize_xaddr(str(discovered.get("xaddr", "")), host)
        name = _scope_label(scopes, host)
        base = {
            "host": host,
            "name": name,
            "xaddr": _public_uri(xaddr),
            "profiles": [],
            "suggested_profiles": suggested_rtsp_profiles(host, name, scopes),
            "error": "",
            "auth_required": False,
        }
        try:
            capabilities_xml = await _soap_request(
                self._http,
                xaddr,
                "http://www.onvif.org/ver10/device/wsdl/GetCapabilities",
                "<tds:GetCapabilities><tds:Category>All</tds:Category>"
                "</tds:GetCapabilities>",
                username,
                password,
                timeout=1.8,
            )
            media_url, ptz_url = parse_onvif_capability_urls(capabilities_xml)
            if not media_url:
                return {**base, "error": "Camera không cung cấp ONVIF Media profile"}
            media_url = _normalize_xaddr(media_url, host)
            profiles_xml = await _soap_request(
                self._http,
                media_url,
                "http://www.onvif.org/ver10/media/wsdl/GetProfiles",
                "<trt:GetProfiles/>",
                username,
                password,
                timeout=1.8,
            )
            profiles = parse_onvif_profiles(profiles_xml)[:16]

            async def resolve_uri(profile: dict[str, Any]) -> dict[str, Any] | None:
                stream_xml = await _soap_request(
                    self._http,
                    media_url,
                    "http://www.onvif.org/ver10/media/wsdl/GetStreamUri",
                    "<trt:GetStreamUri><trt:StreamSetup>"
                    "<tt:Stream>RTP-Unicast</tt:Stream><tt:Transport>"
                    "<tt:Protocol>RTSP</tt:Protocol></tt:Transport>"
                    "</trt:StreamSetup><trt:ProfileToken>"
                    f"{escape(str(profile['token']))}</trt:ProfileToken>"
                    "</trt:GetStreamUri>",
                    username,
                    password,
                    timeout=1.8,
                )
                uri = parse_onvif_stream_uri(stream_xml)
                if not uri:
                    return None
                uri = _normalize_xaddr(uri, host)
                public_uri = _public_uri(uri)
                parsed_uri = urlsplit(public_uri)
                path = parsed_uri.path or "/"
                if parsed_uri.query:
                    path += "?" + parsed_uri.query
                return {
                    **profile,
                    "rtsp_url": public_uri,
                    "path": path,
                    "ptz": bool(ptz_url and profile.get("ptz")),
                }

            resolved = await asyncio.gather(
                *(resolve_uri(profile) for profile in profiles),
                return_exceptions=True,
            )
            public_profiles = [
                item for item in resolved if isinstance(item, dict)
            ]
            return {
                **base,
                "profiles": public_profiles,
                "error": (
                    "Camera không trả về RTSP URI cho các profile ONVIF"
                    if profiles and not public_profiles
                    else ""
                ),
            }
        except httpx.HTTPStatusError as exc:
            if _is_onvif_auth_error(exc.response, username):
                detail = (
                    "Tài khoản hoặc mật khẩu ONVIF không đúng"
                    if username
                    else "Camera yêu cầu tài khoản ONVIF riêng"
                )
                return {**base, "error": detail, "auth_required": True}
            fault = _soap_fault_text(exc.response.text)
            return {
                **base,
                "error": fault or "Camera từ chối yêu cầu ONVIF",
            }
        except (httpx.HTTPError, ElementTree.ParseError, ValueError):
            return {**base, "error": "Không đọc được profile ONVIF từ camera"}

    def credentialed_source(self, source: str) -> str:
        """Attach credentials learned for this host without returning them to UI."""
        parsed = urlsplit(source)
        if parsed.username is not None or not parsed.hostname:
            return source
        credentials = self._onvif_credentials.get(parsed.hostname.casefold())
        if not credentials or not credentials[0]:
            return source
        username, password = credentials
        userinfo = quote(username, safe="")
        if password:
            userinfo += ":" + quote(password, safe="")
        hostname = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
        port = f":{parsed.port}" if parsed.port else ""
        return urlunsplit(
            (
                parsed.scheme,
                f"{userinfo}@{hostname}{port}",
                parsed.path,
                parsed.query,
                parsed.fragment,
            )
        )

    async def capabilities(self, source_type: str, source: str) -> dict[str, Any]:
        if source_type == "camera" and source.startswith("/dev/video"):
            return await asyncio.to_thread(usb_ptz_capabilities, source)
        if source_type == "rtsp":
            target = await self._onvif_target(source)
            if target:
                return {
                    "supported": True,
                    "pan": target.pan,
                    "tilt": target.tilt,
                    "zoom": target.zoom,
                    "transport": "onvif",
                }
        return {
            "supported": False,
            "pan": False,
            "tilt": False,
            "zoom": False,
            "transport": "none",
        }

    async def command(
        self,
        source_type: str,
        source: str,
        payload: dict[str, Any],
    ) -> bool:
        if source_type == "camera" and source.startswith("/dev/video"):
            return await asyncio.to_thread(set_usb_ptz, source, payload)
        if source_type != "rtsp":
            return False
        target = await self._onvif_target(source)
        if target is None:
            return False
        # Preserve move/stop order when UI events arrive close together.
        async with self._command_lock:
            return await self._onvif_command(target, payload)

    async def stop(self, source_type: str, source: str) -> None:
        try:
            if source_type == "rtsp":
                cached = self._onvif_cache.get(source)
                if not cached or cached[1] is None:
                    return
                async with self._command_lock:
                    await self._onvif_command(cached[1], {"operation": "stop"})
            else:
                await self.command(source_type, source, {"operation": "stop"})
        except (httpx.HTTPError, OSError, ValueError):
            pass

    async def _onvif_target(self, source: str) -> OnvifTarget | None:
        cached = self._onvif_cache.get(source)
        if cached and time.monotonic() - cached[0] < (60 if cached[1] else 10):
            return cached[1]
        target = await self._discover_onvif(source)
        self._onvif_cache[source] = (time.monotonic(), target)
        return target

    async def _discover_onvif(self, source: str) -> OnvifTarget | None:
        parsed = urlsplit(source)
        if not parsed.hostname:
            return None
        username = unquote(parsed.username or "")
        password = unquote(parsed.password or "")
        hostname = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
        device_url = f"http://{hostname}/onvif/device_service"
        try:
            capabilities_xml = await _soap_request(
                self._http,
                device_url,
                "http://www.onvif.org/ver10/device/wsdl/GetCapabilities",
                "<tds:GetCapabilities><tds:Category>All</tds:Category>"
                "</tds:GetCapabilities>",
                username,
                password,
                timeout=1.2,
            )
            media_url, ptz_url = parse_onvif_capability_urls(capabilities_xml)
            if not media_url or not ptz_url:
                return None
            media_url = _normalize_xaddr(media_url, parsed.hostname)
            ptz_url = _normalize_xaddr(ptz_url, parsed.hostname)
            profiles_xml = await _soap_request(
                self._http,
                media_url,
                "http://www.onvif.org/ver10/media/wsdl/GetProfiles",
                "<trt:GetProfiles/>",
                username,
                password,
                timeout=1.2,
            )
            profile_token, configuration_token = parse_onvif_profile(profiles_xml)
            if not profile_token:
                return None
            pan = tilt = zoom = True
            if configuration_token:
                try:
                    options_xml = await _soap_request(
                        self._http,
                        ptz_url,
                        "http://www.onvif.org/ver20/ptz/wsdl/"
                        "GetConfigurationOptions",
                        "<tptz:GetConfigurationOptions>"
                        f"<tptz:ConfigurationToken>{escape(configuration_token)}"
                        "</tptz:ConfigurationToken>"
                        "</tptz:GetConfigurationOptions>",
                        username,
                        password,
                        timeout=1.2,
                    )
                    pan, tilt, zoom = parse_onvif_ptz_spaces(options_xml)
                except httpx.HTTPError:
                    pass
            return OnvifTarget(
                media_url=media_url,
                ptz_url=ptz_url,
                profile_token=profile_token,
                username=username,
                password=password,
                pan=pan,
                tilt=tilt,
                zoom=zoom,
            )
        except (httpx.HTTPError, ElementTree.ParseError, ValueError):
            return None

    async def _onvif_command(
        self, target: OnvifTarget, payload: dict[str, Any]
    ) -> bool:
        operation = str(payload.get("operation", ""))
        profile_token = escape(target.profile_token)
        if operation == "stop":
            action = "http://www.onvif.org/ver20/ptz/wsdl/Stop"
            pan_tilt = str(target.pan or target.tilt).lower()
            zoom_stop = str(target.zoom).lower()
            body = (
                "<tptz:Stop>"
                f"<tptz:ProfileToken>{profile_token}</tptz:ProfileToken>"
                f"<tptz:PanTilt>{pan_tilt}</tptz:PanTilt>"
                f"<tptz:Zoom>{zoom_stop}</tptz:Zoom>"
                "</tptz:Stop>"
            )
        elif operation in {"move", "zoom"}:
            try:
                pan = max(-1.0, min(1.0, float(payload.get("pan", 0))))
                tilt = max(-1.0, min(1.0, float(payload.get("tilt", 0))))
                zoom = max(-1.0, min(1.0, float(payload.get("zoom", 0))))
            except (TypeError, ValueError):
                return False
            speed = _speed_value(payload.get("speed", "medium"))
            velocity: list[str] = []
            if (pan or tilt) and (target.pan or target.tilt):
                x = pan * speed if target.pan else 0
                y = tilt * speed if target.tilt else 0
                velocity.append(f'<tt:PanTilt x="{x:.3f}" y="{y:.3f}"/>')
            if zoom and target.zoom:
                velocity.append(f'<tt:Zoom x="{zoom * speed:.3f}"/>')
            if not velocity:
                return False
            action = "http://www.onvif.org/ver20/ptz/wsdl/ContinuousMove"
            body = (
                "<tptz:ContinuousMove>"
                f"<tptz:ProfileToken>{profile_token}</tptz:ProfileToken>"
                f"<tptz:Velocity>{''.join(velocity)}</tptz:Velocity>"
                "</tptz:ContinuousMove>"
            )
        else:
            return False
        try:
            await _soap_request(
                self._http,
                target.ptz_url,
                action,
                body,
                target.username,
                target.password,
            )
            return True
        except httpx.HTTPError:
            return False

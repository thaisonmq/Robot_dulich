import subprocess

import httpx
import pytest

from simulator import camera_ptz


V4L2_PTZ_CONTROLS = """
Camera Controls

                   pan_relative 0x009a0904 (int)    : min=-448 max=448 step=64 default=0 value=0
                  tilt_relative 0x009a0905 (int)    : min=-448 max=448 step=64 default=0 value=0
                   zoom_absolute 0x009a090d (int)    : min=100 max=500 step=1 default=100 value=120
"""


def test_usb_capability_detection_uses_real_v4l2_controls(monkeypatch) -> None:
    monkeypatch.setattr(
        camera_ptz.subprocess,
        "run",
        lambda *_args, **_kwargs: subprocess.CompletedProcess(
            [], 0, stdout=V4L2_PTZ_CONTROLS, stderr=""
        ),
    )

    capabilities = camera_ptz.usb_ptz_capabilities("/dev/video0")

    assert capabilities == {
        "supported": True,
        "pan": True,
        "tilt": True,
        "zoom": True,
        "transport": "uvc",
    }


def test_usb_camera_without_ptz_controls_is_not_advertised(monkeypatch) -> None:
    output = """
                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157
    """
    monkeypatch.setattr(
        camera_ptz.subprocess,
        "run",
        lambda *_args, **_kwargs: subprocess.CompletedProcess(
            [], 0, stdout=output, stderr=""
        ),
    )

    assert camera_ptz.usb_ptz_capabilities("/dev/video0")["supported"] is False


def test_usb_ptz_speed_changes_relative_step(monkeypatch) -> None:
    commands: list[list[str]] = []

    def run(command: list[str], **_kwargs: object) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        if "--list-ctrls" in command:
            return subprocess.CompletedProcess(
                command, 0, stdout=V4L2_PTZ_CONTROLS, stderr=""
            )
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(camera_ptz.subprocess, "run", run)

    assert camera_ptz.set_usb_ptz(
        "/dev/video0",
        {"operation": "move", "pan": 1, "tilt": -1, "speed": "slow"},
    )
    assert commands[-1][-1] == "--set-ctrl=pan_relative=128,tilt_relative=-128"


def test_onvif_capabilities_profiles_and_spaces_are_parsed() -> None:
    capabilities_xml = """
    <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
      xmlns:tt="http://www.onvif.org/ver10/schema"><s:Body><tt:Capabilities>
      <tt:Media><tt:XAddr>http://camera/onvif/media</tt:XAddr></tt:Media>
      <tt:PTZ><tt:XAddr>http://camera/onvif/ptz</tt:XAddr></tt:PTZ>
      </tt:Capabilities></s:Body></s:Envelope>
    """
    profiles_xml = """
    <trt:GetProfilesResponse xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
      xmlns:tt="http://www.onvif.org/ver10/schema">
      <trt:Profiles token="profile-main"><tt:PTZConfiguration token="ptz-main"/>
      </trt:Profiles></trt:GetProfilesResponse>
    """
    options_xml = """
    <tptz:GetConfigurationOptionsResponse xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
      xmlns:tt="http://www.onvif.org/ver10/schema"><tptz:PTZConfigurationOptions>
      <tt:Spaces><tt:ContinuousPanTiltVelocitySpace/><tt:ContinuousZoomVelocitySpace/>
      </tt:Spaces></tptz:PTZConfigurationOptions></tptz:GetConfigurationOptionsResponse>
    """

    assert camera_ptz.parse_onvif_capability_urls(capabilities_xml) == (
        "http://camera/onvif/media",
        "http://camera/onvif/ptz",
    )
    assert camera_ptz.parse_onvif_profile(profiles_xml) == (
        "profile-main",
        "ptz-main",
    )
    assert camera_ptz.parse_onvif_ptz_spaces(options_xml) == (True, True, True)


def test_onvif_scan_parses_all_rtsp_profiles_and_discovery_addresses() -> None:
    discovery_xml = """
    <e:Envelope xmlns:e="http://www.w3.org/2003/05/soap-envelope"
      xmlns:d="http://schemas.xmlsoap.org/ws/2005/04/discovery">
      <e:Body><d:ProbeMatches><d:ProbeMatch>
        <d:Scopes>onvif://www.onvif.org/name/Lobby_Camera</d:Scopes>
        <d:XAddrs>http://192.168.6.128/onvif/device_service</d:XAddrs>
      </d:ProbeMatch></d:ProbeMatches></e:Body>
    </e:Envelope>
    """
    profiles_xml = """
    <trt:GetProfilesResponse xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
      xmlns:tt="http://www.onvif.org/ver10/schema">
      <trt:Profiles token="main"><tt:Name>Main Stream</tt:Name>
        <tt:VideoEncoderConfiguration token="enc-main">
          <tt:Encoding>H264</tt:Encoding>
          <tt:Resolution><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:Resolution>
          <tt:RateControl><tt:FrameRateLimit>25</tt:FrameRateLimit><tt:BitrateLimit>2048</tt:BitrateLimit></tt:RateControl>
        </tt:VideoEncoderConfiguration>
        <tt:PTZConfiguration token="ptz-main"/>
      </trt:Profiles>
      <trt:Profiles token="sub"><tt:Name>Sub Stream</tt:Name>
        <tt:VideoEncoderConfiguration token="enc-sub">
          <tt:Encoding>H264</tt:Encoding>
          <tt:Resolution><tt:Width>704</tt:Width><tt:Height>576</tt:Height></tt:Resolution>
          <tt:RateControl><tt:FrameRateLimit>15</tt:FrameRateLimit></tt:RateControl>
        </tt:VideoEncoderConfiguration>
      </trt:Profiles>
    </trt:GetProfilesResponse>
    """

    matches = camera_ptz.parse_ws_discovery_response(discovery_xml)
    profiles = camera_ptz.parse_onvif_profiles(profiles_xml)

    assert matches[0]["xaddrs"] == [
        "http://192.168.6.128/onvif/device_service"
    ]
    assert camera_ptz._scope_label(matches[0]["scopes"], "fallback") == (
        "Lobby Camera"
    )
    assert [(item["token"], item["width"], item["fps"], item["ptz"]) for item in profiles] == [
        ("main", 1920, 25, True),
        ("sub", 704, 15, False),
    ]


def test_onvif_stream_uri_is_sanitized_but_keeps_path() -> None:
    xml = """
    <trt:GetStreamUriResponse xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
      xmlns:tt="http://www.onvif.org/ver10/schema"><trt:MediaUri>
      <tt:Uri>rtsp://admin:secret@192.168.6.128:554/cam/main?channel=1</tt:Uri>
      </trt:MediaUri></trt:GetStreamUriResponse>
    """
    uri = camera_ptz.parse_onvif_stream_uri(xml)

    assert camera_ptz._public_uri(uri) == (
        "rtsp://192.168.6.128:554/cam/main?channel=1"
    )


def test_locked_onvif_cameras_expose_vendor_paths_without_claiming_profiles() -> None:
    dahua = camera_ptz.suggested_rtsp_profiles(
        "192.168.6.142", "Dahua", []
    )
    hikvision = camera_ptz.suggested_rtsp_profiles(
        "192.168.6.211", "HIKVISION DS-2CD2643G2-IZS", []
    )

    assert [profile["path"] for profile in dahua] == [
        "/cam/realmonitor?channel=1&subtype=0",
        "/cam/realmonitor?channel=1&subtype=1",
    ]
    assert [profile["path"] for profile in hikvision] == [
        "/Streaming/Channels/101",
        "/Streaming/Channels/102",
    ]


@pytest.mark.asyncio
async def test_onvif_scan_never_reuses_current_camera_credentials_for_other_hosts(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        camera_ptz,
        "_ws_discover_onvif",
        lambda _local_ip: [
            {
                "host": "192.168.6.128",
                "xaddr": "http://192.168.6.128/onvif/device_service",
                "scopes": [],
            },
            {
                "host": "192.168.6.211",
                "xaddr": "http://192.168.6.211/onvif/device_service",
                "scopes": [],
            },
        ],
    )
    controller = camera_ptz.CameraPtzController()
    seen: dict[str, tuple[str, str]] = {}

    async def inspect(discovered, username, password):
        host = discovered["host"]
        seen[host] = (username, password)
        return {
            "host": host,
            "name": host,
            "xaddr": discovered["xaddr"],
            "profiles": [],
            "error": "",
            "auth_required": False,
        }

    monkeypatch.setattr(controller, "_inspect_onvif_device", inspect)
    try:
        await controller.scan_onvif(
            "192.168.6.145",
            "rtsp://operator:camera-pass@192.168.6.128/live",
        )
    finally:
        await controller.close()

    assert seen["192.168.6.128"] == ("operator", "camera-pass")
    assert seen["192.168.6.211"] == ("", "")


@pytest.mark.asyncio
async def test_onvif_targeted_retry_uses_only_the_submitted_camera_credentials(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        camera_ptz,
        "_ws_discover_onvif",
        lambda _local_ip: [
            {
                "host": "192.168.6.128",
                "xaddr": "http://192.168.6.128/onvif/device_service",
                "scopes": [],
            },
            {
                "host": "192.168.6.211",
                "xaddr": "http://192.168.6.211/onvif/device_service",
                "scopes": [],
            },
        ],
    )
    controller = camera_ptz.CameraPtzController()
    seen: list[tuple[str, str, str]] = []

    async def inspect(discovered, username, password):
        seen.append((discovered["host"], username, password))
        return {
            "host": discovered["host"],
            "name": discovered["host"],
            "xaddr": discovered["xaddr"],
            "profiles": [{"token": "main"}],
            "error": "",
            "auth_required": False,
        }

    monkeypatch.setattr(controller, "_inspect_onvif_device", inspect)
    try:
        result = await controller.scan_onvif(
            "192.168.6.145",
            "rtsp://current:secret@192.168.6.128/live",
            "192.168.6.211",
            "hik-user",
            "hik-pass",
        )
        credentialed = controller.credentialed_source(
            "rtsp://192.168.6.211:554/Streaming/Channels/101"
        )
    finally:
        await controller.close()

    assert [device["host"] for device in result] == ["192.168.6.211"]
    assert seen == [("192.168.6.211", "hik-user", "hik-pass")]
    assert credentialed == (
        "rtsp://hik-user:hik-pass@192.168.6.211:554/Streaming/Channels/101"
    )


def test_onvif_http_400_without_credentials_is_reported_as_auth_required() -> None:
    response = httpx.Response(
        400,
        text="<Fault><Reason><Text>Bad Request</Text></Reason></Fault>",
        request=httpx.Request("POST", "http://camera/onvif/device_service"),
    )

    assert camera_ptz._is_onvif_auth_error(response, "") is True

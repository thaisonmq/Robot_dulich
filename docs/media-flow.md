# Media flow

Each robot uses room `robot-{robot_id}`.

- Robot control/audio identity: `robot:{robot_id}`.
- Encoded camera identity: `robot:{robot_id}:video`.
- User identity: `user:{user_id}:session:{session_id}`.
- The robot publishes `robot-microphone`; its optimized media process publishes
  `camera` with a separate identity so the two authenticated connections never
  replace each other.
- Browser subscribes to robot tracks and only publishes its microphone after an
  explicit click.
- Simulator subscribes only to `user:*` audio, converts it to 48 kHz mono PCM,
  and plays it through the configured ALSA or PipeWire/PulseAudio speaker. A
  bounded latest-frame queue prevents an interrupted output from accumulating
  stale conversation audio; the output process reconnects with capped backoff.
  Device capture/playback uses native `arecord`/`aplay` or `pacat`: ALSA uses a
  60 ms buffer with a 20 ms period, Pulse playback targets 60/20 ms, and Pulse
  capture targets 20/10 ms. FFmpeg remains the fallback and its Pulse output is
  explicitly capped at 60 ms instead of the roughly two-second default.

The backend signs short-lived, room-scoped tokens for both operator and robot.
LiveKit API secrets never reach the browser, simulator, or Orange Pi. The edge
device authenticates to Center first, then requests a publisher token limited
to `robot-{robot_id}`. When media is unavailable, control and telemetry remain
usable and the dashboard shows a clear no-signal state.

For Internet deployment, set `LIVEKIT_PUBLIC_URL=wss://media.example.com`, put
LiveKit behind TLS, expose the configured UDP range, and configure LiveKit TURN
with the supplied coturn service or a managed TURN endpoint.

## Simulator sources

| Type | Configuration | Runtime |
| --- | --- | --- |
| Test | `SIMULATOR_MEDIA_SOURCE_TYPE=test` | Generated moving frame through the SDK compatibility path |
| MP4 | `file`, `/media/file.mp4` | H.264 is copied; another codec uses the selected hardware/software encoder |
| RTSP | `rtsp`, `rtsp://...` | H.264 is depayloaded and parsed directly, without decode/re-encode |
| USB | `camera`, `/dev/video0` | H.264 is captured directly; MJPEG/raw uses the selected encoder |

Set `SIMULATOR_AUDIO_SOURCE` to a file/stream readable by FFmpeg. Without it,
the simulator publishes a silent audio track so the media contract remains
present. Sources reconnect with capped exponential delay.

The robot speaker is selected separately through `audio_output_type` and
`audio_output`. Hardware discovery probes ALSA playback devices and available
PipeWire/PulseAudio sinks without an audible tone. The explicit speaker
diagnostic plays a short low-volume tone before the operator saves the
configuration. The container runner exposes `/dev/snd` and, when available,
the host Pulse socket to the edge process.

`VIDEO_PIPELINE=auto` is the default. At startup the edge checks the source
codec and probes each encoder by encoding one real test frame. It selects
Rockchip MPP (`h264_rkmpp`) on a working RK3588 environment, VA-API/NVENC/
V4L2M2M when available, and finally `libx264`. Merely seeing an encoder name in
FFmpeg is not treated as hardware support.

On older Intel VA-API drivers that expose only constant-quality rate control,
the edge uses H.264 CQP with a motion-safe quality setting instead of passing an
unsupported bitrate mode. The FFmpeg-to-GStreamer bridge uses an OS pipe, not a
local TCP listener, so startup does not race while the USB camera is being
opened. Its encoded queue applies back-pressure rather than discarding H.264
delta frames.

## Stability

- LiveKit Server and the browser SDK must use compatible protocol generations.
  The Compose profile pins LiveKit Server 1.12.
- `LIVEKIT_NODE_IP` is advertised as the ICE node address for Docker-on-LAN
  deployments; this avoids signaling/ICE recovery loops caused by a Docker
  bridge address.
- H.264 cameras do not pass Full HD raw frames through Python and are not
  decoded/re-encoded. Non-H.264 input is scaled once and encoded before it
  reaches the LiveKit publisher. Full HD uses an 8 Mbps ceiling.
- The RTSP depayloader requests a keyframe and waits for one after packet loss.
  It keeps an 80 ms reorder window and does not deliberately discard encoded
  buffers. SPS/PPS is repeated once per second so a decoder can recover cleanly.
  When a camera omits H.264 frame duration, the publisher derives a stable
  cadence from timestamps/caps instead of emitting invalid RTP timestamp steps.
  The browser keeps the recovery layer visible until WebRTC reports a decoded
  keyframe, preventing undecodable startup delta frames from appearing green.
  For minimum startup latency, configure the RTSP camera's I-frame/GOP interval
  to about one second.
- The browser explicitly requests the highest video layer and monitors decoded
  frame progress. Its adaptive 60–140 ms playout target absorbs normal LAN and
  USB capture jitter without adding a large surveillance-style delay. During a short
  reconnect it captures one reduced recovery frame, then restores the live
  element when decoded frames resume. It does not read pixels or copy Full HD
  frames periodically during healthy playback.
- VA-API runs with a single frame in flight and FFmpeg flushes each muxed frame
  into the local pipe. The encoded bridge keeps only one non-leaky handoff
  frame, avoiding extra encoder/queue delay and quiet-scene AVIO buffering on
  USB cameras.

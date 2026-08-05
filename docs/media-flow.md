# Telepresence media flow

This subsystem is optimized for a live robot view, not surveillance recording.
When compute or network capacity is insufficient, it discards stale raw media or
restarts at a decodable GOP instead of replaying an ever-growing backlog.

## Participants and process boundary

Each session uses room `robot-{robot_id}`:

- `robot:{robot_id}` publishes `robot-microphone` and subscribes to operator audio.
- `robot:{robot_id}:video` publishes `camera` from the native Go/GStreamer process.
- `user:{user_id}:session:{session_id}` subscribes and publishes the operator mic.

There are currently two robot participants because LiveKit does not let two
independent SDK connections share one participant identity: the Python SDK owns
the bidirectional PCM audio connection, while `gstreamer-publisher` owns encoded
H.264 access units. Reusing `robot:{robot_id}` would make the connections replace
each other. Merging them requires moving subscribe/playback audio into the Go
daemon; it must not be simulated by copying encoded video through Python.

Control, telemetry, ROS and motion remain in the Python robot agent. Media tasks
are independently cancellable and all blocking probes run in worker threads.

## Video routes

### USB camera mode selection

USB capture size is not fixed to 1080p. When cameras are scanned, the robot
parses every discrete format, size and frame interval advertised by
`v4l2-ctl --list-formats-ext`. Selecting a camera recomputes the capture mode
against the active video profile. The selector prefers a supported mode near
the requested display size and FPS, penalizes modes below the target FPS and
60/120 fps modes that would waste USB/decoder capacity, and prefers native
H.264 when available. The chosen mode is read back from the driver before the
publisher starts. `SIMULATOR_CAMERA_WIDTH`, `HEIGHT`, `FPS` and `FORMAT` are
optional overrides; their zero/empty defaults mean automatic selection.

### A — compatible H.264 passthrough

```text
RTSP camera
  -> GStreamer rtspsrc (80 ms bounded jitter, drop-on-latency)
  -> rtph264depay (request/wait for keyframe)
  -> h264parse (AU-aligned Annex-B, SPS/PPS every second)
  -> native LiveKit H.264 track
  -> browser WebRTC receiver
```

There is no FFmpeg, MPEG-TS, decode, encode, raw frame, or Python frame copy in
this route. Source PTS/duration is used when valid, including `30000/1001`.
`RTSP_TRANSPORT=auto` allows UDP first and bounded fallback to TCP. Explicit
`udp` and `tcp` remain available for managed deployments.

USB cameras that negotiate H.264 use the equivalent direct route:
`v4l2src -> h264parse -> LiveKit`.

### B — H.264 normalize

```text
RTSP H.264 with bad timing/B-frames/oversized profile
  -> FFmpeg receive/decode
  -> arrival-clock or PTS normalization
  -> fps/scale (surplus raw frames dropped before encode)
  -> low-latency H.264 encoder (bf=0, short GOP, no lookahead)
  -> bounded Annex-B pipe (no MPEG-TS)
  -> h264parse
  -> native LiveKit H.264 track
```

Runtime probing compares advertised FPS with measured packet cadence and detects
repeated/backward timestamps and bursts. Normalization is selected only when the
passthrough route is unsafe.

### B — H.265/HEVC and MJPEG

```text
RTSP H.265 -> decode -> latest-time fps/scale -> H.264 low-latency -> Annex-B pipe -> LiveKit
RTSP/USB MJPEG -> decode -> latest-time fps/scale -> H.264 low-latency -> Annex-B pipe -> LiveKit
```

The encoder preference is a backend that passes an actual two-frame probe,
followed by `libx264 ultrafast/zerolatency`. H.264 output has no B-frames, one
second or shorter GOP, a 250 ms VBV target, and one hardware frame in flight
where supported. If published FPS remains below 65% of target, the watchdog
restarts only video and reduces FPS/resolution in two bounded steps.

### C — unsuitable for realtime on Raspberry Pi 5

Profiles requiring 4K transcode, high-rate H.265/MJPEG, or more than the bounded
software/hardware encoder budget are reported as level C. The UI recommends an
H.264 substream around 720p, 15–25 fps, reasonable CBR, Smart Codec off, no
B-frames, and a 0.5–1 second GOP. The system does not claim realtime for these
sources.

## Backlog, timestamps, keyframes and recovery

- Raw compatibility frames use a two-frame latest-wins queue.
- The transcode handoff is an OS pipe plus a 2–6 access-unit queue. It never
  drops an arbitrary encoded delta frame. If blocked or stalled, the watchdog
  terminates both children and reconnects, thereby flushing the old GOP.
- The native publisher paces only a short RTP burst; the deadline is below one
  frame interval. It does not sleep once per metadata FPS while preserving old
  frames.
- PLI/FIR is debounced and sends `GstForceKeyUnit` upstream. The publisher waits
  for a new IDR; only a three-second keyframe timeout reconnects the video source.
- Startup without access units, a live process without progress, and sustained
  below-realtime publish rate are separate watchdog failures.
- Child processes receive terminate, then kill after a bounded timeout. Video
  reconnect uses capped exponential backoff and never restarts robot control.

Structured periodic video metrics include route, encoder, input/published FPS,
bounded backlog frames/milliseconds, last-frame age and reconnect/degrade events.
Credentials and LiveKit tokens are redacted.

## Full-duplex audio with real AEC

For a configured microphone and speaker, one top-level native GStreamer pipeline
owns both devices:

```text
operator LiveKit audio (48 kHz mono, 10 ms)
  -> 20 ms latest-wins application queue
  -> GStreamer raw PCM queue (leaky before reference)
  -> webrtcechoprobe `robot_echo_reference`
  -> speaker (40 ms device buffer, 10 ms period)
                         |
                         +---- reverse/render reference ----+
                                                            v
microphone (48 kHz mono, 10 ms period)
  -> bounded raw queue
  -> webrtcdsp (AEC + NS + AGC + high-pass filter)
  -> 10 ms PCM frames
  -> LiveKit AudioSource (40 ms maximum queue)
  -> operator browser
```

The probe and DSP are intentionally in the same GstPipeline, as required by the
WebRTC audio processor. If either plugin is unavailable, logs and health report
`aec=false`; the system does not label an unreferenced capture path as echo
cancelled. Capture stall and playback write timeout restart audio processing
without touching video or control. USB Audio/I2S is recommended; Bluetooth
HSP/HFP remains best effort because its latency and device profile are outside
the AEC pipeline's control.

The browser uses separate jitter targets: video adapts from 60–140 ms, while
conversation audio remains at 40 ms. Video jitter therefore cannot silently
inflate mouth-to-ear delay.

## Browser diagnostics

The receiver tracks bytes, decoded/dropped frames, decoded keyframes, FPS,
freezes and jitter. It distinguishes upstream stall, missing keyframe, and
decoder stall. Recovery first resubscribes the video publication, then performs
a room/token reconnect with bounded backoff. Audio and video track loss are
handled independently.

For new viewers, the recovery layer remains visible until `keyFramesDecoded`
advances, avoiding a green delta-frame image. Camera GOP should still be kept at
0.5–1 second for best startup time.

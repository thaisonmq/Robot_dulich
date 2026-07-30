import {
  Room, RoomEvent, Track, VideoQuality, type RemoteTrack, type RemoteTrackPublication,
  type RemoteParticipant,
} from "livekit-client";

export interface IMediaTransport {
  connect(url: string, token: string): Promise<void>;
  enableMicrophone(enabled: boolean): Promise<void>;
  setSpeakerMuted(muted: boolean): void;
  disconnect(): Promise<void>;
}

export class LiveKitMediaTransport implements IMediaTransport {
  private room: Room | null = null;
  private speakerMuted = false;
  private watchdog: number | null = null;
  private frameCallback: number | null = null;
  private lastFrameAt = 0;
  private lastVideoTime = 0;
  private recovering = false;
  private recoveryAttempts = 0;
  private manualDisconnect = false;
  private reconnectTimer: number | null = null;
  private staleSnapshotTimer: number | null = null;
  private connection: { url: string; token: string } | null = null;
  private videoKeyframeReady = false;
  private videoTrackGeneration = 0;

  constructor(
    private readonly videoElement: HTMLVideoElement,
    private readonly audioElement: HTMLAudioElement,
    private readonly onState: (state: string) => void,
    private readonly snapshotCanvas?: HTMLCanvasElement,
  ) {}

  async connect(url: string, token: string): Promise<void> {
    await this.disconnect(false);
    this.manualDisconnect = false;
    this.connection = { url, token };
    this.onState("connecting");
    // The robot publishes one high-quality H.264 layer. Keep that layer stable
    // instead of allowing viewport heuristics to pause or switch quality.
    const room = new Room({ adaptiveStream: false, dynacast: false });
    this.room = room;
    room.on(RoomEvent.TrackSubscribed, (
      track: RemoteTrack,
      publication: RemoteTrackPublication,
      participant: RemoteParticipant,
    ) => {
      if (!participant.identity.startsWith("robot:")) return;
      if (track.kind === Track.Kind.Video) {
        const generation = ++this.videoTrackGeneration;
        this.videoKeyframeReady = false;
        this.stopFrameCallback();
        // A passthrough RTSP stream may begin between two IDR frames. Chromium
        // can render the undecodable delta frames as a green image, so keep the
        // neutral recovery layer visible until WebRTC reports one decoded
        // keyframe.
        this.videoElement.classList.add("is-recovering");
        // The robot publishes a single quality layer. Request it explicitly and
        // tell WebRTC that continuous motion is more important than still-image
        // detail, so people walking do not cause visible frame drops.
        publication.setVideoQuality(VideoQuality.HIGH);
        try {
          track.mediaStreamTrack.contentHint = "motion";
        } catch {
          // Older Safari versions expose contentHint as read-only.
        }
        // Keep a small stability margin. Forcing this to exactly zero makes
        // late H.264 delta frames miss playout and corrupts/freezes the GOP.
        // 50 ms is still suitable for driving while absorbing normal LAN,
        // USB-capture and scheduling jitter.
        track.setPlayoutDelay(0.05);
        if (track.receiver && "jitterBufferTarget" in track.receiver) {
          track.receiver.jitterBufferTarget = 50;
        }
        track.attach(this.videoElement);
        this.lastFrameAt = performance.now();
        this.lastVideoTime = this.videoElement.currentTime;
        this.startFrameMonitor();
        void this.waitForFirstDecodedKeyframe(track, generation);
        void this.videoElement.play().catch(() => undefined);
      }
      if (track.kind === Track.Kind.Audio) {
        // Use the same small target for audio so A/V remains synchronized.
        track.setPlayoutDelay(0.05);
        if (track.receiver && "jitterBufferTarget" in track.receiver) {
          track.receiver.jitterBufferTarget = 50;
        }
        track.attach(this.audioElement);
        this.audioElement.muted = this.speakerMuted;
      }
    });
    room.on(RoomEvent.TrackUnsubscribed, (track, _publication, participant) => {
      if (participant.identity.startsWith("robot:") && track.kind === Track.Kind.Video) {
        this.videoTrackGeneration += 1;
        this.videoKeyframeReady = false;
        this.stopFrameCallback();
        this.showRecoveryFrame();
      }
    });
    room.on(RoomEvent.Reconnecting, () => {
      this.showRecoveryFrame();
      this.onState("reconnecting");
    });
    room.on(RoomEvent.Reconnected, () => {
      this.lastFrameAt = performance.now();
      this.recoveryAttempts = 0;
      this.onState("reconnecting");
    });
    room.on(RoomEvent.Disconnected, () => {
      if (this.manualDisconnect) return;
      this.showRecoveryFrame();
      this.scheduleRoomReconnect();
    });
    await room.connect(url, token);
    this.startWatchdog();
  }

  async enableMicrophone(enabled: boolean): Promise<void> {
    if (!this.room) throw new Error("Media chưa kết nối");
    await this.room.localParticipant.setMicrophoneEnabled(enabled);
  }

  setSpeakerMuted(muted: boolean): void {
    this.speakerMuted = muted;
    this.audioElement.muted = muted;
  }

  async disconnect(clearRecoveryFrame = true): Promise<void> {
    this.manualDisconnect = true;
    this.videoTrackGeneration += 1;
    this.videoKeyframeReady = false;
    this.connection = null;
    this.stopMonitoring();
    if (clearRecoveryFrame) {
      this.clearStaleSnapshotTimer();
      if (this.snapshotCanvas) {
        this.snapshotCanvas.classList.remove("has-frame");
        this.snapshotCanvas.getContext("2d")?.clearRect(
          0, 0, this.snapshotCanvas.width, this.snapshotCanvas.height
        );
      }
    }
    if (this.reconnectTimer !== null) window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
    if (!this.room) {
      this.onState("idle");
      return;
    }
    await this.room.localParticipant.setMicrophoneEnabled(false).catch(() => undefined);
    this.videoElement.srcObject = null;
    this.audioElement.srcObject = null;
    await this.room.disconnect();
    this.room = null;
    this.videoElement.classList.remove("is-recovering");
    this.onState("idle");
  }

  private startFrameMonitor(): void {
    if (!("requestVideoFrameCallback" in this.videoElement) || this.frameCallback !== null) return;
    const frame = (now: number) => {
      this.lastFrameAt = now;
      if (!this.videoKeyframeReady) {
        this.frameCallback = this.videoElement.requestVideoFrameCallback(frame);
        return;
      }
      this.recoveryAttempts = 0;
      this.clearStaleSnapshotTimer();
      this.videoElement.classList.remove("is-recovering");
      this.onState("connected");
      this.frameCallback = this.videoElement.requestVideoFrameCallback(frame);
    };
    this.frameCallback = this.videoElement.requestVideoFrameCallback(frame);
  }

  private async waitForFirstDecodedKeyframe(
    track: RemoteTrack,
    generation: number,
  ): Promise<void> {
    while (
      this.room
      && generation === this.videoTrackGeneration
      && track.kind === Track.Kind.Video
    ) {
      const receiver = track.receiver;
      if (receiver) {
        try {
          const report = await receiver.getStats();
          for (const rawStat of report.values()) {
            if (
              rawStat.type !== "inbound-rtp"
              || (rawStat.kind !== "video" && rawStat.mediaType !== "video")
            ) continue;
            const stat = rawStat as RTCInboundRtpStreamStats & {
              keyFramesDecoded?: number;
              framesDecoded?: number;
            };
            const hasKeyframeCounter = typeof stat.keyFramesDecoded === "number";
            const ready = hasKeyframeCounter
              ? stat.keyFramesDecoded! > 0
              : (stat.framesDecoded ?? 0) > 0;
            if (!ready) continue;
            if (generation !== this.videoTrackGeneration) return;
            this.videoKeyframeReady = true;
            this.lastFrameAt = performance.now();
            this.lastVideoTime = this.videoElement.currentTime;
            this.recoveryAttempts = 0;
            this.clearStaleSnapshotTimer();
            this.videoElement.classList.remove("is-recovering");
            this.onState("connected");
            return;
          }
        } catch {
          // Stats can be temporarily unavailable during ICE negotiation.
        }
      }
      await new Promise((resolve) => window.setTimeout(resolve, 100));
    }
  }

  private rememberFrame(): void {
    if (!this.snapshotCanvas) return;
    const width = this.videoElement.videoWidth;
    const height = this.videoElement.videoHeight;
    if (!width || !height) return;
    const scale = Math.min(1, 1280 / width, 720 / height);
    const snapshotWidth = Math.round(width * scale);
    const snapshotHeight = Math.round(height * scale);
    if (this.snapshotCanvas.width !== snapshotWidth) this.snapshotCanvas.width = snapshotWidth;
    if (this.snapshotCanvas.height !== snapshotHeight) this.snapshotCanvas.height = snapshotHeight;
    this.snapshotCanvas.getContext("2d")?.drawImage(
      this.videoElement, 0, 0, snapshotWidth, snapshotHeight
    );
    this.snapshotCanvas.classList.add("has-frame");
  }

  private showRecoveryFrame(): void {
    if (
      !this.videoElement.classList.contains("is-recovering")
      && this.videoElement.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA
    ) {
      // Capture once when recovery starts. Periodically copying a 1080p frame
      // forced GPU readback on the UI thread and caused a visible micro-stutter.
      this.rememberFrame();
    }
    this.videoElement.classList.add("is-recovering");
    this.onState("reconnecting");
    this.scheduleStaleSnapshotClear();
  }

  private scheduleStaleSnapshotClear(): void {
    if (this.staleSnapshotTimer !== null) return;
    this.staleSnapshotTimer = window.setTimeout(() => {
      this.staleSnapshotTimer = null;
      if (this.snapshotCanvas) {
        this.snapshotCanvas.classList.remove("has-frame");
        this.snapshotCanvas.getContext("2d")?.clearRect(
          0, 0, this.snapshotCanvas.width, this.snapshotCanvas.height
        );
      }
      this.onState("no_video");
    }, 8000);
  }

  private clearStaleSnapshotTimer(): void {
    if (this.staleSnapshotTimer !== null) window.clearTimeout(this.staleSnapshotTimer);
    this.staleSnapshotTimer = null;
  }

  private startWatchdog(): void {
    if (this.watchdog !== null) return;
    this.lastFrameAt = performance.now();
    this.lastVideoTime = this.videoElement.currentTime;
    this.watchdog = window.setInterval(() => {
      if (!this.room) return;
      const currentTime = this.videoElement.currentTime;
      if (currentTime > this.lastVideoTime + .01) {
        this.lastVideoTime = currentTime;
        this.lastFrameAt = performance.now();
        this.recoveryAttempts = 0;
        if (this.videoElement.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA) {
          this.clearStaleSnapshotTimer();
          this.videoElement.classList.remove("is-recovering");
          this.onState("connected");
        }
        return;
      }
      if (performance.now() - this.lastFrameAt < 8000) return;
      this.showRecoveryFrame();
      void this.recoverVideoTrack();
    }, 1000);
  }

  private async recoverVideoTrack(): Promise<void> {
    if (this.recovering || !this.room) return;
    this.recovering = true;
    this.recoveryAttempts += 1;
    try {
      // Audio/control and optimized H.264 video use separate robot
      // participants, so select the participant that actually owns video.
      const publication = [...this.room.remoteParticipants.values()]
        .filter((participant) => participant.identity.startsWith("robot:"))
        .flatMap((participant) => [...participant.videoTrackPublications.values()])
        .find(Boolean);
      if (publication) {
        this.stopFrameCallback();
        publication.setSubscribed(false);
        await new Promise((resolve) => window.setTimeout(resolve, 220));
        publication.setSubscribed(true);
        this.lastFrameAt = performance.now();
      }
      if (!publication) this.scheduleRoomReconnect(900);
    } finally {
      window.setTimeout(() => {
        this.recovering = false;
      }, 900);
    }
  }

  private scheduleRoomReconnect(delay = 900): void {
    if (this.manualDisconnect || !this.connection || this.reconnectTimer !== null) return;
    this.onState("reconnecting");
    const connection = this.connection;
    this.reconnectTimer = window.setTimeout(async () => {
      this.reconnectTimer = null;
      if (this.manualDisconnect) return;
      try {
        const room = this.room;
        this.room = null;
        if (room) await room.disconnect().catch(() => undefined);
        await this.connect(connection.url, connection.token);
      } catch {
        this.connection = connection;
        this.manualDisconnect = false;
        this.scheduleRoomReconnect(1800);
      }
    }, delay);
  }

  private stopMonitoring(): void {
    if (this.watchdog !== null) window.clearInterval(this.watchdog);
    this.watchdog = null;
    this.stopFrameCallback();
    this.recovering = false;
    this.recoveryAttempts = 0;
  }

  private stopFrameCallback(): void {
    if (this.frameCallback !== null && "cancelVideoFrameCallback" in this.videoElement) {
      this.videoElement.cancelVideoFrameCallback(this.frameCallback);
    }
    this.frameCallback = null;
  }
}

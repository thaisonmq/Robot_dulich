export const VIDEO_BUFFER_MIN_TARGET_MS = 60;
export const VIDEO_BUFFER_INITIAL_TARGET_MS = 80;
export const VIDEO_BUFFER_MAX_TARGET_MS = 140;
export const AUDIO_BUFFER_TARGET_MS = 40;

const VIDEO_BUFFER_INCREASE_STEP_MS = 20;
const VIDEO_BUFFER_DECREASE_STEP_MS = 10;
const VIDEO_BUFFER_STABLE_SAMPLES = 15;

export function mediaPlayoutTargets(videoTargetMs: number): {
  videoMs: number;
  audioMs: number;
} {
  return {
    videoMs: clampTarget(videoTargetMs),
    audioMs: AUDIO_BUFFER_TARGET_MS,
  };
}

export interface VideoBufferSample {
  jitterMs: number;
  freezeCount: number;
  totalFreezesDuration: number;
  renderedGapCount: number;
}

export type VideoDecodeState =
  | "unknown"
  | "healthy"
  | "upstream-stalled"
  | "missing-keyframe"
  | "decoder-stalled";

export interface VideoDecodeSample {
  bytesReceived: number;
  framesDecoded: number;
  framesDropped: number;
  freezeCount: number;
  keyFramesDecoded: number;
  framesPerSecond: number;
}

export function nextVideoRecoveryAction(
  completedTrackAttempts: number,
  hasPublication: boolean,
): "resubscribe" | "reconnect" {
  return hasPublication && completedTrackAttempts < 2
    ? "resubscribe"
    : "reconnect";
}

/** Distinguishes an idle upstream from a receiver that gets bytes but cannot decode. */
export class VideoDecodeHealth {
  private previous: VideoDecodeSample | null = null;
  private stagnantSamples = 0;

  reset(): void {
    this.previous = null;
    this.stagnantSamples = 0;
  }

  update(sample: VideoDecodeSample): VideoDecodeState {
    const previous = this.previous;
    this.previous = sample;
    if (!previous) return "unknown";
    if (sample.framesDecoded > previous.framesDecoded) {
      this.stagnantSamples = 0;
      return "healthy";
    }
    this.stagnantSamples += 1;
    if (this.stagnantSamples < 2) return "unknown";
    if (sample.bytesReceived <= previous.bytesReceived) return "upstream-stalled";
    if (sample.keyFramesDecoded === 0) return "missing-keyframe";
    return "decoder-stalled";
  }
}

function clampTarget(targetMs: number): number {
  return Math.min(
    VIDEO_BUFFER_MAX_TARGET_MS,
    Math.max(VIDEO_BUFFER_MIN_TARGET_MS, targetMs),
  );
}

function roundUpToTen(value: number): number {
  return Math.ceil(value / 10) * 10;
}

/**
 * Keeps the lowest safe browser playout target for a live robot video track.
 *
 * The controller reacts quickly to RTP jitter or a rendered-frame gap, then
 * lowers latency only after a sustained stable period. It is intentionally a
 * small state machine rather than a timer-heavy estimator so the same client
 * remains cheap on low-power tablets and ARM boards.
 */
export class AdaptiveVideoBuffer {
  private targetMs = VIDEO_BUFFER_INITIAL_TARGET_MS;
  private stableSamples = 0;
  private lastFreezeCount: number | null = null;
  private lastTotalFreezesDuration: number | null = null;
  private lastRenderedGapCount: number | null = null;

  get currentTargetMs(): number {
    return this.targetMs;
  }

  reset(): void {
    this.targetMs = VIDEO_BUFFER_INITIAL_TARGET_MS;
    this.stableSamples = 0;
    this.lastFreezeCount = null;
    this.lastTotalFreezesDuration = null;
    this.lastRenderedGapCount = null;
  }

  update(sample: VideoBufferSample): number {
    const jitterMs = Number.isFinite(sample.jitterMs)
      ? Math.max(0, sample.jitterMs)
      : 0;
    const freezeCount = Math.max(0, sample.freezeCount);
    const totalFreezesDuration = Math.max(0, sample.totalFreezesDuration);
    const renderedGapCount = Math.max(0, sample.renderedGapCount);

    const disrupted =
      (this.lastFreezeCount !== null && freezeCount > this.lastFreezeCount)
      || (
        this.lastTotalFreezesDuration !== null
        && totalFreezesDuration > this.lastTotalFreezesDuration
      )
      || (
        this.lastRenderedGapCount !== null
        && renderedGapCount > this.lastRenderedGapCount
      );

    this.lastFreezeCount = freezeCount;
    this.lastTotalFreezesDuration = totalFreezesDuration;
    this.lastRenderedGapCount = renderedGapCount;

    // Keep enough room for the measured RTP jitter plus scheduling/decoding
    // time. A rendered gap also raises the target even if browser jitter stats
    // have not caught up yet.
    const underPressure = jitterMs > this.targetMs * 0.65;
    if (disrupted || underPressure) {
      const measuredTarget = roundUpToTen(jitterMs * 1.25 + 20);
      this.targetMs = clampTarget(Math.max(
        this.targetMs + VIDEO_BUFFER_INCREASE_STEP_MS,
        measuredTarget,
      ));
      this.stableSamples = 0;
      return this.targetMs;
    }

    // Avoid oscillation: only trade stability back for latency after fifteen
    // consecutive low-jitter samples (normally fifteen seconds).
    if (jitterMs <= this.targetMs * 0.45) {
      this.stableSamples += 1;
      if (this.stableSamples >= VIDEO_BUFFER_STABLE_SAMPLES) {
        this.targetMs = clampTarget(
          this.targetMs - VIDEO_BUFFER_DECREASE_STEP_MS,
        );
        this.stableSamples = 0;
      }
    } else {
      this.stableSamples = 0;
    }

    return this.targetMs;
  }
}

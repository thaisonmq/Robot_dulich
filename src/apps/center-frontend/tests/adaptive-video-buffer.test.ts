import { describe, expect, it } from "vitest";
import {
  AdaptiveVideoBuffer,
  VIDEO_BUFFER_INITIAL_TARGET_MS,
  VIDEO_BUFFER_MAX_TARGET_MS,
  VIDEO_BUFFER_MIN_TARGET_MS,
} from "../src/transports/AdaptiveVideoBuffer";

function sample(
  jitterMs: number,
  overrides: Partial<{
    freezeCount: number;
    totalFreezesDuration: number;
    renderedGapCount: number;
  }> = {},
) {
  return {
    jitterMs,
    freezeCount: overrides.freezeCount ?? 0,
    totalFreezesDuration: overrides.totalFreezesDuration ?? 0,
    renderedGapCount: overrides.renderedGapCount ?? 0,
  };
}

describe("AdaptiveVideoBuffer", () => {
  it("starts with a small motion-first safety margin", () => {
    const controller = new AdaptiveVideoBuffer();

    expect(controller.currentTargetMs).toBe(VIDEO_BUFFER_INITIAL_TARGET_MS);
  });

  it("raises the target when measured jitter approaches the buffer", () => {
    const controller = new AdaptiveVideoBuffer();

    expect(controller.update(sample(60))).toBe(100);
    expect(controller.update(sample(60))).toBe(100);
  });

  it("reacts to a new rendered gap even when RTP jitter is low", () => {
    const controller = new AdaptiveVideoBuffer();

    controller.update(sample(15, { renderedGapCount: 0 }));

    expect(controller.update(sample(15, { renderedGapCount: 1 }))).toBe(100);
  });

  it("does not mistake the first cumulative freeze counters for a new freeze", () => {
    const controller = new AdaptiveVideoBuffer();

    expect(controller.update(sample(10, {
      freezeCount: 3,
      totalFreezesDuration: 1.2,
    }))).toBe(VIDEO_BUFFER_INITIAL_TARGET_MS);
  });

  it("caps the target under severe jitter", () => {
    const controller = new AdaptiveVideoBuffer();

    expect(controller.update(sample(200))).toBe(VIDEO_BUFFER_MAX_TARGET_MS);
    expect(controller.update(sample(200))).toBe(VIDEO_BUFFER_MAX_TARGET_MS);
  });

  it("lowers latency slowly after a sustained stable period", () => {
    const controller = new AdaptiveVideoBuffer();

    controller.update(sample(60));
    for (let index = 0; index < 14; index += 1) {
      controller.update(sample(20));
    }
    expect(controller.currentTargetMs).toBe(100);

    controller.update(sample(20));
    expect(controller.currentTargetMs).toBe(90);

    for (let index = 0; index < 60; index += 1) {
      controller.update(sample(0));
    }
    expect(controller.currentTargetMs).toBe(VIDEO_BUFFER_MIN_TARGET_MS);
  });

  it("reset removes history from the previous video track", () => {
    const controller = new AdaptiveVideoBuffer();
    controller.update(sample(200));

    controller.reset();

    expect(controller.currentTargetMs).toBe(VIDEO_BUFFER_INITIAL_TARGET_MS);
    expect(controller.update(sample(10, { renderedGapCount: 5 })))
      .toBe(VIDEO_BUFFER_INITIAL_TARGET_MS);
  });
});

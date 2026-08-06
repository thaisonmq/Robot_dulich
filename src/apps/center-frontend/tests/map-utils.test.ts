import { describe, expect, it } from "vitest";
import { pixelToWorld, worldToLeaflet, worldToPixel } from "../../../packages/map-utils";

const map = {
  width_pixels: 1600,
  height_pixels: 1000,
  resolution_m_per_pixel: 0.01,
  origin: { x: 0, y: 0, yaw: 0 },
};

describe("map coordinate conversion", () => {
  it("round trips world and pixel coordinates", () => {
    const pixel = worldToPixel({ x: 4.2, y: 7.5 }, map);
    expect(pixel).toEqual({ px: 420, py: 250 });
    expect(pixelToWorld(pixel, map)).toEqual({ x: 4.2, y: 7.5 });
    expect(worldToLeaflet({ x: 4.2, y: 7.5 }, map)).toEqual([250, 420]);
  });

  it("applies a rotated ROS map origin", () => {
    const rotated = {
      ...map,
      width_pixels: 100,
      height_pixels: 100,
      resolution_m_per_pixel: 0.1,
      origin: { x: 2, y: -1, yaw: Math.PI / 2 },
    };
    const pixel = worldToPixel({ x: 1, y: 1 }, rotated);
    expect(pixel.px).toBeCloseTo(20);
    expect(pixel.py).toBeCloseTo(90);
    const world = pixelToWorld(pixel, rotated);
    expect(world.x).toBeCloseTo(1);
    expect(world.y).toBeCloseTo(1);
  });
});

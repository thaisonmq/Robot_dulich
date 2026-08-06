import { describe, expect, it } from "vitest";
import { fitMapViewport, scaleBarMeters, worldToCanvas } from "../src/utils/mappingViewport";

describe("mapping viewport", () => {
  it("keeps occupancy cells square while centering the map", () => {
    const viewport = fitMapViewport(1000, 500, 160, 216, 10);
    expect(viewport.width / viewport.height).toBeCloseTo(160 / 216);
    expect(viewport.y).toBeCloseTo(10);
    expect(viewport.x).toBeGreaterThan(300);
  });

  it("converts a rotated OccupancyGrid origin into screen coordinates", () => {
    const map = {
      width: 10,
      height: 10,
      resolution: 1,
      origin: { x: 2, y: 3, yaw: Math.PI / 2 },
    };
    const viewport = fitMapViewport(100, 100, map.width, map.height, 0);
    expect(worldToCanvas(2, 4, map, viewport)).toEqual({ x: 10, y: 100 });
    const rotated = worldToCanvas(1, 3, map, viewport);
    expect(rotated.x).toBeCloseTo(0);
    expect(rotated.y).toBeCloseTo(90);
  });

  it("selects a readable metric scale", () => {
    expect(scaleBarMeters(48)).toBe(2);
    expect(scaleBarMeters(10)).toBe(10);
  });
});

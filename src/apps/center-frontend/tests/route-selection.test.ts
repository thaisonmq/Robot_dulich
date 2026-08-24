import type { Route } from "../src/types";
import { authoritativeRouteId } from "../src/utils/routeSelection";

function route(missionId: string, selected = "route-a"): Route {
  return {
    route_id: selected,
    selected_route_id: selected,
    mission_id: missionId,
    robot_id: "ROBOT-1",
    destination_id: "CUSTOM-GOAL",
    points: [{ x: 0, y: 0 }, { x: 1, y: 0 }],
    distance_m: 1,
    estimated_seconds: 5,
    candidates: [
      {
        route_id: "route-a", points: [{ x: 0, y: 0 }, { x: 1, y: 0 }],
        total_length: 1, estimated_time: 5, narrow_segments: 0,
        overlap_with_original: 1, valid: true, recommended: true,
      },
      {
        route_id: "route-b", points: [{ x: 0, y: 0 }, { x: 0, y: 1 }],
        total_length: 1, estimated_time: 6, narrow_segments: 0,
        overlap_with_original: 0.2, valid: true, recommended: false,
      },
    ],
  };
}

describe("authoritative preview route selection", () => {
  it("keeps an alternative selected from the same preview mission", () => {
    const prepared = route("mission-current");
    expect(authoritativeRouteId(prepared, prepared, "route-b")).toBe("route-b");
  });

  it("drops a stale selection after localization recomputes the preview", () => {
    const displayed = route("mission-old", "route-b");
    const recomputed = route("mission-new", "route-a");
    expect(authoritativeRouteId(recomputed, displayed, "route-b")).toBe("route-a");
  });

  it("rejects a browser-only id even on the same mission", () => {
    const prepared = route("mission-current");
    expect(authoritativeRouteId(prepared, prepared, "route-browser")).toBe("route-a");
  });
});

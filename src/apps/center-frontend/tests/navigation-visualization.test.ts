import { mergeNavigationVisualization } from "../src/utils/navigationVisualization";

describe("route-aware navigation visualization merge", () => {
  it("keeps the current path for an obstacle-only update on the same route", () => {
    const first = mergeNavigationVisualization(null, {
      revision: 1, map_id: "MAP-A", map_version: 1, route_id: "R1",
      global_path: [{ x: 0, y: 0 }, { x: 1, y: 0 }], dynamic_obstacles: [],
    });
    const second = mergeNavigationVisualization(first, {
      revision: 2, map_id: "MAP-A", map_version: 1, route_id: "R1",
      dynamic_obstacles: [{ x: 0.5, y: 0 }],
    });

    expect(second.global_path).toEqual([{ x: 0, y: 0 }, { x: 1, y: 0 }]);
    expect(second.dynamic_obstacles).toEqual([{ x: 0.5, y: 0 }]);
  });

  it("replaces path when route identity changes", () => {
    const previous = {
      revision: 1, map_id: "MAP-A", map_version: 1, route_id: "R1",
      global_path: [{ x: 0, y: 0 }, { x: 1, y: 0 }], dynamic_obstacles: [],
    };
    const changed = mergeNavigationVisualization(previous, {
      revision: 2, map_id: "MAP-A", map_version: 1, route_id: "R2",
      global_path: [{ x: 0, y: 0 }, { x: 0, y: 1 }],
    });

    expect(changed.route_id).toBe("R2");
    expect(changed.global_path).toEqual([{ x: 0, y: 0 }, { x: 0, y: 1 }]);
  });

  it("honors an explicit path clear", () => {
    const previous = {
      revision: 1, map_id: "MAP-A", map_version: 1, route_id: "R2",
      global_path: [{ x: 0, y: 0 }, { x: 0, y: 1 }], dynamic_obstacles: [],
    };
    const cleared = mergeNavigationVisualization(previous, {
      revision: 2, map_id: "MAP-A", map_version: 1, route_id: "R2",
      global_path: [],
    });

    expect(cleared.global_path).toEqual([]);
  });
});

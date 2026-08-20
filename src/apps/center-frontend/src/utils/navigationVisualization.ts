import type { NavigationVisualization } from "../types";

export function mergeNavigationVisualization(
  previous: NavigationVisualization | null,
  next: NavigationVisualization,
): NavigationVisualization {
  const sameMap = previous?.map_id === next.map_id
    && previous.map_version === next.map_version;
  const sameRoute = sameMap && next.route_id === previous?.route_id;

  return {
    ...next,
    global_path: next.global_path ?? (sameRoute ? previous?.global_path : []) ?? [],
    dynamic_obstacles: next.dynamic_obstacles
      ?? (sameMap ? previous?.dynamic_obstacles : [])
      ?? [],
  };
}

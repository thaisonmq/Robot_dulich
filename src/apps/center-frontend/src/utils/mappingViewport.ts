export interface MapViewport {
  x: number;
  y: number;
  width: number;
  height: number;
  pixelsPerCell: number;
}

export interface OccupancyGeometry {
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number; yaw: number };
}

export function fitMapViewport(
  canvasWidth: number,
  canvasHeight: number,
  mapWidth: number,
  mapHeight: number,
  padding = 18,
): MapViewport {
  const availableWidth = Math.max(1, canvasWidth - padding * 2);
  const availableHeight = Math.max(1, canvasHeight - padding * 2);
  const pixelsPerCell = Math.min(
    availableWidth / Math.max(1, mapWidth),
    availableHeight / Math.max(1, mapHeight),
  );
  const width = mapWidth * pixelsPerCell;
  const height = mapHeight * pixelsPerCell;
  return {
    x: (canvasWidth - width) / 2,
    y: (canvasHeight - height) / 2,
    width,
    height,
    pixelsPerCell,
  };
}

export function worldToCanvas(
  worldX: number,
  worldY: number,
  map: OccupancyGeometry,
  viewport: MapViewport,
): { x: number; y: number } {
  const deltaX = worldX - map.origin.x;
  const deltaY = worldY - map.origin.y;
  const cosine = Math.cos(map.origin.yaw);
  const sine = Math.sin(map.origin.yaw);
  // OccupancyGrid origin can be rotated. Convert world coordinates back into
  // the map-local frame before applying the screen's inverted Y axis.
  const localX = deltaX * cosine + deltaY * sine;
  const localY = -deltaX * sine + deltaY * cosine;
  return {
    x: viewport.x + localX / map.resolution * viewport.pixelsPerCell,
    y: viewport.y + viewport.height - localY / map.resolution * viewport.pixelsPerCell,
  };
}

export function scaleBarMeters(pixelsPerMeter: number, maxPixels = 120): number {
  const candidates = [0.25, 0.5, 1, 2, 5, 10, 20, 50, 100];
  let selected = candidates[0];
  for (const candidate of candidates) {
    if (candidate * pixelsPerMeter > maxPixels) break;
    selected = candidate;
  }
  return selected;
}

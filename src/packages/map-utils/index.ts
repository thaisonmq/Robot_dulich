export interface MapMetadata {
  width_pixels: number;
  height_pixels: number;
  resolution_m_per_pixel: number;
  origin: { x: number; y: number; yaw: number };
}

export type Point = { x: number; y: number };
export type PixelPoint = { px: number; py: number };
export type LeafletPoint = [number, number];

export function worldToPixel(point: Point, map: MapMetadata): PixelPoint {
  return {
    px: (point.x - map.origin.x) / map.resolution_m_per_pixel,
    py:
      map.height_pixels -
      (point.y - map.origin.y) / map.resolution_m_per_pixel,
  };
}

export function pixelToWorld(point: PixelPoint, map: MapMetadata): Point {
  return {
    x: map.origin.x + point.px * map.resolution_m_per_pixel,
    y:
      map.origin.y +
      (map.height_pixels - point.py) * map.resolution_m_per_pixel,
  };
}

export function worldToLeaflet(point: Point, map: MapMetadata): LeafletPoint {
  const { px, py } = worldToPixel(point, map);
  return [py, px];
}


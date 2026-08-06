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
  const dx = point.x - map.origin.x;
  const dy = point.y - map.origin.y;
  const cosine = Math.cos(map.origin.yaw);
  const sine = Math.sin(map.origin.yaw);
  // ROS map metadata expresses the image origin as a full SE(2) transform.
  // Convert world -> map-local with the inverse rotation before flipping Y.
  const localX = cosine * dx + sine * dy;
  const localY = -sine * dx + cosine * dy;
  return {
    px: localX / map.resolution_m_per_pixel,
    py: map.height_pixels - localY / map.resolution_m_per_pixel,
  };
}

export function pixelToWorld(point: PixelPoint, map: MapMetadata): Point {
  const localX = point.px * map.resolution_m_per_pixel;
  const localY = (map.height_pixels - point.py) * map.resolution_m_per_pixel;
  const cosine = Math.cos(map.origin.yaw);
  const sine = Math.sin(map.origin.yaw);
  return {
    x: map.origin.x + cosine * localX - sine * localY,
    y: map.origin.y + sine * localX + cosine * localY,
  };
}

export function worldToLeaflet(point: Point, map: MapMetadata): LeafletPoint {
  const { px, py } = worldToPixel(point, map);
  return [py, px];
}

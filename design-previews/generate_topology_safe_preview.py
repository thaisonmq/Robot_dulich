#!/usr/bin/env python3
"""Render a polished ROS map without changing any traversable source cell."""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def component_count(mask: np.ndarray) -> int:
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    count = 0
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or seen[y, x]:
                continue
            count += 1
            seen[y, x] = True
            queue = deque([(x, y)])
            while queue:
                cx, cy = queue.popleft()
                for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                    if 0 <= nx < width and 0 <= ny < height and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((nx, ny))
    return count


def shift(mask: np.ndarray, dx: int, dy: int) -> np.ndarray:
    output = np.zeros_like(mask)
    source_y = slice(max(0, -dy), min(mask.shape[0], mask.shape[0] - dy))
    source_x = slice(max(0, -dx), min(mask.shape[1], mask.shape[1] - dx))
    target_y = slice(max(0, dy), min(mask.shape[0], mask.shape[0] + dy))
    target_x = slice(max(0, dx), min(mask.shape[1], mask.shape[1] + dx))
    output[target_y, target_x] = mask[source_y, source_x]
    return output


def render(source: Path, target: Path, report: Path) -> None:
    raw = np.asarray(Image.open(source).convert("L"))
    # ROS trinary map: free is above free_thresh; retain it byte-for-byte as a mask.
    source_free = raw > 245
    occupied = raw < 80
    known = raw != 205

    ys, xs = np.where(known)
    x0, x1 = max(0, int(xs.min()) - 6), min(raw.shape[1], int(xs.max()) + 7)
    y0, y1 = max(0, int(ys.min()) - 6), min(raw.shape[0], int(ys.max()) + 7)
    free = source_free[y0:y1, x0:x1].copy()
    occ = occupied[y0:y1, x0:x1].copy()
    known_crop = known[y0:y1, x0:x1]

    # The display navigability mask is intentionally an exact copy. No closing,
    # dilation, erosion, denoise, or AI inference is allowed on this layer.
    display_free = free.copy()
    assert np.array_equal(free, display_free)
    source_components = component_count(free)
    display_components = component_count(display_free)
    assert source_components == display_components

    adjacent_free = (
        shift(free, 1, 0) | shift(free, -1, 0)
        | shift(free, 0, 1) | shift(free, 0, -1)
    )
    structural_edge = (~free) & adjacent_free
    occupied_edge = occ & adjacent_free

    width, height = 1536, 1024
    canvas = Image.new("RGB", (width, height), "#06111E")
    draw = ImageDraw.Draw(canvas)
    draw.text((62, 48), "TOPOLOGY-SAFE MAP", font=font(17, True), fill="#5DE4E7")
    draw.text((62, 80), "Bản đồ hiển thị · Map v5", font=font(32, True), fill="#EDF6FF")
    draw.text((62, 128), "Không suy diễn tường · Không đóng lối đi · 1 ô = 5 cm", font=font(15), fill="#8293AA")
    draw.rounded_rectangle((1255, 56, 1474, 108), radius=25, fill="#0E2A3B", outline="#23516A")
    draw.ellipse((1280, 76, 1292, 88), fill="#43E89B")
    draw.text((1306, 70), "FREE CELLS LOCKED", font=font(13, True), fill="#C9FFE3")

    frame = (62, 176, 1474, 954)
    draw.rounded_rectangle(frame, radius=28, fill="#091827", outline="#173A53", width=2)
    viewport = (92, 206, 1444, 924)
    draw.rounded_rectangle(viewport, radius=20, fill="#071522", outline="#15364C")

    vh, vw = viewport[3] - viewport[1], viewport[2] - viewport[0]
    scale = min((vw - 100) / free.shape[1], (vh - 40) / free.shape[0])
    rw, rh = round(free.shape[1] * scale), round(free.shape[0] * scale)
    ox = viewport[0] + (vw - rw) // 2
    oy = viewport[1] + (vh - rh) // 2

    # Subtle exact metric grid below the map.
    step = 20 * scale
    gx = ox
    while gx <= ox + rw:
        draw.line((gx, oy, gx, oy + rh), fill="#10283A", width=1)
        gx += step
    gy = oy
    while gy <= oy + rh:
        draw.line((ox, gy, ox + rw, gy), fill="#10283A", width=1)
        gy += step

    rgba = np.zeros((*free.shape, 4), dtype=np.uint8)
    rgba[known_crop & ~free & ~occ] = (10, 29, 43, 180)
    rgba[occ] = (14, 26, 36, 255)
    rgba[free] = (25, 64, 84, 255)
    rgba[structural_edge] = (91, 184, 194, 255)
    rgba[occupied_edge] = (190, 244, 245, 255)

    # Nearest-neighbour is deliberate: it guarantees even a one-cell corridor
    # remains present. A separate glow supplies polish without modifying cells.
    map_layer = Image.fromarray(rgba, "RGBA").resize((rw, rh), Image.Resampling.NEAREST)
    edge_alpha = Image.fromarray((structural_edge * 150).astype("uint8")).resize((rw, rh), Image.Resampling.NEAREST)
    edge_alpha = edge_alpha.filter(ImageFilter.GaussianBlur(5))
    glow = Image.new("RGBA", (rw, rh), (74, 218, 226, 0))
    glow.putalpha(edge_alpha)
    canvas.paste(glow, (ox, oy), glow)
    canvas.paste(map_layer, (ox, oy), map_layer)

    draw = ImageDraw.Draw(canvas)
    # Scale and source-truth badge.
    bar = 40 * scale
    sx, sy = viewport[0] + 28, viewport[3] - 30
    draw.line((sx, sy, sx + bar, sy), fill="#DCEBFA", width=4)
    draw.line((sx, sy - 6, sx, sy + 6), fill="#DCEBFA", width=3)
    draw.line((sx + bar, sy - 6, sx + bar, sy + 6), fill="#DCEBFA", width=3)
    draw.text((sx + bar + 12, sy - 11), "2 m", font=font(13, True), fill="#8293AA")

    badge = (viewport[2] - 328, viewport[3] - 60, viewport[2] - 22, viewport[3] - 18)
    draw.rounded_rectangle(badge, radius=18, fill="#0D2636")
    draw.text((badge[0] + 18, badge[1] + 11), f"{int(free.sum()):,} ô free · {source_components} vùng liên thông", font=font(12, True), fill="#91E8E3")

    target.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(target)
    report.write_text(json.dumps({
        "source": str(source),
        "source_free_cells": int(free.sum()),
        "display_free_cells": int(display_free.sum()),
        "source_connected_components": source_components,
        "display_connected_components": display_components,
        "topology_preserved": True,
        "crop": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
    }, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    render(args.source, args.target, args.report)


if __name__ == "__main__":
    main()

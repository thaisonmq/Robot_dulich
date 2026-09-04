#!/usr/bin/env python3
"""Generate an isolated, presentation-only preview from a ROS occupancy PGM."""

from __future__ import annotations

import argparse
import math
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont


BG = "#07111E"
PANEL = "#0B1727"
PANEL_2 = "#0E1D30"
TEXT = "#EDF6FF"
MUTED = "#8293AA"
CYAN = "#5DE4E7"
BLUE = "#4D8DFF"
AMBER = "#FFBF69"
RED = "#FF6174"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def rounded(draw: ImageDraw.ImageDraw, box, radius: int, fill, outline=None, width=1):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def keep_components(mask: np.ndarray, minimum: int) -> np.ndarray:
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    output = np.zeros_like(mask, dtype=bool)
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or seen[y, x]:
                continue
            stack = [(x, y)]
            seen[y, x] = True
            cells = []
            while stack:
                cx, cy = stack.pop()
                cells.append((cx, cy))
                for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                    if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((nx, ny))
            if len(cells) >= minimum:
                for cx, cy in cells:
                    output[cy, cx] = True
    return output


def farthest_free(free: np.ndarray, start: tuple[int, int]) -> tuple[tuple[int, int], dict]:
    h, w = free.shape
    queue = deque([start])
    parent = {start: None}
    farthest = start
    while queue:
        point = queue.popleft()
        farthest = point
        x, y = point
        for nxt in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            nx, ny = nxt
            if 0 <= nx < w and 0 <= ny < h and free[ny, nx] and nxt not in parent:
                parent[nxt] = point
                queue.append(nxt)
    return farthest, parent


def demo_route(free: np.ndarray) -> list[tuple[int, int]]:
    ys, xs = np.where(free)
    if len(xs) == 0:
        return []
    seed = (int(np.median(xs)), int(np.median(ys)))
    if not free[seed[1], seed[0]]:
        seed = (int(xs[len(xs) // 2]), int(ys[len(ys) // 2]))
    a, _ = farthest_free(free, seed)
    b, parents = farthest_free(free, a)
    path = []
    cursor = b
    while cursor is not None:
        path.append(cursor)
        cursor = parents[cursor]
    path.reverse()
    if len(path) < 2:
        return path
    # Keep turns plus periodic anchors, then soften the orthogonal stair-step.
    reduced = [path[0]]
    last_direction = None
    for index in range(1, len(path)):
        direction = (path[index][0] - path[index - 1][0], path[index][1] - path[index - 1][1])
        if last_direction is not None and direction != last_direction:
            reduced.append(path[index - 1])
        if index % 14 == 0:
            reduced.append(path[index])
        last_direction = direction
    reduced.append(path[-1])
    return reduced


def generate(source: Path, target: Path) -> None:
    raw_image = Image.open(source).convert("L")
    raw = np.asarray(raw_image)
    known = raw != 205
    occupied = raw < 80
    free = raw > 245

    occupied_image = Image.fromarray((occupied * 255).astype("uint8"))
    occupied_image = occupied_image.filter(ImageFilter.MaxFilter(3)).filter(ImageFilter.MinFilter(3))
    occupied = keep_components(np.asarray(occupied_image) > 0, 3)
    free_image = Image.fromarray((free * 255).astype("uint8")).filter(ImageFilter.MedianFilter(3))
    free = np.asarray(free_image) > 0

    ys, xs = np.where(known)
    x0, x1 = max(0, int(xs.min()) - 5), min(raw.shape[1], int(xs.max()) + 6)
    y0, y1 = max(0, int(ys.min()) - 5), min(raw.shape[0], int(ys.max()) + 6)
    occupied = occupied[y0:y1, x0:x1]
    free = free[y0:y1, x0:x1]
    raw_crop = raw[y0:y1, x0:x1]

    width, height = 1600, 1000
    canvas = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(canvas)

    # Header
    draw.text((58, 44), "ROVERA", font=font(18, True), fill=CYAN)
    draw.text((58, 75), "BẢN ĐỒ VẬN HÀNH", font=font(36, True), fill=TEXT)
    draw.text((58, 125), "Map v5  /  0,05 m mỗi pixel  /  chế độ hiển thị nâng cao", font=font(16), fill=MUTED)
    rounded(draw, (1295, 54, 1542, 110), 28, "#10283A", "#23425A", 1)
    draw.ellipse((1318, 74, 1330, 86), fill="#43E89B")
    draw.text((1343, 68), "ROBOT ONLINE", font=font(15, True), fill="#BDF9DA")

    # Main map card
    map_box = (48, 176, 1168, 944)
    rounded(draw, map_box, 28, PANEL, "#183149", 2)
    draw.text((82, 206), "TẦNG 01", font=font(14, True), fill=MUTED)
    draw.text((82, 234), "Không gian đã lập bản đồ", font=font(22, True), fill=TEXT)
    rounded(draw, (930, 208, 1129, 251), 20, "#10283A")
    draw.text((952, 220), "LIVE TELEMETRY", font=font(12, True), fill=CYAN)

    viewport = (78, 282, 1138, 910)
    rounded(draw, viewport, 22, "#091523", "#17344B", 1)
    vx0, vy0, vx1, vy1 = viewport
    inner_w, inner_h = vx1 - vx0 - 54, vy1 - vy0 - 42
    scale = min(inner_w / occupied.shape[1], inner_h / occupied.shape[0])
    render_w = max(1, round(occupied.shape[1] * scale))
    render_h = max(1, round(occupied.shape[0] * scale))
    ox = vx0 + (vx1 - vx0 - render_w) // 2
    oy = vy0 + (vy1 - vy0 - render_h) // 2

    # Metric grid: source pixels are 5 cm, so 20 pixels = 1 metre.
    grid_step = 20 * scale
    gx = ox
    while gx <= ox + render_w:
        draw.line((gx, oy, gx, oy + render_h), fill="#10263A", width=1)
        gx += grid_step
    gy = oy
    while gy <= oy + render_h:
        draw.line((ox, gy, ox + render_w, gy), fill="#10263A", width=1)
        gy += grid_step

    layer = np.zeros((*free.shape, 4), dtype=np.uint8)
    layer[free] = (21, 55, 76, 255)
    layer[occupied] = (191, 242, 244, 255)
    map_layer = Image.fromarray(layer, "RGBA").resize((render_w, render_h), Image.Resampling.NEAREST)
    glow = Image.fromarray((occupied * 180).astype("uint8")).resize((render_w, render_h), Image.Resampling.NEAREST)
    glow = glow.filter(ImageFilter.GaussianBlur(5))
    cyan_glow = Image.new("RGBA", (render_w, render_h), (70, 224, 231, 0))
    cyan_glow.putalpha(glow)
    canvas.paste(cyan_glow, (ox, oy), cyan_glow)
    canvas.paste(map_layer, (ox, oy), map_layer)
    draw = ImageDraw.Draw(canvas)

    def screen(point: tuple[int, int]) -> tuple[float, float]:
        return ox + point[0] * scale, oy + point[1] * scale

    route = demo_route(free & ~occupied)
    if len(route) >= 2:
        route_points = [screen(point) for point in route]
        draw.line(route_points, fill="#163B67", width=14, joint="curve")
        draw.line(route_points, fill=BLUE, width=6, joint="curve")
        for index in range(1, len(route_points), max(1, len(route_points) // 7)):
            px, py = route_points[index]
            draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill="#DDEAFF")

        rx, ry = route_points[len(route_points) // 3]
        heading_target = route_points[min(len(route_points) - 1, len(route_points) // 3 + 1)]
        angle = math.atan2(heading_target[1] - ry, heading_target[0] - rx)
        draw.ellipse((rx - 24, ry - 24, rx + 24, ry + 24), fill="#0A1A2A", outline="#8EBCFF", width=4)
        nose = (rx + math.cos(angle) * 16, ry + math.sin(angle) * 16)
        left = (rx + math.cos(angle + 2.35) * 12, ry + math.sin(angle + 2.35) * 12)
        right = (rx + math.cos(angle - 2.35) * 12, ry + math.sin(angle - 2.35) * 12)
        draw.polygon((nose, left, right), fill=BLUE)
        rounded(draw, (rx + 29, ry - 17, rx + 141, ry + 17), 13, "#10253A")
        draw.text((rx + 43, ry - 10), "ROBOT-001", font=font(12, True), fill=TEXT)

        dx, dy = route_points[-1]
        draw.ellipse((dx - 18, dy - 18, dx + 18, dy + 18), fill="#2B2419", outline=AMBER, width=4)
        draw.ellipse((dx - 5, dy - 5, dx + 5, dy + 5), fill=AMBER)

    # Scale bar
    bar = 2 * 20 * scale
    draw.line((vx0 + 30, vy1 - 30, vx0 + 30 + bar, vy1 - 30), fill=TEXT, width=4)
    draw.line((vx0 + 30, vy1 - 36, vx0 + 30, vy1 - 24), fill=TEXT, width=3)
    draw.line((vx0 + 30 + bar, vy1 - 36, vx0 + 30 + bar, vy1 - 24), fill=TEXT, width=3)
    draw.text((vx0 + 40 + bar, vy1 - 42), "2 m", font=font(13, True), fill=MUTED)

    # Right rail
    side = (1192, 176, 1552, 944)
    rounded(draw, side, 28, PANEL, "#183149", 2)
    draw.text((1224, 210), "TRẠNG THÁI", font=font(13, True), fill=MUTED)
    cards = [
        ("Định vị", "READY", "#43E89B"),
        ("Bản đồ", "v5 · ACTIVE", CYAN),
        ("Độ phân giải", "5 cm", BLUE),
    ]
    cy = 246
    for label, value, color in cards:
        rounded(draw, (1216, cy, 1528, cy + 76), 18, PANEL_2)
        draw.text((1236, cy + 14), label, font=font(13), fill=MUTED)
        draw.text((1236, cy + 39), value, font=font(17, True), fill=color)
        cy += 88

    draw.text((1224, 526), "LỚP HIỂN THỊ", font=font(13, True), fill=MUTED)
    legends = [(CYAN, "Biên tường"), ("#15374C", "Vùng đã quét"), (BLUE, "Lộ trình minh họa"), (AMBER, "Điểm đến")]
    ly = 565
    for color, label in legends:
        draw.rounded_rectangle((1224, ly + 2, 1242, ly + 20), radius=5, fill=color)
        draw.text((1258, ly), label, font=font(14), fill=TEXT)
        ly += 40

    rounded(draw, (1216, 748, 1528, 906), 18, "#101B29", "#203447", 1)
    draw.text((1236, 770), "PREVIEW ONLY", font=font(12, True), fill=AMBER)
    draw.multiline_text(
        (1236, 800),
        "Geometry lấy từ PGM thật.\nRoute và vị trí robot chỉ dùng\nđể minh họa phong cách.\nNav2 map không bị thay đổi.",
        font=font(13), fill=MUTED, spacing=8,
    )

    target.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(target, quality=95)

    # Also save a clean map-only render useful for direct visual review.
    clean_target = target.with_name(target.stem + "-map-only.png")
    clean = canvas.crop((58, 186, 1158, 930))
    clean.save(clean_target, quality=95)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    generate(args.source, args.target)


if __name__ == "__main__":
    main()

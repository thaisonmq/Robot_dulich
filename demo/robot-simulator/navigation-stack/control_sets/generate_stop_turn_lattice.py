#!/usr/bin/env python3
"""Generate the Rovera 5 cm straight + in-place-turn Nav2 Humble lattice."""

from __future__ import annotations

import json
import math
from datetime import date
from pathlib import Path


HEADING_COUNT = 24
GRID_RESOLUTION = 0.05
STRAIGHT_LENGTH = 0.15


def normalized(angle: float) -> float:
    return angle % (2.0 * math.pi)


def primitive(
    trajectory_id: int,
    start_index: int,
    end_index: int,
    *,
    left_turn: bool,
    poses: list[list[float]],
    straight_length: float,
) -> dict[str, object]:
    return {
        "trajectory_id": trajectory_id,
        "start_angle_index": start_index,
        "end_angle_index": end_index,
        "left_turn": left_turn,
        "trajectory_radius": 0.0,
        "trajectory_length": straight_length,
        "arc_length": 0.0,
        "straight_length": straight_length,
        "poses": [[round(x, 8), round(y, 8), normalized(yaw)] for x, y, yaw in poses],
    }


def generate() -> dict[str, object]:
    step = 2.0 * math.pi / HEADING_COUNT
    headings = [index * step for index in range(HEADING_COUNT)]
    primitives: list[dict[str, object]] = []
    trajectory_id = 0
    for start_index, yaw in enumerate(headings):
        for direction in (-1, 1):
            end_index = (start_index + direction) % HEADING_COUNT
            poses = [
                [0.0, 0.0, yaw + direction * step * fraction / 3.0]
                for fraction in (1, 2, 3)
            ]
            primitives.append(primitive(
                trajectory_id,
                start_index,
                end_index,
                left_turn=direction > 0,
                poses=poses,
                straight_length=0.0,
            ))
            trajectory_id += 1
        poses = [
            [distance * math.cos(yaw), distance * math.sin(yaw), yaw]
            for distance in (GRID_RESOLUTION, 2 * GRID_RESOLUTION, STRAIGHT_LENGTH)
        ]
        primitives.append(primitive(
            trajectory_id,
            start_index,
            start_index,
            left_turn=True,
            poses=poses,
            straight_length=STRAIGHT_LENGTH,
        ))
        trajectory_id += 1
    return {
        "version": 1.0,
        "date_generated": date.today().isoformat(),
        "lattice_metadata": {
            "motion_model": "diff",
            "turning_radius": 0.001,
            "grid_resolution": GRID_RESOLUTION,
            "stopping_threshold": 5,
            "num_of_headings": HEADING_COUNT,
            "heading_angles": headings,
            "number_of_trajectories": len(primitives),
        },
        "primitives": primitives,
    }


if __name__ == "__main__":
    output = Path(__file__).with_name("rovera_5cm_24_heading_stop_turn.json")
    output.write_text(json.dumps(generate(), indent=2) + "\n")

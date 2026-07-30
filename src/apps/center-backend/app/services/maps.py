MAP = {
    "map_id": "MAP-001",
    "name": "Khu tham quan demo",
    "image_url": "/maps/map-001.svg",
    "width_pixels": 1600,
    "height_pixels": 1000,
    "resolution_m_per_pixel": 0.01,
    "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "restricted_zones": [
        {"zone_id": "ZONE-001", "name": "Khu vực kỹ thuật", "points": [
            {"x": 12.0, "y": 6.1}, {"x": 14.0, "y": 6.1},
            {"x": 14.0, "y": 7.2}, {"x": 12.0, "y": 7.2}
        ]}
    ],
}

DESTINATIONS = [
    {"destination_id": "DEST-001", "map_id": "MAP-001", "name": "Khu trưng bày A", "x": 10.2, "y": 5.6, "yaw": 1.57, "enabled": True},
    {"destination_id": "DEST-002", "map_id": "MAP-001", "name": "Khu trưng bày B", "x": 13.2, "y": 2.2, "yaw": 3.14, "enabled": True},
    {"destination_id": "DEST-003", "map_id": "MAP-001", "name": "Sảnh chính", "x": 2.4, "y": 1.4, "yaw": 0.0, "enabled": True},
    {"destination_id": "DEST-004", "map_id": "MAP-001", "name": "Thang máy", "x": 8.2, "y": 8.2, "yaw": -1.57, "enabled": True},
]


def destination_by_id(destination_id: str) -> dict | None:
    return next((item for item in DESTINATIONS if item["destination_id"] == destination_id), None)


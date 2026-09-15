"""Coarse image-bbox -> bearing -> laser-scan-correlated distance estimate
(docs/09-perception-architecture.md perception pipeline step 5: "Depth/geometric
estimation ... LaserScan+camera FOV correlation" fallback path used when no depth
source is present, which is the MVP's only source since no depth camera is modeled).

Known MVP simplification, documented rather than hidden: the camera is treated as
co-located with and forward-aligned to `base_link` (a common approximation for a small
mobile-robot front camera) — no separate camera_frame -> base_link TF lookup is
performed for the bearing/distance estimate itself; a real TF lookup IS used one level
up (map -> base_link) to place the final position in `map` frame when localized
(ros_mcp.adapters.perception.perception_adapter).
"""
from __future__ import annotations

import math

from ros_mcp.perception.laser_scan import LaserScanAnalysis


def estimate_bearing_rad(bbox_center_x_px: float, image_width_px: int, horizontal_fov_rad: float) -> float:
    """0 = straight ahead (image center), negative = left, positive = right — matching
    the laser scan's angle convention (ros_mcp.perception.laser_scan sector math)."""
    if image_width_px <= 0:
        return 0.0
    normalized = (bbox_center_x_px / image_width_px) - 0.5
    return normalized * horizontal_fov_rad


def correlate_distance(
    analysis: LaserScanAnalysis, bearing_rad: float, *, tolerance_rad: float
) -> tuple[float | None, str]:
    """Returns (distance_m, distance_source). distance_source is always one of
    "laser_scan_correlation" or "unavailable" here — "depth_image" is reserved for a
    future depth-camera-equipped plugin, never produced by this function."""
    best: float | None = None
    best_delta = tolerance_rad

    # Reuse the already-computed nearest-per-sector summary plus the global
    # nearest/farthest as coarse candidates rather than re-deriving per-beam angles —
    # the MVP's bearing tolerance is wide (several sectors), and a full per-beam scan
    # would just duplicate analyze_laser_scan's own work.
    candidates = [analysis.nearest, analysis.farthest, *analysis.sectors.values()]
    for reading in candidates:
        if reading is None:
            continue
        delta = abs(_angular_delta(reading.angle_rad, bearing_rad))
        if delta <= best_delta:
            best_delta = delta
            best = reading.range_m

    if best is None:
        return None, "unavailable"
    return best, "laser_scan_correlation"


def _angular_delta(a: float, b: float) -> float:
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff <= -math.pi:
        diff += 2 * math.pi
    return diff

"""Pure laser-scan classification/geometry (docs/09-perception-architecture.md Laser
Scan Reasoning). Used both by robot.get_laser_scan (docs/04-mcp-surface.md) and by the
closed-loop motion controller's obstacle check (docs/07-motion-architecture.md step 5) —
one sectoring implementation, not two.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ros_mcp.contracts.results import RangeReading

Sector = str  # "front" | "left" | "right" | "rear"

_SECTOR_BOUNDS_RAD: tuple[tuple[Sector, float, float], ...] = (
    ("front", -math.pi / 6, math.pi / 6),
    ("left", math.pi / 6, 5 * math.pi / 6),
    ("right", -5 * math.pi / 6, -math.pi / 6),
    # rear covers the remaining wedge on both sides of +-pi; handled as a fallback below.
)


def _sector_for_angle(angle_rad: float) -> Sector:
    wrapped = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
    for sector, lo, hi in _SECTOR_BOUNDS_RAD:
        if lo <= wrapped < hi:
            return sector
    return "rear"


@dataclass(frozen=True)
class LaserScanAnalysis:
    stamp_sec: float | None
    frame_id: str | None
    nearest: RangeReading | None
    farthest: RangeReading | None
    sectors: dict[Sector, RangeReading | None]
    raw_ranges: tuple[float, ...]
    invalid_beam_count: int


def analyze_laser_scan(raw_scan: Any) -> LaserScanAnalysis:
    """`raw_scan` is a sensor_msgs/msg/LaserScan instance (or anything exposing the
    same fields: ranges, angle_min, angle_increment, range_min, range_max, header)."""
    ranges = list(raw_scan.ranges)
    angle_min = float(raw_scan.angle_min)
    angle_increment = float(raw_scan.angle_increment)
    range_min = float(raw_scan.range_min)
    range_max = float(raw_scan.range_max)

    nearest: RangeReading | None = None
    farthest: RangeReading | None = None
    sector_nearest: dict[Sector, RangeReading | None] = {
        "front": None,
        "left": None,
        "right": None,
        "rear": None,
    }
    invalid_count = 0

    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_increment
        if math.isnan(r) or r < range_min:
            invalid_count += 1
            continue
        if math.isinf(r) or r > range_max:
            # "No obstacle detected out to sensor max range" — not an obstacle, not an
            # error; contributes to farthest-in-range reasoning only when finite, so
            # infinite readings are excluded from both nearest and farthest here
            # (09-perception-architecture.md: farthest is defined over valid, finite
            # beams only).
            continue

        reading = RangeReading(range_m=r, angle_rad=angle, sector=_sector_for_angle(angle))

        if nearest is None or r < nearest.range_m:
            nearest = reading
        if farthest is None or r > farthest.range_m:
            farthest = reading

        current_sector_nearest = sector_nearest[reading.sector]
        if current_sector_nearest is None or r < current_sector_nearest.range_m:
            sector_nearest[reading.sector] = reading

    header = getattr(raw_scan, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    stamp_sec = None
    if stamp is not None:
        stamp_sec = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
    frame_id = getattr(header, "frame_id", None) if header is not None else None

    return LaserScanAnalysis(
        stamp_sec=stamp_sec,
        frame_id=frame_id,
        nearest=nearest,
        farthest=farthest,
        sectors=sector_nearest,
        raw_ranges=tuple(ranges),
        invalid_beam_count=invalid_count,
    )

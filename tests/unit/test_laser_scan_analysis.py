"""Unit tests for laser scan classification (valid / inf / out-of-range / artifact) —
docs/09-perception-architecture.md Laser Scan Reasoning, distinguishing obstacle vs.
sensor artifact vs. invalid range vs. infinite range."""
from __future__ import annotations

import math

from sensor_msgs.msg import LaserScan

from ros_mcp.perception.laser_scan import analyze_laser_scan


def _scan(ranges: list[float], angle_min: float = -math.pi, angle_increment: float | None = None) -> LaserScan:
    scan = LaserScan()
    scan.angle_min = angle_min
    n = len(ranges)
    scan.angle_increment = angle_increment if angle_increment is not None else (2 * math.pi / max(n, 1))
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = ranges
    scan.header.frame_id = "base_scan"
    return scan


def test_all_valid_ranges_finds_nearest_and_farthest():
    scan = _scan([5.0, 1.0, 3.0, 2.0], angle_min=-0.1, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.nearest.range_m == 1.0
    assert analysis.farthest.range_m == 5.0
    assert analysis.invalid_beam_count == 0


def test_inf_reading_is_not_an_obstacle_and_not_invalid():
    scan = _scan([1.0, float("inf"), 2.0], angle_min=-0.05, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.invalid_beam_count == 0
    assert analysis.nearest.range_m == 1.0
    assert analysis.farthest.range_m == 2.0  # inf excluded from farthest


def test_all_inf_scan_has_no_nearest_or_farthest():
    scan = _scan([float("inf")] * 5, angle_min=-0.1, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.nearest is None
    assert analysis.farthest is None
    assert analysis.invalid_beam_count == 0


def test_nan_reading_is_invalid_and_excluded():
    scan = _scan([1.0, float("nan"), 2.0], angle_min=-0.05, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.invalid_beam_count == 1
    assert analysis.nearest.range_m == 1.0


def test_below_range_min_is_invalid_artifact():
    scan = _scan([0.01, 1.0, 2.0], angle_min=-0.05, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.invalid_beam_count == 1
    assert analysis.nearest.range_m == 1.0


def test_above_range_max_treated_like_no_detection_not_invalid():
    scan = _scan([1.0, 50.0, 2.0], angle_min=-0.05, angle_increment=0.05)  # range_max=10.0
    analysis = analyze_laser_scan(scan)
    assert analysis.invalid_beam_count == 0
    assert analysis.farthest.range_m == 2.0


def test_front_sector_detects_nearest_obstacle_ahead():
    # angle_min=-pi, so index for angle 0 (straight ahead / front sector) is at the
    # midpoint of a full 360-degree scan.
    n = 36
    ranges = [5.0] * n
    angle_increment = 2 * math.pi / n
    front_index = n // 2  # angle ~= 0
    ranges[front_index] = 0.5
    scan = _scan(ranges, angle_min=-math.pi, angle_increment=angle_increment)
    analysis = analyze_laser_scan(scan)
    assert analysis.sectors["front"] is not None
    assert analysis.sectors["front"].range_m == 0.5


def test_raw_ranges_preserved_verbatim():
    scan = _scan([1.0, 2.0, 3.0], angle_min=-0.05, angle_increment=0.05)
    analysis = analyze_laser_scan(scan)
    assert analysis.raw_ranges == (1.0, 2.0, 3.0)

"""Unit tests for robot.get_laser_scan / robot.get_camera_image result assembly."""
from __future__ import annotations

import math
from datetime import datetime, timezone

import numpy as np
from sensor_msgs.msg import Image, LaserScan

from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.subscriptions import CachedMessage
from ros_mcp.mcp.camera_resolver import resolve_camera_image
from ros_mcp.mcp.laser_scan_resolver import resolve_laser_scan

NOW = datetime.now(timezone.utc)


class FakeSubscriptions:
    def __init__(self, cache: dict[str, CachedMessage]) -> None:
        self._cache = cache

    def ensure_subscribed(self, topic, type_name, *, qos=None):
        pass

    def get_latest(self, topic):
        return self._cache.get(topic)

    def is_fresh(self, topic, max_age_s):
        cached = self._cache.get(topic)
        return cached is not None and cached.age_s <= max_age_s


def test_laser_scan_capability_unavailable_when_no_lidar():
    registry = InMemoryCapabilityRegistry()
    result = resolve_laser_scan(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions({}),
        include_raw_ranges=False, sensor_stale_s=1.0,
    )
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"


def test_laser_scan_returns_structured_nearest_and_sectors():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                          backend_id="laser_scan", resolved_topics={"scan": "/scan"}),)
    )
    scan = LaserScan()
    scan.angle_min = -math.pi
    n = 36
    scan.angle_increment = 2 * math.pi / n
    scan.range_min = 0.1
    scan.range_max = 10.0
    ranges = [5.0] * n
    ranges[n // 2] = 0.8  # front
    scan.ranges = ranges
    scan.header.frame_id = "base_scan"

    cache = {"/scan": CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=0.1)}
    result = resolve_laser_scan(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions(cache),
        include_raw_ranges=False, sensor_stale_s=1.0,
    )
    assert result.status == "succeeded"
    assert math.isclose(result.nearest.range_m, 0.8, rel_tol=1e-5)
    assert math.isclose(result.sectors["front"].range_m, 0.8, rel_tol=1e-5)
    assert result.raw_ranges is None
    assert result.stale is False


def test_laser_scan_include_raw_ranges_flag():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                          backend_id="laser_scan", resolved_topics={"scan": "/scan"}),)
    )
    scan = LaserScan()
    scan.angle_min = -0.1
    scan.angle_increment = 0.05
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0, 3.0]
    cache = {"/scan": CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=0.1)}
    result = resolve_laser_scan(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions(cache),
        include_raw_ranges=True, sensor_stale_s=1.0,
    )
    assert result.raw_ranges == (1.0, 2.0, 3.0)


def test_laser_scan_flags_stale_but_still_returns_data():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                          backend_id="laser_scan", resolved_topics={"scan": "/scan"}),)
    )
    scan = LaserScan()
    scan.angle_min = -0.1
    scan.angle_increment = 0.05
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0]
    cache = {"/scan": CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=5.0)}
    result = resolve_laser_scan(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions(cache),
        include_raw_ranges=False, sensor_stale_s=1.0,
    )
    assert result.status == "succeeded"
    assert result.stale is True


def test_camera_image_capability_unavailable_without_camera():
    registry = InMemoryCapabilityRegistry()
    result = resolve_camera_image(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions({}),
        max_width_px=640, max_width_px_cap=1280,
    )
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"


def test_camera_image_returns_downsized_jpeg():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="visual_observation", confidence=Confidence.CONFIRMED,
                          backend_id="camera", resolved_topics={"image": "/camera/image_raw"}),)
    )
    img = Image()
    img.height = 20
    img.width = 40
    img.encoding = "rgb8"
    img.data = (np.random.rand(20, 40, 3) * 255).astype("uint8").tobytes()
    img.header.frame_id = "camera_link"
    cache = {"/camera/image_raw": CachedMessage(value={}, raw=img, stamp=NOW, received_at=NOW, age_s=0.05)}

    result = resolve_camera_image(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions(cache),
        max_width_px=10, max_width_px_cap=1280,
    )
    assert result.status == "succeeded"
    assert result.width_px == 10
    assert result.original_width_px == 40
    assert result.image_jpeg_bytes[:2] == b"\xff\xd8"
    assert result.frame == "camera_link"


def test_camera_image_max_width_capped_by_server_config():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="visual_observation", confidence=Confidence.CONFIRMED,
                          backend_id="camera", resolved_topics={"image": "/camera/image_raw"}),)
    )
    img = Image()
    img.height = 20
    img.width = 2000
    img.encoding = "rgb8"
    img.data = (np.random.rand(20, 2000, 3) * 255).astype("uint8").tobytes()
    cache = {"/camera/image_raw": CachedMessage(value={}, raw=img, stamp=NOW, received_at=NOW, age_s=0.05)}

    result = resolve_camera_image(
        robot_id="r1", command_id="c1", duration_sec=0.01,
        registry=registry, subscriptions=FakeSubscriptions(cache),
        max_width_px=5000,  # caller asked for huge width...
        max_width_px_cap=1280,  # ...but server config caps it
    )
    assert result.width_px == 1280

"""Unit tests for DefaultPerceptionAdapter.detect_objects — the full acquire -> detect
-> geometry -> TF -> structured-result pipeline (docs/09-perception-architecture.md)."""
from __future__ import annotations

import asyncio
import math
from datetime import datetime, timezone

import numpy as np
from sensor_msgs.msg import Image, LaserScan

from ros_mcp.adapters.perception.perception_adapter import DefaultPerceptionAdapter
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.contracts.adapters import DetectedObject2D
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import PerceptionConfig, SafetyConfig
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.plugins import PluginHealth, PluginMetadata
from ros_mcp.contracts.subscriptions import CachedMessage
from ros_mcp.contracts.tf import TransformResult

NOW = datetime.now(timezone.utc)


class FakeSubscriptions:
    def __init__(self, cache: dict[str, CachedMessage]) -> None:
        self._cache = cache
        self.ensure_subscribed_calls: list[tuple[str, str]] = []

    def ensure_subscribed(self, topic, type_name, *, qos=None):
        self.ensure_subscribed_calls.append((topic, type_name))

    def get_latest(self, topic):
        return self._cache.get(topic)

    def is_fresh(self, topic, max_age_s):
        cached = self._cache.get(topic)
        return cached is not None and cached.age_s <= max_age_s


class FixedDetector:
    """Always returns one detection centered in the image (bearing ~= 0, i.e. front)."""

    def __init__(self, boxes):
        self.metadata = PluginMetadata(plugin_id="fixed_test_detector", api_version="1.0.0",
                                        provides_capabilities=("object_detection",), requires=())
        self._boxes = boxes

    async def health_check(self):
        return PluginHealth.HEALTHY

    async def detect(self, image):
        return self._boxes


class NoOpTFAdapter:
    def __init__(self, ok=True, x=0.0, y=0.0, yaw=0.0):
        self._ok = ok
        self._x, self._y, self._yaw = x, y, yaw

    async def lookup_transform(self, target_frame, source_frame, stamp, timeout_s):
        if self._ok:
            return TransformResult(ok=True, x=self._x, y=self._y, yaw=self._yaw, error=None)
        return TransformResult(ok=False, x=None, y=None, yaw=None, error="not_connected")

    def known_frames(self):
        return frozenset({"map", "odom", "base_link"})


def _camera_registry():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="visual_observation", confidence=Confidence.CONFIRMED,
                          backend_id="camera", resolved_topics={"image": "/camera/image_raw"}),)
    )
    return registry


def _image_cache():
    img = Image()
    img.height, img.width = 20, 40
    img.encoding = "rgb8"
    img.data = (np.random.rand(20, 40, 3) * 255).astype("uint8").tobytes()
    img.header.frame_id = "camera_link"
    return {"/camera/image_raw": CachedMessage(value={}, raw=img, stamp=NOW, received_at=NOW, age_s=0.05)}


def _adapter(*, registry, subscriptions, detector, tf_adapter=None, image_topic="/camera/image_raw",
             scan_topic="/scan"):
    return DefaultPerceptionAdapter(
        robot_id="r1",
        registry=registry,
        subscriptions=subscriptions,
        tf_adapter=tf_adapter or NoOpTFAdapter(),
        detector_plugin=detector,
        safety_config_provider=lambda: SafetyConfig(),
        perception_config_provider=lambda: PerceptionConfig(),
        image_topic=image_topic,
        scan_topic=scan_topic,
    )


def test_constructor_eagerly_subscribes_to_configured_image_topic():
    # Regression test: get_camera_image/detect_objects only ever read the last-value
    # cache (SubscriptionManager.get_latest MUST NOT subscribe as a side effect,
    # 13-contracts.md §9) — something must call ensure_subscribed for the camera topic
    # up front, the same way CmdVelMotionPlugin does for odom/cmd_vel/scan.
    subscriptions = FakeSubscriptions({})
    _adapter(registry=_camera_registry(), subscriptions=subscriptions, detector=FixedDetector(()),
             image_topic="/camera/image_raw")
    assert ("/camera/image_raw", "sensor_msgs/msg/Image") in subscriptions.ensure_subscribed_calls


def test_detect_objects_capability_unavailable_without_camera():
    adapter = _adapter(registry=InMemoryCapabilityRegistry(), subscriptions=FakeSubscriptions({}),
                        detector=FixedDetector(()))
    result = asyncio.run(adapter.detect_objects(None))
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"


def test_detect_objects_no_detections_returns_empty_list():
    registry = _camera_registry()
    adapter = _adapter(registry=registry, subscriptions=FakeSubscriptions(_image_cache()),
                        detector=FixedDetector(()))
    result = asyncio.run(adapter.detect_objects(None))
    assert result.status == "succeeded"
    assert result.objects == ()
    assert result.detector_plugin_id == "fixed_test_detector"


def test_detect_objects_filters_by_label():
    registry = _camera_registry()
    boxes = (DetectedObject2D(label="chair", confidence=0.5, bbox_px=(15, 5, 25, 15)),)
    adapter = _adapter(registry=registry, subscriptions=FakeSubscriptions(_image_cache()),
                        detector=FixedDetector(boxes))
    result = asyncio.run(adapter.detect_objects(("person",)))
    assert result.objects == ()

    result2 = asyncio.run(adapter.detect_objects(("chair",)))
    assert len(result2.objects) == 1


def test_detect_objects_correlates_distance_via_laser_scan_no_localization():
    registry = _camera_registry()
    # centered bbox -> bearing ~0 (front)
    boxes = (DetectedObject2D(label="chair", confidence=0.7, bbox_px=(15, 5, 25, 15)),)

    scan = LaserScan()
    scan.angle_min = -math.pi
    n = 360
    scan.angle_increment = 2 * math.pi / n
    scan.range_min, scan.range_max = 0.1, 10.0
    ranges = [8.0] * n
    ranges[n // 2] = 2.5  # front
    scan.ranges = ranges

    cache = _image_cache()
    cache["/scan"] = CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=0.05)
    registry.update(
        registry.current()
        + (CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                            backend_id="laser_scan", resolved_topics={"scan": "/scan"}),)
    )

    adapter = _adapter(registry=registry, subscriptions=FakeSubscriptions(cache), detector=FixedDetector(boxes))
    result = asyncio.run(adapter.detect_objects(None))
    assert len(result.objects) == 1
    obj = result.objects[0]
    assert obj.distance_source == "laser_scan_correlation"
    assert math.isclose(obj.distance_m, 2.5, rel_tol=1e-4)
    assert obj.position is not None
    assert obj.position.frame == "base_link"


def test_detect_objects_position_in_map_frame_when_localized():
    registry = _camera_registry()
    boxes = (DetectedObject2D(label="chair", confidence=0.7, bbox_px=(15, 5, 25, 15)),)

    scan = LaserScan()
    scan.angle_min = -math.pi
    n = 360
    scan.angle_increment = 2 * math.pi / n
    scan.range_min, scan.range_max = 0.1, 10.0
    ranges = [8.0] * n
    ranges[n // 2] = 2.0
    scan.ranges = ranges

    cache = _image_cache()
    cache["/scan"] = CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=0.05)
    registry.update(
        registry.current()
        + (
            CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                             backend_id="laser_scan", resolved_topics={"scan": "/scan"}),
            CapabilityEntry(capability_id="localization", confidence=Confidence.CONFIRMED, backend_id="amcl"),
        )
    )

    tf_adapter = NoOpTFAdapter(ok=True, x=10.0, y=0.0, yaw=0.0)
    adapter = _adapter(registry=registry, subscriptions=FakeSubscriptions(cache), detector=FixedDetector(boxes),
                        tf_adapter=tf_adapter)
    result = asyncio.run(adapter.detect_objects(None))
    obj = result.objects[0]
    assert obj.position.frame == "map"
    assert math.isclose(obj.position.x, 12.0, abs_tol=0.1)  # robot at map x=10, object 2m ahead


def test_detect_objects_tf_unavailable_when_localized_but_lookup_fails():
    registry = _camera_registry()
    boxes = (DetectedObject2D(label="chair", confidence=0.7, bbox_px=(15, 5, 25, 15)),)
    scan = LaserScan()
    scan.angle_min = -math.pi
    n = 360
    scan.angle_increment = 2 * math.pi / n
    scan.range_min, scan.range_max = 0.1, 10.0
    ranges = [8.0] * n
    ranges[n // 2] = 2.0  # a distinct front spike, matching the bearing~0 test bbox
    scan.ranges = ranges

    cache = _image_cache()
    cache["/scan"] = CachedMessage(value={}, raw=scan, stamp=NOW, received_at=NOW, age_s=0.05)
    registry.update(
        registry.current()
        + (
            CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED,
                             backend_id="laser_scan", resolved_topics={"scan": "/scan"}),
            CapabilityEntry(capability_id="localization", confidence=Confidence.CONFIRMED, backend_id="amcl"),
        )
    )
    adapter = _adapter(registry=registry, subscriptions=FakeSubscriptions(cache), detector=FixedDetector(boxes),
                        tf_adapter=NoOpTFAdapter(ok=False))
    result = asyncio.run(adapter.detect_objects(None))
    obj = result.objects[0]
    assert obj.position is None
    assert obj.position_error == "TF_UNAVAILABLE"
    assert obj.distance_m is not None  # distance was found even though TF placement failed

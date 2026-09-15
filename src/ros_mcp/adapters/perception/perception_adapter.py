"""DefaultPerceptionAdapter — structurally satisfies
ros_mcp.contracts.adapters.PerceptionAdapter (docs/13-contracts.md §7,
docs/09-perception-architecture.md).

MUST NOT depend on ExecutionManager, CommandPlanner, or SemanticCommandFactory — and
does not: every method here takes only the plain parameters the frozen Protocol
specifies and returns ToolResult data, never a SemanticCommand (10-safety-and-trust.md
trust-boundary rule, ADR-012). `command_id`/`duration_sec` are placeholders (this
adapter has no SemanticCommand to draw them from); the caller — the read_handler wired
into ExecutionManager — re-stamps both onto the returned frozen dataclass via
`dataclasses.replace` before it reaches the MCP layer.
"""
from __future__ import annotations

import math
from typing import Any

from ros_mcp.contracts.adapters import DetectedObject2D, ObjectDetectorPlugin, RawImage
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.config import PerceptionConfig, SafetyConfig
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.results import (
    CameraImageResult,
    DetectedObject,
    DetectObjectsResult,
    LaserScanResult,
    ObjectPosition,
)
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.execution.error_results import build_error_result
from ros_mcp.mcp.camera_resolver import resolve_camera_image
from ros_mcp.mcp.laser_scan_resolver import resolve_laser_scan
from ros_mcp.perception.detection_geometry import correlate_distance, estimate_bearing_rad
from ros_mcp.perception.laser_scan import analyze_laser_scan

_DEFAULT_CAMERA_HORIZONTAL_FOV_RAD = math.radians(60.0)
_DEFAULT_BEARING_TOLERANCE_RAD = math.radians(15.0)
_TF_LOOKUP_TIMEOUT_S = 0.5


def _to_raw_image(raw_msg: Any, *, is_compressed: bool) -> RawImage:
    from datetime import datetime, timezone

    header = getattr(raw_msg, "header", None)
    frame_id = getattr(header, "frame_id", "") or ""

    if is_compressed:
        from ros_mcp.perception.camera import to_pil_image

        pil_image = to_pil_image(raw_msg, is_compressed=True).convert("RGB")
        import numpy as np

        arr = np.asarray(pil_image)
        return RawImage(
            width_px=pil_image.width,
            height_px=pil_image.height,
            encoding="rgb8",
            data=arr.tobytes(),
            frame_id=frame_id,
            stamp=datetime.now(timezone.utc),
        )

    return RawImage(
        width_px=raw_msg.width,
        height_px=raw_msg.height,
        encoding=raw_msg.encoding,
        data=bytes(raw_msg.data),
        frame_id=frame_id,
        stamp=datetime.now(timezone.utc),
    )


class DefaultPerceptionAdapter:
    """Structurally satisfies ros_mcp.contracts.adapters.PerceptionAdapter."""

    def __init__(
        self,
        *,
        robot_id: str,
        registry: CapabilityRegistry,
        subscriptions: SubscriptionManager,
        tf_adapter: TFAdapter,
        detector_plugin: ObjectDetectorPlugin,
        safety_config_provider: Any,
        perception_config_provider: Any,
        camera_horizontal_fov_rad: float = _DEFAULT_CAMERA_HORIZONTAL_FOV_RAD,
        bearing_tolerance_rad: float = _DEFAULT_BEARING_TOLERANCE_RAD,
        base_frame: str = "base_link",
        image_topic: str | None = None,
        scan_topic: str | None = None,
    ) -> None:
        """`base_frame`: see the matching note on Nav2NavigationPlugin — platform-
        specific TF frame name, not a frozen-contract field.

        `image_topic`/`scan_topic`, when given, are eagerly subscribed here (matching
        CmdVelMotionPlugin's pattern of subscribing to its configured topics at
        construction, docs/13-contracts.md §9 ensure_subscribed) — without this, nothing
        would ever call ensure_subscribed for them purely to serve
        get_laser_scan_summary/get_camera_image/detect_objects, since those methods
        only read the last-value cache and the contract forbids get_latest from
        subscribing as a side effect. (In practice CmdVelMotionPlugin also subscribes
        to the scan topic for its own obstacle check, but PerceptionAdapter must not
        depend on that coincidence to function.)
        """
        self._robot_id = robot_id
        self._registry = registry
        self._subscriptions = subscriptions
        self._tf_adapter = tf_adapter
        self._detector_plugin = detector_plugin
        self._safety_config_provider = safety_config_provider
        self._perception_config_provider = perception_config_provider
        self._camera_horizontal_fov_rad = camera_horizontal_fov_rad
        self._bearing_tolerance_rad = bearing_tolerance_rad
        self._base_frame = base_frame

        if image_topic is not None:
            subscriptions.ensure_subscribed(image_topic, "sensor_msgs/msg/Image")
        if scan_topic is not None:
            subscriptions.ensure_subscribed(scan_topic, "sensor_msgs/msg/LaserScan")

    async def get_laser_scan_summary(self, include_raw: bool) -> LaserScanResult:
        safety: SafetyConfig = self._safety_config_provider()
        return resolve_laser_scan(
            robot_id=self._robot_id,
            command_id="",
            duration_sec=0.0,
            registry=self._registry,
            subscriptions=self._subscriptions,
            include_raw_ranges=include_raw,
            sensor_stale_s=safety.sensor_stale_s,
        )

    async def get_camera_image(self, max_width_px: int) -> CameraImageResult:
        perception: PerceptionConfig = self._perception_config_provider()
        return resolve_camera_image(
            robot_id=self._robot_id,
            command_id="",
            duration_sec=0.0,
            registry=self._registry,
            subscriptions=self._subscriptions,
            max_width_px=max_width_px,
            max_width_px_cap=perception.camera_max_width_px_cap,
        )

    async def detect_objects(self, labels: tuple[str, ...] | None) -> DetectObjectsResult:
        camera_cap = self._registry.get("visual_observation")
        if camera_cap is None or "image" not in camera_cap.resolved_topics:
            return self._detect_error(ErrorCode.CAPABILITY_UNAVAILABLE, "no camera available")

        image_topic = camera_cap.resolved_topics["image"]
        cached_image = self._subscriptions.get_latest(image_topic)
        if cached_image is None:
            return self._detect_error(ErrorCode.SENSOR_STALE, f"no camera image ever received on {image_topic}")

        is_compressed = "Compressed" in type(cached_image.raw).__name__
        raw_image = _to_raw_image(cached_image.raw, is_compressed=is_compressed)

        detected: tuple[DetectedObject2D, ...] = await self._detector_plugin.detect(raw_image)
        if labels:
            allowed = set(labels)
            detected = tuple(d for d in detected if d.label in allowed)

        scan_analysis = self._current_laser_scan_analysis()
        localization_cap = self._registry.get("localization")

        objects: list[DetectedObject] = []
        for det in detected:
            bbox_center_x = (det.bbox_px[0] + det.bbox_px[2]) / 2.0
            bearing_rad = estimate_bearing_rad(bbox_center_x, raw_image.width_px, self._camera_horizontal_fov_rad)

            distance_m: float | None = None
            distance_source = "unavailable"
            if scan_analysis is not None:
                distance_m, distance_source = correlate_distance(
                    scan_analysis, bearing_rad, tolerance_rad=self._bearing_tolerance_rad
                )

            position: ObjectPosition | None = None
            position_error: str | None = None
            if distance_m is not None:
                x_local = distance_m * math.cos(bearing_rad)
                y_local = distance_m * math.sin(bearing_rad)
                if localization_cap is not None:
                    tf_result = await self._tf_adapter.lookup_transform(
                        "map", self._base_frame, cached_image.stamp, _TF_LOOKUP_TIMEOUT_S
                    )
                    if tf_result.ok:
                        assert tf_result.x is not None and tf_result.y is not None and tf_result.yaw is not None
                        cos_yaw, sin_yaw = math.cos(tf_result.yaw), math.sin(tf_result.yaw)
                        position = ObjectPosition(
                            x=tf_result.x + x_local * cos_yaw - y_local * sin_yaw,
                            y=tf_result.y + x_local * sin_yaw + y_local * cos_yaw,
                            frame="map",
                        )
                    else:
                        position_error = "TF_UNAVAILABLE"
                else:
                    position = ObjectPosition(x=x_local, y=y_local, frame=self._base_frame)

            objects.append(
                DetectedObject(
                    label=det.label,
                    confidence=det.confidence,
                    distance_m=distance_m,
                    distance_source=distance_source,  # type: ignore[arg-type]
                    position=position,
                    position_error=position_error,
                    bbox_px=det.bbox_px,
                )
            )

        return DetectObjectsResult(
            status="succeeded",
            command_id="",
            robot_id=self._robot_id,
            duration_sec=0.0,
            stamp=cached_image.stamp,
            objects=tuple(objects),
            detector_plugin_id=self._detector_plugin.metadata.plugin_id,
        )

    def _current_laser_scan_analysis(self) -> Any:
        scan_cap = self._registry.get("range_sensing")
        if scan_cap is None or "scan" not in scan_cap.resolved_topics:
            return None
        cached = self._subscriptions.get_latest(scan_cap.resolved_topics["scan"])
        if cached is None:
            return None
        return analyze_laser_scan(cached.raw)

    def _detect_error(self, code: ErrorCode, message: str) -> DetectObjectsResult:
        result = build_error_result(
            DetectObjectsResult,
            command_id="",
            robot_id=self._robot_id,
            duration_sec=0.0,
            error=ToolError(code=code, message=message),
        )
        assert isinstance(result, DetectObjectsResult)
        return result

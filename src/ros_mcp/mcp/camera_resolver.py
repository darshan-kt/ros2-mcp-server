"""Builds CameraImageResult from the CapabilityRegistry + SubscriptionManager
(docs/04-mcp-surface.md robot.get_camera_image)."""
from __future__ import annotations

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.results import CameraImageResult
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.execution.error_results import build_error_result
from ros_mcp.perception.camera import encode_jpeg_thumbnail


def resolve_camera_image(
    *,
    robot_id: str,
    command_id: str,
    duration_sec: float,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    max_width_px: int,
    max_width_px_cap: int,
) -> CameraImageResult:
    cap = registry.get("visual_observation")
    if cap is None or "image" not in cap.resolved_topics:
        result = build_error_result(
            CameraImageResult,
            command_id=command_id,
            robot_id=robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.CAPABILITY_UNAVAILABLE, message="no camera available"),
        )
        assert isinstance(result, CameraImageResult)
        return result

    topic = cap.resolved_topics["image"]
    cached = subscriptions.get_latest(topic)
    if cached is None:
        result = build_error_result(
            CameraImageResult,
            command_id=command_id,
            robot_id=robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.SENSOR_STALE, message=f"no camera image ever received on {topic}"),
        )
        assert isinstance(result, CameraImageResult)
        return result

    effective_max_width = min(max_width_px, max_width_px_cap)
    is_compressed = "Compressed" in type(cached.raw).__name__
    try:
        jpeg_bytes, width_px, height_px, original_width, original_height = encode_jpeg_thumbnail(
            cached.raw, max_width_px=effective_max_width, is_compressed=is_compressed
        )
    except ValueError as exc:
        result = build_error_result(
            CameraImageResult,
            command_id=command_id,
            robot_id=robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.ROS_INTERFACE_ERROR, message=str(exc)),
        )
        assert isinstance(result, CameraImageResult)
        return result

    frame_id = getattr(getattr(cached.raw, "header", None), "frame_id", None)

    return CameraImageResult(
        status="succeeded",
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        stamp=cached.stamp,
        frame=frame_id,
        data_age_s=cached.age_s,
        image_jpeg_bytes=jpeg_bytes,
        width_px=width_px,
        height_px=height_px,
        original_width_px=original_width,
        original_height_px=original_height,
    )

"""Builds a `command.expected_result_type` instance for failures that occur before an
adapter is ever dispatched (safety denial, capability unavailable, internal error) — the
Execution Manager must still return the *correctly typed* result for whichever tool was
called, not a generic envelope, so the MCP layer never has to special-case a mismatched
type (docs/13-contracts.md §1 `expected_result_type` rationale)."""
from __future__ import annotations

from typing import Any, Callable, Literal

from ros_mcp.contracts.errors import ToolError, ToolResult
from ros_mcp.contracts.results import (
    CameraImageResult,
    CapabilitiesResult,
    DetectObjectsResult,
    LaserScanResult,
    MotionResult,
    NavigateResult,
    RobotStateResult,
    StopResult,
)

_EXTRA_FIELDS_BY_TYPE: dict[type[ToolResult], Callable[[], dict[str, Any]]] = {
    RobotStateResult: lambda: {
        "pose": None,
        "linear_velocity_mps": None,
        "angular_velocity_rps": None,
        "is_moving": False,
        "active_command": None,
    },
    CapabilitiesResult: lambda: {"capabilities": (), "snapshot_version": 0},
    MotionResult: lambda: {"distance_traveled_m": 0.0, "final_pose": None, "stop_reason": None},
    StopResult: lambda: {"cancelled_command_id": None, "final_pose": None},
    NavigateResult: lambda: {
        "final_pose": None,
        "distance_remaining_m": None,
        "fail_reason": None,
    },
    LaserScanResult: lambda: {
        "stamp": None,
        "frame": None,
        "nearest": None,
        "farthest": None,
        "sectors": {},
        "raw_ranges": None,
        "data_age_s": None,
        "stale": True,
        "invalid_beam_count": 0,
    },
    CameraImageResult: lambda: {
        "stamp": None,
        "frame": None,
        "data_age_s": None,
        "image_jpeg_bytes": b"",
        "width_px": 0,
        "height_px": 0,
        "original_width_px": 0,
        "original_height_px": 0,
    },
    DetectObjectsResult: lambda: {"stamp": None, "objects": (), "detector_plugin_id": ""},
}


def build_result_with_defaults(
    expected_result_type: type[ToolResult],
    *,
    status: Literal["succeeded", "failed", "awaiting_approval"],
    command_id: str,
    robot_id: str,
    duration_sec: float,
    error: ToolError | None,
) -> ToolResult:
    """Builds a `expected_result_type` instance with placeholder/failure defaults for
    every subtype-specific field, used whenever a result must be returned before an
    adapter ever ran (safety denial, capability unavailable, awaiting approval,
    internal error)."""
    extra_fn = _EXTRA_FIELDS_BY_TYPE.get(expected_result_type)
    extra = extra_fn() if extra_fn is not None else {}
    return expected_result_type(
        status=status,
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        error=error,
        **extra,
    )


def build_error_result(
    expected_result_type: type[ToolResult],
    *,
    command_id: str,
    robot_id: str,
    duration_sec: float,
    error: ToolError,
) -> ToolResult:
    return build_result_with_defaults(
        expected_result_type,
        status="failed",
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        error=error,
    )

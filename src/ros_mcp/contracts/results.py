"""Concrete ToolResult subtypes — one per MVP tool, per docs/13-contracts.md §2 note:

    "Full result models ... are dataclasses subclassing ToolResult, field-shaped exactly
    as the JSON examples in 04-mcp-surface.md. They live in ros_mcp.contracts.results."

Field shapes follow the JSON examples in docs/04-mcp-surface.md verbatim. Small
supporting dataclasses (CommandSummary, RangeReading, DetectedObject, ObjectPosition)
are named in that document's prose but not given field-level schemas there; they are
defined here to fulfil the documented result shape without altering any frozen
Protocol/dataclass in 13-contracts.md.

Every subclass fixes the ToolResult base fields as required-positional (inherited) and
adds its own fields as keyword-only (`kw_only=True`) — this is purely a dataclass field-
ordering mechanism (Python 3.10 PEP 681 kw_only), not a change to any frozen field name,
type, or the base ToolResult shape itself.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Literal

from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.core import ExecutionState, Operation, Pose2D
from ros_mcp.contracts.errors import ToolResult


@dataclass(frozen=True)
class CommandSummary:
    command_id: str
    operation: Operation
    execution_state: ExecutionState


@dataclass(frozen=True, kw_only=True)
class RobotStateResult(ToolResult):
    pose: Pose2D | None
    linear_velocity_mps: float | None
    angular_velocity_rps: float | None
    is_moving: bool
    active_command: CommandSummary | None


@dataclass(frozen=True, kw_only=True)
class CapabilitiesResult(ToolResult):
    capabilities: tuple[CapabilityEntry, ...]
    snapshot_version: int


StopReason = Literal["target_reached", "obstacle", "timeout", "cancelled", "safety_stop"]


@dataclass(frozen=True, kw_only=True)
class MotionResult(ToolResult):
    distance_traveled_m: float
    final_pose: Pose2D | None
    stop_reason: StopReason | None


@dataclass(frozen=True, kw_only=True)
class StopResult(ToolResult):
    cancelled_command_id: str | None
    final_pose: Pose2D | None


FailReason = Literal[
    "goal_rejected",
    "planner_failure",
    "controller_failure",
    "obstacle_timeout",
    "cancelled",
    "localization_lost",
]


@dataclass(frozen=True, kw_only=True)
class NavigateResult(ToolResult):
    final_pose: Pose2D | None
    distance_remaining_m: float | None
    fail_reason: FailReason | None


@dataclass(frozen=True)
class RangeReading:
    range_m: float
    angle_rad: float
    sector: str


@dataclass(frozen=True, kw_only=True)
class LaserScanResult(ToolResult):
    stamp: datetime | None
    frame: str | None
    nearest: RangeReading | None
    farthest: RangeReading | None
    sectors: dict[str, RangeReading | None]
    raw_ranges: tuple[float, ...] | None
    data_age_s: float | None
    stale: bool
    invalid_beam_count: int


@dataclass(frozen=True, kw_only=True)
class CameraImageResult(ToolResult):
    stamp: datetime | None
    frame: str | None
    data_age_s: float | None
    image_jpeg_bytes: bytes
    width_px: int
    height_px: int
    original_width_px: int
    original_height_px: int


@dataclass(frozen=True)
class ObjectPosition:
    x: float
    y: float
    frame: str


DistanceSource = Literal["laser_scan_correlation", "depth_image", "unavailable"]


@dataclass(frozen=True)
class DetectedObject:
    label: str
    confidence: float
    distance_m: float | None
    distance_source: DistanceSource
    position: ObjectPosition | None
    position_error: str | None
    bbox_px: tuple[int, int, int, int]


@dataclass(frozen=True, kw_only=True)
class DetectObjectsResult(ToolResult):
    stamp: datetime | None
    objects: tuple[DetectedObject, ...]
    detector_plugin_id: str

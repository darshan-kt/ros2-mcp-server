"""Adapter (backend) contracts — frozen by docs/13-contracts.md §7."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Protocol, runtime_checkable

from ros_mcp.contracts.core import SemanticCommand
from ros_mcp.contracts.plugins import PluginHealth, PluginMetadata
from ros_mcp.contracts.results import CameraImageResult, DetectObjectsResult, LaserScanResult
from ros_mcp.contracts.results import MotionResult, NavigateResult, StopResult


@dataclass(frozen=True)
class RawImage:
    """Concrete shape for the `image` parameter ObjectDetectorPlugin.detect() forward-
    references as "RawImage" in 13-contracts.md §7 — not itself redefined there, so this
    is the supplying type, not a signature change."""

    width_px: int
    height_px: int
    encoding: str  # e.g. "rgb8", "bgr8", "mono8" (sensor_msgs/Image encoding string)
    data: bytes
    frame_id: str
    stamp: datetime


@runtime_checkable
class CancellationToken(Protocol):
    def is_cancelled(self) -> bool: ...
    def deadline_exceeded(self) -> bool: ...


class MotionBackend(Protocol):
    """Implemented by CmdVelMotionPlugin (MVP)."""

    async def move(self, command: SemanticCommand, token: CancellationToken) -> MotionResult:
        """MUST publish an explicit zero-velocity command on every exit path
        (07-motion-architecture.md). MUST poll `token` at <= one control-loop period."""
        ...

    async def stop(self) -> StopResult:
        """Immediate: publishes zero velocity, does not itself run a control loop."""
        ...


class NavigationBackend(Protocol):
    """Implemented by Nav2NavigationPlugin (MVP)."""

    async def navigate(self, command: SemanticCommand, token: CancellationToken) -> NavigateResult: ...

    async def cancel_all(self) -> None:
        """Cancels any outstanding Nav2 goal; called unconditionally by robot.stop's
        planner fan-out (06-execution.md)."""
        ...

    def is_available(self) -> bool:
        """Cheap, synchronous, backed by the current CapabilityRegistry — used by the
        Planner before deciding to route NAVIGATE here."""
        ...


# The set of backend Protocols the CommandPlanner may resolve an operation to
# (13-contracts.md §6 `CommandPlanner.select_backend -> "Backend | None"`).
Backend = MotionBackend | NavigationBackend


class PerceptionAdapter(Protocol):
    """MUST NOT depend on ExecutionManager, CommandPlanner, or SemanticCommandFactory —
    perception produces ToolResult data only, never commands (10-safety-and-trust.md
    trust-boundary rule)."""

    async def get_laser_scan_summary(self, include_raw: bool) -> LaserScanResult: ...

    async def get_camera_image(self, max_width_px: int) -> CameraImageResult: ...

    async def detect_objects(self, labels: tuple[str, ...] | None) -> DetectObjectsResult: ...


@dataclass(frozen=True)
class DetectedObject2D:
    label: str
    confidence: float
    bbox_px: tuple[int, int, int, int]
    text_content: str | None = None


class ObjectDetectorPlugin(Protocol):
    """Perception plugin seam (09-perception-architecture.md, 12-plugin-architecture.md)."""

    metadata: PluginMetadata

    async def detect(self, image: RawImage) -> tuple[DetectedObject2D, ...]: ...

    async def health_check(self) -> PluginHealth: ...

"""Result / error model — frozen by docs/13-contracts.md §2."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Literal


class ErrorCode(str, Enum):
    INVALID_ARGUMENT = "INVALID_ARGUMENT"
    INVALID_FRAME = "INVALID_FRAME"
    TF_UNAVAILABLE = "TF_UNAVAILABLE"
    CAPABILITY_UNAVAILABLE = "CAPABILITY_UNAVAILABLE"
    NAVIGATION_FAILED = "NAVIGATION_FAILED"
    ACTION_TIMEOUT = "ACTION_TIMEOUT"
    SAFETY_REJECTED = "SAFETY_REJECTED"
    ROBOT_NOT_READY = "ROBOT_NOT_READY"
    SENSOR_STALE = "SENSOR_STALE"
    PERMISSION_DENIED = "PERMISSION_DENIED"
    ROS_INTERFACE_ERROR = "ROS_INTERFACE_ERROR"
    CANCELLED = "CANCELLED"
    INTERNAL_ERROR = "INTERNAL_ERROR"


@dataclass(frozen=True)
class ToolError:
    code: ErrorCode
    message: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ToolResult:
    """Base envelope. Every MCP tool handler returns a subclass of this, serialized to
    the MCP tool result content — never a bare string."""

    status: Literal["succeeded", "failed", "awaiting_approval"]
    command_id: str
    robot_id: str
    duration_sec: float
    error: ToolError | None = None

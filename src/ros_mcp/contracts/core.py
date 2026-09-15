"""Core data models — frozen by docs/13-contracts.md §1.

Every symbol in this module is a verbatim implementation of a signature in
13-contracts.md. Do not add fields, rename fields, or change types here without a new
ADR amending that document.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Literal

from ros_mcp.contracts.errors import ToolResult


class Confidence(str, Enum):
    CONFIRMED = "confirmed"
    LIKELY = "likely"
    AMBIGUOUS = "ambiguous"


class Operation(str, Enum):
    MOVE = "move"
    STOP = "stop"
    NAVIGATE = "navigate"
    GET_STATE = "get_state"
    GET_CAPABILITIES = "get_capabilities"
    GET_LASER_SCAN = "get_laser_scan"
    GET_CAMERA_IMAGE = "get_camera_image"
    DETECT_OBJECTS = "detect_objects"
    RAW_READ_TOPIC = "raw_read_topic"
    RAW_PUBLISH = "raw_publish"
    RAW_CALL_SERVICE = "raw_call_service"
    RAW_SEND_ACTION_GOAL = "raw_send_action_goal"
    RAW_GET_PARAMETER = "raw_get_parameter"
    RAW_SET_PARAMETER = "raw_set_parameter"


class CommandClass(str, Enum):
    READ = "read"
    LOW_RISK = "low_risk"
    MOTION = "motion"
    HIGH_RISK = "high_risk"


class Priority(str, Enum):
    NORMAL = "normal"
    SAFETY = "safety"


class ExecutionState(str, Enum):
    RECEIVED = "received"
    VALIDATING = "validating"
    REJECTED = "rejected"
    SAFETY_CHECK = "safety_check"
    SAFETY_REJECTED = "safety_rejected"
    AWAITING_APPROVAL = "awaiting_approval"
    PLANNING = "planning"
    CAPABILITY_UNAVAILABLE = "capability_unavailable"
    EXECUTING = "executing"
    MONITORING = "monitoring"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"
    SAFETY_STOP = "safety_stop"
    CANCELLED = "cancelled"


# Terminal states: once reached, an ExecutionManager must never transition further.
TERMINAL_STATES: frozenset[ExecutionState] = frozenset(
    {
        ExecutionState.REJECTED,
        ExecutionState.SAFETY_REJECTED,
        ExecutionState.CAPABILITY_UNAVAILABLE,
        ExecutionState.SUCCEEDED,
        ExecutionState.FAILED,
        ExecutionState.TIMEOUT,
        ExecutionState.SAFETY_STOP,
        ExecutionState.CANCELLED,
    }
)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame: str
    stamp: datetime


@dataclass(frozen=True)
class Provenance:
    source: Literal["mcp_tool_call"]
    tool_name: str
    raw_arguments: dict[str, Any]
    client_session_id: str


@dataclass(frozen=True)
class SafetyConstraints:
    max_linear_mps: float
    max_angular_rps: float
    max_linear_acceleration_mps2: float
    max_angular_acceleration_rps2: float
    max_move_distance_m: float
    max_navigation_distance_m: float
    obstacle_stop_distance_m: float


@dataclass(frozen=True)
class MoveTarget:
    direction: Literal["forward", "backward", "left", "right", "rotate_left", "rotate_right"]
    distance_m: float | None
    angle_deg: float | None


@dataclass(frozen=True)
class NavigateTarget:
    x: float
    y: float
    yaw: float | None


Target = MoveTarget | NavigateTarget | None


@dataclass  # mutable: execution_state is updated in place by ExecutionManager only
class SemanticCommand:
    command_id: str
    session_id: str
    robot_id: str
    operation: Operation
    command_class: CommandClass
    target: Target
    frame: str | None
    timeout_s: float
    priority: Priority
    safety_constraints: SafetyConstraints | None
    provenance: Provenance
    confidence: float | None
    execution_state: ExecutionState
    created_at: datetime
    expected_result_type: type[ToolResult]

"""SemanticCommandFactory — the single place raw MCP tool arguments become a
SemanticCommand (docs/05-semantic-command-model.md, Non-Bypass Rule in
docs/13-contracts.md §5). Not itself a frozen Protocol/dataclass, but the concrete
realization the Non-Bypass Rule names as the only route from arguments to a command.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from ros_mcp.commands.ulid import new_ulid
from ros_mcp.contracts.config import SafetyConfig, TimeoutConfig
from ros_mcp.contracts.core import (
    CommandClass,
    ExecutionState,
    MoveTarget,
    NavigateTarget,
    Operation,
    Priority,
    Provenance,
    SafetyConstraints,
    SemanticCommand,
    Target,
)
from ros_mcp.contracts.errors import ToolResult
from ros_mcp.contracts.results import (
    CapabilitiesResult,
    CameraImageResult,
    DetectObjectsResult,
    LaserScanResult,
    MotionResult,
    NavigateResult,
    RobotStateResult,
    StopResult,
)

_READ_DEFAULT_TIMEOUT_S = 5.0

_RESULT_TYPE_BY_OPERATION: dict[Operation, type[ToolResult]] = {
    Operation.GET_STATE: RobotStateResult,
    Operation.GET_CAPABILITIES: CapabilitiesResult,
    Operation.MOVE: MotionResult,
    Operation.STOP: StopResult,
    Operation.NAVIGATE: NavigateResult,
    Operation.GET_LASER_SCAN: LaserScanResult,
    Operation.GET_CAMERA_IMAGE: CameraImageResult,
    Operation.DETECT_OBJECTS: DetectObjectsResult,
}

_MOTION_OPERATIONS = frozenset({Operation.MOVE, Operation.STOP, Operation.NAVIGATE})


def _build_target(operation: Operation, arguments: dict[str, Any]) -> Target:
    if operation is Operation.MOVE:
        return MoveTarget(
            direction=arguments["direction"],
            distance_m=arguments.get("distance_m"),
            angle_deg=arguments.get("angle_deg"),
        )
    if operation is Operation.NAVIGATE:
        return NavigateTarget(
            x=arguments["x"], y=arguments["y"], yaw=arguments.get("yaw")
        )
    return None


def _resolve_timeout_s(
    operation: Operation, arguments: dict[str, Any], timeouts: TimeoutConfig
) -> float:
    if operation is Operation.MOVE:
        requested = arguments.get("timeout_s")
        if requested is not None:
            return min(float(requested), timeouts.move_max_s)
        return timeouts.move_default_s
    if operation is Operation.NAVIGATE:
        return timeouts.navigate_default_s
    if operation is Operation.STOP:
        return 5.0
    return _READ_DEFAULT_TIMEOUT_S


def _resolve_safety_constraints(safety: SafetyConfig) -> SafetyConstraints:
    return SafetyConstraints(
        max_linear_mps=safety.max_linear_mps,
        max_angular_rps=safety.max_angular_rps,
        max_linear_acceleration_mps2=safety.max_linear_acceleration_mps2,
        max_angular_acceleration_rps2=safety.max_angular_acceleration_rps2,
        max_move_distance_m=safety.max_move_distance_m,
        max_navigation_distance_m=safety.max_navigation_distance_m,
        obstacle_stop_distance_m=safety.obstacle_stop_distance_m,
    )


class SemanticCommandFactory:
    def __init__(self, *, robot_id: str) -> None:
        self._robot_id = robot_id

    def create(
        self,
        *,
        tool_name: str,
        operation: Operation,
        command_class: CommandClass,
        arguments: dict[str, Any],
        session_id: str,
        safety_config: SafetyConfig,
        timeout_config: TimeoutConfig,
        confidence: float | None = None,
    ) -> SemanticCommand:
        target = _build_target(operation, arguments)
        frame = arguments.get("frame") if operation is Operation.NAVIGATE else None
        priority = Priority.SAFETY if operation is Operation.STOP else Priority.NORMAL
        safety_constraints = (
            _resolve_safety_constraints(safety_config)
            if operation in _MOTION_OPERATIONS
            else None
        )
        return SemanticCommand(
            command_id=new_ulid(),
            session_id=session_id,
            robot_id=self._robot_id,
            operation=operation,
            command_class=command_class,
            target=target,
            frame=frame,
            timeout_s=_resolve_timeout_s(operation, arguments, timeout_config),
            priority=priority,
            safety_constraints=safety_constraints,
            provenance=Provenance(
                source="mcp_tool_call",
                tool_name=tool_name,
                raw_arguments=dict(arguments),
                client_session_id=session_id,
            ),
            confidence=confidence,
            execution_state=ExecutionState.RECEIVED,
            created_at=datetime.now(timezone.utc),
            expected_result_type=_RESULT_TYPE_BY_OPERATION[operation],
        )

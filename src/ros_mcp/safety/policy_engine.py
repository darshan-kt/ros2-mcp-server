"""DefaultSafetyPolicyEngine — structurally satisfies
ros_mcp.contracts.safety.SafetyPolicyEngine (docs/13-contracts.md §5,
docs/10-safety-and-trust.md Safety & Policy Engine Checks).

Checks run in the documented order; the first failure short-circuits with a specific
ErrorCode. `robot.stop` (Operation.STOP) always ALLOWs immediately — "stop is never
itself rejected by policy" (04-mcp-surface.md).
"""
from __future__ import annotations

import time
from collections import defaultdict
from typing import Any

from ros_mcp.contracts.config import SafetyConfig
from ros_mcp.contracts.core import MoveTarget, NavigateTarget, Operation, Pose2D, SemanticCommand
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.safety import SafetyDecision, SafetyOutcome
from ros_mcp.geometry import euclidean_distance

_ROTATION_DIRECTIONS = frozenset({"rotate_left", "rotate_right"})
_TRANSLATION_DELTA_XY: dict[str, tuple[float, float]] = {
    # (forward, lateral) displacement in the robot's own frame, rotated by current yaw
    # before being added to the current pose.
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}


def _resolve_constraints(safety: SafetyConfig) -> Any:
    from ros_mcp.contracts.core import SafetyConstraints

    return SafetyConstraints(
        max_linear_mps=safety.max_linear_mps,
        max_angular_rps=safety.max_angular_rps,
        max_linear_acceleration_mps2=safety.max_linear_acceleration_mps2,
        max_angular_acceleration_rps2=safety.max_angular_acceleration_rps2,
        max_move_distance_m=safety.max_move_distance_m,
        max_navigation_distance_m=safety.max_navigation_distance_m,
        obstacle_stop_distance_m=safety.obstacle_stop_distance_m,
    )


def _predicted_move_endpoint(target: MoveTarget, current_pose: Pose2D) -> tuple[float, float]:
    if target.direction in _ROTATION_DIRECTIONS or target.distance_m is None:
        return current_pose.x, current_pose.y
    import math

    forward, lateral = _TRANSLATION_DELTA_XY[target.direction]
    dx_local = forward * target.distance_m
    dy_local = lateral * target.distance_m
    yaw = current_pose.yaw
    dx_world = dx_local * math.cos(yaw) - dy_local * math.sin(yaw)
    dy_world = dx_local * math.sin(yaw) + dy_local * math.cos(yaw)
    return current_pose.x + dx_world, current_pose.y + dy_world


def _within_geofence(x: float, y: float, safety: SafetyConfig) -> bool:
    fence = safety.geofence
    return fence.min_x <= x <= fence.max_x and fence.min_y <= y <= fence.max_y


class DefaultSafetyPolicyEngine:
    """Structurally satisfies ros_mcp.contracts.safety.SafetyPolicyEngine."""

    def __init__(self, safety_config_provider: Any) -> None:
        """`safety_config_provider` is a zero-arg callable returning the current
        SafetyConfig (typically ConfigProvider.safety) — read fresh on every check so a
        config reload takes effect for the *next* command without retroactively
        changing one already resolved (16-configuration.md, ADR-013)."""
        self._safety_config_provider = safety_config_provider
        self._rate_limit_history: dict[str, list[float]] = defaultdict(list)

    async def check(
        self, command: SemanticCommand, current_pose: Pose2D | None
    ) -> SafetyOutcome:
        safety = self._safety_config_provider()
        constraints = _resolve_constraints(safety)

        if command.operation is Operation.STOP:
            return SafetyOutcome(
                decision=SafetyDecision.ALLOW,
                error=None,
                approval_reason=None,
                resolved_constraints=constraints,
            )

        # 1. Rate limit.
        rate_error = self._check_rate_limit(command.session_id, safety.rate_limit_rps)
        if rate_error is not None:
            return SafetyOutcome(
                decision=SafetyDecision.DENY,
                error=rate_error,
                approval_reason=None,
                resolved_constraints=constraints,
            )

        distance_m: float | None = None
        predicted_xy: tuple[float, float] | None = None

        if command.operation is Operation.MOVE:
            assert isinstance(command.target, MoveTarget)
            if command.target.direction not in _ROTATION_DIRECTIONS:
                distance_m = command.target.distance_m
                # 3. Distance limit.
                if distance_m is not None and distance_m > safety.max_move_distance_m:
                    return SafetyOutcome(
                        decision=SafetyDecision.DENY,
                        error=ToolError(
                            code=ErrorCode.SAFETY_REJECTED,
                            message=(
                                f"requested move distance {distance_m}m exceeds the "
                                f"configured limit of {safety.max_move_distance_m}m"
                            ),
                            details={
                                "requested_distance_m": distance_m,
                                "max_move_distance_m": safety.max_move_distance_m,
                            },
                        ),
                        approval_reason=None,
                        resolved_constraints=constraints,
                    )
            if current_pose is not None:
                predicted_xy = _predicted_move_endpoint(command.target, current_pose)

        elif command.operation is Operation.NAVIGATE:
            assert isinstance(command.target, NavigateTarget)
            if current_pose is None:
                return SafetyOutcome(
                    decision=SafetyDecision.DENY,
                    error=ToolError(
                        code=ErrorCode.ROBOT_NOT_READY,
                        message="cannot validate navigation distance without a known current pose",
                    ),
                    approval_reason=None,
                    resolved_constraints=constraints,
                )
            distance_m = euclidean_distance(
                current_pose.x, current_pose.y, command.target.x, command.target.y
            )
            predicted_xy = (command.target.x, command.target.y)
            # 3. Distance limit.
            if distance_m > safety.max_navigation_distance_m:
                return SafetyOutcome(
                    decision=SafetyDecision.DENY,
                    error=ToolError(
                        code=ErrorCode.SAFETY_REJECTED,
                        message=(
                            f"requested navigation distance {distance_m:.2f}m exceeds "
                            f"the configured limit of {safety.max_navigation_distance_m}m"
                        ),
                        details={
                            "requested_distance_m": distance_m,
                            "max_navigation_distance_m": safety.max_navigation_distance_m,
                        },
                    ),
                    approval_reason=None,
                    resolved_constraints=constraints,
                )

        # 4. Geofence.
        if safety.geofence.enabled and predicted_xy is not None:
            if not _within_geofence(predicted_xy[0], predicted_xy[1], safety):
                return SafetyOutcome(
                    decision=SafetyDecision.DENY,
                    error=ToolError(
                        code=ErrorCode.SAFETY_REJECTED,
                        message="predicted position lies outside the configured geofence",
                        details={"predicted_x": predicted_xy[0], "predicted_y": predicted_xy[1]},
                    ),
                    approval_reason=None,
                    resolved_constraints=constraints,
                )

        # 6. Human-in-the-loop policy.
        approval_reason = self._check_hitl(safety, distance_m)
        if approval_reason is not None:
            return SafetyOutcome(
                decision=SafetyDecision.REQUIRE_APPROVAL,
                error=None,
                approval_reason=approval_reason,
                resolved_constraints=constraints,
            )

        return SafetyOutcome(
            decision=SafetyDecision.ALLOW,
            error=None,
            approval_reason=None,
            resolved_constraints=constraints,
        )

    def _check_rate_limit(self, session_id: str, rate_limit_rps: float) -> ToolError | None:
        now = time.monotonic()
        window_start = now - 1.0
        history = self._rate_limit_history[session_id]
        history[:] = [t for t in history if t >= window_start]
        if len(history) >= rate_limit_rps:
            return ToolError(
                code=ErrorCode.PERMISSION_DENIED,
                message=f"rate limit exceeded ({rate_limit_rps} commands/sec)",
                details={"session_id": session_id},
            )
        history.append(now)
        return None

    def _check_hitl(self, safety: SafetyConfig, distance_m: float | None) -> str | None:
        if safety.approval_policy == "automatic":
            return None
        if safety.approval_policy == "always_human":
            return "human approval is required for all motion commands (approval_policy=always_human)"
        if safety.approval_policy == "policy_based" and distance_m is not None:
            threshold = safety.human_approval.required_above_distance_m
            if distance_m > threshold:
                return (
                    f"this command will move the robot approximately {distance_m:.2f}m; "
                    f"the configured limit for automatic approval is {threshold}m"
                )
        return None

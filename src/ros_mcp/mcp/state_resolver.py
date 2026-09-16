"""Builds RobotStateResult from the CapabilityRegistry + SubscriptionManager last-value
cache (docs/04-mcp-surface.md robot.get_state)."""
from __future__ import annotations

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Pose2D
from ros_mcp.contracts.results import CommandSummary, RobotStateResult
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.robot_state import resolve_current_pose


async def resolve_robot_state(
    *,
    robot_id: str,
    command_id: str,
    duration_sec: float,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    requested_frame: str | None,
    active_command: CommandSummary | None,
    tf_adapter: TFAdapter | None = None,
    base_frame: str | None = None,
) -> RobotStateResult:
    motion_cap = registry.get("differential_drive_motion")

    pose: Pose2D | None = await resolve_current_pose(
        registry=registry,
        subscriptions=subscriptions,
        requested_frame=requested_frame,
        tf_adapter=tf_adapter,
        base_frame=base_frame,
    )
    linear_velocity: float | None = None
    angular_velocity: float | None = None
    is_moving = False

    if motion_cap is not None and "odom" in motion_cap.resolved_topics:
        cached = subscriptions.get_latest(motion_cap.resolved_topics["odom"])
        if cached is not None:
            linear_velocity = float(cached.raw.twist.twist.linear.x)
            angular_velocity = float(cached.raw.twist.twist.angular.z)
            is_moving = abs(linear_velocity) > 1e-3 or abs(angular_velocity) > 1e-3

    return RobotStateResult(
        status="succeeded",
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        pose=pose,
        linear_velocity_mps=linear_velocity,
        angular_velocity_rps=angular_velocity,
        is_moving=is_moving,
        active_command=active_command,
    )

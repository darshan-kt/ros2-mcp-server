"""Builds RobotStateResult from the CapabilityRegistry + SubscriptionManager last-value
cache (docs/04-mcp-surface.md robot.get_state). Prefers a localization-derived (map
frame) pose when available and requested, falling back to raw odometry (odom frame)."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Pose2D
from ros_mcp.contracts.results import CommandSummary, RobotStateResult
from ros_mcp.geometry import Quaternion, quaternion_to_yaw
from ros_mcp.contracts.subscriptions import SubscriptionManager


def _pose_from_odometry(raw_odom: Any, *, frame: str, stamp: datetime) -> Pose2D:
    pos = raw_odom.pose.pose.position
    orientation = raw_odom.pose.pose.orientation
    yaw = quaternion_to_yaw(
        Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
    )
    return Pose2D(x=pos.x, y=pos.y, yaw=yaw, frame=frame, stamp=stamp)


def _pose_from_pose_with_covariance_stamped(raw_msg: Any, *, frame: str, stamp: datetime) -> Pose2D:
    pos = raw_msg.pose.pose.position
    orientation = raw_msg.pose.pose.orientation
    yaw = quaternion_to_yaw(
        Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
    )
    return Pose2D(x=pos.x, y=pos.y, yaw=yaw, frame=frame, stamp=stamp)


def resolve_robot_state(
    *,
    robot_id: str,
    command_id: str,
    duration_sec: float,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    requested_frame: str | None,
    active_command: CommandSummary | None,
) -> RobotStateResult:
    motion_cap = registry.get("differential_drive_motion")
    localization_cap = registry.get("localization")

    pose: Pose2D | None = None
    linear_velocity: float | None = None
    angular_velocity: float | None = None
    is_moving = False

    want_map_frame = requested_frame in (None, "map")
    if (
        want_map_frame
        and localization_cap is not None
        and "amcl_pose" in localization_cap.resolved_topics
    ):
        cached = subscriptions.get_latest(localization_cap.resolved_topics["amcl_pose"])
        if cached is not None:
            pose = _pose_from_pose_with_covariance_stamped(cached.raw, frame="map", stamp=cached.stamp)

    if motion_cap is not None and "odom" in motion_cap.resolved_topics:
        cached = subscriptions.get_latest(motion_cap.resolved_topics["odom"])
        if cached is not None:
            linear_velocity = float(cached.raw.twist.twist.linear.x)
            angular_velocity = float(cached.raw.twist.twist.angular.z)
            is_moving = abs(linear_velocity) > 1e-3 or abs(angular_velocity) > 1e-3
            if pose is None:
                pose = _pose_from_odometry(
                    cached.raw, frame=requested_frame or "odom", stamp=cached.stamp
                )

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

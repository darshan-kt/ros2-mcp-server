"""Shared "what is the robot's current pose right now" resolution, used both by
robot.get_state's result assembly (ros_mcp.mcp.state_resolver) and by the Execution
Manager's SafetyPolicyEngine.check() current_pose argument (docs/13-contracts.md §5) —
one implementation, not two, so the safety check and the reported state can never
disagree about what "current pose" means.
"""
from __future__ import annotations

from datetime import datetime
from typing import Any

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Pose2D
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.geometry import Quaternion, quaternion_to_yaw


def pose_from_odometry(raw_odom: Any, *, frame: str, stamp: datetime) -> Pose2D:
    pos = raw_odom.pose.pose.position
    orientation = raw_odom.pose.pose.orientation
    yaw = quaternion_to_yaw(
        Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
    )
    return Pose2D(x=pos.x, y=pos.y, yaw=yaw, frame=frame, stamp=stamp)


def pose_from_pose_with_covariance_stamped(raw_msg: Any, *, frame: str, stamp: datetime) -> Pose2D:
    pos = raw_msg.pose.pose.position
    orientation = raw_msg.pose.pose.orientation
    yaw = quaternion_to_yaw(
        Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
    )
    return Pose2D(x=pos.x, y=pos.y, yaw=yaw, frame=frame, stamp=stamp)


def resolve_current_pose(
    *,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    requested_frame: str | None = None,
) -> Pose2D | None:
    """Prefers a localization-derived (map frame) pose when localized and a map-frame
    pose was requested (or no frame was specified); falls back to raw odometry."""
    motion_cap = registry.get("differential_drive_motion")
    localization_cap = registry.get("localization")

    want_map_frame = requested_frame in (None, "map")
    if (
        want_map_frame
        and localization_cap is not None
        and "amcl_pose" in localization_cap.resolved_topics
    ):
        cached = subscriptions.get_latest(localization_cap.resolved_topics["amcl_pose"])
        if cached is not None:
            return pose_from_pose_with_covariance_stamped(cached.raw, frame="map", stamp=cached.stamp)

    if motion_cap is not None and "odom" in motion_cap.resolved_topics:
        cached = subscriptions.get_latest(motion_cap.resolved_topics["odom"])
        if cached is not None:
            return pose_from_odometry(cached.raw, frame=requested_frame or "odom", stamp=cached.stamp)

    return None

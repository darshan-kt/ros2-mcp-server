"""Shared "what is the robot's current pose right now" resolution, used both by
robot.get_state's result assembly (ros_mcp.mcp.state_resolver) and by the Execution
Manager's SafetyPolicyEngine.check() current_pose argument (docs/13-contracts.md §5) —
one implementation, not two, so the safety check and the reported state can never
disagree about what "current pose" means.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Pose2D
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.geometry import Quaternion, quaternion_to_yaw

# Sentinel meaning "the latest available transform" (mirrors
# ros_mcp.adapters.perception.tf_adapter.LATEST_TRANSFORM_STAMP) — kept here too since
# this module must stay adapter-agnostic (depends only on the TFAdapter Protocol, never
# on a concrete adapter class).
_LATEST_TRANSFORM_STAMP = datetime.fromtimestamp(0, tz=timezone.utc)

# Bug fix (see docs/../acceptance/RESULTS.md Finding 1): a snapshot-gated capability
# check ("has discovery confirmed 'localization' yet?") can lag the live TF tree by up
# to one discovery poll_interval_s. get_state's pose used to be entirely gated behind
# that lagging capability check via the cached /amcl_pose subscription below, so it
# could report pose=None for several seconds (or indefinitely, if the amcl_pose
# subscription itself never received a first message in time) even while `map` was
# already live-resolvable in TF. The live TF lookup below is the same fallback Finding 1
# recommended for robot.navigate's frame check, applied here to pose resolution too, so
# get_state (and the safety engine, which shares this function) don't share that gap.
_DEFAULT_TF_TIMEOUT_S = 0.5


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


async def _pose_from_live_tf(
    *, tf_adapter: TFAdapter, base_frame: str, timeout_s: float
) -> Pose2D | None:
    result = await tf_adapter.lookup_transform(
        "map", base_frame, _LATEST_TRANSFORM_STAMP, timeout_s
    )
    if not result.ok or result.x is None or result.y is None or result.yaw is None:
        return None
    return Pose2D(
        x=result.x, y=result.y, yaw=result.yaw, frame="map",
        stamp=datetime.now(timezone.utc),
    )


async def resolve_current_pose(
    *,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    requested_frame: str | None = None,
    tf_adapter: TFAdapter | None = None,
    base_frame: str | None = None,
    tf_timeout_s: float = _DEFAULT_TF_TIMEOUT_S,
) -> Pose2D | None:
    """Prefers a localization-derived (map frame) pose when localized and a map-frame
    pose was requested (or no frame was specified); falls back to raw odometry.

    `tf_adapter`/`base_frame` are optional (default None => old cache-only behavior,
    fully backward compatible for any caller/test that doesn't pass them). When
    supplied, a live TF lookup is tried as a fallback between the two cache-based
    branches, so a real, already-resolvable `map` transform is never hidden behind a
    lagging discovery snapshot or a not-yet-populated subscription cache.
    """
    motion_cap = registry.get("differential_drive_motion")
    localization_cap = registry.get("localization")

    want_map_frame = requested_frame in (None, "map")
    if want_map_frame:
        if (
            localization_cap is not None
            and "amcl_pose" in localization_cap.resolved_topics
        ):
            cached = subscriptions.get_latest(localization_cap.resolved_topics["amcl_pose"])
            if cached is not None:
                return pose_from_pose_with_covariance_stamped(cached.raw, frame="map", stamp=cached.stamp)

        if tf_adapter is not None and base_frame is not None:
            live_pose = await _pose_from_live_tf(
                tf_adapter=tf_adapter, base_frame=base_frame, timeout_s=tf_timeout_s
            )
            if live_pose is not None:
                return live_pose

    if motion_cap is not None and "odom" in motion_cap.resolved_topics:
        cached = subscriptions.get_latest(motion_cap.resolved_topics["odom"])
        if cached is not None:
            return pose_from_odometry(cached.raw, frame=requested_frame or "odom", stamp=cached.stamp)

    return None

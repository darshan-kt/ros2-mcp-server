"""Unit tests for robot.get_state / robot.get_capabilities result assembly."""
from __future__ import annotations

from datetime import datetime, timezone

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.subscriptions import CachedMessage
from ros_mcp.mcp.capabilities_resolver import resolve_capabilities
from ros_mcp.mcp.state_resolver import resolve_robot_state

NOW = datetime.now(timezone.utc)


class FakeSubscriptions:
    def __init__(self, cache: dict[str, CachedMessage]) -> None:
        self._cache = cache

    def ensure_subscribed(self, topic, type_name, *, qos=None):
        pass

    def get_latest(self, topic):
        return self._cache.get(topic)

    def is_fresh(self, topic, max_age_s):
        cached = self._cache.get(topic)
        return cached is not None and cached.age_s <= max_age_s


def test_resolve_capabilities_reports_registry_snapshot():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="range_sensing", confidence=Confidence.CONFIRMED, backend_id="laser_scan"),)
    )
    result = resolve_capabilities(
        robot_id="turtlebot3_waffle", command_id="c1", duration_sec=0.01, registry=registry
    )
    assert result.status == "succeeded"
    assert result.snapshot_version == 1
    assert result.capabilities[0].capability_id == "range_sensing"


def test_resolve_robot_state_with_no_odometry_yet_reports_no_pose():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(
                capability_id="differential_drive_motion",
                confidence=Confidence.CONFIRMED,
                backend_id="cmd_vel",
                resolved_topics={"cmd_vel": "/cmd_vel", "odom": "/odom"},
            ),
        )
    )
    result = resolve_robot_state(
        robot_id="r1",
        command_id="c1",
        duration_sec=0.01,
        registry=registry,
        subscriptions=FakeSubscriptions({}),
        requested_frame=None,
        active_command=None,
    )
    assert result.pose is None
    assert result.is_moving is False


def test_resolve_robot_state_from_odometry():
    odom = Odometry()
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.orientation.w = 1.0  # identity -> yaw 0
    odom.twist.twist.linear.x = 0.2
    odom.twist.twist.angular.z = 0.0

    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(
                capability_id="differential_drive_motion",
                confidence=Confidence.CONFIRMED,
                backend_id="cmd_vel",
                resolved_topics={"cmd_vel": "/cmd_vel", "odom": "/odom"},
            ),
        )
    )
    cache = {
        "/odom": CachedMessage(value={}, raw=odom, stamp=NOW, received_at=NOW, age_s=0.05)
    }
    result = resolve_robot_state(
        robot_id="r1",
        command_id="c1",
        duration_sec=0.01,
        registry=registry,
        subscriptions=FakeSubscriptions(cache),
        requested_frame=None,
        active_command=None,
    )
    assert result.pose is not None
    assert result.pose.x == 1.0
    assert result.pose.y == 2.0
    assert result.pose.frame == "odom"
    assert result.linear_velocity_mps == 0.2
    assert result.is_moving is True


def test_resolve_robot_state_prefers_map_frame_when_localized():
    amcl = PoseWithCovarianceStamped()
    amcl.pose.pose.position.x = 5.0
    amcl.pose.pose.position.y = 6.0
    amcl.pose.pose.orientation.w = 1.0

    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(
                capability_id="localization",
                confidence=Confidence.CONFIRMED,
                backend_id="amcl",
                resolved_topics={"amcl_pose": "/amcl_pose"},
            ),
        )
    )
    cache = {
        "/amcl_pose": CachedMessage(value={}, raw=amcl, stamp=NOW, received_at=NOW, age_s=0.02)
    }
    result = resolve_robot_state(
        robot_id="r1",
        command_id="c1",
        duration_sec=0.01,
        registry=registry,
        subscriptions=FakeSubscriptions(cache),
        requested_frame="map",
        active_command=None,
    )
    assert result.pose is not None
    assert result.pose.frame == "map"
    assert result.pose.x == 5.0

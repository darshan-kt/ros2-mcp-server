"""Unit tests for capability inference — pure functions over synthetic GraphSnapshots,
no ROS/rclpy involved at all (03-capability-discovery.md, ADR-003)."""
from __future__ import annotations

from datetime import datetime, timezone

from ros_mcp.capabilities.inference import DefaultCapabilityInferenceEngine
from ros_mcp.contracts.config import CapabilityConfig, SafetyConfig
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.discovery import ActionInfo, GraphSnapshot, TopicInfo

NOW = datetime.now(timezone.utc)


def _topic(name: str, type_name: str) -> TopicInfo:
    return TopicInfo(
        name=name,
        type_name=type_name,
        publishers=("/some_node",),
        subscribers=(),
        qos_profiles=(),
        measured_hz=None,
        last_stamp=None,
        frame_id=None,
    )


def _snapshot(topics=(), actions=(), tf_frames=()) -> GraphSnapshot:
    return GraphSnapshot(
        snapshot_version=1,
        taken_at=NOW,
        nodes=(),
        topics=tuple(topics),
        services=(),
        actions=tuple(actions),
        tf_frames=tuple(tf_frames),
    )


def _engine() -> DefaultCapabilityInferenceEngine:
    return DefaultCapabilityInferenceEngine(SafetyConfig())


def test_no_topics_yields_no_capabilities():
    entries = _engine().infer(_snapshot(), CapabilityConfig())
    assert entries == ()


def test_confirmed_differential_drive_motion_at_default_topic_names():
    snapshot = _snapshot(
        topics=[
            _topic("/cmd_vel", "geometry_msgs/msg/Twist"),
            _topic("/odom", "nav_msgs/msg/Odometry"),
        ]
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    motion = next(e for e in entries if e.capability_id == "differential_drive_motion")
    assert motion.confidence == Confidence.CONFIRMED
    assert motion.resolved_topics == {"cmd_vel": "/cmd_vel", "odom": "/odom"}
    assert motion.limits["max_linear_mps"] == 0.5


def test_likely_differential_drive_motion_at_nonstandard_topic_name():
    snapshot = _snapshot(
        topics=[
            _topic("/base/twist_cmd", "geometry_msgs/msg/Twist"),
            _topic("/base/odometry", "nav_msgs/msg/Odometry"),
        ]
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    motion = next(e for e in entries if e.capability_id == "differential_drive_motion")
    assert motion.confidence == Confidence.LIKELY
    assert motion.resolved_topics == {"cmd_vel": "/base/twist_cmd", "odom": "/base/odometry"}


def test_ambiguous_differential_drive_motion_with_two_candidates():
    snapshot = _snapshot(
        topics=[
            _topic("/left/cmd_vel", "geometry_msgs/msg/Twist"),
            _topic("/right/cmd_vel", "geometry_msgs/msg/Twist"),
            _topic("/odom", "nav_msgs/msg/Odometry"),
        ]
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    motion = next(e for e in entries if e.capability_id == "differential_drive_motion")
    assert motion.confidence == Confidence.AMBIGUOUS
    assert motion.resolved_topics == {}


def test_motion_disabled_by_config_yields_no_entry():
    snapshot = _snapshot(
        topics=[
            _topic("/cmd_vel", "geometry_msgs/msg/Twist"),
            _topic("/odom", "nav_msgs/msg/Odometry"),
        ]
    )
    cfg = CapabilityConfig()
    cfg.motion.enabled = False
    entries = _engine().infer(snapshot, cfg)
    assert not any(e.capability_id == "differential_drive_motion" for e in entries)


def test_confirmed_autonomous_navigation():
    snapshot = _snapshot(
        actions=[ActionInfo(name="/navigate_to_pose", type_name="nav2_msgs/action/NavigateToPose")]
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    nav = next(e for e in entries if e.capability_id == "autonomous_navigation")
    assert nav.confidence == Confidence.CONFIRMED
    assert nav.limits["max_navigation_distance_m"] == 10.0


def test_no_nav2_action_server_yields_no_navigation_capability():
    entries = _engine().infer(_snapshot(), CapabilityConfig())
    assert not any(e.capability_id == "autonomous_navigation" for e in entries)


def test_localization_via_amcl_pose_topic():
    snapshot = _snapshot(
        topics=[_topic("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped")]
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    loc = next(e for e in entries if e.capability_id == "localization")
    assert loc.confidence == Confidence.CONFIRMED


def test_localization_via_tf_map_frame():
    snapshot = _snapshot(tf_frames=["map", "odom", "base_link"])
    entries = _engine().infer(snapshot, CapabilityConfig())
    loc = next(e for e in entries if e.capability_id == "localization")
    assert loc.confidence == Confidence.LIKELY


def test_range_sensing_confirmed_at_default_scan_topic():
    snapshot = _snapshot(topics=[_topic("/scan", "sensor_msgs/msg/LaserScan")])
    entries = _engine().infer(snapshot, CapabilityConfig())
    scan = next(e for e in entries if e.capability_id == "range_sensing")
    assert scan.confidence == Confidence.CONFIRMED
    assert scan.resolved_topics == {"scan": "/scan"}


def test_visual_observation_and_object_detection_follow_camera_presence():
    snapshot = _snapshot(topics=[_topic("/camera/image_raw", "sensor_msgs/msg/Image")])
    entries = _engine().infer(snapshot, CapabilityConfig())
    vis = next(e for e in entries if e.capability_id == "visual_observation")
    det = next(e for e in entries if e.capability_id == "object_detection")
    assert vis.confidence == Confidence.CONFIRMED
    assert det.confidence == Confidence.CONFIRMED
    assert det.backend_id == "stub_detector"


def test_object_detection_absent_without_camera():
    entries = _engine().infer(_snapshot(), CapabilityConfig())
    assert not any(e.capability_id == "object_detection" for e in entries)


def test_full_turtlebot3_graph_yields_all_five_mvp_capabilities():
    snapshot = _snapshot(
        topics=[
            _topic("/cmd_vel", "geometry_msgs/msg/Twist"),
            _topic("/odom", "nav_msgs/msg/Odometry"),
            _topic("/scan", "sensor_msgs/msg/LaserScan"),
            _topic("/camera/image_raw", "sensor_msgs/msg/Image"),
            _topic("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
        ],
        actions=[ActionInfo(name="/navigate_to_pose", type_name="nav2_msgs/action/NavigateToPose")],
    )
    entries = _engine().infer(snapshot, CapabilityConfig())
    ids = {e.capability_id for e in entries}
    assert ids == {
        "differential_drive_motion",
        "autonomous_navigation",
        "localization",
        "range_sensing",
        "visual_observation",
        "object_detection",
    }
    assert all(e.confidence == Confidence.CONFIRMED for e in entries)

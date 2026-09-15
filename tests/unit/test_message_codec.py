"""Unit tests for RosidlMessageCodec. Uses real ROS message classes (Twist, Odometry,
LaserScan) directly — no rclpy.init(), no node, no live graph required, satisfying the
MVP's "ROS mocked" unit-test requirement."""
from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from ros_mcp.codec.message_codec import RosidlMessageCodec


def test_twist_round_trip():
    codec = RosidlMessageCodec()
    original = Twist()
    original.linear.x = 0.5
    original.angular.z = -0.25

    as_dict = codec.to_dict(original)
    assert as_dict == {
        "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": -0.25},
    }

    restored = codec.from_dict("geometry_msgs/msg/Twist", as_dict)
    assert math.isclose(restored.linear.x, 0.5)
    assert math.isclose(restored.angular.z, -0.25)


def test_odometry_round_trip_nested_and_header():
    codec = RosidlMessageCodec()
    original = Odometry()
    original.header.frame_id = "odom"
    original.child_frame_id = "base_link"
    original.pose.pose.position.x = 1.5
    original.pose.pose.position.y = -2.0
    original.pose.pose.orientation.w = 1.0

    as_dict = codec.to_dict(original)
    assert as_dict["header"]["frame_id"] == "odom"
    assert as_dict["child_frame_id"] == "base_link"
    assert as_dict["pose"]["pose"]["position"]["x"] == 1.5
    assert as_dict["pose"]["pose"]["position"]["y"] == -2.0

    restored = codec.from_dict("nav_msgs/msg/Odometry", as_dict)
    assert restored.header.frame_id == "odom"
    assert math.isclose(restored.pose.pose.position.x, 1.5)
    assert math.isclose(restored.pose.pose.position.y, -2.0)


def test_laser_scan_round_trip_small_array():
    codec = RosidlMessageCodec()
    original = LaserScan()
    original.range_min = 0.1
    original.range_max = 10.0
    original.ranges = [1.0, 2.0, float("inf")]

    as_dict = codec.to_dict(original)
    assert as_dict["ranges"] == [1.0, 2.0, float("inf")]
    assert as_dict["range_min"] == 0.1

    restored = codec.from_dict("sensor_msgs/msg/LaserScan", as_dict)
    assert list(restored.ranges) == [1.0, 2.0, float("inf")]


def test_large_array_is_truncated_not_inlined():
    codec = RosidlMessageCodec()
    scan = LaserScan()
    scan.ranges = [1.0] * 5000  # above the default element_limit of 4096

    as_dict = codec.to_dict(scan, element_limit=4096)
    assert as_dict["ranges"] == {
        "truncated": True,
        "length": 5000,
        "note": "use a dedicated perception tool or raise element_limit explicitly",
    }


def test_small_array_under_limit_is_not_truncated():
    codec = RosidlMessageCodec()
    scan = LaserScan()
    scan.ranges = [1.0] * 10

    as_dict = codec.to_dict(scan, element_limit=4096)
    assert as_dict["ranges"] == [1.0] * 10


def test_schema_for_twist_reports_nested_object():
    codec = RosidlMessageCodec()
    schema = codec.schema_for("geometry_msgs/msg/Twist")
    assert schema["type"] == "object"
    assert schema["properties"]["linear"]["type"] == "object"
    assert schema["properties"]["linear"]["properties"]["x"] == {"type": "number"}


def test_schema_for_laser_scan_reports_array_of_number():
    codec = RosidlMessageCodec()
    schema = codec.schema_for("sensor_msgs/msg/LaserScan")
    assert schema["properties"]["ranges"] == {"type": "array", "items": {"type": "number"}}


def test_schema_is_cached_per_type_name():
    codec = RosidlMessageCodec()
    first = codec.schema_for("geometry_msgs/msg/Twist")
    second = codec.schema_for("geometry_msgs/msg/Twist")
    assert first is second

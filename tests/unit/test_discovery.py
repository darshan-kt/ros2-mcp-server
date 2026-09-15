"""Unit tests for ros_mcp.discovery — ROS mocked via a duck-typed fake node, no live
ROS graph or rclpy.init() required (17-mvp.md acceptance criterion)."""
from __future__ import annotations

import asyncio

from ros_mcp.discovery.engine import RclpyDiscoveryEngine
from ros_mcp.discovery.snapshot import collect_graph_snapshot


class FakeNode:
    """Duck-typed stand-in for rclpy.node.Node exposing only the graph-introspection
    method names collect_graph_snapshot actually calls."""

    def __init__(self) -> None:
        self._nodes = [("robot_state_publisher", "/"), ("turtlebot3_node", "/")]
        self._pubs_by_node = {
            "robot_state_publisher": [("/tf", ["tf2_msgs/msg/TFMessage"])],
            "turtlebot3_node": [
                ("/odom", ["nav_msgs/msg/Odometry"]),
                ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ],
        }
        self._subs_by_node = {
            "robot_state_publisher": [],
            "turtlebot3_node": [("/cmd_vel", ["geometry_msgs/msg/Twist"])],
        }
        self._services_by_node = {"robot_state_publisher": [], "turtlebot3_node": []}
        self._clients_by_node = {"robot_state_publisher": [], "turtlebot3_node": []}

    def get_node_names_and_namespaces(self):
        return list(self._nodes)

    def get_publisher_names_and_types_by_node(self, name, namespace):
        return list(self._pubs_by_node.get(name, []))

    def get_subscriber_names_and_types_by_node(self, name, namespace):
        return list(self._subs_by_node.get(name, []))

    def get_service_names_and_types_by_node(self, name, namespace):
        return list(self._services_by_node.get(name, []))

    def get_client_names_and_types_by_node(self, name, namespace):
        return list(self._clients_by_node.get(name, []))


def _fake_action_names_and_types(node):
    return [("/navigate_to_pose", ["nav2_msgs/action/NavigateToPose"])]


def _fake_action_server_names_and_types_by_node(node, name, namespace):
    if name == "turtlebot3_node":
        return [("/navigate_to_pose", ["nav2_msgs/action/NavigateToPose"])]
    return []


def test_collect_graph_snapshot_aggregates_topics_across_nodes():
    snapshot = collect_graph_snapshot(
        FakeNode(),
        prev_version=0,
        action_names_and_types_fn=_fake_action_names_and_types,
        action_server_names_and_types_by_node_fn=_fake_action_server_names_and_types_by_node,
    )

    assert snapshot.snapshot_version == 1
    topic_names = {t.name for t in snapshot.topics}
    assert topic_names == {"/tf", "/odom", "/scan", "/cmd_vel"}

    odom = next(t for t in snapshot.topics if t.name == "/odom")
    assert odom.type_name == "nav_msgs/msg/Odometry"
    assert odom.publishers == ("/turtlebot3_node",)

    cmd_vel = next(t for t in snapshot.topics if t.name == "/cmd_vel")
    assert cmd_vel.subscribers == ("/turtlebot3_node",)

    assert len(snapshot.nodes) == 2
    assert {n.name for n in snapshot.nodes} == {"robot_state_publisher", "turtlebot3_node"}

    assert snapshot.actions == (
        __import__("ros_mcp.contracts.discovery", fromlist=["ActionInfo"]).ActionInfo(
            name="/navigate_to_pose", type_name="nav2_msgs/action/NavigateToPose"
        ),
    )


def test_collect_graph_snapshot_handles_empty_graph():
    class EmptyNode(FakeNode):
        def get_node_names_and_namespaces(self):
            return []

    snapshot = collect_graph_snapshot(
        EmptyNode(),
        prev_version=3,
        action_names_and_types_fn=lambda node: [],
        action_server_names_and_types_by_node_fn=lambda node, n, ns: [],
    )
    assert snapshot.snapshot_version == 4
    assert snapshot.nodes == ()
    assert snapshot.topics == ()
    assert snapshot.actions == ()


def test_collect_graph_snapshot_tf_frames_from_buffer():
    class FakeTfBuffer:
        def all_frames_as_yaml(self):
            return "base_link:\n  parent: 'odom'\n  broadcaster: 'x'\nodom:\n  parent: 'map'\n  broadcaster: 'y'\n"

    snapshot = collect_graph_snapshot(
        FakeNode(),
        tf_buffer=FakeTfBuffer(),
        prev_version=0,
        action_names_and_types_fn=lambda node: [],
        action_server_names_and_types_by_node_fn=lambda node, n, ns: [],
    )
    assert set(snapshot.tf_frames) == {"base_link", "odom", "map"}


class FakeSyncRosBridge:
    """Runs the callable inline — sufficient for unit-testing engine orchestration
    without a real rclpy executor thread."""

    async def call_ros_from_asyncio(self, fn):
        return fn()

    def call_soon_threadsafe_from_ros(self, fn):
        fn()


def test_discovery_engine_start_populates_snapshot_and_fires_callback():
    async def scenario():
        engine = RclpyDiscoveryEngine(
            ros_bridge=FakeSyncRosBridge(),
            node=FakeNode(),
            poll_interval_s=1000.0,
            action_names_and_types_fn=_fake_action_names_and_types,
            action_server_names_and_types_by_node_fn=_fake_action_server_names_and_types_by_node,
        )
        seen = []
        engine.on_snapshot_changed(seen.append)
        await engine.start()
        snapshot = await engine.get_current_snapshot()
        await engine.stop()
        return snapshot, seen

    snapshot, seen = asyncio.run(scenario())
    assert snapshot.snapshot_version == 1
    assert len(seen) == 1
    assert seen[0] is snapshot


def test_discovery_engine_no_robot_present_yields_empty_snapshot():
    class NoRobotNode(FakeNode):
        def get_node_names_and_namespaces(self):
            return []

    async def scenario():
        engine = RclpyDiscoveryEngine(
            ros_bridge=FakeSyncRosBridge(),
            node=NoRobotNode(),
            poll_interval_s=1000.0,
            action_names_and_types_fn=lambda node: [],
            action_server_names_and_types_by_node_fn=lambda node, n, ns: [],
        )
        await engine.start()
        snapshot = await engine.get_current_snapshot()
        await engine.stop()
        return snapshot

    snapshot = asyncio.run(scenario())
    assert snapshot.nodes == ()
    assert snapshot.topics == ()

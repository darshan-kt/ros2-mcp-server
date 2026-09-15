"""Unit tests for Nav2NavigationPlugin — a fake ActionClient/GoalHandle stands in for
Nav2, driven synchronously through the same ThreadSafeRosBridge-shaped fake used
elsewhere, so no live ROS or Nav2 stack is required."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from ros_mcp.adapters.navigation.nav2_plugin import Nav2NavigationPlugin
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import SafetyConfig, TimeoutConfig
from ros_mcp.contracts.core import CommandClass, Confidence, Operation
from ros_mcp.contracts.tf import TransformResult
from ros_mcp.execution.cancellation import SimpleCancellationToken


class FakeBridge:
    async def call_ros_from_asyncio(self, fn):
        return fn()

    def call_soon_threadsafe_from_ros(self, fn):
        fn()


async def _instant_sleep(_s: float) -> None:
    # Must still cede control to the event loop (a bare `return None` coroutine never
    # suspends and would busy-loop navigate()'s polling `while` forever whenever another
    # task, e.g. the internal get_result_async() bridge, needs a scheduling turn).
    await asyncio.sleep(0)


async def _pump_until(predicate, *, max_iterations: int = 200) -> None:
    """Repeatedly yields one event-loop tick until `predicate()` is true. navigate()
    crosses several internal await hops (ensure_action_client -> wait_for_server ->
    localization TF lookup -> send_goal_async) before a fake ActionClient's
    last_goal_handle is populated; a single `await asyncio.sleep(0)` is not reliably
    enough hops, so tests that inspect action_client state concurrently with a running
    navigate() task pump until that state actually appears (or fail loudly instead of
    hanging)."""
    for _ in range(max_iterations):
        if predicate():
            return
        await asyncio.sleep(0)
    raise AssertionError("condition never became true within max_iterations event-loop ticks")


class FakeRclpyFuture:
    def __init__(self) -> None:
        self._callbacks = []
        self._result = None
        self._exception = None
        self._done = False

    def add_done_callback(self, cb) -> None:
        self._callbacks.append(cb)
        if self._done:
            cb(self)

    def set_result(self, result) -> None:
        self._result = result
        self._done = True
        for cb in self._callbacks:
            cb(self)

    def result(self):
        return self._result

    def exception(self):
        return self._exception


class FakeGoalHandle:
    def __init__(self, *, accepted: bool) -> None:
        self.accepted = accepted
        self.cancel_called = False
        self.result_future = FakeRclpyFuture()

    def get_result_async(self) -> FakeRclpyFuture:
        return self.result_future

    def cancel_goal_async(self) -> FakeRclpyFuture:
        self.cancel_called = True
        fut = FakeRclpyFuture()
        fut.set_result(SimpleNamespace())
        return fut


class FakeActionClient:
    def __init__(self, node, action_type, action_name) -> None:
        self.action_name = action_name
        self.server_ready = True
        self.next_accepted = True
        self.sent_goals: list = []
        self.last_goal_handle: FakeGoalHandle | None = None

    def wait_for_server(self, timeout_sec=None) -> bool:
        return self.server_ready

    def send_goal_async(self, goal, feedback_callback=None) -> FakeRclpyFuture:
        self.sent_goals.append(goal)
        self._feedback_callback = feedback_callback
        goal_handle = FakeGoalHandle(accepted=self.next_accepted)
        self.last_goal_handle = goal_handle
        fut = FakeRclpyFuture()
        fut.set_result(goal_handle)
        return fut

    def emit_feedback(self, *, distance_remaining, number_of_recoveries=0, x=0.0, y=0.0):
        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = distance_remaining
        feedback.number_of_recoveries = number_of_recoveries
        feedback.current_pose.pose.position.x = x
        feedback.current_pose.pose.position.y = y
        feedback.current_pose.pose.orientation.w = 1.0
        msg = SimpleNamespace(feedback=feedback)
        self._feedback_callback(msg)


class AlwaysOkTFAdapter:
    def __init__(self, frames=("map", "odom", "base_link"), transform_ok=True):
        self._frames = frozenset(frames)
        self._transform_ok = transform_ok

    async def lookup_transform(self, target_frame, source_frame, stamp, timeout_s):
        if self._transform_ok:
            return TransformResult(ok=True, x=1.0, y=2.0, yaw=0.0, error=None)
        return TransformResult(ok=False, x=None, y=None, yaw=None, error="not_connected")

    def known_frames(self):
        return self._frames


def _plugin(*, tf_adapter, registry, action_client, sleep_fn=_instant_sleep, progress_sink=None):
    return Nav2NavigationPlugin(
        ros_bridge=FakeBridge(),
        node=None,
        tf_adapter=tf_adapter,
        capability_registry=registry,
        action_name="/navigate_to_pose",
        progress_sink=progress_sink,
        action_client_class=lambda node, action_type, action_name: action_client,
        poll_interval_s=0.001,
        sleep_fn=sleep_fn,
    )


def _registry_with_nav_and_localization():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(capability_id="autonomous_navigation", confidence=Confidence.CONFIRMED, backend_id="nav2"),
            CapabilityEntry(capability_id="localization", confidence=Confidence.CONFIRMED, backend_id="amcl"),
        )
    )
    return registry


def _navigate_command(x=1.0, y=2.0, timeout_s=10.0):
    safety = SafetyConfig()
    factory = SemanticCommandFactory(robot_id="turtlebot3_waffle")
    return factory.create(
        tool_name="robot.navigate",
        operation=Operation.NAVIGATE,
        command_class=CommandClass.MOTION,
        arguments={"x": x, "y": y, "timeout_s": timeout_s},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )


def test_invalid_frame_rejected_before_touching_nav2():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(frames=("odom",)), registry=registry, action_client=action_client)
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.navigate(cmd, token))
    assert result.status == "failed"
    assert result.error.code.value == "INVALID_FRAME"
    assert action_client.sent_goals == []


def test_action_server_not_ready_is_capability_unavailable():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    action_client.server_ready = False
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.navigate(cmd, token))
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"


def test_no_localization_capability_is_capability_unavailable():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (CapabilityEntry(capability_id="autonomous_navigation", confidence=Confidence.CONFIRMED, backend_id="nav2"),)
    )
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.navigate(cmd, token))
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"
    assert action_client.sent_goals == []


def test_tf_unavailable_is_capability_unavailable():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(
        tf_adapter=AlwaysOkTFAdapter(transform_ok=False), registry=registry, action_client=action_client
    )
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.navigate(cmd, token))
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"


def test_goal_rejected_by_nav2():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    action_client.next_accepted = False
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.navigate(cmd, token))
    assert result.status == "failed"
    assert result.fail_reason == "goal_rejected"


def test_successful_navigation_reports_final_pose_and_progress():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    cmd = _navigate_command()
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    progress_events = []

    async def scenario():
        plugin_local = _plugin(
            tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client,
            progress_sink=lambda cid, payload: progress_events.append((cid, payload)),
        )
        nav_task = asyncio.ensure_future(plugin_local.navigate(cmd, token))
        await _pump_until(lambda: action_client.last_goal_handle is not None)
        action_client.emit_feedback(distance_remaining=5.0)
        await asyncio.sleep(0)
        action_client.last_goal_handle.result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace())
        )
        return await nav_task

    result = asyncio.run(scenario())
    assert result.status == "succeeded"
    assert result.distance_remaining_m == 0.0
    assert result.final_pose is not None
    assert progress_events, "expected at least one progress notification"
    assert progress_events[0][0] == cmd.command_id


def test_navigation_cancelled_mid_flight():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    token = SimpleCancellationToken(deadline_monotonic=1e18)
    tick_count = {"n": 0}

    async def cancel_after_a_few_ticks(_s: float) -> None:
        tick_count["n"] += 1
        if tick_count["n"] >= 2:
            token.cancel()

    plugin = _plugin(
        tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client,
        sleep_fn=cancel_after_a_few_ticks,
    )
    cmd = _navigate_command()
    # Never completes the result future -> the plugin must rely on the cancel path.
    result = asyncio.run(plugin.navigate(cmd, token))

    assert result.status == "failed"
    assert result.error.code.value == "CANCELLED"
    assert action_client.last_goal_handle.cancel_called is True


def test_navigation_aborted_maps_to_navigation_failed():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    async def scenario():
        plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
        nav_task = asyncio.ensure_future(plugin.navigate(cmd, token))
        await _pump_until(lambda: action_client.last_goal_handle is not None)
        action_client.last_goal_handle.result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=SimpleNamespace())
        )
        return await nav_task

    cmd = _navigate_command()
    result = asyncio.run(scenario())
    assert result.status == "failed"
    assert result.error.code.value == "NAVIGATION_FAILED"
    assert result.fail_reason == "planner_failure"


def test_cancel_all_cancels_active_goal():
    registry = _registry_with_nav_and_localization()
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    token = SimpleCancellationToken(deadline_monotonic=1e18)
    cmd = _navigate_command()

    async def scenario():
        nav_task = asyncio.ensure_future(plugin.navigate(cmd, token))
        # Pump until the plugin has actually recorded the goal as active (not merely
        # until the fake client received it) — cancel_all() only acts on
        # self._active_goal_handle, set one await-hop after send_goal_async returns.
        await _pump_until(lambda: plugin._active_goal_handle is not None)
        await plugin.cancel_all()
        action_client.last_goal_handle.result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=SimpleNamespace())
        )
        return await nav_task

    result = asyncio.run(scenario())
    assert action_client.last_goal_handle.cancel_called is True
    assert result.status == "failed"
    assert result.error.code.value == "CANCELLED"


def test_is_available_reflects_registry():
    registry = InMemoryCapabilityRegistry()
    action_client = FakeActionClient(None, None, None)
    plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    assert plugin.is_available() is False

    registry.update(
        (CapabilityEntry(capability_id="autonomous_navigation", confidence=Confidence.CONFIRMED, backend_id="nav2"),)
    )
    assert plugin.is_available() is True

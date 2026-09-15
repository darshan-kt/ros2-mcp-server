"""Full-pipeline NAVIGATE tests: SemanticCommandFactory -> DefaultSafetyPolicyEngine ->
DefaultCommandPlanner -> DefaultExecutionManager -> Nav2NavigationPlugin."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import SafetyConfig, TimeoutConfig
from ros_mcp.contracts.core import CommandClass, Confidence, ExecutionState, Operation, Pose2D
from ros_mcp.execution.manager import DefaultExecutionManager
from ros_mcp.execution.planner import DefaultCommandPlanner

from test_nav2_plugin import AlwaysOkTFAdapter, FakeActionClient, _instant_sleep, _plugin, _pump_until


def _navigate_command(x=1.0, y=2.0, max_navigation_distance_m=50.0, timeout_s=10.0):
    safety = SafetyConfig(max_navigation_distance_m=max_navigation_distance_m)
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


def _wire(*, registry, navigation_backend, safety):
    async def current_pose_provider() -> Pose2D | None:
        return Pose2D(x=0.0, y=0.0, yaw=0.0, frame="odom", stamp=datetime.now(timezone.utc))

    planner = DefaultCommandPlanner(motion_backend=None, navigation_backend=navigation_backend)
    from ros_mcp.safety.policy_engine import DefaultSafetyPolicyEngine

    manager = DefaultExecutionManager(
        registry=registry,
        command_planner=planner,
        safety_policy_engine=DefaultSafetyPolicyEngine(lambda: safety),
        current_pose_provider=current_pose_provider,
        read_handlers={},
        motion_backend=None,
        navigation_backend=navigation_backend,
    )
    return manager


def test_navigate_returns_capability_unavailable_when_nav2_absent():
    registry = InMemoryCapabilityRegistry()  # no autonomous_navigation capability at all
    safety = SafetyConfig()
    manager = _wire(registry=registry, navigation_backend=None, safety=safety)

    cmd = _navigate_command()
    result = asyncio.run(manager.submit(cmd))

    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"
    assert cmd.execution_state == ExecutionState.CAPABILITY_UNAVAILABLE


def test_navigate_exceeding_distance_limit_is_safety_rejected_before_touching_nav2():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(capability_id="autonomous_navigation", confidence=Confidence.CONFIRMED, backend_id="nav2"),
            CapabilityEntry(capability_id="localization", confidence=Confidence.CONFIRMED, backend_id="amcl"),
        )
    )
    action_client = FakeActionClient(None, None, None)
    nav_plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    safety = SafetyConfig(max_navigation_distance_m=5.0)
    manager = _wire(registry=registry, navigation_backend=nav_plugin, safety=safety)

    cmd = _navigate_command(x=100.0, y=100.0, max_navigation_distance_m=5.0)
    result = asyncio.run(manager.submit(cmd))

    assert result.status == "failed"
    assert result.error.code.value == "SAFETY_REJECTED"
    assert action_client.sent_goals == []


def test_navigate_end_to_end_success():
    registry = InMemoryCapabilityRegistry()
    registry.update(
        (
            CapabilityEntry(capability_id="autonomous_navigation", confidence=Confidence.CONFIRMED, backend_id="nav2"),
            CapabilityEntry(capability_id="localization", confidence=Confidence.CONFIRMED, backend_id="amcl"),
        )
    )
    action_client = FakeActionClient(None, None, None)
    nav_plugin = _plugin(tf_adapter=AlwaysOkTFAdapter(), registry=registry, action_client=action_client)
    safety = SafetyConfig()
    manager = _wire(registry=registry, navigation_backend=nav_plugin, safety=safety)
    cmd = _navigate_command(x=1.0, y=1.0)

    async def scenario():
        submit_task = asyncio.ensure_future(manager.submit(cmd))
        await _pump_until(lambda: action_client.last_goal_handle is not None)
        action_client.last_goal_handle.result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace())
        )
        return await submit_task

    result = asyncio.run(scenario())
    assert result.status == "succeeded"
    assert result.final_pose is not None
    assert cmd.execution_state == ExecutionState.SUCCEEDED

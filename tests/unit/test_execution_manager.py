"""Full-pipeline unit tests: SemanticCommandFactory -> DefaultSafetyPolicyEngine ->
DefaultCommandPlanner -> DefaultExecutionManager -> CmdVelMotionPlugin. Exercises the
Non-Bypass Rule end to end and the MVP's headline safety acceptance criterion."""
from __future__ import annotations

import asyncio

from ros_mcp.adapters.motion.cmd_vel_plugin import CmdVelMotionPlugin
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import SafetyConfig, TimeoutConfig
from ros_mcp.contracts.core import CommandClass, Confidence, ExecutionState, Operation, Pose2D
from ros_mcp.execution.manager import DefaultExecutionManager
from ros_mcp.execution.planner import DefaultCommandPlanner
from ros_mcp.safety.policy_engine import DefaultSafetyPolicyEngine

from test_cmd_vel_motion_plugin import FakeBridge, FakePublisher, KinematicWorld, SimulatedSubscriptions, _instant_sleep


def _wire(*, world: KinematicWorld, safety: SafetyConfig):
    subs = SimulatedSubscriptions(world, odom_topic="/odom", scan_topic="/scan")
    publisher_holder: dict[str, FakePublisher] = {}

    def get_publisher_fn(twist_class, topic):
        pub = FakePublisher(subs.on_publish)
        publisher_holder["pub"] = pub
        return pub

    motion_plugin = CmdVelMotionPlugin(
        ros_bridge=FakeBridge(),
        node=None,
        subscriptions=subs,
        robot_id="turtlebot3_waffle",
        cmd_vel_topic="/cmd_vel",
        odom_topic="/odom",
        laser_scan_topic="/scan",
        safety_config_provider=lambda: safety,
        control_rate_hz=50.0,
        get_publisher_fn=get_publisher_fn,
        sleep_fn=_instant_sleep,
    )

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

    planner = DefaultCommandPlanner(motion_backend=motion_plugin, navigation_backend=None)
    safety_engine = DefaultSafetyPolicyEngine(lambda: safety)

    async def current_pose_provider() -> Pose2D | None:
        from ros_mcp.robot_state import resolve_current_pose

        return resolve_current_pose(registry=registry, subscriptions=subs)

    manager = DefaultExecutionManager(
        registry=registry,
        command_planner=planner,
        safety_policy_engine=safety_engine,
        current_pose_provider=current_pose_provider,
        read_handlers={},
        motion_backend=motion_plugin,
        navigation_backend=None,
    )
    return manager, publisher_holder, subs


def _move_command(direction, distance_m, *, session_id="s1", max_move_distance_m=50.0, timeout_s=10.0):
    safety = SafetyConfig(max_move_distance_m=max_move_distance_m)
    factory = SemanticCommandFactory(robot_id="turtlebot3_waffle")
    return factory.create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments={"direction": direction, "distance_m": distance_m, "timeout_s": timeout_s},
        session_id=session_id,
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )


def test_move_exceeding_limit_is_safety_rejected_and_publishes_nothing():
    safety = SafetyConfig(max_move_distance_m=5.0)
    world = KinematicWorld()
    manager, publisher_holder, _subs = _wire(world=world, safety=safety)

    cmd = _move_command("forward", 20.0, max_move_distance_m=5.0)

    result = asyncio.run(manager.submit(cmd))

    assert result.status == "failed"
    assert result.error.code.value == "SAFETY_REJECTED"
    assert cmd.execution_state == ExecutionState.SAFETY_REJECTED
    # The motion backend's publisher was never even constructed — proof that
    # CmdVelMotionPlugin.move() (and therefore any /cmd_vel publish) never ran.
    assert "pub" not in publisher_holder


def test_move_within_limit_succeeds_and_does_publish():
    safety = SafetyConfig(max_move_distance_m=5.0)
    world = KinematicWorld()
    manager, publisher_holder, _subs = _wire(world=world, safety=safety)

    cmd = _move_command("forward", 1.0, max_move_distance_m=5.0)
    result = asyncio.run(manager.submit(cmd))

    assert result.status == "succeeded"
    assert cmd.execution_state == ExecutionState.SUCCEEDED
    assert "pub" in publisher_holder
    assert len(publisher_holder["pub"].messages) > 0


def test_stop_halts_and_publishes_zero_twist():
    safety = SafetyConfig()
    world = KinematicWorld()
    manager, publisher_holder, _subs = _wire(world=world, safety=safety)

    factory = SemanticCommandFactory(robot_id="turtlebot3_waffle")
    stop_cmd = factory.create(
        tool_name="robot.stop",
        operation=Operation.STOP,
        command_class=CommandClass.MOTION,
        arguments={},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    result = asyncio.run(manager.submit(stop_cmd))
    assert result.status == "succeeded"
    assert "pub" in publisher_holder
    last = publisher_holder["pub"].messages[-1]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_submit_dedupes_by_command_id():
    safety = SafetyConfig()
    world = KinematicWorld()
    manager, publisher_holder, _subs = _wire(world=world, safety=safety)
    cmd = _move_command("forward", 1.0)

    async def scenario():
        return await asyncio.gather(manager.submit(cmd), manager.submit(cmd))

    result_a, result_b = asyncio.run(scenario())
    assert result_a is result_b


def test_capability_unavailable_when_registry_lacks_motion():
    from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry

    safety = SafetyConfig()
    world = KinematicWorld()
    manager, publisher_holder, subs = _wire(world=world, safety=safety)
    # Replace the registry with an empty one (no differential_drive_motion capability).
    manager._registry = InMemoryCapabilityRegistry()  # type: ignore[attr-defined]

    cmd = _move_command("forward", 1.0)
    result = asyncio.run(manager.submit(cmd))
    assert result.status == "failed"
    assert result.error.code.value == "CAPABILITY_UNAVAILABLE"
    assert "pub" not in publisher_holder

"""Unit tests for DefaultSafetyPolicyEngine — the mandatory safety choke point
(docs/10-safety-and-trust.md, ADR-002)."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone

from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.contracts.config import SafetyConfig
from ros_mcp.contracts.core import CommandClass, Operation, Pose2D
from ros_mcp.contracts.safety import SafetyDecision
from ros_mcp.contracts.errors import ErrorCode
from ros_mcp.safety.policy_engine import DefaultSafetyPolicyEngine
from ros_mcp.contracts.config import TimeoutConfig

NOW = datetime.now(timezone.utc)
HOME_POSE = Pose2D(x=0.0, y=0.0, yaw=0.0, frame="odom", stamp=NOW)


def _factory() -> SemanticCommandFactory:
    return SemanticCommandFactory(robot_id="r1")


def _run(coro):
    return asyncio.run(coro)


def test_move_within_limit_is_allowed():
    safety = SafetyConfig()
    cmd = _factory().create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments={"direction": "forward", "distance_m": 1.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.ALLOW


def test_move_exceeding_distance_limit_is_safety_rejected():
    safety = SafetyConfig(max_move_distance_m=5.0)
    cmd = _factory().create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments={"direction": "forward", "distance_m": 20.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.DENY
    assert outcome.error.code == ErrorCode.SAFETY_REJECTED
    assert "20.0" in outcome.error.message or "20" in outcome.error.message


def test_navigate_exceeding_distance_limit_is_safety_rejected():
    safety = SafetyConfig(max_navigation_distance_m=10.0)
    cmd = _factory().create(
        tool_name="robot.navigate",
        operation=Operation.NAVIGATE,
        command_class=CommandClass.MOTION,
        arguments={"x": 100.0, "y": 100.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.DENY
    assert outcome.error.code == ErrorCode.SAFETY_REJECTED


def test_navigate_without_current_pose_is_robot_not_ready():
    safety = SafetyConfig()
    cmd = _factory().create(
        tool_name="robot.navigate",
        operation=Operation.NAVIGATE,
        command_class=CommandClass.MOTION,
        arguments={"x": 1.0, "y": 1.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, None))
    assert outcome.decision == SafetyDecision.DENY
    assert outcome.error.code == ErrorCode.ROBOT_NOT_READY


def test_stop_always_allowed_even_with_always_human_policy():
    safety = SafetyConfig(approval_policy="always_human")
    cmd = _factory().create(
        tool_name="robot.stop",
        operation=Operation.STOP,
        command_class=CommandClass.MOTION,
        arguments={},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.ALLOW


def test_geofence_rejects_out_of_bounds_target():
    safety = SafetyConfig()
    safety.geofence.enabled = True
    safety.geofence.min_x = -1.0
    safety.geofence.max_x = 1.0
    safety.geofence.min_y = -1.0
    safety.geofence.max_y = 1.0
    cmd = _factory().create(
        tool_name="robot.navigate",
        operation=Operation.NAVIGATE,
        command_class=CommandClass.MOTION,
        arguments={"x": 5.0, "y": 5.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.DENY
    assert outcome.error.code == ErrorCode.SAFETY_REJECTED


def test_policy_based_approval_requires_approval_above_threshold():
    safety = SafetyConfig(approval_policy="policy_based", max_move_distance_m=50.0)
    safety.human_approval.required_above_distance_m = 5.0
    cmd = _factory().create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments={"direction": "forward", "distance_m": 20.0},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.REQUIRE_APPROVAL
    assert "5.0" in outcome.approval_reason


def test_rate_limit_denies_excess_commands_in_one_second():
    safety = SafetyConfig(rate_limit_rps=2.0, max_move_distance_m=50.0)
    engine = DefaultSafetyPolicyEngine(lambda: safety)

    async def scenario():
        outcomes = []
        for _ in range(4):
            cmd = _factory().create(
                tool_name="robot.move",
                operation=Operation.MOVE,
                command_class=CommandClass.MOTION,
                arguments={"direction": "forward", "distance_m": 0.5},
                session_id="same-session",
                safety_config=safety,
                timeout_config=TimeoutConfig(),
            )
            outcomes.append(await engine.check(cmd, HOME_POSE))
        return outcomes

    outcomes = _run(scenario())
    decisions = [o.decision for o in outcomes]
    assert decisions.count(SafetyDecision.DENY) >= 2


def test_rotation_move_has_no_distance_limit_check():
    safety = SafetyConfig(max_move_distance_m=0.1)
    cmd = _factory().create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments={"direction": "rotate_left", "angle_deg": 90},
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )
    engine = DefaultSafetyPolicyEngine(lambda: safety)
    outcome = _run(engine.check(cmd, HOME_POSE))
    assert outcome.decision == SafetyDecision.ALLOW

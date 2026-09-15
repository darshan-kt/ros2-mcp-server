"""CmdVelMotionPlugin — structurally satisfies ros_mcp.contracts.adapters.MotionBackend
and ros_mcp.contracts.plugins.Plugin (docs/13-contracts.md §7 §12,
docs/07-motion-architecture.md).

Implements the closed-loop control algorithm exactly as specified: read odometry, check
freshness, check the obstacle sector matching the actual direction of travel, compute a
clamped (velocity + acceleration) command, publish, and repeat — with cancellation and
timeout checked every tick and an explicit zero-velocity publish on every exit path. No
code path in this class can produce an unbounded/open-loop velocity stream.

`forward`/`backward` hold the heading present at move start and drive along it (positive
or negative linear.x respectively) — TurtleBot3's LiDAR is a full 360° sensor, so a
straight reverse is exactly as observable as a straight advance and needs no turn.
`left`/`right` cannot be realized as a lateral Twist on a differential-drive base, so
they are realized as a two-phase maneuver: rotate in place to face +90°/-90° off the
start heading, then drive forward along that new heading — the same primitives as
forward/rotate, chained.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time
from datetime import datetime, timezone
from typing import Any, Literal

from ros_mcp.contracts.adapters import CancellationToken
from ros_mcp.contracts.config import SafetyConfig
from ros_mcp.contracts.core import MoveTarget, Pose2D, SemanticCommand
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.plugins import PluginHealth, PluginMetadata
from ros_mcp.contracts.results import MotionResult, StopResult
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.geometry import (
    Quaternion,
    angular_difference,
    euclidean_distance,
    normalize_angle,
    project_translation,
    quaternion_to_yaw,
)
from ros_mcp.perception.laser_scan import analyze_laser_scan

logger = logging.getLogger(__name__)

Phase = Literal["turning", "driving"]


def _pose_from_odometry(raw_odom: Any, *, frame: str, stamp: datetime) -> Pose2D:
    pos = raw_odom.pose.pose.position
    o = raw_odom.pose.pose.orientation
    yaw = quaternion_to_yaw(Quaternion(x=o.x, y=o.y, z=o.z, w=o.w))
    return Pose2D(x=pos.x, y=pos.y, yaw=yaw, frame=frame, stamp=stamp)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _clamp_delta(new: float, prev: float, max_delta: float) -> float:
    if max_delta <= 0:
        return prev
    delta = new - prev
    if delta > max_delta:
        return prev + max_delta
    if delta < -max_delta:
        return prev - max_delta
    return new


class CmdVelMotionPlugin:
    """Structurally satisfies ros_mcp.contracts.adapters.MotionBackend and
    ros_mcp.contracts.plugins.Plugin."""

    def __init__(
        self,
        *,
        ros_bridge: Any,
        node: Any,
        subscriptions: SubscriptionManager,
        robot_id: str,
        cmd_vel_topic: str,
        odom_topic: str,
        laser_scan_topic: str | None,
        safety_config_provider: Any,
        control_rate_hz: float = 20.0,
        position_tolerance_m: float = 0.05,
        angle_tolerance_rad: float = math.radians(2.0),
        kp_linear: float = 1.0,
        kp_angular: float = 1.5,
        twist_class: Any = None,
        get_publisher_fn: Any = None,
        sleep_fn: Any = None,
    ) -> None:
        self.metadata = PluginMetadata(
            plugin_id="cmd_vel_motion",
            api_version="1.0.0",
            provides_capabilities=("differential_drive_motion",),
            requires=("topic:geometry_msgs/msg/Twist", "topic:nav_msgs/msg/Odometry"),
        )
        self._ros_bridge = ros_bridge
        self._node = node
        self._subscriptions = subscriptions
        self._robot_id = robot_id
        self._cmd_vel_topic = cmd_vel_topic
        self._odom_topic = odom_topic
        self._laser_scan_topic = laser_scan_topic
        self._safety_config_provider = safety_config_provider
        self._control_rate_hz = control_rate_hz
        self._tick_period_s = 1.0 / control_rate_hz
        self._position_tolerance_m = position_tolerance_m
        self._angle_tolerance_rad = angle_tolerance_rad
        self._kp_linear = kp_linear
        self._kp_angular = kp_angular

        if twist_class is None:
            from geometry_msgs.msg import Twist as _Twist

            twist_class = _Twist
        self._twist_class = twist_class
        self._get_publisher_fn = get_publisher_fn
        self._publisher: Any = None
        self._last_published_at: float | None = None
        # DI seam for tests only: the control loop's pacing between ticks (never its
        # deadline/cancellation math, which stays wall-clock-real via time.monotonic())
        # — defaults to real asyncio.sleep.
        self._sleep_fn = sleep_fn or asyncio.sleep

        subscriptions.ensure_subscribed(odom_topic, "nav_msgs/msg/Odometry")
        if laser_scan_topic is not None:
            subscriptions.ensure_subscribed(laser_scan_topic, "sensor_msgs/msg/LaserScan")

    async def health_check(self) -> PluginHealth:
        return PluginHealth.HEALTHY

    async def _ensure_publisher(self) -> Any:
        if self._publisher is None:
            if self._get_publisher_fn is not None:
                self._publisher = await self._ros_bridge.call_ros_from_asyncio(
                    lambda: self._get_publisher_fn(self._twist_class, self._cmd_vel_topic)
                )
            else:
                self._publisher = await self._ros_bridge.call_ros_from_asyncio(
                    lambda: self._node.create_publisher(self._twist_class, self._cmd_vel_topic, 10)
                )
        return self._publisher

    async def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        publisher = await self._ensure_publisher()
        msg = self._twist_class()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        await self._ros_bridge.call_ros_from_asyncio(lambda: publisher.publish(msg))
        self._last_published_at = time.monotonic()

    async def _publish_zero(self) -> None:
        await self._publish_twist(0.0, 0.0)

    def _current_pose(self) -> tuple[Pose2D | None, bool]:
        """Returns (pose, fresh). pose is None if odometry was never received."""
        safety = self._safety_config_provider()
        cached = self._subscriptions.get_latest(self._odom_topic)
        if cached is None:
            return None, False
        fresh = self._subscriptions.is_fresh(self._odom_topic, safety.odom_stale_s)
        pose = _pose_from_odometry(cached.raw, frame="odom", stamp=cached.stamp)
        return pose, fresh

    def _obstacle_ahead(self, sector: str, obstacle_stop_distance_m: float) -> float | None:
        if self._laser_scan_topic is None:
            return None
        cached = self._subscriptions.get_latest(self._laser_scan_topic)
        if cached is None:
            return None
        analysis = analyze_laser_scan(cached.raw)
        reading = analysis.sectors.get(sector)
        if reading is not None and reading.range_m < obstacle_stop_distance_m:
            return reading.range_m
        return None

    async def move(self, command: SemanticCommand, token: CancellationToken) -> MotionResult:
        start_monotonic = time.monotonic()
        target = command.target
        assert isinstance(target, MoveTarget)
        constraints = command.safety_constraints
        assert constraints is not None

        def elapsed() -> float:
            return time.monotonic() - start_monotonic

        def fail(error: ToolError, *, distance_traveled: float, final_pose: Pose2D | None,
                  stop_reason: str) -> MotionResult:
            return MotionResult(
                status="failed",
                command_id=command.command_id,
                robot_id=command.robot_id,
                duration_sec=elapsed(),
                error=error,
                distance_traveled_m=distance_traveled,
                final_pose=final_pose,
                stop_reason=stop_reason,  # type: ignore[arg-type]
            )

        start_pose, fresh = self._current_pose()
        if start_pose is None or not fresh:
            await self._publish_zero()
            return fail(
                ToolError(code=ErrorCode.SENSOR_STALE, message="no fresh odometry available for closed-loop motion"),
                distance_traveled=0.0,
                final_pose=None,
                stop_reason="safety_stop",
            )

        is_rotation = target.direction in ("rotate_left", "rotate_right")
        if is_rotation:
            assert target.angle_deg is not None
            sign = 1.0 if target.direction == "rotate_left" else -1.0
            heading_target = normalize_angle(start_pose.yaw + sign * math.radians(target.angle_deg))
            phase: Phase = "turning"
            drive_sign = 0.0
            distance_target_m = 0.0
        elif target.direction in ("forward", "backward"):
            assert target.distance_m is not None
            heading_target = start_pose.yaw
            phase = "driving"
            drive_sign = 1.0 if target.direction == "forward" else -1.0
            distance_target_m = target.distance_m
        else:  # left / right
            assert target.distance_m is not None
            offset = math.pi / 2 if target.direction == "left" else -math.pi / 2
            heading_target = normalize_angle(start_pose.yaw + offset)
            phase = "turning"
            drive_sign = 1.0
            distance_target_m = target.distance_m

        phase_start_pose = start_pose
        distance_traveled_total = 0.0
        last_pose = start_pose
        prev_linear_cmd = 0.0
        prev_angular_cmd = 0.0
        deadline = start_monotonic + command.timeout_s

        while True:
            if token.is_cancelled():
                await self._publish_zero()
                return fail(
                    ToolError(code=ErrorCode.CANCELLED, message="move cancelled"),
                    distance_traveled=distance_traveled_total,
                    final_pose=last_pose,
                    stop_reason="cancelled",
                )
            if token.deadline_exceeded() or time.monotonic() >= deadline:
                await self._publish_zero()
                return fail(
                    ToolError(code=ErrorCode.ACTION_TIMEOUT, message="move timed out before reaching target"),
                    distance_traveled=distance_traveled_total,
                    final_pose=last_pose,
                    stop_reason="timeout",
                )

            current_pose, fresh = self._current_pose()
            if current_pose is None or not fresh:
                await self._publish_zero()
                return fail(
                    ToolError(code=ErrorCode.SENSOR_STALE, message="odometry became stale during motion"),
                    distance_traveled=distance_traveled_total,
                    final_pose=last_pose,
                    stop_reason="safety_stop",
                )
            distance_traveled_total += euclidean_distance(
                last_pose.x, last_pose.y, current_pose.x, current_pose.y
            )
            last_pose = current_pose

            if phase == "driving":
                sector = "front" if drive_sign >= 0 else "rear"
                obstacle_range = self._obstacle_ahead(sector, constraints.obstacle_stop_distance_m)
                if obstacle_range is not None:
                    await self._publish_zero()
                    return fail(
                        ToolError(
                            code=ErrorCode.SAFETY_REJECTED,
                            message=(
                                f"obstacle detected {obstacle_range:.2f}m away, within the "
                                f"{constraints.obstacle_stop_distance_m}m stop distance"
                            ),
                            details={"obstacle_range_m": obstacle_range, "sector": sector},
                        ),
                        distance_traveled=distance_traveled_total,
                        final_pose=current_pose,
                        stop_reason="safety_stop",
                    )

            if phase == "turning":
                error = angular_difference(heading_target, current_pose.yaw)
                if abs(error) <= self._angle_tolerance_rad:
                    if is_rotation:
                        await self._publish_zero()
                        return MotionResult(
                            status="succeeded",
                            command_id=command.command_id,
                            robot_id=command.robot_id,
                            duration_sec=elapsed(),
                            distance_traveled_m=distance_traveled_total,
                            final_pose=current_pose,
                            stop_reason="target_reached",
                        )
                    phase = "driving"
                    phase_start_pose = current_pose
                    prev_angular_cmd = 0.0
                    await self._sleep_fn(self._tick_period_s)
                    continue
                angular_cmd = _clamp(
                    self._kp_angular * error, -constraints.max_angular_rps, constraints.max_angular_rps
                )
                angular_cmd = _clamp_delta(
                    angular_cmd,
                    prev_angular_cmd,
                    constraints.max_angular_acceleration_rps2 * self._tick_period_s,
                )
                linear_cmd = 0.0
            else:  # driving
                remaining = distance_target_m - euclidean_distance(
                    phase_start_pose.x, phase_start_pose.y, current_pose.x, current_pose.y
                )
                if remaining <= self._position_tolerance_m:
                    await self._publish_zero()
                    return MotionResult(
                        status="succeeded",
                        command_id=command.command_id,
                        robot_id=command.robot_id,
                        duration_sec=elapsed(),
                        distance_traveled_m=distance_traveled_total,
                        final_pose=current_pose,
                        stop_reason="target_reached",
                    )
                heading_error = angular_difference(heading_target, current_pose.yaw)
                angular_cmd = _clamp(
                    self._kp_angular * heading_error,
                    -constraints.max_angular_rps,
                    constraints.max_angular_rps,
                )
                angular_cmd = _clamp_delta(
                    angular_cmd,
                    prev_angular_cmd,
                    constraints.max_angular_acceleration_rps2 * self._tick_period_s,
                )
                speed = _clamp(self._kp_linear * remaining, 0.0, constraints.max_linear_mps)
                linear_cmd = drive_sign * speed
                linear_cmd = _clamp_delta(
                    linear_cmd,
                    prev_linear_cmd,
                    constraints.max_linear_acceleration_mps2 * self._tick_period_s,
                )

            await self._publish_twist(linear_cmd, angular_cmd)
            prev_linear_cmd, prev_angular_cmd = linear_cmd, angular_cmd
            await self._sleep_fn(self._tick_period_s)

    async def stop(self) -> StopResult:
        await self._publish_zero()
        pose, _fresh = self._current_pose()
        return StopResult(
            status="succeeded",
            command_id="",
            robot_id=self._robot_id,
            duration_sec=0.0,
            cancelled_command_id=None,
            final_pose=pose,
        )

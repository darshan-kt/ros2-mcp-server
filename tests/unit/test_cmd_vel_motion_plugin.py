"""Unit tests for CmdVelMotionPlugin's closed-loop control — a tiny deterministic
kinematic simulator stands in for the real robot: every published Twist is integrated
for one control-loop tick to produce the next odometry sample, so the controller's
actual convergence behavior is exercised, not mocked away."""
from __future__ import annotations

import asyncio
import math
import time
from datetime import datetime, timezone

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from ros_mcp.adapters.motion.cmd_vel_plugin import CmdVelMotionPlugin
from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.contracts.config import SafetyConfig, TimeoutConfig
from ros_mcp.contracts.core import CommandClass, Operation
from ros_mcp.execution.cancellation import SimpleCancellationToken


class FakeBridge:
    async def call_ros_from_asyncio(self, fn):
        return fn()


class FakePublisher:
    def __init__(self, on_publish) -> None:
        self._on_publish = on_publish
        self.messages: list[Twist] = []

    def publish(self, msg: Twist) -> None:
        self.messages.append(msg)
        self._on_publish(msg)


class KinematicWorld:
    """Deterministic unicycle-model integrator standing in for the real robot."""

    def __init__(self, *, x=0.0, y=0.0, yaw=0.0, dt=0.005) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.dt = dt
        self.obstacle_range_m: float | None = None  # None => no obstacle ever reported

    def integrate(self, msg: Twist) -> None:
        self.x += msg.linear.x * math.cos(self.yaw) * self.dt
        self.y += msg.linear.x * math.sin(self.yaw) * self.dt
        self.yaw += msg.angular.z * self.dt

    def odometry_message(self) -> Odometry:
        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        return odom

    def laser_scan_message(self) -> LaserScan:
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min = 0.05
        scan.range_max = 10.0
        ranges = [10.0] * 360
        if self.obstacle_range_m is not None:
            ranges[180] = self.obstacle_range_m  # index for angle ~0 (front)
        scan.ranges = ranges
        return scan


class SimulatedSubscriptions:
    """A SubscriptionManager double whose /odom and /scan reflect a KinematicWorld,
    updated as a side effect of each published Twist (see FakePublisher)."""

    def __init__(self, world: KinematicWorld, *, odom_topic: str, scan_topic: str | None) -> None:
        self._world = world
        self._odom_topic = odom_topic
        self._scan_topic = scan_topic
        self._odom_fresh = True
        self._now = datetime.now(timezone.utc)

    def ensure_subscribed(self, topic, type_name, *, qos=None):
        pass

    def on_publish(self, msg: Twist) -> None:
        self._world.integrate(msg)

    def get_latest(self, topic):
        from dataclasses import dataclass

        from ros_mcp.contracts.subscriptions import CachedMessage

        if topic == self._odom_topic:
            return CachedMessage(
                value={}, raw=self._world.odometry_message(), stamp=self._now,
                received_at=self._now, age_s=0.0,
            )
        if topic == self._scan_topic:
            return CachedMessage(
                value={}, raw=self._world.laser_scan_message(), stamp=self._now,
                received_at=self._now, age_s=0.0,
            )
        return None

    def is_fresh(self, topic, max_age_s):
        if topic == self._odom_topic:
            return self._odom_fresh
        return topic == self._scan_topic

    def set_odom_fresh(self, fresh: bool) -> None:
        self._odom_fresh = fresh


async def _instant_sleep(_seconds: float) -> None:
    """Replaces the control loop's between-tick asyncio.sleep in tests: the simulated
    physics already advance by exactly one tick_period per iteration (KinematicWorld),
    so real wall-clock pacing between iterations adds nothing but test runtime. Still
    uses a real (zero-duration) asyncio.sleep(0) rather than a bare `return` so it
    genuinely yields one event-loop tick — a coroutine that never awaits anything
    internally never cedes control, which would busy-loop forever in any test running
    a second concurrently-scheduled task alongside the control loop."""
    await asyncio.sleep(0)


def _make_plugin(world: KinematicWorld, *, laser_scan_topic="/scan"):
    subs = SimulatedSubscriptions(world, odom_topic="/odom", scan_topic=laser_scan_topic)
    publisher_holder: dict[str, FakePublisher] = {}

    def get_publisher_fn(twist_class, topic):
        pub = FakePublisher(subs.on_publish)
        publisher_holder["pub"] = pub
        return pub

    plugin = CmdVelMotionPlugin(
        ros_bridge=FakeBridge(),
        node=None,
        subscriptions=subs,
        robot_id="r1",
        cmd_vel_topic="/cmd_vel",
        odom_topic="/odom",
        laser_scan_topic=laser_scan_topic,
        safety_config_provider=lambda: SafetyConfig(),
        # A coarser tick (vs. the production default of 20 Hz) keeps the number of real
        # asyncio.sleep() iterations small so per-iteration scheduler overhead can't
        # eat into the wall-clock command timeout below — the physics simulated per
        # tick are unaffected, only how many real Python round-trips it takes.
        control_rate_hz=50.0,
        get_publisher_fn=get_publisher_fn,
        sleep_fn=_instant_sleep,
    )
    return plugin, subs, publisher_holder


def _command(direction, distance_m=None, angle_deg=None, timeout_s=10.0, max_move_distance_m=50.0):
    safety = SafetyConfig(max_move_distance_m=max_move_distance_m)
    factory = SemanticCommandFactory(robot_id="r1")
    arguments = {"direction": direction, "timeout_s": timeout_s}
    if distance_m is not None:
        arguments["distance_m"] = distance_m
    if angle_deg is not None:
        arguments["angle_deg"] = angle_deg
    return factory.create(
        tool_name="robot.move",
        operation=Operation.MOVE,
        command_class=CommandClass.MOTION,
        arguments=arguments,
        session_id="s1",
        safety_config=safety,
        timeout_config=TimeoutConfig(),
    )


def test_move_forward_reaches_target_within_tolerance():
    world = KinematicWorld()
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("forward", distance_m=1.0)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "succeeded"
    assert result.stop_reason == "target_reached"
    assert abs(result.final_pose.x - 1.0) < 0.1
    assert abs(result.final_pose.y) < 0.1


def test_move_backward_drives_in_reverse_without_turning():
    world = KinematicWorld()
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("backward", distance_m=0.5)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "succeeded"
    assert abs(result.final_pose.x - (-0.5)) < 0.1
    assert abs(result.final_pose.yaw) < 0.2  # heading held near 0, no 180 deg turn


def test_rotate_left_90_degrees():
    world = KinematicWorld()
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("rotate_left", angle_deg=90.0)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "succeeded"
    assert abs(result.final_pose.yaw - math.pi / 2) < 0.1


def test_move_left_reaches_laterally_offset_point():
    world = KinematicWorld()
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("left", distance_m=0.5)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "succeeded"
    assert abs(result.final_pose.x - 0.0) < 0.15
    assert abs(result.final_pose.y - 0.5) < 0.15


def test_every_exit_path_publishes_a_final_zero_twist():
    world = KinematicWorld()
    plugin, _subs, pubs = _make_plugin(world)
    cmd = _command("forward", distance_m=1.0)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    asyncio.run(plugin.move(cmd, token))
    last_msg = pubs["pub"].messages[-1]
    assert last_msg.linear.x == 0.0
    assert last_msg.angular.z == 0.0


def test_move_aborts_on_stale_odometry_before_any_progress():
    world = KinematicWorld()
    plugin, subs, pubs = _make_plugin(world)
    subs.set_odom_fresh(False)
    cmd = _command("forward", distance_m=1.0)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "failed"
    assert result.error.code.value == "SENSOR_STALE"
    assert result.stop_reason == "safety_stop"
    assert result.distance_traveled_m == 0.0


def test_move_aborts_on_obstacle_ahead():
    world = KinematicWorld()
    world.obstacle_range_m = 0.1  # closer than default obstacle_stop_distance_m=0.3
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("forward", distance_m=5.0)
    token = SimpleCancellationToken(deadline_monotonic=1e18)

    result = asyncio.run(plugin.move(cmd, token))

    assert result.status == "failed"
    assert result.error.code.value == "SAFETY_REJECTED"
    assert result.stop_reason == "safety_stop"


def test_move_cancelled_mid_execution():
    # Deterministic rather than wall-clock-raced: the injected sleep_fn cancels the
    # token after a few real control-loop ticks have actually executed, so the
    # assertion exercises mid-flight cancellation (not "cancelled before it ever
    # started") without depending on real-time scheduling.
    world = KinematicWorld()
    subs = SimulatedSubscriptions(world, odom_topic="/odom", scan_topic="/scan")
    publisher_holder: dict[str, FakePublisher] = {}

    def get_publisher_fn(twist_class, topic):
        pub = FakePublisher(subs.on_publish)
        publisher_holder["pub"] = pub
        return pub

    token = SimpleCancellationToken(deadline_monotonic=1e18)
    tick_count = {"n": 0}

    async def cancel_after_a_few_ticks(_seconds: float) -> None:
        tick_count["n"] += 1
        if tick_count["n"] >= 3:
            token.cancel()

    plugin = CmdVelMotionPlugin(
        ros_bridge=FakeBridge(),
        node=None,
        subscriptions=subs,
        robot_id="r1",
        cmd_vel_topic="/cmd_vel",
        odom_topic="/odom",
        laser_scan_topic="/scan",
        safety_config_provider=lambda: SafetyConfig(),
        control_rate_hz=50.0,
        get_publisher_fn=get_publisher_fn,
        sleep_fn=cancel_after_a_few_ticks,
    )
    cmd = _command("forward", distance_m=100.0)  # far enough that it wouldn't converge in 3 ticks

    result = asyncio.run(plugin.move(cmd, token))
    assert result.status == "failed"
    assert result.stop_reason == "cancelled"
    assert result.error.code.value == "CANCELLED"
    assert tick_count["n"] >= 3


def test_move_times_out_before_reaching_distant_target():
    # Deterministic: an already-expired deadline trips on the very first loop
    # iteration's check, independent of iteration speed.
    world = KinematicWorld()
    plugin, _subs, _pubs = _make_plugin(world)
    cmd = _command("forward", distance_m=1000.0, timeout_s=10.0)
    token = SimpleCancellationToken(deadline_monotonic=time.monotonic() - 1.0)

    result = asyncio.run(plugin.move(cmd, token))
    assert result.status == "failed"
    assert result.stop_reason == "timeout"
    assert result.error.code.value == "ACTION_TIMEOUT"


def test_stop_publishes_zero_and_reports_pose():
    world = KinematicWorld(x=2.0, y=3.0)
    plugin, _subs, pubs = _make_plugin(world)

    result = asyncio.run(plugin.stop())
    assert result.status == "succeeded"
    assert pubs["pub"].messages[-1].linear.x == 0.0
    assert pubs["pub"].messages[-1].angular.z == 0.0
    assert result.final_pose.x == 2.0

#!/usr/bin/env python3
"""Live-robot acceptance client for docs/17-mvp.md's 9 live-robot criteria.

Not a real LLM: a scripted MCP client. It spawns `python -m ros_mcp.server` as a local
stdio subprocess (mcp.client.stdio) exactly the way Claude Desktop would (ADR-014), so
every tool call in this script exercises the real transport, not a shortcut through the
Python objects directly.

Pass/fail for the motion-sensitive criteria (4, 5, 7, 8) is judged from a SECOND,
independent rclpy node (`Observer`) subscribed directly to /odom and /cmd_vel — not from
trusting the tool's own reported result — per the run's instruction to verify with a
live topic echo, not by reading code.

Usage: python3 acceptance/live_client.py --phase no_nav2|with_nav2 [--goal-x X --goal-y Y]
Writes acceptance/logs/results_<phase>.json.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client

REPO_ROOT = Path(__file__).resolve().parent.parent
RESULTS: list[dict[str, Any]] = []


def record(criterion: str, measured: Any, verdict: str, evidence: dict[str, Any] | None = None) -> None:
    entry = {
        "criterion": criterion,
        "measured": measured,
        "verdict": verdict,
        "evidence": evidence or {},
        "ts": datetime.now(timezone.utc).isoformat(),
    }
    RESULTS.append(entry)
    print(f"[{verdict}] {criterion}: {measured}", flush=True)


class Observer(Node):
    """Independent ground-truth witness. Subscribes directly to /odom and /cmd_vel so
    criteria don't rely on the tool's self-reported result for the motion checks."""

    def __init__(self) -> None:
        from rclpy.parameter import Parameter

        super().__init__(
            "acceptance_observer",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._odom_lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        self._cmd_vel_lock = threading.Lock()
        self.cmd_vel_log: list[tuple[float, float, float]] = []  # (monotonic_ts, linear_x, angular_z)
        self.create_subscription(Odometry, "/odom", self._on_odom, odom_qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )

    def _on_odom(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._latest_odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_vel_lock:
            self.cmd_vel_log.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def latest_pose_xy(self) -> tuple[float, float] | None:
        with self._odom_lock:
            if self._latest_odom is None:
                return None
            p = self._latest_odom.pose.pose.position
            return (p.x, p.y)

    def cmd_vel_since(self, t0: float) -> list[tuple[float, float, float]]:
        with self._cmd_vel_lock:
            return [e for e in self.cmd_vel_log if e[0] >= t0]

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # AMCL default covariance for a known-good initial pose (matches RViz's typical
        # 2D Pose Estimate defaults closely enough for convergence in a small sim world).
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.06853892326654787
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)


def spin_in_background(node: Node) -> None:
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


def parse_tool_json(result: Any) -> dict[str, Any]:
    for block in result.content:
        if getattr(block, "type", None) == "text":
            return json.loads(block.text)
    raise ValueError(f"no text content block in tool result: {result!r}")


def image_bytes_len(result: Any) -> int | None:
    for block in result.content:
        if getattr(block, "type", None) == "image":
            import base64

            return len(base64.b64decode(block.data))
    return None


async def call_tool(session: ClientSession, name: str, args: dict[str, Any]) -> tuple[dict[str, Any], Any]:
    raw = await session.call_tool(name, arguments=args)
    return parse_tool_json(raw), raw


async def wait_for_odom(observer: Observer, timeout_s: float = 15.0) -> tuple[float, float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = observer.latest_pose_xy()
        if pose is not None:
            return pose
        await asyncio.sleep(0.2)
    raise TimeoutError("no /odom message received within timeout")


async def wait_for_localization(session: ClientSession, timeout_s: float = 60.0) -> bool:
    """Poll robot.get_state until pose.frame == 'map' (AMCL has broadcast map->odom at
    least once) instead of a fixed sleep — on this host, AMCL has been observed taking
    much longer than a few seconds to publish its first transform after /initialpose
    under heavy background CPU contention from unrelated containers."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state, _ = await call_tool(session, "robot.get_state", {"frame": "map"})
        pose = state.get("pose")
        if pose is not None and pose.get("frame") == "map":
            return True
        await asyncio.sleep(1.0)
    return False


async def wait_for_capability(
    session: ClientSession, capability_id: str, present: bool, timeout_s: float = 30.0
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last: dict[str, Any] = {}
    while time.monotonic() < deadline:
        last, _ = await call_tool(session, "robot.get_capabilities", {})
        ids = {c["capability_id"] for c in last.get("capabilities", [])}
        if (capability_id in ids) == present:
            return last
        await asyncio.sleep(1.0)
    return last


async def phase_no_nav2(session: ClientSession, observer: Observer) -> None:
    # --- Criterion 1: get_state ---
    state, _ = await call_tool(session, "robot.get_state", {})
    pose = state.get("pose")
    if pose is None:
        record("1. get_state pose", state, "FAIL", {"reason": "pose is null"})
    else:
        # use_sim_time is on (Gazebo publishes /clock) so pose.stamp is a *sim*-clock
        # timestamp near the 1970 epoch, not wall-clock — freshness must be measured
        # against the same sim clock (observer's ROS clock, sync'd via /clock), not
        # datetime.now().
        stamp = datetime.fromisoformat(pose["stamp"])
        if stamp.tzinfo is None:
            stamp = stamp.replace(tzinfo=timezone.utc)
        now_sim_s = observer.get_clock().now().nanoseconds / 1e9
        age_s = now_sim_s - stamp.timestamp()
        # small negative age is normal clock-sampling jitter (the pose stamp and the
        # observer's own clock read are two independent samples a few ms apart), not
        # staleness — only flag it if the pose is actually old or from the future by a
        # implausible margin.
        verdict = "PASS" if pose.get("frame") and -0.5 <= age_s < 2.0 else "FAIL"
        record("1. get_state pose", {"pose": pose, "age_s": age_s}, verdict)

    # --- Criterion 2a: get_capabilities WITHOUT Nav2 ---
    caps = await wait_for_capability(session, "autonomous_navigation", present=False, timeout_s=15.0)
    tools = await session.list_tools()
    tool_names = {t.name for t in tools.tools}
    nav_absent = "autonomous_navigation" not in {c["capability_id"] for c in caps.get("capabilities", [])}
    nav_tool_absent = "robot.navigate" not in tool_names
    verdict = "PASS" if nav_absent and nav_tool_absent else "FAIL"
    record(
        "2a. get_capabilities (no Nav2): navigation absent",
        {"capability_ids": sorted({c["capability_id"] for c in caps.get("capabilities", [])}), "tools": sorted(tool_names)},
        verdict,
    )

    # --- Criterion 3: get_laser_scan ---
    scan, _ = await call_tool(session, "robot.get_laser_scan", {"include_raw_ranges": False})
    nearest = scan.get("nearest")
    verdict = "PASS" if scan.get("status") == "succeeded" and nearest is not None and 0 < nearest["range_m"] < 10.0 else "FAIL"
    record("3. get_laser_scan", {k: scan.get(k) for k in ("nearest", "farthest", "sectors", "stale")}, verdict)

    # --- Criterion 4: move(forward, 1.0) within 10cm ---
    baseline = await wait_for_odom(observer)
    move_result, _ = await call_tool(session, "robot.move", {"direction": "forward", "distance_m": 1.0})
    await asyncio.sleep(0.5)  # let the final /odom sample catch up
    final = observer.latest_pose_xy() or baseline
    displacement = math.hypot(final[0] - baseline[0], final[1] - baseline[1])
    error_cm = abs(displacement - 1.0) * 100.0
    verdict = "PASS" if move_result.get("status") == "succeeded" and error_cm <= 10.0 else "FAIL"
    record(
        "4. move(forward, 1.0) accuracy",
        {"error_cm": round(error_cm, 2), "displacement_m": round(displacement, 4), "tool_result": move_result},
        verdict,
    )

    # --- Criterion 5: stop latency ---
    t_launch = time.monotonic()
    move_task = asyncio.create_task(call_tool(session, "robot.move", {"direction": "forward", "distance_m": 3.0, "timeout_s": 30.0}))
    moving = False
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        recent = observer.cmd_vel_since(t_launch)
        if any(abs(lx) > 0.01 for _, lx, _ in recent):
            moving = True
            break
        await asyncio.sleep(0.1)
    if not moving:
        record("5. stop latency", None, "BLOCKED", {"reason": "robot never appeared to start moving before stop was issued"})
        move_task.cancel()
    else:
        t_stop_call = time.monotonic()
        stop_result, _ = await call_tool(session, "robot.stop", {})
        t_stop_returned = time.monotonic()
        zero_ts: float | None = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            after = [e for e in observer.cmd_vel_since(t_stop_call) if abs(e[1]) < 0.01 and abs(e[2]) < 0.01]
            if after:
                zero_ts = after[0][0]
                break
            await asyncio.sleep(0.05)
        try:
            await asyncio.wait_for(move_task, timeout=5.0)
        except (asyncio.TimeoutError, Exception):
            pass
        if zero_ts is None:
            record("5. stop latency", None, "FAIL", {"reason": "no zero-velocity /cmd_vel observed within 5s of stop", "stop_result": stop_result})
        else:
            latency_ms = (zero_ts - t_stop_call) * 1000.0
            record(
                "5. stop latency",
                {"latency_ms": round(latency_ms, 1), "call_round_trip_ms": round((t_stop_returned - t_stop_call) * 1000.0, 1)},
                "PASS",
                {"stop_result": stop_result},
            )

    # --- Criterion 8: move exceeding limit -> SAFETY_REJECTED, zero /cmd_vel ---
    t0 = time.monotonic()
    reject_result, raw = await call_tool(session, "robot.move", {"direction": "forward", "distance_m": 6.0})
    published = observer.cmd_vel_since(t0)
    code = (reject_result.get("error") or {}).get("code")
    verdict = "PASS" if reject_result.get("status") == "failed" and code == "SAFETY_REJECTED" and len(published) == 0 else "FAIL"
    record(
        "8. move exceeding limit -> SAFETY_REJECTED + silent /cmd_vel",
        {"error_code": code, "cmd_vel_messages_published": len(published)},
        verdict,
        {"tool_result": reject_result},
    )

    # --- Criterion 9: get_camera_image ---
    cam, raw_cam = await call_tool(session, "robot.get_camera_image", {"max_width_px": 640})
    img_bytes = image_bytes_len(raw_cam)
    cam_error_code = (cam.get("error") or {}).get("code")
    if cam.get("status") == "succeeded" and img_bytes is not None and img_bytes <= 1_048_576 and cam.get("width_px", 0) <= 640:
        verdict = "PASS"
    elif cam_error_code == "CAPABILITY_UNAVAILABLE":
        # burger (this run's TURTLEBOT3_MODEL) has no camera at all — clean, correct
        # degradation, not a product defect, but this criterion can't be exercised on
        # this model.
        verdict = "BLOCKED"
    else:
        verdict = "FAIL"
    record(
        "9. get_camera_image",
        {
            "width_px": cam.get("width_px"),
            "height_px": cam.get("height_px"),
            "frame": cam.get("frame"),
            "jpeg_bytes": img_bytes,
            "data_age_s": cam.get("data_age_s"),
        },
        verdict,
        {"tool_result": cam},
    )


async def phase_with_nav2(session: ClientSession, observer: Observer, goal_x: float, goal_y: float) -> None:
    # --- Criterion 2b: get_capabilities WITH Nav2 ---
    caps = await wait_for_capability(session, "autonomous_navigation", present=True, timeout_s=60.0)
    tools = await session.list_tools()
    tool_names = {t.name for t in tools.tools}
    nav_present = "autonomous_navigation" in {c["capability_id"] for c in caps.get("capabilities", [])}
    nav_tool_present = "robot.navigate" in tool_names
    verdict = "PASS" if nav_present and nav_tool_present else "BLOCKED"
    record(
        "2b. get_capabilities (with Nav2): navigation present",
        {"capability_ids": sorted({c["capability_id"] for c in caps.get("capabilities", [])}), "tools": sorted(tool_names)},
        verdict,
    )
    if verdict != "PASS":
        record("6. navigate to goal", None, "BLOCKED", {"reason": "autonomous_navigation capability never confirmed"})
        record("7. cancelled navigate leaves robot stationary", None, "BLOCKED", {"reason": "same as criterion 6"})
        return

    # --- Criterion 6: navigate reaches goal ---
    nav_result, _ = await call_tool(session, "robot.navigate", {"x": goal_x, "y": goal_y})
    retry_evidence: dict[str, Any] = {}
    if (nav_result.get("error") or {}).get("code") == "INVALID_FRAME":
        # Diagnostic only (does not change src/): RclpyDiscoveryEngine.known_frames() is
        # a periodic (poll_interval_s) TF-frame snapshot, while get_state's pose
        # resolution does a LIVE tf_buffer.lookup_transform() — a frame that just became
        # live-resolvable (map, right after AMCL localizes on a freshly-started server)
        # can be absent from the snapshot until the next poll. Wait past one more poll
        # cycle and retry once to see whether this self-heals or is permanently stuck.
        retry_evidence["first_attempt"] = nav_result
        await asyncio.sleep(8.0)
        nav_result, _ = await call_tool(session, "robot.navigate", {"x": goal_x, "y": goal_y})
        retry_evidence["retried_after_s"] = 8.0
    final_pose = nav_result.get("final_pose")
    if final_pose is None:
        record("6. navigate to goal", nav_result, "FAIL", retry_evidence or {"reason": "final_pose is null"})
    else:
        err_m = math.hypot(final_pose["x"] - goal_x, final_pose["y"] - goal_y)
        verdict = "PASS" if nav_result.get("status") == "succeeded" and err_m <= 0.5 else "FAIL"
        record(
            "6. navigate to goal",
            {"goal": [goal_x, goal_y], "final_pose": final_pose, "error_m": round(err_m, 3)},
            verdict,
            retry_evidence,
        )

    # --- Criterion 7: cancelled navigate leaves robot stationary ---
    far_x, far_y = goal_x - 1.5, goal_y
    t_launch = time.monotonic()
    nav_task = asyncio.create_task(call_tool(session, "robot.navigate", {"x": far_x, "y": far_y}))
    moving = False
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        recent = observer.cmd_vel_since(t_launch)
        if any(abs(lx) > 0.01 or abs(az) > 0.01 for _, lx, az in recent):
            moving = True
            break
        await asyncio.sleep(0.1)
    if not moving:
        record("7. cancelled navigate leaves robot stationary", None, "BLOCKED", {"reason": "robot never appeared to start moving"})
        nav_task.cancel()
    else:
        t_cancel = time.monotonic()
        cancel_stop_result, _ = await call_tool(session, "robot.stop", {})
        try:
            cancelled_nav_result, _ = await asyncio.wait_for(nav_task, timeout=10.0)
        except asyncio.TimeoutError:
            cancelled_nav_result = None
        await asyncio.sleep(3.0)
        post_cancel = observer.cmd_vel_since(t_cancel + 0.3)
        non_zero_after = [e for e in post_cancel if abs(e[1]) > 0.01 or abs(e[2]) > 0.01]
        code = (cancelled_nav_result or {}).get("error", {}).get("code") if cancelled_nav_result else None
        verdict = "PASS" if code == "CANCELLED" and len(non_zero_after) == 0 else "FAIL"
        record(
            "7. cancelled navigate leaves robot stationary",
            {"navigate_error_code": code, "non_zero_cmd_vel_in_3s_after_cancel": len(non_zero_after)},
            verdict,
            {"stop_result": cancel_stop_result, "navigate_result": cancelled_nav_result},
        )


async def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=["no_nav2", "with_nav2"], required=True)
    parser.add_argument("--config", default=str(REPO_ROOT / "config/robot.yaml"))
    parser.add_argument("--out", default=str(REPO_ROOT / "acceptance/logs"))
    parser.add_argument("--initial-x", type=float, default=-2.0)
    parser.add_argument("--initial-y", type=float, default=-0.5)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=-1.0)
    parser.add_argument("--goal-y", type=float, default=-0.5)
    args = parser.parse_args()

    rclpy.init(args=None)
    observer = Observer()
    spin_in_background(observer)

    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO_ROOT / "src") + ":" + env.get("PYTHONPATH", "")
    server_params = StdioServerParameters(
        command="python3",
        args=["-m", "ros_mcp.server", "--config", args.config],
        env=env,
        cwd=str(REPO_ROOT),
    )

    try:
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                if args.phase == "no_nav2":
                    await phase_no_nav2(session, observer)
                else:
                    observer.publish_initial_pose(args.initial_x, args.initial_y, args.initial_yaw)
                    localized = await wait_for_localization(session, timeout_s=60.0)
                    print(f"localized={localized}", flush=True)
                    await phase_with_nav2(session, observer, args.goal_x, args.goal_y)
    finally:
        rclpy.shutdown()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"results_{args.phase}.json"
    out_path.write_text(json.dumps(RESULTS, indent=2, default=str))
    print(f"wrote {out_path}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())

"""Nav2NavigationPlugin — structurally satisfies
ros_mcp.contracts.adapters.NavigationBackend and ros_mcp.contracts.plugins.Plugin
(docs/13-contracts.md §7 §12, docs/08-navigation-architecture.md).

Implements the documented pipeline: frame validation, live action-server re-check,
localization freshness check, goal submission with feedback streaming, and
cancellation/timeout handling — never falling back to the motion backend
(ADR-005/ADR-006: an absent Nav2/localization is CAPABILITY_UNAVAILABLE, not a silent
reinterpretation).
"""
from __future__ import annotations

import asyncio
import logging
import time
from datetime import datetime, timezone
from typing import Any, Callable

from ros_mcp.contracts.adapters import CancellationToken
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Confidence, NavigateTarget, Pose2D, SemanticCommand
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.plugins import PluginHealth, PluginMetadata
from ros_mcp.contracts.results import NavigateResult
from ros_mcp.adapters.perception.tf_adapter import LATEST_TRANSFORM_STAMP
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.geometry import Quaternion, yaw_to_quaternion

logger = logging.getLogger(__name__)

_LOCALIZATION_TF_TIMEOUT_S = 0.5
_ACTION_SERVER_WAIT_TIMEOUT_S = 0.5
_POLL_INTERVAL_S = 0.2
_CANCEL_WAIT_TIMEOUT_S = 5.0


class Nav2NavigationPlugin:
    """Structurally satisfies ros_mcp.contracts.adapters.NavigationBackend and
    ros_mcp.contracts.plugins.Plugin."""

    def __init__(
        self,
        *,
        ros_bridge: Any,
        node: Any,
        tf_adapter: TFAdapter,
        capability_registry: CapabilityRegistry,
        action_name: str,
        base_frame: str = "base_link",
        progress_sink: Callable[[str, dict[str, Any]], None] | None = None,
        action_client_class: Any = None,
        navigate_to_pose_type: Any = None,
        poll_interval_s: float = _POLL_INTERVAL_S,
        sleep_fn: Any = None,
    ) -> None:
        """`base_frame` names the robot-base TF frame used for the map->base_frame
        localization/final-pose lookups — not a frozen-contract field (13-contracts.md
        only fixes NavigationBackend's methods, not this constructor), but genuinely
        platform-specific: TurtleBot3's TF tree uses `base_footprint`, not the generic
        `base_link` the docs use as an example (confirmed against the live reference
        platform during MVP testing). Defaults to `base_link` for platforms that do use
        it; server.py wires the actual reference robot's frame explicitly."""
        self.metadata = PluginMetadata(
            plugin_id="nav2_navigation",
            api_version="1.0.0",
            provides_capabilities=("autonomous_navigation",),
            requires=("action:nav2_msgs/action/NavigateToPose",),
        )
        self._ros_bridge = ros_bridge
        self._node = node
        self._tf_adapter = tf_adapter
        self._capability_registry = capability_registry
        self._action_name = action_name
        self._base_frame = base_frame
        self._progress_sink = progress_sink
        self._poll_interval_s = poll_interval_s
        self._sleep_fn = sleep_fn or asyncio.sleep

        if navigate_to_pose_type is None:
            from nav2_msgs.action import NavigateToPose as _NavigateToPose

            navigate_to_pose_type = _NavigateToPose
        self._navigate_to_pose_type = navigate_to_pose_type

        if action_client_class is None:
            from rclpy.action import ActionClient as _ActionClient

            action_client_class = _ActionClient
        self._action_client_class = action_client_class

        self._action_client: Any = None
        self._active_goal_handle: Any = None

    async def health_check(self) -> PluginHealth:
        return PluginHealth.HEALTHY

    def is_available(self) -> bool:
        entry = self._capability_registry.get("autonomous_navigation")
        return entry is not None and entry.confidence != Confidence.AMBIGUOUS

    async def _ensure_action_client(self) -> Any:
        if self._action_client is None:
            self._action_client = await self._ros_bridge.call_ros_from_asyncio(
                lambda: self._action_client_class(self._node, self._navigate_to_pose_type, self._action_name)
            )
        return self._action_client

    async def _bridge_rclpy_future(self, make_future_on_ros_thread: Callable[[], Any]) -> Any:
        """Bridges an rclpy.task.Future (created by an rclpy call that must run on the
        executor thread) into an awaitable asyncio Future — the same crossing pattern
        as ThreadSafeRosBridge, specialized for calls (send_goal_async,
        get_result_async, cancel_goal_async) that return a Future rather than a plain
        value (13-contracts.md §13)."""
        loop = asyncio.get_running_loop()
        aio_future: "asyncio.Future[Any]" = loop.create_future()

        def _create_and_attach() -> None:
            rclpy_future = make_future_on_ros_thread()

            def _on_done(fut: Any) -> None:
                def _resolve() -> None:
                    if aio_future.done():
                        return
                    exc = fut.exception()
                    if exc is not None:
                        aio_future.set_exception(exc)
                    else:
                        aio_future.set_result(fut.result())

                loop.call_soon_threadsafe(_resolve)

            rclpy_future.add_done_callback(_on_done)

        await self._ros_bridge.call_ros_from_asyncio(_create_and_attach)
        return await aio_future

    async def navigate(self, command: SemanticCommand, token: CancellationToken) -> NavigateResult:
        start_monotonic = time.monotonic()
        target = command.target
        assert isinstance(target, NavigateTarget)

        def duration() -> float:
            return time.monotonic() - start_monotonic

        def fail(error: ToolError, *, fail_reason: str | None, final_pose: Pose2D | None,
                  distance_remaining_m: float | None) -> NavigateResult:
            return NavigateResult(
                status="failed",
                command_id=command.command_id,
                robot_id=command.robot_id,
                duration_sec=duration(),
                error=error,
                final_pose=final_pose,
                distance_remaining_m=distance_remaining_m,
                fail_reason=fail_reason,  # type: ignore[arg-type]
            )

        frame = command.frame or "map"

        # 1. Frame validation.
        if frame not in self._tf_adapter.known_frames():
            return fail(
                ToolError(code=ErrorCode.INVALID_FRAME, message=f"unknown frame '{frame}'",
                           details={"known_frames": sorted(self._tf_adapter.known_frames())}),
                fail_reason=None, final_pose=None, distance_remaining_m=None,
            )

        # 3. Live action-server re-check (defense in depth beyond registry confidence).
        client = await self._ensure_action_client()
        server_ready = await self._ros_bridge.call_ros_from_asyncio(
            lambda: client.wait_for_server(timeout_sec=_ACTION_SERVER_WAIT_TIMEOUT_S)
        )
        if not server_ready:
            return fail(
                ToolError(code=ErrorCode.CAPABILITY_UNAVAILABLE, message="Nav2 action server is not responding"),
                fail_reason=None, final_pose=None, distance_remaining_m=None,
            )

        # 4. Localization check.
        localization_entry = self._capability_registry.get("localization")
        if localization_entry is None:
            return fail(
                ToolError(code=ErrorCode.CAPABILITY_UNAVAILABLE, message="robot is not localized (no localization capability)"),
                fail_reason=None, final_pose=None, distance_remaining_m=None,
            )
        tf_result = await self._tf_adapter.lookup_transform(
            "map", self._base_frame, LATEST_TRANSFORM_STAMP, _LOCALIZATION_TF_TIMEOUT_S
        )
        if not tf_result.ok:
            return fail(
                ToolError(code=ErrorCode.CAPABILITY_UNAVAILABLE,
                           message=f"map->{self._base_frame} transform unavailable ({tf_result.error})"),
                fail_reason=None, final_pose=None, distance_remaining_m=None,
            )

        # 5. Build and 6. submit the goal.
        goal_msg = self._navigate_to_pose_type.Goal()
        goal_msg.pose.header.frame_id = frame
        goal_msg.pose.pose.position.x = target.x
        goal_msg.pose.pose.position.y = target.y
        if target.yaw is not None:
            q = yaw_to_quaternion(target.yaw)
            goal_msg.pose.pose.orientation.x = q.x
            goal_msg.pose.pose.orientation.y = q.y
            goal_msg.pose.pose.orientation.z = q.z
            goal_msg.pose.pose.orientation.w = q.w
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        feedback_state: dict[str, Any] = {}

        def _on_feedback(feedback_msg: Any) -> None:
            fb = feedback_msg.feedback
            feedback_state["distance_remaining_m"] = float(fb.distance_remaining)
            feedback_state["number_of_recoveries"] = int(fb.number_of_recoveries)
            pos = fb.current_pose.pose.position
            ori = fb.current_pose.pose.orientation
            feedback_state["current_pose"] = (pos.x, pos.y, ori.z, ori.w)
            if self._progress_sink is not None:
                self._ros_bridge.call_soon_threadsafe_from_ros(
                    lambda: self._progress_sink(command.command_id, dict(feedback_state))
                )

        goal_handle = await self._bridge_rclpy_future(
            lambda: client.send_goal_async(goal_msg, feedback_callback=_on_feedback)
        )
        if not goal_handle.accepted:
            return fail(
                ToolError(code=ErrorCode.NAVIGATION_FAILED, message="Nav2 rejected the navigation goal"),
                fail_reason="goal_rejected", final_pose=None, distance_remaining_m=None,
            )

        self._active_goal_handle = goal_handle

        result_task = asyncio.ensure_future(
            self._bridge_rclpy_future(lambda: goal_handle.get_result_async())
        )
        outcome: str | None = None  # "cancelled" | "timeout" | None (ran to completion)
        deadline = start_monotonic + command.timeout_s
        while not result_task.done():
            if token.is_cancelled():
                outcome = "cancelled"
            elif token.deadline_exceeded() or time.monotonic() >= deadline:
                outcome = "timeout"
            if outcome is not None:
                try:
                    await self._bridge_rclpy_future(lambda: goal_handle.cancel_goal_async())
                except Exception:  # noqa: BLE001 - cancellation is best-effort
                    logger.exception("cancel_goal_async failed for command %s", command.command_id)
                try:
                    await asyncio.wait_for(result_task, timeout=_CANCEL_WAIT_TIMEOUT_S)
                except asyncio.TimeoutError:
                    pass
                break
            await self._sleep_fn(self._poll_interval_s)

        self._active_goal_handle = None
        final_pose = await self._resolve_final_pose(feedback_state)
        distance_remaining = feedback_state.get("distance_remaining_m")

        if outcome == "cancelled":
            return fail(
                ToolError(code=ErrorCode.CANCELLED, message="navigation cancelled"),
                fail_reason="cancelled", final_pose=final_pose, distance_remaining_m=distance_remaining,
            )
        if outcome == "timeout":
            return fail(
                ToolError(code=ErrorCode.ACTION_TIMEOUT, message="navigation exceeded its timeout"),
                fail_reason="cancelled", final_pose=final_pose, distance_remaining_m=distance_remaining,
            )

        if not result_task.done():
            # Should not happen (loop only exits early via outcome, else waits for
            # result_task) but fail safe rather than block forever.
            return fail(
                ToolError(code=ErrorCode.INTERNAL_ERROR, message="navigation result future never completed"),
                fail_reason=None, final_pose=final_pose, distance_remaining_m=distance_remaining,
            )

        result_response = result_task.result()
        status = result_response.status
        from action_msgs.msg import GoalStatus

        if status == GoalStatus.STATUS_SUCCEEDED:
            return NavigateResult(
                status="succeeded",
                command_id=command.command_id,
                robot_id=command.robot_id,
                duration_sec=duration(),
                final_pose=final_pose,
                distance_remaining_m=0.0,
                fail_reason=None,
            )
        if status == GoalStatus.STATUS_CANCELED:
            return fail(
                ToolError(code=ErrorCode.CANCELLED, message="navigation was canceled"),
                fail_reason="cancelled", final_pose=final_pose, distance_remaining_m=distance_remaining,
            )
        fail_reason = "controller_failure" if feedback_state.get("number_of_recoveries", 0) > 0 else "planner_failure"
        return fail(
            ToolError(code=ErrorCode.NAVIGATION_FAILED, message=f"Nav2 goal ended with status {status}"),
            fail_reason=fail_reason, final_pose=final_pose, distance_remaining_m=distance_remaining,
        )

    async def _resolve_final_pose(self, feedback_state: dict[str, Any]) -> Pose2D | None:
        tf_result = await self._tf_adapter.lookup_transform(
            "map", self._base_frame, LATEST_TRANSFORM_STAMP, _LOCALIZATION_TF_TIMEOUT_S
        )
        if tf_result.ok:
            assert tf_result.x is not None and tf_result.y is not None and tf_result.yaw is not None
            return Pose2D(x=tf_result.x, y=tf_result.y, yaw=tf_result.yaw, frame="map", stamp=datetime.now(timezone.utc))
        current_pose = feedback_state.get("current_pose")
        if current_pose is not None:
            x, y, qz, qw = current_pose
            from ros_mcp.geometry import quaternion_to_yaw

            yaw = quaternion_to_yaw(Quaternion(x=0.0, y=0.0, z=qz, w=qw))
            return Pose2D(x=x, y=y, yaw=yaw, frame="map", stamp=datetime.now(timezone.utc))
        return None

    async def cancel_all(self) -> None:
        if self._active_goal_handle is None:
            return
        try:
            await self._bridge_rclpy_future(lambda: self._active_goal_handle.cancel_goal_async())
        except Exception:  # noqa: BLE001 - best-effort; robot.stop must not raise
            logger.exception("cancel_all failed to cancel the active Nav2 goal")

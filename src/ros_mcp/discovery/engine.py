"""RclpyDiscoveryEngine — structurally satisfies ros_mcp.contracts.discovery.DiscoveryEngine
(docs/13-contracts.md §3).

Owns the startup + periodic re-poll of the live ROS graph. All actual rclpy graph calls
are dispatched through the injected `RosBridge.call_ros_from_asyncio` (ADR-011); this
class itself never touches `rclpy` synchronously from the asyncio loop.
"""
from __future__ import annotations

import asyncio
import logging
from datetime import datetime, timezone
from typing import Any, Callable

from ros_mcp.contracts.discovery import GraphSnapshot
from ros_mcp.discovery.snapshot import collect_graph_snapshot

logger = logging.getLogger(__name__)


def _content_equal(a: GraphSnapshot, b: GraphSnapshot) -> bool:
    """Compares everything except snapshot_version/taken_at, which change every poll
    even when the graph itself hasn't (03-capability-discovery.md: re-publish tool list
    only when the capability-relevant content actually changes)."""
    return (
        a.nodes == b.nodes
        and a.topics == b.topics
        and a.services == b.services
        and a.actions == b.actions
        and a.tf_frames == b.tf_frames
    )


class RclpyDiscoveryEngine:
    """Structurally satisfies ros_mcp.contracts.discovery.DiscoveryEngine."""

    def __init__(
        self,
        *,
        ros_bridge: Any,
        node: Any,
        tf_buffer: Any = None,
        poll_interval_s: float = 5.0,
        action_names_and_types_fn: Any = None,
        action_server_names_and_types_by_node_fn: Any = None,
    ) -> None:
        """`action_names_and_types_fn`/`action_server_names_and_types_by_node_fn` are a
        dependency-injection seam (not part of the frozen DiscoveryEngine Protocol,
        which only requires get_current_snapshot/on_snapshot_changed) so unit tests can
        supply fakes instead of the real rclpy.action graph calls, which require a live
        rclpy Node handle. Default to the real rclpy.action functions."""
        self._bridge = ros_bridge
        self._node = node
        self._tf_buffer = tf_buffer
        self._poll_interval_s = poll_interval_s
        self._action_names_and_types_fn = action_names_and_types_fn
        self._action_server_names_and_types_by_node_fn = action_server_names_and_types_by_node_fn
        self._latest = GraphSnapshot(
            snapshot_version=0,
            taken_at=datetime.now(timezone.utc),
            nodes=(),
            topics=(),
            services=(),
            actions=(),
            tf_frames=(),
        )
        self._callbacks: list[Callable[[GraphSnapshot], None]] = []
        self._poll_task: asyncio.Task[None] | None = None

    async def get_current_snapshot(self) -> GraphSnapshot:
        return self._latest

    def on_snapshot_changed(self, callback: Callable[[GraphSnapshot], None]) -> None:
        self._callbacks.append(callback)

    async def refresh_once(self) -> GraphSnapshot:
        """Performs exactly one discovery poll, dispatched to the rclpy executor thread,
        and updates the cached snapshot. Public so start-up can await the first result
        before the server reports readiness."""
        prev_version = self._latest.snapshot_version
        new_snapshot = await self._bridge.call_ros_from_asyncio(
            lambda: collect_graph_snapshot(
                self._node,
                tf_buffer=self._tf_buffer,
                prev_version=prev_version,
                action_names_and_types_fn=self._action_names_and_types_fn,
                action_server_names_and_types_by_node_fn=self._action_server_names_and_types_by_node_fn,
            )
        )
        changed = not _content_equal(self._latest, new_snapshot)
        self._latest = new_snapshot
        if changed:
            for callback in self._callbacks:
                try:
                    callback(new_snapshot)
                except Exception:  # noqa: BLE001 - one bad subscriber must not break discovery
                    logger.exception("on_snapshot_changed callback raised")
        return new_snapshot

    async def start(self) -> None:
        """Runs the first poll synchronously (so capabilities are populated before the
        server reports ready) then schedules periodic re-polling."""
        await self.refresh_once()
        if self._poll_task is None:
            self._poll_task = asyncio.create_task(self._poll_loop(), name="discovery-poll")

    async def stop(self) -> None:
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
            self._poll_task = None

    async def _poll_loop(self) -> None:
        while True:
            await asyncio.sleep(self._poll_interval_s)
            try:
                await self.refresh_once()
            except Exception:  # noqa: BLE001 - a failed poll must not kill the loop
                logger.exception("discovery poll failed")

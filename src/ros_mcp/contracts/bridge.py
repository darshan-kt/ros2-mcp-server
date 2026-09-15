"""Async/threading boundary contract — frozen by docs/13-contracts.md §13.

This is the ONLY permitted crossing point between the rclpy executor thread and the
MCP/asyncio event loop (ADR-011). No module outside `ros_mcp.ros` may call a blocking
rclpy API directly, and no rclpy callback may await or call asyncio code directly.
"""
from __future__ import annotations

from typing import Any, Callable, Protocol


class RosBridge(Protocol):
    def call_soon_threadsafe_from_ros(self, fn: Callable[[], None]) -> None:
        """Used by rclpy callbacks to hand data to the asyncio loop
        (loop.call_soon_threadsafe)."""
        ...

    async def call_ros_from_asyncio(self, fn: Callable[[], Any]) -> Any:
        """Used by asyncio-side code that must invoke a synchronous rclpy call
        (e.g. create_publisher, send_goal_async's underlying future) — dispatched onto
        the rclpy executor thread and awaited via a bridged future. Never calls rclpy
        APIs directly from the asyncio thread."""
        ...

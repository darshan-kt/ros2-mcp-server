"""RosBridge implementation — the sole legal crossing point between the rclpy executor
thread and the MCP asyncio loop (docs/13-contracts.md §13, ADR-011)."""
from __future__ import annotations

import asyncio
from typing import Any, Callable

from ros_mcp.ros.executor_thread import RclpyExecutorThread


class ThreadSafeRosBridge:
    """Structurally satisfies ros_mcp.contracts.bridge.RosBridge."""

    def __init__(self, executor_thread: RclpyExecutorThread, loop: asyncio.AbstractEventLoop) -> None:
        self._executor_thread = executor_thread
        self._loop = loop

    def call_soon_threadsafe_from_ros(self, fn: Callable[[], None]) -> None:
        self._loop.call_soon_threadsafe(fn)

    async def call_ros_from_asyncio(self, fn: Callable[[], Any]) -> Any:
        future: asyncio.Future[Any] = self._loop.create_future()

        def _run_on_ros_thread() -> None:
            try:
                result = fn()
            except Exception as exc:  # noqa: BLE001 - deliberately broad: bridged to caller
                self._loop.call_soon_threadsafe(_set_exception_if_pending, future, exc)
            else:
                self._loop.call_soon_threadsafe(_set_result_if_pending, future, result)

        self._executor_thread.submit(_run_on_ros_thread)
        return await future


def _set_result_if_pending(future: asyncio.Future[Any], result: Any) -> None:
    if not future.done():
        future.set_result(result)


def _set_exception_if_pending(future: asyncio.Future[Any], exc: Exception) -> None:
    if not future.done():
        future.set_exception(exc)

"""Dedicated rclpy executor thread (ADR-011, 13-contracts.md §13).

One OS thread spins a `SingleThreadedExecutor` (MVP's callback count does not warrant
`MultiThreadedExecutor` — see docs/19-deployment-and-scalability.md) and, between spin
iterations, drains a work queue of callables submitted from the asyncio side via
`RosBridge.call_ros_from_asyncio`. This is the only thread that ever touches `rclpy`
Node/publisher/subscription/action-client state directly.
"""
from __future__ import annotations

import queue
import threading
from typing import Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

_SPIN_TIMEOUT_S = 0.05


class RclpyExecutorThread:
    """Owns the rclpy node + executor and the single OS thread that spins them."""

    def __init__(self, node: Node, executor: SingleThreadedExecutor | None = None) -> None:
        self._node = node
        self._executor = executor or SingleThreadedExecutor()
        self._executor.add_node(node)
        self._work_queue: "queue.SimpleQueue[Callable[[], None]]" = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def node(self) -> Node:
        return self._node

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="ros-mcp-rclpy-executor", daemon=True
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None

    def submit(self, fn: Callable[[], None]) -> None:
        """Enqueue a callable to run on the executor thread at the next spin iteration.
        Non-blocking; the callable is responsible for reporting its own result/exception
        back across the RosBridge (see ros_mcp.ros.bridge.ThreadSafeRosBridge)."""
        self._work_queue.put(fn)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._executor.spin_once(timeout_sec=_SPIN_TIMEOUT_S)
            self._drain_work_queue()
        self._drain_work_queue()

    def _drain_work_queue(self) -> None:
        while True:
            try:
                fn = self._work_queue.get_nowait()
            except queue.Empty:
                return
            fn()

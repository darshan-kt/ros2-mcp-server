"""InMemoryCapabilityRegistry — structurally satisfies
ros_mcp.contracts.capabilities.CapabilityRegistry (docs/13-contracts.md §3).

Single-writer (the CapabilityCoordinator, via `update()`), safe-to-read-from-any-thread
by always handing out an already-built immutable tuple snapshot rather than a live
mutable structure (ADR-011: no shared mutable dict crossing the rclpy/asyncio boundary).
"""
from __future__ import annotations

import threading
from typing import Callable

from ros_mcp.contracts.capabilities import CapabilityEntry


class InMemoryCapabilityRegistry:
    """Structurally satisfies ros_mcp.contracts.capabilities.CapabilityRegistry."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._entries: tuple[CapabilityEntry, ...] = ()
        self._version = 0
        self._callbacks: list[Callable[[tuple[CapabilityEntry, ...]], None]] = []

    def current(self) -> tuple[CapabilityEntry, ...]:
        with self._lock:
            return self._entries

    def get(self, capability_id: str) -> CapabilityEntry | None:
        with self._lock:
            for entry in self._entries:
                if entry.capability_id == capability_id:
                    return entry
        return None

    def snapshot_version(self) -> int:
        with self._lock:
            return self._version

    def on_changed(self, callback: Callable[[tuple[CapabilityEntry, ...]], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def update(self, entries: tuple[CapabilityEntry, ...]) -> bool:
        """Not part of the frozen CapabilityRegistry Protocol — the write-side seam used
        only by CapabilityCoordinator. Returns True if the entry set actually changed."""
        with self._lock:
            changed = entries != self._entries
            if changed:
                self._entries = entries
                self._version += 1
            callbacks = list(self._callbacks)
        if changed:
            for callback in callbacks:
                callback(entries)
        return changed

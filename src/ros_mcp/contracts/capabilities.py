"""Capability contracts — frozen by docs/13-contracts.md §3 (capability half)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Protocol

from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.discovery import GraphSnapshot

if TYPE_CHECKING:
    from ros_mcp.contracts.config import CapabilityConfig


@dataclass(frozen=True)
class CapabilityEntry:
    capability_id: str
    confidence: Confidence
    backend_id: str | None
    resolved_topics: dict[str, str] = field(default_factory=dict)
    limits: dict[str, float] = field(default_factory=dict)


class CapabilityRegistry(Protocol):
    def current(self) -> tuple[CapabilityEntry, ...]:
        """Immutable snapshot, safe to call from any thread."""
        ...

    def get(self, capability_id: str) -> CapabilityEntry | None: ...

    def snapshot_version(self) -> int: ...

    def on_changed(self, callback: Callable[[tuple[CapabilityEntry, ...]], None]) -> None: ...


class CapabilityInferenceEngine(Protocol):
    def infer(
        self, snapshot: GraphSnapshot, overrides: "CapabilityConfig"
    ) -> tuple[CapabilityEntry, ...]:
        """Pure function: graph snapshot + config overrides -> capability entries.
        No I/O; safe to unit test with a synthetic GraphSnapshot."""
        ...

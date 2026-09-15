"""Discovery & capability contracts — frozen by docs/13-contracts.md §3 (discovery half)."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Protocol


@dataclass(frozen=True)
class TopicInfo:
    name: str
    type_name: str
    publishers: tuple[str, ...]
    subscribers: tuple[str, ...]
    qos_profiles: tuple[dict[str, Any], ...]
    measured_hz: float | None
    last_stamp: datetime | None
    frame_id: str | None


@dataclass(frozen=True)
class ServiceInfo:
    name: str
    type_name: str


@dataclass(frozen=True)
class ActionInfo:
    name: str
    type_name: str


@dataclass(frozen=True)
class NodeInfo:
    name: str
    namespace: str
    publishers: tuple[str, ...]
    subscribers: tuple[str, ...]
    services: tuple[str, ...]
    clients: tuple[str, ...]
    actions: tuple[str, ...]


@dataclass(frozen=True)
class GraphSnapshot:
    snapshot_version: int
    taken_at: datetime
    nodes: tuple[NodeInfo, ...]
    topics: tuple[TopicInfo, ...]
    services: tuple[ServiceInfo, ...]
    actions: tuple[ActionInfo, ...]
    tf_frames: tuple[str, ...]


class DiscoveryEngine(Protocol):
    async def get_current_snapshot(self) -> GraphSnapshot:
        """Return the latest snapshot; never triggers a blocking ROS call itself —
        reads from the rclpy-side cache updated by the discovery timer."""
        ...

    def on_snapshot_changed(self, callback: Callable[[GraphSnapshot], None]) -> None:
        """Register a callback invoked (on the asyncio loop, via the thread-safe bridge)
        whenever a new snapshot differs from the previous one."""
        ...

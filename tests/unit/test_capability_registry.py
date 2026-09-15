"""Unit tests for InMemoryCapabilityRegistry and CapabilityCoordinator wiring."""
from __future__ import annotations

import asyncio

from ros_mcp.capabilities.coordinator import CapabilityCoordinator
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import CapabilityConfig
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.discovery import GraphSnapshot
from datetime import datetime, timezone


def _entry(cap_id: str) -> CapabilityEntry:
    return CapabilityEntry(capability_id=cap_id, confidence=Confidence.CONFIRMED, backend_id="x")


def test_registry_update_increments_version_and_fires_callback():
    registry = InMemoryCapabilityRegistry()
    seen = []
    registry.on_changed(seen.append)

    assert registry.snapshot_version() == 0
    changed = registry.update((_entry("a"),))
    assert changed is True
    assert registry.snapshot_version() == 1
    assert registry.current() == (_entry("a"),)
    assert seen == [(_entry("a"),)]


def test_registry_update_with_identical_entries_is_a_noop():
    registry = InMemoryCapabilityRegistry()
    registry.update((_entry("a"),))
    changed = registry.update((_entry("a"),))
    assert changed is False
    assert registry.snapshot_version() == 1


def test_registry_get_returns_none_for_unknown_capability():
    registry = InMemoryCapabilityRegistry()
    registry.update((_entry("a"),))
    assert registry.get("a") is not None
    assert registry.get("nonexistent") is None


_EMPTY_SNAPSHOT = GraphSnapshot(
    snapshot_version=1,
    taken_at=datetime.now(timezone.utc),
    nodes=(),
    topics=(),
    services=(),
    actions=(),
    tf_frames=(),
)


class FakeDiscoveryEngine:
    def __init__(self, snapshot: GraphSnapshot) -> None:
        self._snapshot = snapshot
        self._callback = None

    async def get_current_snapshot(self) -> GraphSnapshot:
        return self._snapshot

    def on_snapshot_changed(self, callback):
        self._callback = callback

    def fire(self, snapshot: GraphSnapshot) -> None:
        self._snapshot = snapshot
        assert self._callback is not None
        self._callback(snapshot)


class FakeInferenceEngine:
    def infer(self, snapshot, overrides):
        if snapshot.topics:
            return (_entry("differential_drive_motion"),)
        return ()


def test_coordinator_updates_registry_on_snapshot_change():
    discovery = FakeDiscoveryEngine(_EMPTY_SNAPSHOT)
    registry = InMemoryCapabilityRegistry()
    coordinator = CapabilityCoordinator(
        discovery, FakeInferenceEngine(), registry, lambda: CapabilityConfig()
    )
    assert registry.current() == ()

    from ros_mcp.contracts.discovery import TopicInfo

    new_snapshot = GraphSnapshot(
        snapshot_version=2,
        taken_at=datetime.now(timezone.utc),
        nodes=(),
        topics=(
            TopicInfo(
                name="/cmd_vel",
                type_name="geometry_msgs/msg/Twist",
                publishers=(),
                subscribers=(),
                qos_profiles=(),
                measured_hz=None,
                last_stamp=None,
                frame_id=None,
            ),
        ),
        services=(),
        actions=(),
        tf_frames=(),
    )
    discovery.fire(new_snapshot)
    assert registry.current() == (_entry("differential_drive_motion"),)


def test_coordinator_refresh_from_current_snapshot_at_startup():
    async def scenario():
        discovery = FakeDiscoveryEngine(_EMPTY_SNAPSHOT)
        registry = InMemoryCapabilityRegistry()
        coordinator = CapabilityCoordinator(
            discovery, FakeInferenceEngine(), registry, lambda: CapabilityConfig()
        )
        await coordinator.refresh_from_current_snapshot()
        return registry.current()

    result = asyncio.run(scenario())
    assert result == ()  # empty snapshot -> FakeInferenceEngine returns no entries

"""Unit tests for RclpyTFAdapter — every tf2 failure mode maps to a TransformResult
value, never a raised exception across the adapter boundary (docs/13-contracts.md §8)."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from types import SimpleNamespace

import tf2_ros

from ros_mcp.adapters.perception.tf_adapter import RclpyTFAdapter
from ros_mcp.contracts.discovery import GraphSnapshot


class FakeBridge:
    async def call_ros_from_asyncio(self, fn):
        return fn()


def _fake_transform_message() -> SimpleNamespace:
    translation = SimpleNamespace(x=1.5, y=2.5, z=0.0)
    rotation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    return SimpleNamespace(transform=SimpleNamespace(translation=translation, rotation=rotation))


class FakeBuffer:
    def __init__(self, mode: str = "ok") -> None:
        self._mode = mode

    def lookup_transform(self, target_frame, source_frame, time, timeout):
        if self._mode == "ok":
            return _fake_transform_message()
        if self._mode == "lookup":
            raise tf2_ros.LookupException("no such frame")
        if self._mode == "connectivity":
            raise tf2_ros.ConnectivityException("not connected")
        if self._mode == "extrapolation":
            raise tf2_ros.ExtrapolationException("extrapolation into the future")
        raise tf2_ros.TimeoutException("timed out")


def _run(coro):
    return asyncio.run(coro)


def test_successful_lookup_returns_xy_yaw():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("ok"))
    result = _run(adapter.lookup_transform("map", "base_link", datetime.now(timezone.utc), 1.0))
    assert result.ok is True
    assert result.x == 1.5
    assert result.y == 2.5
    assert result.error is None


def test_lookup_exception_maps_to_frame_unknown():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("lookup"))
    result = _run(adapter.lookup_transform("map", "nope", datetime.now(timezone.utc), 1.0))
    assert result.ok is False
    assert result.error == "frame_unknown"


def test_connectivity_exception_maps_to_not_connected():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("connectivity"))
    result = _run(adapter.lookup_transform("map", "base_link", datetime.now(timezone.utc), 1.0))
    assert result.error == "not_connected"


def test_extrapolation_exception_maps_to_extrapolation():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("extrapolation"))
    result = _run(adapter.lookup_transform("map", "base_link", datetime.now(timezone.utc), 1.0))
    assert result.error == "extrapolation"


def test_timeout_exception_maps_to_timeout():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("timeout"))
    result = _run(adapter.lookup_transform("map", "base_link", datetime.now(timezone.utc), 1.0))
    assert result.error == "timeout"


def test_known_frames_from_graph_snapshot_provider():
    snapshot = GraphSnapshot(
        snapshot_version=1, taken_at=datetime.now(timezone.utc), nodes=(), topics=(),
        services=(), actions=(), tf_frames=("map", "odom", "base_link"),
    )
    adapter = RclpyTFAdapter(
        ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("ok"), graph_snapshot_provider=lambda: snapshot
    )
    assert adapter.known_frames() == frozenset({"map", "odom", "base_link"})


def test_known_frames_empty_without_provider():
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=FakeBuffer("ok"))
    assert adapter.known_frames() == frozenset()

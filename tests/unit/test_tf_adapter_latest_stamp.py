"""Regression test: querying TF at `datetime.now(timezone.utc)` (real wall-clock) fails
with ExtrapolationException against a sim-time-stamped buffer (`use_sim_time`), which is
exactly the MVP reference platform's normal operating mode — confirmed against the live
TurtleBot3/Nav2 stack during MVP testing. `LATEST_TRANSFORM_STAMP` must map to a zero
rclpy Time() ("give me whatever you have"), not an attempted conversion of the sentinel
itself, and any other stamp must still convert normally."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from types import SimpleNamespace

from ros_mcp.adapters.perception.tf_adapter import LATEST_TRANSFORM_STAMP, RclpyTFAdapter


class FakeBridge:
    async def call_ros_from_asyncio(self, fn):
        return fn()


class RecordingBuffer:
    def __init__(self) -> None:
        self.seen_times: list[object] = []

    def lookup_transform(self, target_frame, source_frame, time, timeout):
        self.seen_times.append(time)
        translation = SimpleNamespace(x=1.0, y=2.0, z=0.0)
        rotation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        return SimpleNamespace(transform=SimpleNamespace(translation=translation, rotation=rotation))


def test_latest_transform_stamp_maps_to_zero_rclpy_time():
    from rclpy.time import Time

    buffer = RecordingBuffer()
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=buffer)

    result = asyncio.run(
        adapter.lookup_transform("map", "base_link", LATEST_TRANSFORM_STAMP, 1.0)
    )
    assert result.ok is True
    assert len(buffer.seen_times) == 1
    seen = buffer.seen_times[0]
    assert isinstance(seen, Time)
    assert seen.seconds_nanoseconds() == (0, 0)


def test_specific_stamp_converts_to_matching_rclpy_time():
    from rclpy.time import Time

    buffer = RecordingBuffer()
    adapter = RclpyTFAdapter(ros_bridge=FakeBridge(), tf_buffer=buffer)

    specific = datetime(2024, 1, 1, tzinfo=timezone.utc)
    asyncio.run(adapter.lookup_transform("map", "base_link", specific, 1.0))

    seen = buffer.seen_times[0]
    assert isinstance(seen, Time)
    sec, _ = seen.seconds_nanoseconds()
    assert sec == int(specific.timestamp())

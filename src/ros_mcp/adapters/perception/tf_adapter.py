"""RclpyTFAdapter — structurally satisfies ros_mcp.contracts.tf.TFAdapter
(docs/13-contracts.md §8, docs/13-contracts.md §13 async boundary).

Wraps one shared `tf2_ros.Buffer` for the whole process (docs/02-ros2-architecture.md
TF Adapter section: "not one per adapter"); the lookup itself always runs through
`RosBridge.call_ros_from_asyncio` since `tf2_ros.Buffer.lookup_transform` is a
blocking call. Never raises a tf2 exception across the adapter boundary — every
failure mode becomes a `TransformResult.error` value.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Callable

from ros_mcp.contracts.discovery import GraphSnapshot
from ros_mcp.contracts.tf import TransformResult
from ros_mcp.geometry import Quaternion, quaternion_to_yaw

# Sentinel `stamp` value meaning "the latest available transform" rather than a specific
# instant — the epoch-zero `datetime` maps to rclpy's zero `Time()`, which tf2 already
# treats as "give me the most recent transform you have" (its own conventional
# shorthand for "now"). Callers that want "now" MUST use this, not
# `datetime.now(timezone.utc)`: with `use_sim_time` (true for the MVP reference sim),
# TF data is stamped in simulation time, which can be far from wall-clock "now" — a
# wall-clock stamp reliably produces `ExtrapolationException` ("extrapolation") against
# a sim-time-stamped buffer. Only pass an actual non-sentinel `stamp` when you have one
# genuinely drawn from the same (sim or wall) clock the TF data itself uses — e.g. a
# cached sensor message's own `header.stamp` (ros_mcp.adapters.perception.perception_adapter).
LATEST_TRANSFORM_STAMP = datetime.fromtimestamp(0, tz=timezone.utc)


class RclpyTFAdapter:
    """Structurally satisfies ros_mcp.contracts.tf.TFAdapter."""

    def __init__(
        self,
        *,
        ros_bridge: Any,
        tf_buffer: Any,
        graph_snapshot_provider: Callable[[], GraphSnapshot] | None = None,
    ) -> None:
        self._ros_bridge = ros_bridge
        self._tf_buffer = tf_buffer
        self._graph_snapshot_provider = graph_snapshot_provider

    async def lookup_transform(
        self, target_frame: str, source_frame: str, stamp: datetime, timeout_s: float
    ) -> TransformResult:
        def _do_lookup() -> TransformResult:
            import tf2_ros
            from rclpy.duration import Duration
            from rclpy.time import Time

            try:
                ros_time = Time() if stamp == LATEST_TRANSFORM_STAMP else Time(seconds=stamp.timestamp())
                transform = self._tf_buffer.lookup_transform(
                    target_frame, source_frame, ros_time, timeout=Duration(seconds=timeout_s)
                )
            except tf2_ros.LookupException:
                return TransformResult(ok=False, x=None, y=None, yaw=None, error="frame_unknown")
            except tf2_ros.ConnectivityException:
                return TransformResult(ok=False, x=None, y=None, yaw=None, error="not_connected")
            except tf2_ros.ExtrapolationException:
                return TransformResult(ok=False, x=None, y=None, yaw=None, error="extrapolation")
            except tf2_ros.TransformException:
                # Covers tf2_ros.TimeoutException and any other transform failure not
                # more specifically classified above — "timeout" is the closest of the
                # four frozen error values for an unspecified transform failure.
                return TransformResult(ok=False, x=None, y=None, yaw=None, error="timeout")

            t = transform.transform.translation
            r = transform.transform.rotation
            yaw = quaternion_to_yaw(Quaternion(x=r.x, y=r.y, z=r.z, w=r.w))
            return TransformResult(ok=True, x=t.x, y=t.y, yaw=yaw, error=None)

        return await self._ros_bridge.call_ros_from_asyncio(_do_lookup)

    def known_frames(self) -> frozenset[str]:
        if self._graph_snapshot_provider is None:
            return frozenset()
        return frozenset(self._graph_snapshot_provider().tf_frames)

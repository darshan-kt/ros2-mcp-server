"""PooledSubscriptionManager — structurally satisfies
ros_mcp.contracts.subscriptions.SubscriptionManager (docs/13-contracts.md §9, ADR-008).

`ensure_subscribed` is frozen as a *synchronous* method, yet creating an rclpy
subscription is a blocking rclpy call that, per the Async/Threading Boundary contract
(13-contracts.md §13), must never be invoked directly from the asyncio thread. This is
resolved by having the synchronous method mark the topic as claimed immediately
(idempotency, no duplicate subscriptions even under concurrent callers) and then
schedule the actual rclpy call as a fire-and-forget asyncio task that awaits
`RosBridge.call_ros_from_asyncio` — the same legal crossing point every other
asyncio-side ROS call uses, just not awaited by `ensure_subscribed` itself since its
signature has no `async`/return value to await. `get_latest` remains fully non-blocking
and returns None until that task completes and the first message arrives, which is
within the contract ("Returns None if never received").
"""
from __future__ import annotations

import asyncio
import logging
import threading
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any

from ros_mcp.codec.message_codec import RosidlMessageCodec
from ros_mcp.contracts.subscriptions import CachedMessage, QosPolicy

logger = logging.getLogger(__name__)

_SENSOR_MSGS_PREFIX = "sensor_msgs/"


def _extract_stamp(msg: Any) -> datetime | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return datetime.fromtimestamp(sec + nanosec / 1e9, tz=timezone.utc)


class PooledSubscriptionManager:
    """Structurally satisfies ros_mcp.contracts.subscriptions.SubscriptionManager."""

    def __init__(
        self,
        *,
        ros_bridge: Any,
        node: Any,
        codec: RosidlMessageCodec | None = None,
        default_element_limit: int = 4096,
        get_message_fn: Any = None,
    ) -> None:
        self._ros_bridge = ros_bridge
        self._node = node
        self._codec = codec or RosidlMessageCodec()
        self._default_element_limit = default_element_limit
        if get_message_fn is None:
            from rosidl_runtime_py.utilities import get_message as _get_message

            get_message_fn = _get_message
        self._get_message_fn = get_message_fn

        self._lock = threading.Lock()
        self._claimed_topics: set[str] = set()
        self._cache: dict[str, CachedMessage] = {}
        self._pending_tasks: set[asyncio.Task[None]] = set()

    def ensure_subscribed(
        self, topic: str, type_name: str, *, qos: QosPolicy | None = None
    ) -> None:
        with self._lock:
            if topic in self._claimed_topics:
                return
            self._claimed_topics.add(topic)

        async def _do_subscribe() -> None:
            try:
                await self._ros_bridge.call_ros_from_asyncio(
                    lambda: self._create_subscription_sync(topic, type_name, qos)
                )
            except Exception:  # noqa: BLE001 - a failed subscribe must not crash the caller
                logger.exception("failed to subscribe to %s (%s)", topic, type_name)
                with self._lock:
                    self._claimed_topics.discard(topic)

        loop = asyncio.get_running_loop()
        task = loop.create_task(_do_subscribe(), name=f"subscribe:{topic}")
        self._pending_tasks.add(task)
        task.add_done_callback(self._pending_tasks.discard)

    def _create_subscription_sync(self, topic: str, type_name: str, qos: QosPolicy | None) -> None:
        """Runs on the rclpy executor thread (dispatched via call_ros_from_asyncio)."""
        msg_class = self._get_message_fn(type_name)
        rclpy_qos = self._resolve_qos(type_name, qos)

        def _on_message(msg: Any) -> None:
            converted = self._codec.to_dict(msg, element_limit=self._default_element_limit)
            stamp = _extract_stamp(msg) or datetime.now(timezone.utc)
            cached = CachedMessage(
                value=converted,
                raw=msg,
                stamp=stamp,
                received_at=datetime.now(timezone.utc),
                age_s=0.0,
            )
            with self._lock:
                self._cache[topic] = cached

        self._node.create_subscription(msg_class, topic, _on_message, rclpy_qos)

    def _resolve_qos(self, type_name: str, qos: QosPolicy | None) -> Any:
        import rclpy.qos as rclpy_qos_mod

        if qos is not None:
            return rclpy_qos_mod.QoSProfile(
                reliability=(
                    rclpy_qos_mod.ReliabilityPolicy.BEST_EFFORT
                    if qos.reliability == "best_effort"
                    else rclpy_qos_mod.ReliabilityPolicy.RELIABLE
                ),
                durability=(
                    rclpy_qos_mod.DurabilityPolicy.TRANSIENT_LOCAL
                    if qos.durability == "transient_local"
                    else rclpy_qos_mod.DurabilityPolicy.VOLATILE
                ),
                depth=qos.depth,
            )
        if type_name.startswith(_SENSOR_MSGS_PREFIX):
            return rclpy_qos_mod.qos_profile_sensor_data
        return 10  # rclpy default: RELIABLE/VOLATILE history depth 10

    def get_latest(self, topic: str) -> CachedMessage | None:
        with self._lock:
            cached = self._cache.get(topic)
        if cached is None:
            return None
        age_s = (datetime.now(timezone.utc) - cached.received_at).total_seconds()
        return replace(cached, age_s=age_s)

    def is_fresh(self, topic: str, max_age_s: float) -> bool:
        cached = self.get_latest(topic)
        if cached is None:
            return False
        return cached.age_s <= max_age_s

"""Subscription manager contract — frozen by docs/13-contracts.md §9."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Literal, Protocol


@dataclass(frozen=True)
class QosPolicy:
    """Concrete shape for the `qos` forward-reference in 13-contracts.md §9
    (`ensure_subscribed(..., qos: "QosPolicy | None" = None)`) — supplying the
    referenced type, not altering the signature. Mirrors the rclpy QoSProfile fields
    this system actually varies (02-ros2-architecture.md QoS Handling)."""

    reliability: Literal["reliable", "best_effort"] = "reliable"
    durability: Literal["volatile", "transient_local"] = "volatile"
    depth: int = 10


@dataclass(frozen=True)
class CachedMessage:
    value: Any
    raw: Any
    stamp: datetime
    received_at: datetime
    age_s: float


class SubscriptionManager(Protocol):
    def ensure_subscribed(
        self, topic: str, type_name: str, *, qos: QosPolicy | None = None
    ) -> None:
        """Idempotent: creates the underlying rclpy subscription at most once per
        (topic, type_name) for the process lifetime. Called by adapters at init, never
        per-request."""
        ...

    def get_latest(self, topic: str) -> CachedMessage | None:
        """Non-blocking. Returns None if never received. MUST NOT create a subscription
        as a side effect (ensure_subscribed is a separate, explicit step) — no new
        subscription per MCP request."""
        ...

    def is_fresh(self, topic: str, max_age_s: float) -> bool: ...

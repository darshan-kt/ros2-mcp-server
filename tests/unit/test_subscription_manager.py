"""Unit tests for PooledSubscriptionManager — fake bridge/node, real ROS message
instances (Twist), no rclpy.init() or live graph required."""
from __future__ import annotations

import asyncio

from geometry_msgs.msg import Twist

from ros_mcp.subscriptions.manager import PooledSubscriptionManager


class FakeBridge:
    async def call_ros_from_asyncio(self, fn):
        return fn()

    def call_soon_threadsafe_from_ros(self, fn):
        fn()


class FakeNode:
    def __init__(self) -> None:
        self.create_subscription_calls: list[tuple[str, object]] = []
        self.callbacks: dict[str, object] = {}

    def create_subscription(self, msg_class, topic, callback, qos):
        self.create_subscription_calls.append((topic, msg_class))
        self.callbacks[topic] = callback


def _manager(node: FakeNode) -> PooledSubscriptionManager:
    return PooledSubscriptionManager(
        ros_bridge=FakeBridge(),
        node=node,
        get_message_fn=lambda type_name: Twist,
    )


async def _settle():
    # let the fire-and-forget subscribe task scheduled by ensure_subscribed run.
    await asyncio.sleep(0)
    await asyncio.sleep(0)


def test_get_latest_returns_none_before_any_subscription():
    async def scenario():
        manager = _manager(FakeNode())
        return manager.get_latest("/cmd_vel")

    assert asyncio.run(scenario()) is None


def test_ensure_subscribed_is_idempotent():
    async def scenario():
        node = FakeNode()
        manager = _manager(node)
        manager.ensure_subscribed("/cmd_vel", "geometry_msgs/msg/Twist")
        manager.ensure_subscribed("/cmd_vel", "geometry_msgs/msg/Twist")
        manager.ensure_subscribed("/cmd_vel", "geometry_msgs/msg/Twist")
        await _settle()
        return node.create_subscription_calls

    calls = asyncio.run(scenario())
    assert len(calls) == 1
    assert calls[0][0] == "/cmd_vel"


def test_get_latest_does_not_create_subscription_as_side_effect():
    async def scenario():
        node = FakeNode()
        manager = _manager(node)
        result = manager.get_latest("/never/subscribed")
        return result, node.create_subscription_calls

    result, calls = asyncio.run(scenario())
    assert result is None
    assert calls == []


def test_get_latest_returns_cached_value_after_message_arrives():
    async def scenario():
        node = FakeNode()
        manager = _manager(node)
        manager.ensure_subscribed("/cmd_vel", "geometry_msgs/msg/Twist")
        await _settle()

        msg = Twist()
        msg.linear.x = 0.3
        node.callbacks["/cmd_vel"](msg)

        return manager.get_latest("/cmd_vel")

    cached = asyncio.run(scenario())
    assert cached is not None
    assert cached.value["linear"]["x"] == 0.3
    assert cached.raw is not None
    assert cached.age_s >= 0.0


def test_is_fresh_true_for_recent_message_false_for_zero_budget():
    async def scenario():
        node = FakeNode()
        manager = _manager(node)
        manager.ensure_subscribed("/cmd_vel", "geometry_msgs/msg/Twist")
        await _settle()
        node.callbacks["/cmd_vel"](Twist())
        return manager.is_fresh("/cmd_vel", max_age_s=1000.0), manager.is_fresh(
            "/cmd_vel", max_age_s=-1.0
        )

    fresh, stale = asyncio.run(scenario())
    assert fresh is True
    assert stale is False


def test_is_fresh_false_when_never_subscribed():
    async def scenario():
        manager = _manager(FakeNode())
        return manager.is_fresh("/nope", max_age_s=1000.0)

    assert asyncio.run(scenario()) is False

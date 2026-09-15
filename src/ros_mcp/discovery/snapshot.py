"""Pure graph-snapshot assembly (docs/02-ros2-architecture.md).

`collect_graph_snapshot` takes a node-like object and returns a `GraphSnapshot`. It is a
plain function operating only on the standard `rclpy.node.Node` graph-introspection
method names (`get_node_names_and_namespaces`, `get_publisher_names_and_types_by_node`,
...) so it is unit-testable against a duck-typed fake exposing those same method names,
with no live ROS graph and no rclpy.init() required — the MVP acceptance requirement for
ROS-mocked unit tests.

This module performs no I/O scheduling and must never be called directly from the
asyncio loop against a real rclpy Node — callers on the asyncio side must route through
`RosBridge.call_ros_from_asyncio` (docs/13-contracts.md §13); that dispatch lives in
`ros_mcp.discovery.engine`, not here.
"""
from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any

from ros_mcp.contracts.discovery import (
    ActionInfo,
    GraphSnapshot,
    NodeInfo,
    ServiceInfo,
    TopicInfo,
)

logger = logging.getLogger(__name__)


def _full_node_name(name: str, namespace: str) -> str:
    ns = namespace.rstrip("/")
    return f"{ns}/{name}" if ns else f"/{name}"


def _first_type(types: list[str]) -> str:
    return types[0] if types else ""


def _qos_profiles_for_topic(node: Any, topic_name: str) -> tuple[dict[str, Any], ...]:
    """Best-effort QoS introspection (docs/02-ros2-architecture.md QoS Handling). Not
    every RMW/endpoint reports this; failures degrade to an empty tuple, never raise."""
    getter = getattr(node, "get_publishers_info_by_topic", None)
    if getter is None:
        return ()
    try:
        infos = getter(topic_name)
    except Exception:  # noqa: BLE001 - introspection is best-effort, never fatal
        logger.debug("QoS introspection failed for %s", topic_name, exc_info=True)
        return ()
    profiles: list[dict[str, Any]] = []
    for info in infos:
        qos = getattr(info, "qos_profile", None)
        if qos is None:
            continue
        profiles.append(
            {
                "reliability": getattr(getattr(qos, "reliability", None), "name", None),
                "durability": getattr(getattr(qos, "durability", None), "name", None),
                "depth": getattr(qos, "depth", None),
                "history": getattr(getattr(qos, "history", None), "name", None),
            }
        )
    return tuple(profiles)


def _tf_frames(tf_buffer: Any) -> tuple[str, ...]:
    if tf_buffer is None:
        return ()
    try:
        import yaml

        text = tf_buffer.all_frames_as_yaml()
        parsed = yaml.safe_load(text) or {}
    except Exception:  # noqa: BLE001 - TF may not be warmed up yet; never fatal
        logger.debug("TF frame introspection failed", exc_info=True)
        return ()
    frames: set[str] = set()
    for frame_id, info in parsed.items():
        frames.add(frame_id)
        parent = (info or {}).get("parent")
        if parent:
            frames.add(parent)
    return tuple(sorted(frames))


def collect_graph_snapshot(
    node: Any,
    *,
    tf_buffer: Any = None,
    prev_version: int = 0,
    action_names_and_types_fn: Any = None,
    action_server_names_and_types_by_node_fn: Any = None,
) -> GraphSnapshot:
    """Walk the live graph exactly once and return a new immutable snapshot.

    `action_names_and_types_fn`/`action_server_names_and_types_by_node_fn` default to
    the real `rclpy.action` module functions but are injectable so unit tests never
    need rclpy.action's own node-graph wiring.
    """
    if action_names_and_types_fn is None or action_server_names_and_types_by_node_fn is None:
        from rclpy.action import get_action_names_and_types, get_action_server_names_and_types_by_node

        action_names_and_types_fn = action_names_and_types_fn or get_action_names_and_types
        action_server_names_and_types_by_node_fn = (
            action_server_names_and_types_by_node_fn or get_action_server_names_and_types_by_node
        )

    node_infos: list[NodeInfo] = []
    topic_acc: dict[str, dict[str, Any]] = {}
    service_acc: dict[str, str] = {}

    for name, namespace in node.get_node_names_and_namespaces():
        full_name = _full_node_name(name, namespace)
        pubs = node.get_publisher_names_and_types_by_node(name, namespace)
        subs = node.get_subscriber_names_and_types_by_node(name, namespace)
        services = node.get_service_names_and_types_by_node(name, namespace)
        clients = node.get_client_names_and_types_by_node(name, namespace)
        try:
            action_servers = action_server_names_and_types_by_node_fn(node, name, namespace)
        except Exception:  # noqa: BLE001 - action graph introspection is best-effort
            action_servers = []

        for topic, types in pubs:
            entry = topic_acc.setdefault(topic, {"type": _first_type(types), "pubs": set(), "subs": set()})
            entry["pubs"].add(full_name)
        for topic, types in subs:
            entry = topic_acc.setdefault(topic, {"type": _first_type(types), "pubs": set(), "subs": set()})
            entry["subs"].add(full_name)
        for service, types in services:
            service_acc[service] = _first_type(types)

        node_infos.append(
            NodeInfo(
                name=name,
                namespace=namespace,
                publishers=tuple(t for t, _ in pubs),
                subscribers=tuple(t for t, _ in subs),
                services=tuple(s for s, _ in services),
                clients=tuple(c for c, _ in clients),
                actions=tuple(a for a, _ in action_servers),
            )
        )

    topics = tuple(
        TopicInfo(
            name=topic,
            type_name=data["type"],
            publishers=tuple(sorted(data["pubs"])),
            subscribers=tuple(sorted(data["subs"])),
            qos_profiles=_qos_profiles_for_topic(node, topic),
            measured_hz=None,
            last_stamp=None,
            frame_id=None,
        )
        for topic, data in sorted(topic_acc.items())
    )
    services = tuple(
        ServiceInfo(name=service, type_name=type_name)
        for service, type_name in sorted(service_acc.items())
    )
    try:
        action_pairs = action_names_and_types_fn(node)
    except Exception:  # noqa: BLE001 - action graph introspection is best-effort
        action_pairs = []
    actions = tuple(ActionInfo(name=a, type_name=_first_type(t)) for a, t in action_pairs)

    return GraphSnapshot(
        snapshot_version=prev_version + 1,
        taken_at=datetime.now(timezone.utc),
        nodes=tuple(node_infos),
        topics=topics,
        services=services,
        actions=actions,
        tf_frames=_tf_frames(tf_buffer),
    )

"""DefaultResourceProvider — structurally satisfies
ros_mcp.contracts.mcp_surface.ResourceProvider (docs/13-contracts.md §4,
docs/04-mcp-surface.md MVP Resources: robot://state, robot://capabilities,
robot://graph-summary).

`robot://graph-summary` is a condensed view — node/topic counts by rough category,
action servers, TF frame count — never a full per-topic dump
(docs/11-context-and-streaming.md). Known MVP limitation: `GraphSnapshot.tf_frames` is a
flat frame-name set (docs/13-contracts.md §3), so true TF-tree root/leaf identification
(needing parent/child edges) isn't derivable from it; this resource reports the frame
set instead of fabricating a root/leaf claim the frozen data model can't support.
"""
from __future__ import annotations

import time
from typing import Any

from ros_mcp.commands.ulid import new_ulid
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.config import ConfigProvider
from ros_mcp.contracts.discovery import DiscoveryEngine
from ros_mcp.contracts.results import CommandSummary
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.contracts.execution import ExecutionManager
from ros_mcp.contracts.results import CapabilitiesResult, RobotStateResult
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.mcp.capabilities_resolver import resolve_capabilities
from ros_mcp.mcp.serialization import to_json_safe
from ros_mcp.mcp.state_resolver import resolve_robot_state

RESOURCE_URIS = ("robot://state", "robot://capabilities", "robot://graph-summary")


def _categorize_topic(type_name: str) -> str:
    if type_name.startswith("sensor_msgs/"):
        return "sensor"
    if type_name.startswith("tf2_msgs/"):
        return "tf"
    if type_name.startswith("diagnostic_msgs/"):
        return "diagnostic"
    if type_name in ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"):
        return "control"
    return "other"


class DefaultResourceProvider:
    """Structurally satisfies ros_mcp.contracts.mcp_surface.ResourceProvider."""

    def __init__(
        self,
        *,
        registry: CapabilityRegistry,
        subscriptions: SubscriptionManager,
        discovery_engine: DiscoveryEngine,
        execution_manager: ExecutionManager,
        config_provider: ConfigProvider,
        tf_adapter: TFAdapter | None = None,
        base_frame: str | None = None,
    ) -> None:
        self._registry = registry
        self._subscriptions = subscriptions
        self._discovery_engine = discovery_engine
        self._execution_manager = execution_manager
        self._config_provider = config_provider
        self._tf_adapter = tf_adapter
        self._base_frame = base_frame

    def current_resources(self) -> tuple[str, ...]:
        return RESOURCE_URIS

    async def read(self, uri: str) -> dict[str, Any]:
        if uri == "robot://state":
            return self._to_dict(await self._read_state())
        if uri == "robot://capabilities":
            return self._to_dict(await self._read_capabilities())
        if uri == "robot://graph-summary":
            return await self._read_graph_summary()
        raise ValueError(f"unknown resource URI: {uri!r}")

    async def _read_state(self) -> RobotStateResult:
        robot_id = self._config_provider.robot().id
        active = self._execution_manager.active_command(robot_id)
        active_summary = (
            CommandSummary(command_id=active.command_id, operation=active.operation,
                            execution_state=active.execution_state)
            if active is not None else None
        )
        return await resolve_robot_state(
            robot_id=robot_id,
            command_id=new_ulid(),
            duration_sec=0.0,
            registry=self._registry,
            subscriptions=self._subscriptions,
            requested_frame=None,
            active_command=active_summary,
            tf_adapter=self._tf_adapter,
            base_frame=self._base_frame,
        )

    async def _read_capabilities(self) -> CapabilitiesResult:
        robot_id = self._config_provider.robot().id
        return resolve_capabilities(
            robot_id=robot_id, command_id=new_ulid(), duration_sec=0.0, registry=self._registry
        )

    async def _read_graph_summary(self) -> dict[str, Any]:
        snapshot = await self._discovery_engine.get_current_snapshot()
        topic_counts: dict[str, int] = {}
        for topic in snapshot.topics:
            category = _categorize_topic(topic.type_name)
            topic_counts[category] = topic_counts.get(category, 0) + 1
        return {
            "snapshot_version": snapshot.snapshot_version,
            "taken_at": snapshot.taken_at.isoformat(),
            "node_count": len(snapshot.nodes),
            "topic_count": len(snapshot.topics),
            "topic_count_by_category": topic_counts,
            "service_count": len(snapshot.services),
            "action_servers": sorted(a.name for a in snapshot.actions),
            "tf_frame_count": len(snapshot.tf_frames),
            "tf_frames": sorted(snapshot.tf_frames),
        }

    @staticmethod
    def _to_dict(result: Any) -> dict[str, Any]:
        return to_json_safe(result)  # type: ignore[no-any-return]

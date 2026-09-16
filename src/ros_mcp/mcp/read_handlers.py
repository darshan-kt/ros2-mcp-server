"""Builds the `read_handlers` dict DefaultExecutionManager dispatches READ-class
operations to (docs/06-execution.md: "GET_* routes directly to the relevant read-only
adapter/Context Manager"). Each handler receives the already-validated SemanticCommand
and returns a fully-stamped ToolResult.
"""
from __future__ import annotations

import dataclasses
import time
from typing import Any, Awaitable, Callable

from ros_mcp.contracts.adapters import PerceptionAdapter
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.config import PerceptionConfig
from ros_mcp.contracts.core import Operation, SemanticCommand
from ros_mcp.contracts.errors import ToolResult
from ros_mcp.contracts.execution import ExecutionManager
from ros_mcp.contracts.results import CommandSummary
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.mcp.capabilities_resolver import resolve_capabilities
from ros_mcp.mcp.state_resolver import resolve_robot_state


def build_read_handlers(
    *,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    perception_adapter: PerceptionAdapter,
    execution_manager: ExecutionManager,
    perception_config_provider: Callable[[], PerceptionConfig],
    tf_adapter: TFAdapter | None = None,
    base_frame: str | None = None,
) -> dict[Operation, Callable[[SemanticCommand], Awaitable[ToolResult]]]:
    async def handle_get_state(command: SemanticCommand) -> ToolResult:
        start = time.monotonic()
        requested_frame = command.provenance.raw_arguments.get("frame")
        active = execution_manager.active_command(command.robot_id)
        active_summary = (
            CommandSummary(
                command_id=active.command_id,
                operation=active.operation,
                execution_state=active.execution_state,
            )
            if active is not None and active.command_id != command.command_id
            else None
        )
        return await resolve_robot_state(
            robot_id=command.robot_id,
            command_id=command.command_id,
            duration_sec=time.monotonic() - start,
            registry=registry,
            subscriptions=subscriptions,
            requested_frame=requested_frame,
            active_command=active_summary,
            tf_adapter=tf_adapter,
            base_frame=base_frame,
        )

    async def handle_get_capabilities(command: SemanticCommand) -> ToolResult:
        start = time.monotonic()
        return resolve_capabilities(
            robot_id=command.robot_id,
            command_id=command.command_id,
            duration_sec=time.monotonic() - start,
            registry=registry,
        )

    async def handle_get_laser_scan(command: SemanticCommand) -> ToolResult:
        start = time.monotonic()
        include_raw = bool(command.provenance.raw_arguments.get("include_raw_ranges", False))
        result = await perception_adapter.get_laser_scan_summary(include_raw)
        return dataclasses.replace(
            result, command_id=command.command_id, duration_sec=time.monotonic() - start
        )

    async def handle_get_camera_image(command: SemanticCommand) -> ToolResult:
        start = time.monotonic()
        perception_config = perception_config_provider()
        max_width = int(
            command.provenance.raw_arguments.get(
                "max_width_px", perception_config.camera_max_width_px_default
            )
        )
        result = await perception_adapter.get_camera_image(max_width)
        return dataclasses.replace(
            result, command_id=command.command_id, duration_sec=time.monotonic() - start
        )

    async def handle_detect_objects(command: SemanticCommand) -> ToolResult:
        start = time.monotonic()
        raw_labels = command.provenance.raw_arguments.get("labels")
        labels = tuple(raw_labels) if raw_labels else None
        result = await perception_adapter.detect_objects(labels)
        return dataclasses.replace(
            result, command_id=command.command_id, duration_sec=time.monotonic() - start
        )

    return {
        Operation.GET_STATE: handle_get_state,
        Operation.GET_CAPABILITIES: handle_get_capabilities,
        Operation.GET_LASER_SCAN: handle_get_laser_scan,
        Operation.GET_CAMERA_IMAGE: handle_get_camera_image,
        Operation.DETECT_OBJECTS: handle_detect_objects,
    }

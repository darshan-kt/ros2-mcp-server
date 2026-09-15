"""Builds CapabilitiesResult from the CapabilityRegistry (docs/04-mcp-surface.md
robot.get_capabilities)."""
from __future__ import annotations

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.results import CapabilitiesResult


def resolve_capabilities(
    *,
    robot_id: str,
    command_id: str,
    duration_sec: float,
    registry: CapabilityRegistry,
) -> CapabilitiesResult:
    return CapabilitiesResult(
        status="succeeded",
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        capabilities=registry.current(),
        snapshot_version=registry.snapshot_version(),
    )

"""MCP surface contracts — frozen by docs/13-contracts.md §4."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from ros_mcp.contracts.core import CommandClass
from ros_mcp.contracts.errors import ToolResult


@dataclass(frozen=True)
class McpToolSpec:
    name: str
    description: str
    input_schema: dict[str, Any]
    command_class: CommandClass


class ToolProvider(Protocol):
    def current_tools(self) -> tuple[McpToolSpec, ...]:
        """Derived from CapabilityRegistry.current(); MUST exclude any capability at
        Confidence.AMBIGUOUS with no config resolution (03-capability-discovery.md)."""
        ...

    async def handle_call(
        self, tool_name: str, arguments: dict[str, Any], session_id: str
    ) -> ToolResult: ...


class ResourceProvider(Protocol):
    def current_resources(self) -> tuple[str, ...]:
        """URIs: robot://state, robot://capabilities, robot://graph-summary (MVP)."""
        ...

    async def read(self, uri: str) -> dict[str, Any]: ...

"""Execution contracts — frozen by docs/13-contracts.md §6."""
from __future__ import annotations

from typing import Any, Callable, Protocol

from ros_mcp.contracts.adapters import Backend
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Operation, SemanticCommand
from ros_mcp.contracts.errors import ToolResult


class ExecutionManager(Protocol):
    async def submit(self, command: SemanticCommand) -> ToolResult:
        """Runs the full RECEIVED -> ... -> terminal pipeline (06-execution.md) and
        returns the terminal ToolResult. Dedupes by command_id: a second submit() with
        an already in-flight command_id returns the SAME awaited result, not a second
        execution."""
        ...

    async def cancel(self, command_id: str, reason: str) -> bool:
        """Signals cancellation_token for an in-flight command. Returns False if no
        such in-flight command exists (not an error)."""
        ...

    def active_command(self, robot_id: str) -> SemanticCommand | None:
        """Read-only; used by robot.get_state and robot.stop."""
        ...

    def on_progress(
        self, command_id: str, callback: Callable[[dict[str, Any]], None]
    ) -> None:
        """Registers a progress-notification sink for streaming (11-context-and-streaming.md)."""
        ...


class CommandPlanner(Protocol):
    def select_backend(
        self, operation: Operation, registry: CapabilityRegistry
    ) -> Backend | None:
        """Pure function of operation + current registry snapshot. Returns None ->
        Execution Manager reports CAPABILITY_UNAVAILABLE. Encodes the NAVIGATE-never-
        falls-back-to-MOVE policy (08-navigation-architecture.md)."""
        ...

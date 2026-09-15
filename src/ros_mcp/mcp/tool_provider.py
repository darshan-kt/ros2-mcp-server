"""DefaultToolProvider — structurally satisfies
ros_mcp.contracts.mcp_surface.ToolProvider (docs/13-contracts.md §4,
docs/03-capability-discovery.md non-speculation rule).

Owns: which tools are currently exposed (derived from the CapabilityRegistry, excluding
AMBIGUOUS-confidence capabilities), and the Validate -> build SemanticCommand -> submit
pipeline for every call. Does NOT own safety/planning/dispatch — that is
ExecutionManager's job (docs/06-execution.md), reached only via `submit()`.
"""
from __future__ import annotations

from typing import Any

from ros_mcp.commands.factory import RESULT_TYPE_BY_OPERATION, SemanticCommandFactory
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.config import ConfigProvider
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.errors import ErrorCode, ToolError, ToolResult
from ros_mcp.contracts.execution import ExecutionManager
from ros_mcp.contracts.mcp_surface import McpToolSpec
from ros_mcp.contracts.safety import ValidationEngine
from ros_mcp.execution.error_results import build_error_result
from ros_mcp.mcp.schemas import TOOL_DEFINITIONS, TOOL_DEFINITIONS_BY_NAME


class DefaultToolProvider:
    """Structurally satisfies ros_mcp.contracts.mcp_surface.ToolProvider."""

    def __init__(
        self,
        *,
        registry: CapabilityRegistry,
        validation_engine: ValidationEngine,
        execution_manager: ExecutionManager,
        command_factory: SemanticCommandFactory,
        config_provider: ConfigProvider,
    ) -> None:
        self._registry = registry
        self._validation_engine = validation_engine
        self._execution_manager = execution_manager
        self._command_factory = command_factory
        self._config_provider = config_provider

    def current_tools(self) -> tuple[McpToolSpec, ...]:
        entries_by_id = {e.capability_id: e for e in self._registry.current()}
        tools: list[McpToolSpec] = []
        for definition in TOOL_DEFINITIONS:
            if definition.required_capability_id is None:
                tools.append(definition.spec)
                continue
            entry = entries_by_id.get(definition.required_capability_id)
            if entry is not None and entry.confidence != Confidence.AMBIGUOUS:
                tools.append(definition.spec)
        return tuple(tools)

    async def handle_call(
        self, tool_name: str, arguments: dict[str, Any], session_id: str
    ) -> ToolResult:
        definition = TOOL_DEFINITIONS_BY_NAME.get(tool_name)
        if definition is None:
            return ToolResult(
                status="failed",
                command_id="",
                robot_id=self._config_provider.robot().id,
                duration_sec=0.0,
                error=ToolError(code=ErrorCode.INVALID_ARGUMENT, message=f"unknown tool '{tool_name}'"),
            )

        validation = self._validation_engine.validate(tool_name, arguments)
        if not validation.ok:
            assert validation.error is not None
            expected_result_type = RESULT_TYPE_BY_OPERATION[definition.operation]
            return build_error_result(
                expected_result_type,
                command_id="",
                robot_id=self._config_provider.robot().id,
                duration_sec=0.0,
                error=validation.error,
            )

        command = self._command_factory.create(
            tool_name=tool_name,
            operation=definition.operation,
            command_class=definition.spec.command_class,
            arguments=arguments,
            session_id=session_id,
            safety_config=self._config_provider.safety(),
            timeout_config=self._config_provider.timeouts(),
        )
        return await self._execution_manager.submit(command)

"""DefaultValidationEngine — structurally satisfies
ros_mcp.contracts.safety.ValidationEngine (docs/13-contracts.md §5)."""
from __future__ import annotations

from typing import Any

from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.safety import ValidationOutcome
from ros_mcp.mcp.schemas import TOOL_DEFINITIONS_BY_NAME
from ros_mcp.validation.schema_check import check_schema

_TRANSLATION_DIRECTIONS = frozenset({"forward", "backward", "left", "right"})
_ROTATION_DIRECTIONS = frozenset({"rotate_left", "rotate_right"})


def _semantic_check(tool_name: str, arguments: dict[str, Any]) -> str | None:
    """Cross-field checks the flat JSON schema can't express (docs/13-contracts.md §5:
    "semantic range checks (e.g. angle_deg in [0, 360])" plus robot.move's
    direction-dependent required-field rule)."""
    if tool_name == "robot.move":
        direction = arguments.get("direction")
        if direction in _TRANSLATION_DIRECTIONS and arguments.get("distance_m") is None:
            return f"distance_m is required when direction is '{direction}'"
        if direction in _ROTATION_DIRECTIONS and arguments.get("angle_deg") is None:
            return f"angle_deg is required when direction is '{direction}'"
    return None


class DefaultValidationEngine:
    """Structurally satisfies ros_mcp.contracts.safety.ValidationEngine."""

    def validate(self, tool_name: str, arguments: dict[str, Any]) -> ValidationOutcome:
        definition = TOOL_DEFINITIONS_BY_NAME.get(tool_name)
        if definition is None:
            return ValidationOutcome(
                ok=False,
                error=ToolError(
                    code=ErrorCode.INVALID_ARGUMENT,
                    message=f"unknown tool '{tool_name}'",
                    details={"tool_name": tool_name},
                ),
            )

        schema_error = check_schema(definition.spec.input_schema, arguments)
        if schema_error is not None:
            return ValidationOutcome(
                ok=False,
                error=ToolError(
                    code=ErrorCode.INVALID_ARGUMENT,
                    message=schema_error,
                    details={"tool_name": tool_name, "arguments": arguments},
                ),
            )

        semantic_error = _semantic_check(tool_name, arguments)
        if semantic_error is not None:
            return ValidationOutcome(
                ok=False,
                error=ToolError(
                    code=ErrorCode.INVALID_ARGUMENT,
                    message=semantic_error,
                    details={"tool_name": tool_name, "arguments": arguments},
                ),
            )

        return ValidationOutcome(ok=True, error=None)

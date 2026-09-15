"""Validation & safety contracts — frozen by docs/13-contracts.md §5."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

from ros_mcp.contracts.core import Pose2D, SafetyConstraints, SemanticCommand
from ros_mcp.contracts.errors import ToolError


@dataclass(frozen=True)
class ValidationOutcome:
    ok: bool
    error: ToolError | None


class ValidationEngine(Protocol):
    def validate(self, tool_name: str, arguments: dict[str, Any]) -> ValidationOutcome:
        """Pure, synchronous, stateless: JSON-schema + semantic range checks
        (e.g. angle_deg in [0, 360]). No ROS access, no I/O."""
        ...


class SafetyDecision(str, Enum):
    ALLOW = "allow"
    DENY = "deny"
    REQUIRE_APPROVAL = "require_approval"


@dataclass(frozen=True)
class SafetyOutcome:
    decision: SafetyDecision
    error: ToolError | None
    approval_reason: str | None
    resolved_constraints: SafetyConstraints


class SafetyPolicyEngine(Protocol):
    async def check(
        self, command: SemanticCommand, current_pose: Pose2D | None
    ) -> SafetyOutcome:
        """The single mandatory choke point. MUST be called by the Execution Manager
        before any adapter dispatch for CommandClass.MOTION and CommandClass.HIGH_RISK.
        MUST NOT be reachable from any other call path (13-contracts §Non-Bypass Rule)."""
        ...

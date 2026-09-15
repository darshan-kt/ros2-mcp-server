"""DefaultCommandPlanner — structurally satisfies
ros_mcp.contracts.execution.CommandPlanner (docs/13-contracts.md §6,
docs/08-navigation-architecture.md "Why robot.navigate Never Falls Back to /cmd_vel").
"""
from __future__ import annotations

from typing import Any

from ros_mcp.contracts.adapters import Backend
from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import Confidence, Operation

_MOTION_OPERATIONS = frozenset({Operation.MOVE, Operation.STOP})


class DefaultCommandPlanner:
    """Structurally satisfies ros_mcp.contracts.execution.CommandPlanner."""

    def __init__(self, *, motion_backend: Any, navigation_backend: Any) -> None:
        self._motion_backend = motion_backend
        self._navigation_backend = navigation_backend

    def select_backend(
        self, operation: Operation, registry: CapabilityRegistry
    ) -> Backend | None:
        if operation in _MOTION_OPERATIONS:
            entry = registry.get("differential_drive_motion")
            if entry is None or entry.confidence == Confidence.AMBIGUOUS:
                return None
            return self._motion_backend

        if operation is Operation.NAVIGATE:
            # Deliberately never falls back to the motion backend when Nav2/
            # localization is unavailable (08-navigation-architecture.md, ADR-005) —
            # returns None so the Execution Manager reports CAPABILITY_UNAVAILABLE.
            nav_entry = registry.get("autonomous_navigation")
            localization_entry = registry.get("localization")
            if nav_entry is None or nav_entry.confidence == Confidence.AMBIGUOUS:
                return None
            if localization_entry is None:
                return None
            if self._navigation_backend is None or not self._navigation_backend.is_available():
                return None
            return self._navigation_backend

        return None

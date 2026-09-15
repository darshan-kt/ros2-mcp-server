"""DefaultCapabilityInferenceEngine — structurally satisfies
ros_mcp.contracts.capabilities.CapabilityInferenceEngine (docs/13-contracts.md §3).

The frozen `infer(snapshot, overrides)` signature does not carry a SafetyConfig
parameter, but two ontology entries (differential_drive_motion, autonomous_navigation)
report configured limits as part of their CapabilityEntry.limits (see
docs/04-mcp-surface.md's robot.get_capabilities example, which includes
max_linear_mps/max_navigation_distance_m in the result). Rather than widen the frozen
method signature, SafetyConfig is supplied once at construction (a normal
dependency-injection seam) and `infer()` itself remains a pure function of its two
parameters for any given instance — satisfying the "no I/O, pure function" contract note
while still being able to report limits.
"""
from __future__ import annotations

from ros_mcp.capabilities.ontology import (
    infer_autonomous_navigation,
    infer_differential_drive_motion,
    infer_localization,
    infer_object_detection,
    infer_range_sensing,
    infer_visual_observation,
)
from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import CapabilityConfig, SafetyConfig
from ros_mcp.contracts.discovery import GraphSnapshot


class DefaultCapabilityInferenceEngine:
    """Structurally satisfies ros_mcp.contracts.capabilities.CapabilityInferenceEngine."""

    def __init__(self, safety_config: SafetyConfig) -> None:
        self._safety_config = safety_config

    def infer(
        self, snapshot: GraphSnapshot, overrides: CapabilityConfig
    ) -> tuple[CapabilityEntry, ...]:
        entries: list[CapabilityEntry] = []

        motion = infer_differential_drive_motion(snapshot, overrides, self._safety_config)
        if motion is not None:
            entries.append(motion)

        navigation = infer_autonomous_navigation(snapshot, overrides, self._safety_config)
        if navigation is not None:
            entries.append(navigation)

        localization = infer_localization(snapshot, overrides)
        if localization is not None:
            entries.append(localization)

        range_sensing = infer_range_sensing(snapshot, overrides)
        if range_sensing is not None:
            entries.append(range_sensing)

        visual_observation = infer_visual_observation(snapshot, overrides)
        if visual_observation is not None:
            entries.append(visual_observation)

        object_detection = infer_object_detection(visual_observation, overrides)
        if object_detection is not None:
            entries.append(object_detection)

        return tuple(entries)

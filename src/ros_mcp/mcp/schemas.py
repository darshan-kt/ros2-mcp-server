"""Static MCP tool schemas — verbatim from docs/04-mcp-surface.md. Each entry's
`capability_id` (not part of McpToolSpec itself) says which CapabilityRegistry entry
must be CONFIRMED/LIKELY for ToolProvider.current_tools() to include it (03-capability-
discovery.md, ADR-003 non-speculation rule); `None` means always available."""
from __future__ import annotations

from dataclasses import dataclass

from ros_mcp.contracts.core import CommandClass, Operation
from ros_mcp.contracts.mcp_surface import McpToolSpec


@dataclass(frozen=True)
class ToolDefinition:
    spec: McpToolSpec
    operation: Operation
    required_capability_id: str | None


TOOL_DEFINITIONS: tuple[ToolDefinition, ...] = (
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.get_capabilities",
            description=(
                "List this robot's currently available semantic capabilities and the "
                "tools/limits each one exposes. Call this first if unsure what the "
                "robot can do."
            ),
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            command_class=CommandClass.READ,
        ),
        operation=Operation.GET_CAPABILITIES,
        required_capability_id=None,
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.get_state",
            description=(
                "Get the robot's current pose, velocity, and whether it is currently "
                "executing a command."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "frame": {
                        "type": "string",
                        "description": (
                            "Frame to express pose in. Defaults to 'map' if localized, "
                            "else 'odom'."
                        ),
                    }
                },
                "additionalProperties": False,
            },
            command_class=CommandClass.READ,
        ),
        operation=Operation.GET_STATE,
        required_capability_id=None,
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.move",
            description=(
                "Move the robot a relative straight-line distance using closed-loop "
                "velocity control against odometry, or rotate in place. Enforces "
                "configured velocity/acceleration/distance/timeout limits and stops on "
                "obstacle or stale odometry. Prefer robot.navigate for absolute goals "
                "when Nav2 is available."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": [
                            "forward",
                            "backward",
                            "left",
                            "right",
                            "rotate_left",
                            "rotate_right",
                        ],
                    },
                    "distance_m": {
                        "type": "number",
                        "minimum": 0,
                        "description": "Required for forward/backward/left/right.",
                    },
                    "angle_deg": {
                        "type": "number",
                        "minimum": 0,
                        "maximum": 360,
                        "description": "Required for rotate_left/rotate_right.",
                    },
                    "timeout_s": {
                        "type": "number",
                        "minimum": 0.1,
                        "description": "Optional; server default/limit applies if omitted.",
                    },
                },
                "required": ["direction"],
                "additionalProperties": False,
            },
            command_class=CommandClass.MOTION,
        ),
        operation=Operation.MOVE,
        required_capability_id="differential_drive_motion",
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.stop",
            description=(
                "Immediately halt all robot motion: cancels any in-flight move/navigate "
                "command and publishes a zero-velocity command."
            ),
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            command_class=CommandClass.MOTION,
        ),
        operation=Operation.STOP,
        required_capability_id="differential_drive_motion",
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.navigate",
            description=(
                "Navigate autonomously to an absolute goal pose using Nav2 (path "
                "planning + obstacle avoidance). Returns CAPABILITY_UNAVAILABLE if Nav2 "
                "or localization is not available — use robot.move for local motion "
                "instead."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "yaw": {
                        "type": "number",
                        "description": "Optional final heading in radians.",
                    },
                    "frame": {"type": "string", "default": "map"},
                },
                "required": ["x", "y"],
                "additionalProperties": False,
            },
            command_class=CommandClass.MOTION,
        ),
        operation=Operation.NAVIGATE,
        required_capability_id="autonomous_navigation",
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.get_laser_scan",
            description=(
                "Get a processed summary of the current laser scan: nearest/farthest "
                "obstacle overall and by sector (front/left/right/rear), plus raw "
                "ranges if needed."
            ),
            input_schema={
                "type": "object",
                "properties": {"include_raw_ranges": {"type": "boolean", "default": False}},
                "additionalProperties": False,
            },
            command_class=CommandClass.READ,
        ),
        operation=Operation.GET_LASER_SCAN,
        required_capability_id="range_sensing",
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.get_camera_image",
            description=(
                "Get the most recent camera image as a downsized JPEG thumbnail plus "
                "metadata. Not a live stream — returns the latest cached frame."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "max_width_px": {"type": "integer", "default": 640, "maximum": 1280}
                },
                "additionalProperties": False,
            },
            command_class=CommandClass.READ,
        ),
        operation=Operation.GET_CAMERA_IMAGE,
        required_capability_id="visual_observation",
    ),
    ToolDefinition(
        spec=McpToolSpec(
            name="robot.detect_objects",
            description=(
                "Detect labeled objects visible to the robot's camera and estimate "
                "their position relative to the robot. Accuracy depends on the "
                "configured detector plugin (MVP ships a stub detector for pipeline "
                "validation)."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "labels": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": 'Optional label filter, e.g. ["chair", "person"].',
                    }
                },
                "additionalProperties": False,
            },
            command_class=CommandClass.READ,
        ),
        operation=Operation.DETECT_OBJECTS,
        required_capability_id="object_detection",
    ),
)

TOOL_DEFINITIONS_BY_NAME: dict[str, ToolDefinition] = {d.spec.name: d for d in TOOL_DEFINITIONS}

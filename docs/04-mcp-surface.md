# 04 — MCP Surface

## Mapping Principle

MCP primitives map onto ROS 2 concepts as follows — this mapping is what keeps the
surface small and semantic instead of a 1:1 topic proxy:

| MCP primitive | Backed by | Not backed by |
|---|---|---|
| **Tool** | A semantic command class dispatched through the Command Planner (§[05](05-semantic-command-model.md), §[06](06-execution.md)) | A single ROS topic/service/action, exposed 1:1 |
| **Resource** | A read-only, cached/computed view assembled by the Context Manager from the Capability Registry + Subscription Manager last-value cache | A live raw topic echo |
| **Prompt** | A reusable multi-tool workflow template (post-MVP; not in the MVP tool set) | An LLM system-prompt hack |

**Explicit anti-pattern rejected**: exposing `ros2 topic list` as an MCP tool list. The
Tool Provider only ever emits tools that a `CONFIRMED`/`LIKELY` capability unlocks
(§[03](03-capability-discovery.md)), plus the deliberately-separate, gated `ros.*` raw
tools (§Raw ROS Tools below) which exist for advanced/debugging use and are individually
disable-able in config.

## MVP Tool Set

Exactly eight tools ship in the MVP (frozen scope, see [17-mvp.md](17-mvp.md)). Each tool
call, on the wire, becomes one `SemanticCommand` (§[05](05-semantic-command-model.md))
that flows through Validation → Safety → Planner → Execution Manager → Adapter
(§[06](06-execution.md)). Every tool returns the structured `ToolResult` envelope defined
in [13-contracts.md](13-contracts.md) — never a bare string.

### `robot.get_capabilities`

Command class: **READ**. No robot-side side effect; reads the Capability Registry.

```json
{
  "name": "robot.get_capabilities",
  "description": "List this robot's currently available semantic capabilities and the tools/limits each one exposes. Call this first if unsure what the robot can do.",
  "inputSchema": { "type": "object", "properties": {}, "additionalProperties": false }
}
```

Result (see full schema in [13-contracts.md](13-contracts.md) §`CapabilitiesResult`):

```json
{
  "status": "succeeded",
  "capabilities": [
    {"id": "differential_drive_motion", "confidence": "confirmed", "backend": "cmd_vel", "limits": {"max_linear_mps": 0.5, "max_angular_rps": 1.0, "max_move_distance_m": 5.0}},
    {"id": "autonomous_navigation", "confidence": "confirmed", "backend": "nav2", "limits": {"max_navigation_distance_m": 10.0}},
    {"id": "range_sensing", "confidence": "confirmed", "backend": "laser_scan", "topic": "/scan"},
    {"id": "visual_observation", "confidence": "confirmed", "backend": "camera", "topic": "/camera/image_raw"},
    {"id": "object_detection", "confidence": "confirmed", "backend": "stub_detector"}
  ],
  "robot_id": "turtlebot3_waffle",
  "snapshot_version": 4
}
```

### `robot.get_state`

Command class: **READ**.

```json
{
  "name": "robot.get_state",
  "description": "Get the robot's current pose, velocity, and whether it is currently executing a command.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "frame": {"type": "string", "description": "Frame to express pose in. Defaults to 'map' if localized, else 'odom'."}
    },
    "additionalProperties": false
  }
}
```

Result: `RobotStateResult` — pose (x, y, yaw, frame, stamp), linear/angular velocity from
odometry, `is_moving: bool`, `active_command: CommandSummary | null`.

### `robot.move`

Command class: **MOTION**.

```json
{
  "name": "robot.move",
  "description": "Move the robot a relative straight-line distance using closed-loop velocity control against odometry, or rotate in place. Enforces configured velocity/acceleration/distance/timeout limits and stops on obstacle or stale odometry. Prefer robot.navigate for absolute goals when Nav2 is available.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "direction": {"type": "string", "enum": ["forward", "backward", "left", "right", "rotate_left", "rotate_right"]},
      "distance_m": {"type": "number", "minimum": 0, "description": "Required for forward/backward/left/right."},
      "angle_deg": {"type": "number", "minimum": 0, "maximum": 360, "description": "Required for rotate_left/rotate_right."},
      "timeout_s": {"type": "number", "minimum": 0.1, "description": "Optional; server default/limit applies if omitted."}
    },
    "required": ["direction"],
    "additionalProperties": false
  }
}
```

Result: `MotionResult` — `status`, `distance_traveled_m`, `duration_sec`, `final_pose`,
`stop_reason` (`target_reached | obstacle | timeout | cancelled | safety_stop`).

### `robot.stop`

Command class: **MOTION** (but always **ALLOW**, see [10](10-safety-and-trust.md) — stop
is never itself rejected by policy).

```json
{
  "name": "robot.stop",
  "description": "Immediately halt all robot motion: cancels any in-flight move/navigate command and publishes a zero-velocity command.",
  "inputSchema": { "type": "object", "properties": {}, "additionalProperties": false }
}
```

Result: `StopResult` — `status`, `cancelled_command_id | null`, `final_pose`.

### `robot.navigate`

Command class: **MOTION**.

```json
{
  "name": "robot.navigate",
  "description": "Navigate autonomously to an absolute goal pose using Nav2 (path planning + obstacle avoidance). Returns CAPABILITY_UNAVAILABLE if Nav2 or localization is not available — use robot.move for local motion instead.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "x": {"type": "number"},
      "y": {"type": "number"},
      "yaw": {"type": "number", "description": "Optional final heading in radians."},
      "frame": {"type": "string", "default": "map"}
    },
    "required": ["x", "y"],
    "additionalProperties": false
  }
}
```

Result: `NavigateResult` — `status`, `duration_sec`, `final_pose`,
`distance_remaining_m | null`, `fail_reason` (`goal_rejected | planner_failure |
controller_failure | obstacle_timeout | cancelled | localization_lost | null`).
Progress is additionally streamed as MCP notifications while the command is in flight
(§[11-context-and-streaming.md](11-context-and-streaming.md)).

### `robot.get_laser_scan`

Command class: **READ** (LOW_RISK per §[10](10-safety-and-trust.md) classification table
— still READ, no actuation).

```json
{
  "name": "robot.get_laser_scan",
  "description": "Get a processed summary of the current laser scan: nearest/farthest obstacle overall and by sector (front/left/right/rear), plus raw ranges if needed.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "include_raw_ranges": {"type": "boolean", "default": false}
    },
    "additionalProperties": false
  }
}
```

Result: `LaserScanResult` — `stamp`, `frame`, `nearest` / `farthest` objects
(`{range_m, angle_rad, sector}`), per-sector nearest, `raw_ranges: number[] | null`,
`data_age_s`.

### `robot.get_camera_image`

Command class: **READ**.

```json
{
  "name": "robot.get_camera_image",
  "description": "Get the most recent camera image as a downsized JPEG thumbnail plus metadata. Not a live stream — returns the latest cached frame.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "max_width_px": {"type": "integer", "default": 640, "maximum": 1280}
    },
    "additionalProperties": false
  }
}
```

Result: `CameraImageResult` — `stamp`, `frame`, `data_age_s`, `image` (MCP image content
block, JPEG, resized per Binary Data Policy §[09](09-perception-architecture.md)),
`width_px`, `height_px`, `original_width_px`, `original_height_px`.

### `robot.detect_objects`

Command class: **READ**.

```json
{
  "name": "robot.detect_objects",
  "description": "Detect labeled objects visible to the robot's camera and estimate their position relative to the robot. Accuracy depends on the configured detector plugin (MVP ships a stub detector for pipeline validation).",
  "inputSchema": {
    "type": "object",
    "properties": {
      "labels": {"type": "array", "items": {"type": "string"}, "description": "Optional label filter, e.g. [\"chair\", \"person\"]."}
    },
    "additionalProperties": false
  }
}
```

Result: `DetectObjectsResult` — `stamp`, `objects: [{label, confidence, distance_m,
position: {x, y, frame}, bbox_px}]`, `detector_plugin_id`.

## MVP Resources

Resources are read via MCP `resources/read`; they are computed by the Context Manager,
never a raw topic echo.

| URI | Contents | Backed by |
|---|---|---|
| `robot://state` | Current pose/velocity/active-command summary | `RobotStateResult` (same shape as `robot.get_state`) |
| `robot://capabilities` | Current capability list + confidence + limits | Capability Registry snapshot |
| `robot://graph-summary` | Condensed ROS graph summary: node count, topic count by type category, action servers found, TF root/leaf frames — **not** a full topic dump | Discovery Engine snapshot, compressed (§[11](11-context-and-streaming.md)) |

## Raw ROS Tools (gated, post-discussion for MVP)

Defined in the architecture for completeness; **disabled by default and out of MVP
scope** (see [17-mvp.md](17-mvp.md) exclusions). When enabled via
`raw_ros_access.enabled: true` in config:

```text
ros.read_topic(topic, timeout_s) → last cached value + schema
ros.publish(topic, message)       → requires raw_ros_access.allow_publish: true, per-topic allowlist
ros.call_service(service, request)
ros.send_action_goal(action, goal)
ros.get_parameter(node, name)
ros.set_parameter(node, name, value) → requires raw_ros_access.allow_param_write: true
```

All six pass through the same Validation → Safety pipeline as semantic tools (command
class defaults to **HIGH_RISK** for `publish`/`set_parameter`/`send_action_goal`, **READ**
for the others), are individually toggleable, and every invocation is audit-logged
(§[10](10-safety-and-trust.md), §[26 security notes](14-multi-robot.md)).

**When to use semantic vs. raw tools**: semantic tools are the default and only path for
normal operation; raw tools are for operator-authorized debugging/inspection or for
capabilities the ontology doesn't yet model (e.g. a bespoke robot service) — see
[ADR-001](adr/ADR-001-semantic-tool-surface.md).

## Prompts (post-MVP)

Reserved names for the workflow layer, not implemented in MVP: `navigation_assistant`,
`inspection_workflow`, `object_search`, `robot_diagnostics`. See
[20-roadmap.md](20-roadmap.md) Phase 4/5.

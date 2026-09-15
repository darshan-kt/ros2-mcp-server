# 13 — Contracts

**This document freezes every module boundary.** It is the implementation contract:
concrete code must implement these signatures exactly (types, field names, exception
behavior). Signatures are given as Python `Protocol`/`ABC`/`dataclass` declarations with
no method bodies — this is an interface specification, not an implementation. A change to
anything in this document requires a new ADR (see [adr/README.md](adr/README.md)).

Conventions used throughout:
- All cross-module calls that can fail return a typed result/error, never raise for
  *expected* failure modes (`CAPABILITY_UNAVAILABLE`, `TF_UNAVAILABLE`, etc. are values,
  not exceptions). Only programming errors (bad type passed internally) raise.
- All async methods are native `async def` running on the MCP-side `asyncio` event loop.
  Anything that must reach into `rclpy` crosses the boundary defined in
  §Async/Threading Boundary below — callers never call `rclpy` APIs directly.
- All dataclasses are `frozen=True` unless explicitly noted `mutable`.

---

## 1. Core Data Models

```python
from __future__ import annotations
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Literal, Protocol, runtime_checkable


class Confidence(str, Enum):
    CONFIRMED = "confirmed"
    LIKELY = "likely"
    AMBIGUOUS = "ambiguous"


class Operation(str, Enum):
    MOVE = "move"
    STOP = "stop"
    NAVIGATE = "navigate"
    GET_STATE = "get_state"
    GET_CAPABILITIES = "get_capabilities"
    GET_LASER_SCAN = "get_laser_scan"
    GET_CAMERA_IMAGE = "get_camera_image"
    DETECT_OBJECTS = "detect_objects"
    RAW_READ_TOPIC = "raw_read_topic"
    RAW_PUBLISH = "raw_publish"
    RAW_CALL_SERVICE = "raw_call_service"
    RAW_SEND_ACTION_GOAL = "raw_send_action_goal"
    RAW_GET_PARAMETER = "raw_get_parameter"
    RAW_SET_PARAMETER = "raw_set_parameter"


class CommandClass(str, Enum):
    READ = "read"
    LOW_RISK = "low_risk"
    MOTION = "motion"
    HIGH_RISK = "high_risk"


class Priority(str, Enum):
    NORMAL = "normal"
    SAFETY = "safety"


class ExecutionState(str, Enum):
    RECEIVED = "received"
    VALIDATING = "validating"
    REJECTED = "rejected"
    SAFETY_CHECK = "safety_check"
    SAFETY_REJECTED = "safety_rejected"
    AWAITING_APPROVAL = "awaiting_approval"
    PLANNING = "planning"
    CAPABILITY_UNAVAILABLE = "capability_unavailable"
    EXECUTING = "executing"
    MONITORING = "monitoring"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"
    SAFETY_STOP = "safety_stop"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame: str
    stamp: datetime


@dataclass(frozen=True)
class Provenance:
    source: Literal["mcp_tool_call"]
    tool_name: str
    raw_arguments: dict[str, Any]
    client_session_id: str


@dataclass(frozen=True)
class SafetyConstraints:
    max_linear_mps: float
    max_angular_rps: float
    max_linear_acceleration_mps2: float
    max_angular_acceleration_rps2: float
    max_move_distance_m: float
    max_navigation_distance_m: float
    obstacle_stop_distance_m: float


@dataclass(frozen=True)
class MoveTarget:
    direction: Literal["forward", "backward", "left", "right", "rotate_left", "rotate_right"]
    distance_m: float | None
    angle_deg: float | None


@dataclass(frozen=True)
class NavigateTarget:
    x: float
    y: float
    yaw: float | None


Target = MoveTarget | NavigateTarget | None


@dataclass  # mutable: execution_state is updated in place by ExecutionManager only
class SemanticCommand:
    command_id: str
    session_id: str
    robot_id: str
    operation: Operation
    command_class: CommandClass
    target: Target
    frame: str | None
    timeout_s: float
    priority: Priority
    safety_constraints: SafetyConstraints | None
    provenance: Provenance
    confidence: float | None
    execution_state: ExecutionState
    created_at: datetime
    expected_result_type: type["ToolResult"]
```

## 2. Result / Error Model

```python
class ErrorCode(str, Enum):
    INVALID_ARGUMENT = "INVALID_ARGUMENT"
    INVALID_FRAME = "INVALID_FRAME"
    TF_UNAVAILABLE = "TF_UNAVAILABLE"
    CAPABILITY_UNAVAILABLE = "CAPABILITY_UNAVAILABLE"
    NAVIGATION_FAILED = "NAVIGATION_FAILED"
    ACTION_TIMEOUT = "ACTION_TIMEOUT"
    SAFETY_REJECTED = "SAFETY_REJECTED"
    ROBOT_NOT_READY = "ROBOT_NOT_READY"
    SENSOR_STALE = "SENSOR_STALE"
    PERMISSION_DENIED = "PERMISSION_DENIED"
    ROS_INTERFACE_ERROR = "ROS_INTERFACE_ERROR"
    CANCELLED = "CANCELLED"
    INTERNAL_ERROR = "INTERNAL_ERROR"


@dataclass(frozen=True)
class ToolError:
    code: ErrorCode
    message: str               # human/LLM-readable explanation
    details: dict[str, Any] = field(default_factory=dict)  # structured extra context


@dataclass(frozen=True)
class ToolResult:
    """Base envelope. Every MCP tool handler returns a subclass of this, serialized to
    the MCP tool result content — never a bare string."""
    status: Literal["succeeded", "failed", "awaiting_approval"]
    command_id: str
    robot_id: str
    duration_sec: float
    error: ToolError | None = None
```

Full result models (`RobotStateResult`, `CapabilitiesResult`, `MotionResult`,
`StopResult`, `NavigateResult`, `LaserScanResult`, `CameraImageResult`,
`DetectObjectsResult`) are dataclasses subclassing `ToolResult`, field-shaped exactly as
the JSON examples in [04-mcp-surface.md](04-mcp-surface.md). They live in
`ros_mcp.contracts.results` at implementation time.

**Error surfacing rule**: `status == "failed"` always carries a non-null `error` with one
`ErrorCode`. The MCP layer never raises an unhandled exception out of a tool handler —
any exception not already converted to a `ToolError` by the Execution Manager is caught
at the outermost tool-dispatch boundary and converted to
`ToolError(code=INTERNAL_ERROR, ...)` with the exception logged (never leaked verbatim
into the message field beyond a sanitized summary).

## 3. Discovery & Capability Contracts

```python
@dataclass(frozen=True)
class TopicInfo:
    name: str
    type_name: str
    publishers: tuple[str, ...]
    subscribers: tuple[str, ...]
    qos_profiles: tuple[dict[str, Any], ...]
    measured_hz: float | None
    last_stamp: datetime | None
    frame_id: str | None


@dataclass(frozen=True)
class ServiceInfo:
    name: str
    type_name: str


@dataclass(frozen=True)
class ActionInfo:
    name: str
    type_name: str


@dataclass(frozen=True)
class NodeInfo:
    name: str
    namespace: str
    publishers: tuple[str, ...]
    subscribers: tuple[str, ...]
    services: tuple[str, ...]
    clients: tuple[str, ...]
    actions: tuple[str, ...]


@dataclass(frozen=True)
class GraphSnapshot:
    snapshot_version: int
    taken_at: datetime
    nodes: tuple[NodeInfo, ...]
    topics: tuple[TopicInfo, ...]
    services: tuple[ServiceInfo, ...]
    actions: tuple[ActionInfo, ...]
    tf_frames: tuple[str, ...]


class DiscoveryEngine(Protocol):
    async def get_current_snapshot(self) -> GraphSnapshot:
        """Return the latest snapshot; never triggers a blocking ROS call itself —
        reads from the rclpy-side cache updated by the discovery timer."""
        ...

    def on_snapshot_changed(self, callback: "Callable[[GraphSnapshot], None]") -> None:
        """Register a callback invoked (on the asyncio loop, via the thread-safe bridge)
        whenever a new snapshot differs from the previous one."""
        ...


@dataclass(frozen=True)
class CapabilityEntry:
    capability_id: str
    confidence: Confidence
    backend_id: str | None      # which registered plugin/backend would serve it
    resolved_topics: dict[str, str] = field(default_factory=dict)
    limits: dict[str, float] = field(default_factory=dict)


class CapabilityRegistry(Protocol):
    def current(self) -> tuple[CapabilityEntry, ...]:
        """Immutable snapshot, safe to call from any thread."""
        ...

    def get(self, capability_id: str) -> CapabilityEntry | None: ...

    def snapshot_version(self) -> int: ...

    def on_changed(self, callback: "Callable[[tuple[CapabilityEntry, ...]], None]") -> None: ...


class CapabilityInferenceEngine(Protocol):
    def infer(
        self, snapshot: GraphSnapshot, overrides: "CapabilityConfig"
    ) -> tuple[CapabilityEntry, ...]:
        """Pure function: graph snapshot + config overrides -> capability entries.
        No I/O; safe to unit test with a synthetic GraphSnapshot."""
        ...
```

## 4. MCP Surface Contracts

```python
@dataclass(frozen=True)
class McpToolSpec:
    name: str
    description: str
    input_schema: dict[str, Any]   # JSON Schema, matches 04-mcp-surface.md exactly
    command_class: CommandClass


class ToolProvider(Protocol):
    def current_tools(self) -> tuple[McpToolSpec, ...]:
        """Derived from CapabilityRegistry.current(); MUST exclude any capability at
        Confidence.AMBIGUOUS with no config resolution (03-capability-discovery.md)."""
        ...

    async def handle_call(
        self, tool_name: str, arguments: dict[str, Any], session_id: str
    ) -> ToolResult:
        ...


class ResourceProvider(Protocol):
    def current_resources(self) -> tuple[str, ...]:
        """URIs: robot://state, robot://capabilities, robot://graph-summary (MVP)."""
        ...

    async def read(self, uri: str) -> dict[str, Any]: ...
```

## 5. Validation & Safety Contracts

```python
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
    error: ToolError | None              # set when decision == DENY
    approval_reason: str | None          # set when decision == REQUIRE_APPROVAL
    resolved_constraints: SafetyConstraints


class SafetyPolicyEngine(Protocol):
    async def check(self, command: SemanticCommand, current_pose: Pose2D | None) -> SafetyOutcome:
        """The single mandatory choke point. MUST be called by the Execution Manager
        before any adapter dispatch for CommandClass.MOTION and CommandClass.HIGH_RISK.
        MUST NOT be reachable from any other call path (13-contracts §Non-Bypass Rule)."""
        ...
```

### Non-Bypass Rule (structural, not a convention)

`ExecutionManager.submit()` is the **only** public entry point that results in an adapter
being invoked for a MOTION/HIGH_RISK command, and its implementation is required to call
`SafetyPolicyEngine.check()` before any adapter reference is dereferenced. Adapters
(`MotionBackend`, `NavigationBackend`, …) accept only a `command_id` +
already-validated fields, not raw tool arguments — so even a future bug in a new tool
handler cannot construct a path that reaches an adapter without having gone through a
`SemanticCommand` built by the (also fixed) `SemanticCommandFactory`, validated, and
safety-checked. This is enforced by module boundaries: adapters live in
`ros_mcp.adapters.*` and import nothing from `ros_mcp.mcp.*`.

## 6. Execution Contracts

```python
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
        self, command_id: str, callback: "Callable[[dict[str, Any]], None]"
    ) -> None:
        """Registers a progress-notification sink for streaming (11-context-and-streaming.md)."""
        ...


class CommandPlanner(Protocol):
    def select_backend(
        self, operation: Operation, registry: CapabilityRegistry
    ) -> "Backend | None":
        """Pure function of operation + current registry snapshot. Returns None ->
        Execution Manager reports CAPABILITY_UNAVAILABLE. Encodes the NAVIGATE-never-
        falls-back-to-MOVE policy (08-navigation-architecture.md)."""
        ...
```

## 7. Adapter (Backend) Contracts

```python
@runtime_checkable
class CancellationToken(Protocol):
    def is_cancelled(self) -> bool: ...
    def deadline_exceeded(self) -> bool: ...


class MotionBackend(Protocol):
    """Implemented by CmdVelMotionPlugin (MVP)."""

    async def move(
        self, command: SemanticCommand, token: CancellationToken
    ) -> "MotionResult":
        """MUST publish an explicit zero-velocity command on every exit path
        (07-motion-architecture.md). MUST poll `token` at <= one control-loop period."""
        ...

    async def stop(self) -> "StopResult":
        """Immediate: publishes zero velocity, does not itself run a control loop."""
        ...


class NavigationBackend(Protocol):
    """Implemented by Nav2NavigationPlugin (MVP)."""

    async def navigate(
        self, command: SemanticCommand, token: CancellationToken
    ) -> "NavigateResult":
        ...

    async def cancel_all(self) -> None:
        """Cancels any outstanding Nav2 goal; called unconditionally by robot.stop's
        planner fan-out (06-execution.md)."""
        ...

    def is_available(self) -> bool:
        """Cheap, synchronous, backed by the current CapabilityRegistry — used by the
        Planner before deciding to route NAVIGATE here."""
        ...


class PerceptionAdapter(Protocol):
    """MUST NOT depend on ExecutionManager, CommandPlanner, or SemanticCommandFactory —
    perception produces ToolResult data only, never commands (10-safety-and-trust.md
    trust-boundary rule)."""

    async def get_laser_scan_summary(self, include_raw: bool) -> "LaserScanResult": ...

    async def get_camera_image(self, max_width_px: int) -> "CameraImageResult": ...

    async def detect_objects(self, labels: tuple[str, ...] | None) -> "DetectObjectsResult": ...


class ObjectDetectorPlugin(Protocol):
    """Perception plugin seam (09-perception-architecture.md, 12-plugin-architecture.md)."""

    metadata: "PluginMetadata"

    async def detect(self, image: "RawImage") -> tuple["DetectedObject2D", ...]:
        ...

    async def health_check(self) -> "PluginHealth": ...


@dataclass(frozen=True)
class DetectedObject2D:
    label: str
    confidence: float
    bbox_px: tuple[int, int, int, int]   # x_min, y_min, x_max, y_max
    text_content: str | None = None       # set only by OCR-capable detectors; always
                                           # surfaced as plain data, never re-parsed as
                                           # instructions (10-safety-and-trust.md)
```

## 8. TF Contract

```python
@dataclass(frozen=True)
class TransformResult:
    ok: bool
    x: float | None
    y: float | None
    yaw: float | None
    error: Literal["frame_unknown", "not_connected", "extrapolation", "timeout"] | None


class TFAdapter(Protocol):
    async def lookup_transform(
        self, target_frame: str, source_frame: str, stamp: datetime, timeout_s: float
    ) -> TransformResult:
        """Never raises tf2 exceptions across this boundary; all failure modes are
        values in TransformResult.error."""
        ...

    def known_frames(self) -> frozenset[str]: ...
```

## 9. Subscription Manager Contract

```python
@dataclass(frozen=True)
class CachedMessage:
    value: Any                 # already codec-converted (message_to_ordereddict-shaped)
    raw: Any                   # original rclpy message instance, for adapters that need it
    stamp: datetime            # header.stamp if present, else reception wall-clock
    received_at: datetime      # wall-clock reception time
    age_s: float                # computed relative to caller's now()


class SubscriptionManager(Protocol):
    def ensure_subscribed(self, topic: str, type_name: str, *, qos: "QosPolicy | None" = None) -> None:
        """Idempotent: creates the underlying rclpy subscription at most once per
        (topic, type_name) for the process lifetime. Called by adapters at init, never
        per-request."""
        ...

    def get_latest(self, topic: str) -> CachedMessage | None:
        """Non-blocking. Returns None if never received. MUST NOT create a subscription
        as a side effect (ensure_subscribed is a separate, explicit step) —
        22-topic-subscriptions.md rule: no new subscription per MCP request."""
        ...

    def is_fresh(self, topic: str, max_age_s: float) -> bool: ...
```

## 10. Message Codec Contract

```python
class MessageCodec(Protocol):
    def schema_for(self, type_name: str) -> dict[str, Any]:
        """JSON-Schema-shaped dict for a ROS type string. Cached per type_name for
        process lifetime (02-ros2-architecture.md)."""
        ...

    def to_dict(self, message: Any, *, element_limit: int = 4096) -> dict[str, Any]:
        """Recursive conversion; arrays longer than element_limit are truncated per the
        Binary Data Policy (09-perception-architecture.md), never silently included
        whole."""
        ...

    def from_dict(self, type_name: str, data: dict[str, Any]) -> Any:
        """Inverse of to_dict, used by ros.publish / ros.call_service / ros.send_action_goal."""
        ...
```

## 11. Config Contract

```python
class ConfigProvider(Protocol):
    def robot(self) -> "RobotConfig": ...
    def safety(self) -> "SafetyConfig": ...
    def capabilities_overrides(self) -> "CapabilityConfig": ...
    def raw_ros_access(self) -> "RawRosAccessConfig": ...
    def reload(self) -> None:
        """Re-reads robot.yaml from disk; does NOT retroactively affect
        safety_constraints already resolved into in-flight SemanticCommands
        (05-semantic-command-model.md)."""
        ...
```

Typed config models (`RobotConfig`, `SafetyConfig`, `CapabilityConfig`,
`RawRosAccessConfig`) are `pydantic.BaseModel` subclasses (the one place `pydantic` is
used, per the approved dependency list) — schema given in full in
[16-configuration.md](16-configuration.md).

## 12. Plugin Contract

```python
@dataclass(frozen=True)
class PluginMetadata:
    plugin_id: str
    api_version: str
    provides_capabilities: tuple[str, ...]
    requires: tuple[str, ...]   # dependency strings, e.g. "topic:LaserScan", "package:nav2_msgs"


class PluginHealth(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class Plugin(Protocol):
    metadata: PluginMetadata

    async def health_check(self) -> PluginHealth: ...
```

`MotionBackend`, `NavigationBackend`, and `ObjectDetectorPlugin` all additionally satisfy
`Plugin`.

## 13. Async / Threading Boundary Contract

Two runtimes coexist in one process (see
[ADR-011](adr/ADR-011-async-boundary.md)):

- **rclpy runtime**: a dedicated OS thread running a `rclpy.executors.MultiThreadedExecutor`
  (or `SingleThreadedExecutor` for the MVP's modest callback count — see
  [19](19-deployment-and-scalability.md)) spinning the node. All `rclpy` callbacks
  (subscription callbacks, timer callbacks, action feedback callbacks) execute here.
- **MCP/asyncio runtime**: the main process thread running the MCP server's `asyncio`
  event loop. All tool handlers, the Execution Manager, Safety Engine, Planner run here.

**Bridge contract**: the only permitted crossing points are:

```python
class RosBridge(Protocol):
    def call_soon_threadsafe_from_ros(self, fn: "Callable[[], None]") -> None:
        """Used by rclpy callbacks to hand data to the asyncio loop
        (loop.call_soon_threadsafe)."""
        ...

    async def call_ros_from_asyncio(self, fn: "Callable[[], Any]") -> Any:
        """Used by asyncio-side code that must invoke a synchronous rclpy call
        (e.g. create_publisher, send_goal_async's underlying future) — dispatched onto
        the rclpy executor thread and awaited via a bridged future. Never calls rclpy
        APIs directly from the asyncio thread."""
        ...
```

**Hard rule**: no `rclpy` callback (subscription, timer, action feedback) may `await`
anything or call into `asyncio`/MCP code synchronously — it may only push data through
`call_soon_threadsafe_from_ros` (cheap, non-blocking). No coroutine on the asyncio side
may call a blocking `rclpy` API directly — it must go through `call_ros_from_asyncio`.
This is what guarantees "ROS callbacks never block on MCP work" (MVP acceptance
criterion) structurally rather than by discipline.

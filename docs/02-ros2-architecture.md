# 02 — ROS 2 Architecture

## ROS Graph Discovery Engine

Runs at startup and on a configurable poll interval (default 5s) plus on graph-change
events where `rclpy` exposes them. Responsible for enumerating, via standard `rclpy`
node-graph APIs only (`get_node_names_and_namespaces`, `get_publisher_names_and_types_by_node`,
`get_subscriber_names_and_types_by_node`, `get_service_names_and_types_by_node`,
`get_action_names_and_types` via `rclpy.action`, `get_parameter_names`/parameter services,
TF via `tf2_ros.Buffer`):

- **Nodes**: name, namespace, publishers, subscribers, services, clients, actions, declared parameters.
- **Topics**: name, type string, publisher/subscriber node lists, QoS profile of each endpoint
  (best-effort; not all RMWs report full QoS for remote endpoints), measured frequency
  (rolling window once subscribed), latest message timestamp, `frame_id` when the message
  has a `Header`.
- **Services**: name, type string. Request/response *schema* is derived on demand via
  interface introspection (§Generic Message Handling), not eagerly for every service.
- **Actions**: name, type string (goal/feedback/result triplet), discovered via the
  `<name>/_action/*` topic/service convention `rclpy.action` already relies on.
- **Parameters**: names + types per node, values fetched lazily.
- **TF**: static and dynamic frame list, observed via `tf2_ros.Buffer.all_frames_as_yaml()`
  equivalent (frame existence only — transform *values* are fetched on demand by the TF
  Adapter, not cached wholesale here).

Discovery output is a plain snapshot (`GraphSnapshot`, see [13-contracts.md](13-contracts.md))
— it does not decide meaning. That is the Capability Inference Engine's job
([03-capability-discovery.md](03-capability-discovery.md)).

**No discovery call may block a ROS callback.** Discovery runs on its own timer callback
in the dedicated rclpy executor thread and never awaits MCP/asyncio work (ADR-011).

## Generic Message Handling

The system must operate on **unknown message types** (custom robot messages, custom
actions) without per-type code. This rules out hardcoding a Python class per message.

**Mechanism**: `rosidl` runtime type introspection via `rosidl_runtime_py.utilities` and
`rosidl_parser`/`rosidl_typesupport_introspection_*`. Concretely:

1. Resolve a type string (`"sensor_msgs/msg/LaserScan"`) to the generated Python message
   class via `rosidl_runtime_py.utilities.get_message` / `get_service` / `get_action`
   (this still requires the interface package to be installed/sourced on the server host —
   see Limitations below).
2. For **conversion to JSON/MCP-safe dict**, use
   `rosidl_runtime_py.message_to_ordereddict` recursively; do not hand-roll per-field
   walkers.
3. For **schema generation** (used for `ros.call_service`/`ros.send_action_goal` argument
   validation and for MCP `inputSchema` of generic tools), walk
   `rosidl_parser.definition` field types recursively into a JSON-Schema-shaped dict:
   - primitives → `number`/`integer`/`string`/`boolean`
   - fixed/bounded/unbounded arrays → `array` with `items` and, where bounded, `maxItems`
   - nested messages → nested `object` (recursive, with cycle guard — ROS messages are a
     DAG in practice but the walker still caps recursion depth defensively)
   - `builtin_interfaces/Time`/`Duration` → `{sec: integer, nanosec: integer}` object,
     additionally exposed as a computed ISO-8601 string for readability
   - byte arrays (`uint8[]`, `sensor_msgs/Image.data`, …) → **not** inlined as JSON arrays
     of numbers; represented per the Binary Data Policy ([09](09-perception-architecture.md))
   - string fields conventionally holding UUIDs (`unique_identifier_msgs/UUID.uuid`,
     action `goal_id`) → hex-string representation
4. Conversion is symmetric: the same schema walker underlies both serialization
   (message → dict) and deserialization (dict → message, for `ros.publish` /
   `ros.call_service` / `ros.send_action_goal`), so there is one source of truth for the
   mapping, not two.

This logic lives behind a single seam, `MessageCodec` (frozen in
[13-contracts.md](13-contracts.md)), so it can later be swapped for a faster
introspection strategy (e.g. precompiled per-type codecs generated at discovery time)
without touching callers.

### Limitations of Introspection-Based Handling

- The interface package (`.msg`/`.srv`/`.action` Python bindings) must be importable on
  the ROS-MCP-Server host — i.e. sourced in the same workspace as the robot's custom
  interfaces. This is not a source-code modification of the robot, but it **is** a
  deployment requirement: the server's environment must have the robot's interface
  packages built and on `AMENT_PREFIX_PATH`. Documented explicitly as a zero-modification
  caveat (see [00-overview.md](00-overview.md) principle 2 and ADR-007).
- Very large arrays (raw point clouds, uncompressed images) convert correctly but are
  never surfaced whole into MCP — see the Binary Data Policy.
- Introspection cost (recursive walk) is paid once per *type* and cached
  (`MessageCodec` caches schema-per-typestring for process lifetime); it is not repeated
  per message instance beyond the actual `message_to_ordereddict` call.

## Topic / Service / Action Adapters

Three thin, generic wrappers over `rclpy`, each stateless with respect to business logic
and used both by semantic adapters (Motion, Nav2, Perception) and by the gated raw-ROS
tools ([04-mcp-surface.md](04-mcp-surface.md) §Raw ROS Tools):

- **TopicAdapter** — creates/reuses a publisher or, for reads, delegates to the
  Subscription Manager rather than creating a bare subscription (see below).
- **ServiceAdapter** — resolves type, builds request from a validated dict via
  `MessageCodec`, calls async, converts response, enforces a per-call timeout.
- **ActionAdapter** — resolves type, sends goal via `ActionClient`, exposes
  goal-handle/feedback/result as an `asyncio`-friendly interface bridged from
  rclpy's callback-based `ActionClient` (see ADR-011 for the bridging mechanism).

## QoS Handling

Discovery records each endpoint's QoS where the RMW reports it. When the server creates
its **own** subscriptions (via the Subscription Manager) it does not blindly assume
`SensorDataQoS` or `reliable`:

- If the topic already has publishers, the Subscription Manager queries their QoS and
  creates a *QoS-compatible* subscription (matching reliability/durability, using
  `qos_profile_sensor_data` defaults specifically for image/scan/pointcloud topics per
  REP-2003 unless the robot's own publisher disagrees).
- If no publisher exists yet (subscribing ahead of the source node starting), it falls
  back to type-based conventions (sensor topics → `SensorDataQoS`; everything else →
  `QoSProfile` default reliable/volatile) and logs the assumption at `WARNING`.
- QoS mismatches that silently drop all messages are a known ROS 2 failure mode; the
  Subscription Manager's freshness check (data never arrives) surfaces this as
  `SENSOR_STALE` rather than hanging silently — see
  [15-observability-and-failure-handling.md](15-observability-and-failure-handling.md).

## TF Adapter

Wraps one shared `tf2_ros.Buffer` + `TransformListener` for the whole process (not one
per adapter). Provides `lookup_transform(target, source, time, timeout)` returning either
a transform or a typed `TFUnavailable` outcome (frames not yet connected, extrapolation
into the future, timeout) — never raises a raw `tf2` exception across the adapter
boundary. See [13-contracts.md](13-contracts.md) for the exact signature and
[03-safety semantics] in [10-safety-and-trust.md](10-safety-and-trust.md) for how
`TF_UNAVAILABLE` propagates as a structured tool error.

## Why This Layer Exists Separately From "Adapters"

Discovery/introspection/QoS/TF are **mechanical ROS concerns** — they know nothing about
"navigate" or "move." Motion/Nav2/Perception adapters are **semantic concerns** — they
know nothing about how to walk a `rosidl` type tree. Keeping the boundary here means a
new ROS distribution (Humble → Jazzy → …) or RMW only touches this layer
([ADR-007](adr/ADR-007-generic-message-introspection.md),
[ADR-013 versioning notes](adr/ADR-013-config-overrides-discovery.md)).

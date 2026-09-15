# 15 — Observability & Failure Handling

## Telemetry Subsystem

- **Structured logs**: every log line is structured (JSON in production, human-readable
  in dev via a formatter toggle), carrying at minimum `command_id`/`session_id` when
  applicable — implemented with Python's `logging` + a structured adapter (no new
  dependency: stdlib `logging` with a `JSONFormatter` the project owns; see
  [19](19-deployment-and-scalability.md) tech choices).
- **Metrics**: counters/histograms for tool-call latency (per tool), command outcomes by
  terminal state, safety rejections by reason, sensor staleness events, Nav2
  success/failure/cancel counts. MVP exposes these via a `MetricsSink` interface
  (contracts in [13](13-contracts.md), implementation may be a simple in-memory
  counter + periodic log line for MVP; OpenTelemetry export is the Phase-5 target, not
  required for MVP — see [20-roadmap.md](20-roadmap.md)).
- **Tracing**: each `SemanticCommand`'s `command_id` is the trace-correlation key across
  Validation → Safety → Planner → Execution → Adapter → Result; OpenTelemetry spans are
  the intended future backing (`opentelemetry-sdk` would be an added dependency —
  requires approval per the constraints, not added in MVP).
- **Command audit history**: append-only log (file-backed for MVP, structured JSON lines)
  of every `SemanticCommand` from `RECEIVED` to terminal state, including `provenance`
  and the resolved `safety_constraints` it was checked against — this is the record that
  answers "why did the robot move" after the fact.
- **Sensor freshness**: the Subscription Manager exposes `is_fresh()` (§[13](13-contracts.md));
  telemetry samples per-topic freshness on each discovery poll and logs a `WARNING` (and
  increments a counter) the first time a previously-fresh topic goes stale, rather than
  logging every tick.

## Failure Handling Matrix

| Failure | Detected by | Behavior | Surfaced as |
|---|---|---|---|
| ROS node unavailable at startup | Discovery Engine finds zero matching interfaces | Capability simply absent from registry; server still starts | `robot.get_capabilities` shows nothing for it; any tool needing it returns `CAPABILITY_UNAVAILABLE` |
| Topic disappears mid-session | Subscription Manager stops receiving; freshness check trips | Cached last-value ages out | `SENSOR_STALE` on reads; Motion Adapter safety-stops if it was mid-move |
| Action server unavailable (Nav2 down) | Nav2 Adapter's live server-presence check before `send_goal_async` | No goal sent | `CAPABILITY_UNAVAILABLE` |
| Stale sensor | Subscription Manager `age_s > threshold` | Read tools flag `stale: true`; Motion Adapter hard-fails | `SENSOR_STALE` (motion), soft flag (read-only queries) |
| TF unavailable | TF Adapter lookup fails/timeouts | Transform not computed | `TF_UNAVAILABLE` (perception objects still returned with `position: null`; navigate returns `CAPABILITY_UNAVAILABLE`) |
| Nav2 unavailable | Planner + Nav2 Adapter availability check | No dispatch | `CAPABILITY_UNAVAILABLE` |
| Invalid goal (bad frame, NaN, out of geofence) | Validation Engine / Safety Engine | Rejected before dispatch | `INVALID_ARGUMENT` / `INVALID_FRAME` / `SAFETY_REJECTED` |
| Timeout (any command) | Execution Manager deadline + adapter self-timeout | Cancellation signaled, zero-velocity published for motion | `TIMEOUT`, `stop_reason`/`fail_reason` populated |
| Robot emergency-stop tripped | Configured e-stop signal check | All MOTION commands rejected pre-dispatch | `ROBOT_NOT_READY` |
| Network/DDS disconnect | rclpy executor / subscription silence | Same as "topic disappears" | `SENSOR_STALE`, watchdog zero-publish |
| LLM/MCP client disconnect mid-command | MCP transport close event | In-flight MOTION command is cancelled by the server (never left running unsupervised) | N/A (no client to receive result; audit log still records `CANCELLED`, reason `client_disconnected`) |
| MCP reconnect | New session established | Fresh `session_id`; prior session's in-flight commands (if still running) remain owned by their original `command_id`/`session_id`, queryable via `robot.get_state` | N/A |
| Malformed ROS message (introspection failure) | MessageCodec raises during `to_dict` | Caught at adapter boundary | `ROS_INTERFACE_ERROR` |
| Incompatible ROS distro / missing interface package | Discovery/Codec import failure for a type | That capability/topic excluded, logged at `ERROR` once | Absent from capabilities; `ROS_INTERFACE_ERROR` if explicitly requested via raw tools |
| Unknown message type (raw tools) | MessageCodec `schema_for` fails to resolve | Rejected before any publish/call | `ROS_INTERFACE_ERROR` |

**Fail-safe default**: every row above resolves to "robot stops/does not move and the
caller gets a clear structured error" — never "robot does something unspecified."

## Error Surfacing to the LLM

`ToolError.code` is one of the closed `ErrorCode` enum values (§[13](13-contracts.md));
`message` is a short human-readable sentence; `details` carries structured extras (e.g.
`{"requested_frame": "kitchen", "known_frames": ["map", "odom", "base_link", ...]}` for
`INVALID_FRAME`) so the LLM can reason about *why* and potentially retry with corrected
arguments rather than just seeing a failure.

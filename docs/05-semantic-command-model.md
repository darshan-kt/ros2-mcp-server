# 05 — Semantic Command Model

Every MCP tool invocation that has a robot-side effect (i.e. every tool except pure
resource reads) is first translated into exactly one `SemanticCommand`. This is the
single data structure that flows through Validation → Safety → Planner → Execution
Manager → Adapter. Its full typed definition is frozen in
[13-contracts.md](13-contracts.md) §`SemanticCommand`; this document explains the *why*
of each field.

## Fields and Rationale

| Field | Type | Why |
|---|---|---|
| `command_id` | `str` (ULID) | Globally unique, time-sortable. Used for idempotency/dedup (§[06](06-execution.md)), correlation in logs/telemetry, and as the cancellation handle. |
| `session_id` | `str` | Ties the command to an MCP session; lets per-session rate limiting and audit scoping work (§[10](10-safety-and-trust.md)). |
| `robot_id` | `str` | Even in the single-robot MVP, every command is robot-scoped from day one so multi-robot (§[14](14-multi-robot.md)) is additive, not a breaking change. |
| `operation` | `Operation` enum: `MOVE, STOP, NAVIGATE, GET_STATE, GET_CAPABILITIES, GET_LASER_SCAN, GET_CAMERA_IMAGE, DETECT_OBJECTS, RAW_*` | Backend-agnostic verb. The Planner, not the tool layer, decides which adapter realizes it. |
| `target` | `Target \| None` | Structured goal (e.g. `{x, y, yaw, frame}` for `NAVIGATE`, `{direction, distance_m}` for `MOVE`). Typed per-operation, never a free-form dict the adapter has to sniff. |
| `frame` | `str \| None` | Coordinate frame for `target`, when spatial. Validated against the TF Adapter's known frames before execution (`INVALID_FRAME` otherwise). |
| `timeout_s` | `float` | Every command has one, even if the client didn't ask — defaulted from `robot.yaml` per operation. No command runs unbounded (§[07](07-motion-architecture.md), §[08](08-navigation-architecture.md)). |
| `priority` | `Priority` enum: `NORMAL, SAFETY` | `STOP`/cancellation-triggering commands are `SAFETY` priority and pre-empt queued `NORMAL` commands in the Execution Manager. |
| `safety_constraints` | `SafetyConstraints` | Snapshot of the *resolved* limits (max velocity/accel/distance) this specific command will be checked against — resolved once at validation time from config, so later config reloads can't retroactively change an in-flight command's limits. |
| `cancellation_token` | internal, not client-supplied | Created by the Execution Manager; adapters poll/react to it. Not part of the wire schema. |
| `expected_result_type` | `type[ToolResult subtype]` | Lets the Execution Manager type-check the adapter's result before returning it — a coding error in an adapter can't silently produce a malformed MCP response. |
| `provenance` | `Provenance` | `{source: "mcp_tool_call", tool_name, raw_arguments, client_session_id}` — full traceability from "why did the robot move" back to the exact tool call, required for the audit log (§[10](10-safety-and-trust.md)). |
| `confidence` | `float \| None` | Only set when the command was produced from an ambiguous upstream inference (e.g. a future "navigate to the object I just detected" workflow substituting a perception-derived coordinate). `None`/`1.0` for direct user-specified numeric targets. Low confidence can trigger human-approval policy (§[10](10-safety-and-trust.md) HITL). |
| `execution_state` | `ExecutionState` enum | See [06-execution.md](06-execution.md) for the full state machine; this is the live/mutable field the Execution Manager updates. |
| `created_at` | `datetime` (UTC) | Used for timeout computation and audit ordering. |

## Example

```text
User: "Navigate to x=1, y=2."

SemanticCommand(
  command_id="01J...ULID",
  session_id="sess-abc123",
  robot_id="turtlebot3_waffle",
  operation=Operation.NAVIGATE,
  target=NavigateTarget(x=1.0, y=2.0, yaw=None),
  frame="map",
  timeout_s=120.0,               # from config nav2.default_timeout_s
  priority=Priority.NORMAL,
  safety_constraints=SafetyConstraints(max_navigation_distance_m=10.0, ...),
  expected_result_type=NavigateResult,
  provenance=Provenance(source="mcp_tool_call", tool_name="robot.navigate",
                         raw_arguments={"x": 1.0, "y": 2.0}, client_session_id="sess-abc123"),
  confidence=None,
  execution_state=ExecutionState.RECEIVED,
  created_at=<utcnow>,
)
```

## Design Note: Why Not Just Pass the Tool Arguments Through?

Tool arguments are client-facing and minimal by design (§[04](04-mcp-surface.md)). The
`SemanticCommand` is server-internal and carries everything downstream stages need so
that:

- the Safety Engine never re-derives limits from config mid-flight (race-free — limits
  are resolved once, at validation, into `safety_constraints`);
- the audit log has one canonical record per command, independent of tool-schema
  evolution;
- cancellation, dedup, and state transitions have one stable identity
  (`command_id`) instead of being inferred from tool-call shape.

See [ADR-004](adr/ADR-004-semantic-command-and-state-machine.md).

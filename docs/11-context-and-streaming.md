# 11 — Context & Streaming Architecture

## Context Management Strategy

A robot's ROS graph can have hundreds of topics; none of that is dumped into LLM context.

```text
Robot
 ├── capabilities        ← robot.get_capabilities (small, ranked list)
 ├── state               ← robot.get_state (pose/velocity/active command only)
 ├── graph-summary        ← robot://graph-summary resource (counts, not a topic dump)
 └── (perception/nav results are returned only when the matching tool is called)
```

Principles:

1. **Lazy discovery, eager capability summary.** Full `GraphSnapshot` detail
   (§[02](02-ros2-architecture.md)) stays server-side; only the Capability Registry's
   compressed view — a handful of capability IDs with confidence/limits — is exposed
   through `robot.get_capabilities`/`robot://capabilities`.
2. **Semantic indexing, not raw listing.** `robot://graph-summary` reports aggregate
   counts (`{node_count: 12, topic_count_by_category: {sensor: 4, control: 2, tf: 2,
   diagnostic: 1, other: 3}, action_servers: ["/navigate_to_pose"], tf_root: "map",
   tf_leaves: ["base_scan", "camera_link"]}`), never a per-topic enumeration — an
   operator debugging via raw ROS tools can still get the full list via
   `ros.read_topic`/CLI, but that is not what ships into LLM context by default.
3. **Caching + freshness, not re-computation.** All resource reads are served from the
   Subscription Manager's last-value cache / Capability Registry snapshot — computing a
   resource never triggers a fresh ROS round-trip inline in the read path except for TF
   lookups (bounded, timeout-guarded) and camera/scan acquisition (already
   cache-first, §[09](09-perception-architecture.md)).
4. **Context prioritization**: when a tool result must be trimmed to stay under
   `max_result_bytes` (§[09](09-perception-architecture.md) Binary Data Policy), the
   trim order is: raw arrays first (e.g. `raw_ranges`, full detection bbox pixel masks if
   ever added), then lower-confidence detections, never the top-level status/error
   fields.
5. **Event-driven updates over polling.** The LLM should not need to call
   `robot.get_state` in a loop to know a `navigate` call is progressing — see Streaming
   below.

## Streaming / Event Architecture

ROS-side events map onto MCP as follows:

| ROS-side event | MCP-side representation |
|---|---|
| Nav2 action feedback (`distance_remaining`, …) | MCP progress notification correlated to the tool call's `command_id`, throttled (default 1 Hz) |
| Motion adapter control-loop tick | **not** streamed by default (20 Hz is too fast to be useful); only terminal result is returned. A future `verbose: true` flag could opt in, post-MVP. |
| Command reaching a terminal state (`SUCCEEDED`/`FAILED`/…) | The tool call's own response (MVP tools are request/response with progress notifications in between — not fire-and-forget) |
| Sensor becoming stale (`SENSOR_STALE` transition) | Logged + counted in telemetry (§[15](15-observability-and-failure-handling.md)); **not** pushed as an unsolicited MCP message in MVP (no ambient "battery low" push channel without an explicit subscription mechanism, which is post-MVP — see [20-roadmap.md](20-roadmap.md)) |

MVP explicitly ships **request/response + in-flight progress notifications for
`robot.navigate`** (the one command that can meaningfully run for tens of seconds) and
**not** an ambient event-subscription tool (`robot.subscribe_to_events`) — that is a
Phase-2+ extension once multiple concurrent clients/sessions exist and a "who gets which
events" model is needed (§[20-roadmap.md](20-roadmap.md)).

## Cancellation Across the Streaming Boundary

An in-flight `robot.navigate` progress stream stops the moment the command reaches a
terminal state; a client-initiated `robot.stop` call is guaranteed (by the Execution
Manager's priority pre-emption, §[06](06-execution.md)) to produce one final terminal
notification (`cancelled`) for the in-flight command before `robot.stop`'s own result is
returned.

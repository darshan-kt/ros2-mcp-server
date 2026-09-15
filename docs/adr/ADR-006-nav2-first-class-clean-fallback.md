# ADR-006: Nav2 as First-Class Navigation Backend With Clean `CAPABILITY_UNAVAILABLE`

**Status**: Accepted

## Decision

Nav2 presence and localization health are re-verified at dispatch time (not just at
discovery time) before every `NavigateToPose` goal; absence at either check produces a
structured `CAPABILITY_UNAVAILABLE` result, never a hang or an unhandled exception
(§[08](../08-navigation-architecture.md)).

## Why

Discovery is periodic (default 5 s); Nav2 or AMCL could go down between polls. Sending a
goal to a dead action server has ROS-specific failure modes (the action client may wait
indefinitely for `send_goal_async` to resolve, depending on RMW/QoS) that would otherwise
surface as an opaque timeout to the LLM instead of a clear, immediate, correctly-labeled
error.

## Alternatives Considered

- **Trust the discovery snapshot's capability entry without re-checking at dispatch.**
  Rejected: stale-by-seconds capability state could send a goal into the void.
- **Let `send_goal_async` simply time out and surface a generic `ACTION_TIMEOUT`.**
  Rejected: conflates "Nav2 isn't there" (a capability question, answerable instantly)
  with "Nav2 is there but struggling" (a real timeout, answerable only after waiting) —
  these need different `ErrorCode`s and different latency for the caller.

## Tradeoffs

- Pro: fast, correctly-labeled failure when Nav2/localization isn't ready; no risk of
  indefinitely blocking a `robot.navigate` call against a server that no longer exists.
  Con: one extra liveness check per navigate call — negligible cost (a graph query,
  already cached by Discovery/TF Adapter).

## Failure Modes

- A liveness check that itself blocks (e.g. a synchronous `rclpy` graph call on the
  asyncio thread) would violate the async boundary (ADR-011) — implementation must route
  this check through `RosBridge.call_ros_from_asyncio`.

## Recommendation

Keep the pre-dispatch liveness + localization check as a mandatory step in
`Nav2NavigationPlugin.navigate()`; do not optimize it away for latency without measuring
that it's actually a bottleneck (it should not be, given caching).

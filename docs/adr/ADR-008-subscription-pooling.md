# ADR-008: Subscription Pooling With Last-Value Cache, Not Per-Request Subscriptions

**Status**: Accepted

## Decision

A single `SubscriptionManager` owns at most one `rclpy` subscription per
`(topic, type_name)` for the process lifetime, maintaining a last-value cache with
timestamps. Adapters call `ensure_subscribed` once at initialization and read via
non-blocking `get_latest`; no adapter or tool handler creates a subscription per MCP
request (§[13-contracts.md](../13-contracts.md) §9).

## Why

A camera topic at 30 FPS or a laser scan at 10+ Hz would, under a naive
"subscribe-per-request" model, create and tear down subscriptions constantly (expensive
DDS discovery churn) while still only ever delivering one value per request — the LLM
never needs the full stream rate. Pooling decouples "how fast the sensor publishes" from
"how often the LLM asks."

## Alternatives Considered

- **Create a fresh subscription, wait for one message, destroy it — per tool call.**
  Rejected: DDS discovery overhead per call, unbounded wait if the topic is briefly
  silent, and no shared freshness/staleness policy across callers.
- **Subscribe once but store only the raw rclpy message with no freshness metadata.**
  Rejected: freshness/staleness checking is load-bearing for both safety (stale odometry
  → abort motion) and read-tool honesty (`stale: true` flag) — it has to be a first-class
  part of the cache entry, not bolted on separately per caller.

## Tradeoffs

- Pro: bounded subscription count regardless of request rate; consistent freshness
  semantics everywhere; cheap reads.
  Con: the cache can serve a value slightly older than "right now" — bounded by the
  topic's own publish rate and explicitly reported via `age_s`, which is an acceptable
  and honestly-reported latency, not a hidden one.

## Failure Modes

- A topic that stops publishing silently ages out of freshness — correctly surfaced as
  `SENSOR_STALE` rather than the cache silently returning to fresh once refreshed later
  with a gap unnoticed (fresh comparisons are always against wall-clock `now()`, not
  "was ever fresh").
- QoS mismatch causing zero messages ever received is indistinguishable from "topic not
  yet started" until the freshness/never-received distinction is checked —
  `get_latest` returning `None` vs. a `CachedMessage` with large `age_s` disambiguates
  this for callers.

## Recommendation

Keep pooling mandatory; any new adapter must reuse `SubscriptionManager` rather than
creating its own `rclpy` subscription.

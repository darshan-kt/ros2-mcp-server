# ADR-003: Capability Inference With Confidence Levels + Config Override

**Status**: Accepted

## Decision

Capabilities are inferred from the live ROS graph against a small fixed ontology
(§[03](../03-capability-discovery.md)), each assigned `CONFIRMED`/`LIKELY`/`AMBIGUOUS`
confidence. `AMBIGUOUS` capabilities never become MCP tools automatically. Explicit
`robot.yaml` config always overrides inference at any confidence level.

## Why

"Every `/cmd_vel` means the same thing" is false in general — some robots use `/cmd_vel`
for a base while a second `/cmd_vel`-shaped topic exists for a towed platform, etc.
Blind inference risks wiring the wrong topic to a semantic tool; blind non-inference
(requiring full manual config for every robot) defeats the "zero configuration for the
common case" goal. Confidence levels let the common case (one `Twist` publisher, one
`LaserScan`) auto-configure while ambiguous cases surface for a human decision instead of
guessing.

## Alternatives Considered

- **Always require explicit config, no inference.** Rejected: reintroduces per-robot
  integration work the whole project exists to avoid.
- **Always trust inference, even when ambiguous (pick the "most likely" candidate
  silently).** Rejected: silent wrong guesses on an actuation-relevant mapping
  (which topic is "the" cmd_vel) are exactly the kind of speculation §41 of the source
  brief warns against.

## Tradeoffs

- Pro: zero-config for straightforward robots; safe degradation (tool simply doesn't
  appear) for ambiguous robots rather than a wrong guess.
  Con: a genuinely ambiguous robot requires one-time config work before `robot.move`
  appears — acceptable, since the alternative is a potentially wrong topic being driven.

## Failure Modes

- Ontology pattern too narrow (e.g. assumes `Twist`, robot uses `TwistStamped` only) →
  `LIKELY`/`AMBIGUOUS` catches type mismatches; pattern set is versioned and extensible
  without breaking existing config.
- Config drifts from the real robot after a topic rename → discovery re-polls and will
  flag the configured topic as absent (`CAPABILITY_UNAVAILABLE` for that capability)
  rather than silently keeping a stale mapping.

## Recommendation

Keep the ontology intentionally small; resist adding a new capability ID for every
robot-specific variation — use config overrides and, if a genuinely new semantic category
emerges repeatedly across robots, add it deliberately via a new ADR-level discussion, not
ad hoc.

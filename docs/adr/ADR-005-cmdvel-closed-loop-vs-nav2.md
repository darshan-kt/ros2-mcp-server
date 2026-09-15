# ADR-005: Closed-Loop `/cmd_vel` for Relative Motion; Nav2 for Absolute Goals

**Status**: Accepted

## Decision

`robot.move` is realized as a closed-loop controller against `/odom` (+ `/scan` obstacle
check) publishing `/cmd_vel`, used only for short relative motions. `robot.navigate`
(absolute, `map`-frame goals) always routes to Nav2 and never falls back to a
reinterpreted `/cmd_vel` sequence when Nav2 is unavailable (§[07](../07-motion-architecture.md),
§[08](../08-navigation-architecture.md)).

## Why

Nav2 provides path planning and costmap-based obstacle avoidance that a local velocity
controller fundamentally cannot; using it for absolute goals whenever available is
strictly safer. But requiring Nav2 for *every* motion (including "move forward 1 meter")
would make the simplest, most common request depend on a heavier stack and fail on robots
without Nav2 configured. A closed-loop `/cmd_vel` controller is the right tool for
short, locally-supervised motion specifically because it has real feedback and real
limits — it is not "blindly publish and hope," which is the actual anti-pattern.

## Alternatives Considered

- **Blindly publish `/cmd_vel` for a computed duration.** Rejected outright — see
  [07-motion-architecture.md](../07-motion-architecture.md) "Why Blindly Publishing
  `/cmd_vel` Is Unsafe."
- **Always require Nav2, even for `robot.move`.** Rejected: unnecessary dependency for
  trivial relative motion, and Nav2's `NavigateToPose` is not naturally expressed as
  "rotate 90 degrees in place" without first establishing localization, which many valid
  simple robots won't have.
- **Silently reinterpret a `navigate` goal as a `move` sequence when Nav2 is absent.**
  Rejected: an absolute goal reinterpreted as local motion has no path planning or global
  obstacle awareness — could drive the robot somewhere Nav2 would have avoided. Explicit
  `CAPABILITY_UNAVAILABLE` lets the caller (LLM/user) make an informed choice instead.

## Tradeoffs

- Pro: simple robots (no Nav2) still get safe relative motion; Nav2-equipped robots get
  full planning for absolute goals; the two tools have clearly distinct, non-overlapping
  semantics the LLM can reason about.
  Con: a robot with Nav2 but issued a series of `robot.move` calls to approximate
  navigation loses planning/obstacle-avoidance benefits — acceptable since `robot.move`
  is documented as local/supervised, not a Nav2 substitute.

## Failure Modes

- Closed-loop controller's own obstacle check (single-sector laser threshold) is a
  backstop, not full costmap avoidance — a fast-moving obstacle from the side could still
  be missed between control ticks at 20 Hz; documented as a known limitation, mitigated
  by keeping `robot.move` distances/velocities conservative by default config.

## Recommendation

Keep this split permanently; do not add an automatic Nav2-unavailable → cmd_vel fallback
for `navigate` even as a "best effort" feature — the clean-failure behavior is a
deliberate safety property, not a gap.

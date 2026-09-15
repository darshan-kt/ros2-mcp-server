# 18 — Testing Strategy

## Unit Tests (no live ROS required)

Run via `python -m pytest tests/ -q`. ROS is mocked at the `rclpy` API boundary — tests
construct fake `GraphSnapshot`s, fake `CachedMessage`s, and fake rclpy message instances
(plain objects with the expected fields, since `rosidl` message classes are plain
attribute-bearing objects) rather than requiring a running ROS graph.

| Area | What's tested |
|---|---|
| Message conversion (`MessageCodec`) | `to_dict`/`from_dict` round-trip for `Twist`, `Odometry`, `LaserScan`, nested/array/`Header`/`Time` fields; truncation behavior above `element_limit`. |
| Safety policy | Each rejection path (velocity, acceleration-implied, distance, geofence, e-stop, rate-limit) independently, using a fake `SafetyPolicyEngine` input `SemanticCommand`; asserts `SafetyOutcome.decision == DENY` with the right `ErrorCode`, **and** that no adapter method was ever invoked (spy assertion). |
| Scan processing | Artifact/invalid/inf classification table (§[09](09-perception-architecture.md)) against synthetic `LaserScan` fixtures; sector boundary correctness; nearest/farthest edge cases (all-inf scan, all-invalid scan). |
| Command state machine | Every transition in §[06](06-execution.md), including cancellation pre-empting `EXECUTING`/`MONITORING`, dedup-by-`command_id` returning the same future, timeout firing independent of adapter cooperation. |
| Capability inference | Pure-function tests (`CapabilityInferenceEngine.infer`) against synthetic `GraphSnapshot`s: confirmed/likely/ambiguous cases, config-override precedence. |
| Config loading | Valid/invalid `robot.yaml` fixtures; `pydantic` validation error surfaces clearly. |

## Integration Tests (require a running ROS graph; may use a minimal fake-node harness)

- MCP tool call → Execution Manager → a **fake** `MotionBackend`/`NavigationBackend`
  double, verifying the full pipeline wiring without needing Gazebo.
- Subscription Manager against a real `rclpy` publisher node in-process, verifying
  pooling (one subscription per topic across N concurrent `get_latest` calls) and
  freshness behavior.
- TF Adapter against a real `tf2_ros` static broadcaster, verifying `TransformResult`
  success/`frame_unknown`/`timeout` cases.

## Simulation Tests (Gazebo Classic + `turtlebot3_gazebo`)

Exercises the real `CmdVelMotionPlugin` and `Nav2NavigationPlugin` against a live
simulated robot — this is where the acceptance-criteria behaviors in
[17-mvp.md](17-mvp.md) (`move` stopping within 10 cm, `navigate` reaching a goal) are
actually demonstrated, not merely unit-asserted. Run manually / via a scripted launch for
MVP; CI automation of the sim run is a Phase-5 concern (§[20](20-roadmap.md)).

## End-to-End (natural language → MCP → ROS → robot)

Manual for MVP: drive the server via Claude Desktop against the running sim and observe
the five example flows (§[26 flows in the architecture note](00-overview.md) /
`examples/`) actually execute. Not automated in MVP CI (would require driving an LLM in
CI, out of scope).

## Fault Injection

Unit/integration level:
- Kill the fake odometry publisher mid-`move` → assert `SENSOR_STALE` + zero-velocity
  publish.
- Fake obstacle appearing mid-`move` (inject a close-range scan) → assert
  `stop_reason == "obstacle"`.
- Fake Nav2 action server absent → assert `CAPABILITY_UNAVAILABLE`, no goal ever sent.
- Fake Nav2 goal accepted then never completing → assert `timeout_s` still fires and
  `cancel_goal_async` is called.
- Malformed/unregistered ROS type requested via raw tools (when enabled) → assert
  `ROS_INTERFACE_ERROR`, no crash.

## What "Passing" Means for the MVP

All boxes in [17-mvp.md](17-mvp.md) §Acceptance Criteria checked, with real command
output pasted per the seven build steps — no acceptance criterion is marked done from
code inspection alone.

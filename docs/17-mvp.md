# 17 — MVP (Frozen Scope)

## Purpose

The smallest implementation that genuinely proves the architecture end-to-end:

```text
Claude
 ↓ MCP (stdio)
ROS-MCP-Server
 ↓ rclpy
ROS 2 differential-drive robot (reference: TurtleBot3, Gazebo Classic sim)
```

## Reference Platform

TurtleBot3 (Waffle or Burger) under `turtlebot3_gazebo`, ROS 2 Humble, Gazebo Classic 11.
Assumed topics/actions present, unmodified:

```text
/cmd_vel            geometry_msgs/msg/Twist
/odom               nav_msgs/msg/Odometry
/scan               sensor_msgs/msg/LaserScan
/camera/image_raw   sensor_msgs/msg/Image        (if camera-equipped model/world used)
/camera/camera_info sensor_msgs/msg/CameraInfo
/tf, /tf_static
/navigate_to_pose   nav2_msgs/action/NavigateToPose   (when Nav2 bring-up is running)
/amcl_pose          geometry_msgs/msg/PoseWithCovarianceStamped  (when localized)
```

Nav2 and AMCL are **not** required for the server to start — `robot.navigate` degrades
cleanly to `CAPABILITY_UNAVAILABLE` when they are absent, and the rest of the tool set
(`move`, `stop`, `get_state`, `get_laser_scan`, `get_camera_image`, `detect_objects`,
`get_capabilities`) works against `/cmd_vel` + `/odom` + `/scan` + camera alone.

## MVP Tool Set (exactly these eight — no more, no fewer)

`robot.get_capabilities`, `robot.get_state`, `robot.move`, `robot.stop`,
`robot.navigate`, `robot.get_laser_scan`, `robot.get_camera_image`,
`robot.detect_objects`. Full schemas: [04-mcp-surface.md](04-mcp-surface.md).

## MVP Resources

`robot://state`, `robot://capabilities`, `robot://graph-summary`.

## Required Behaviors (acceptance-relevant, restated from across this doc set)

1. Discovery infers capabilities from the live graph at startup; the tool list reflects
   only what is actually present (§[03](03-capability-discovery.md)).
2. `robot.move` is closed-loop against `/odom`, enforces velocity/acceleration/distance/
   timeout limits, aborts on obstacle (`/scan`) or stale odometry, and never publishes an
   unbounded/open-loop velocity stream (§[07](07-motion-architecture.md)).
3. `robot.navigate` uses `NavigateToPose` with feedback streaming and cancellation, and
   returns `CAPABILITY_UNAVAILABLE` cleanly (not a hang, not a crash) when Nav2 or
   localization is absent (§[08](08-navigation-architecture.md)).
4. Every tool returns the structured result/error model of
   [13-contracts.md](13-contracts.md) — no bare strings.
5. Safety limits load from `config/robot.yaml` and are enforced in the Safety & Policy
   Engine, which the MCP tool-call path cannot bypass by construction
   (§[10](10-safety-and-trust.md), §[13](13-contracts.md) Non-Bypass Rule).
6. ROS callbacks never block on MCP work; the Subscription Manager pools subscriptions
   with a last-value cache and freshness checks (§[13](13-contracts.md) §9, §13).
7. `robot.detect_objects` is served by a stub detector plugin behind the real
   `ObjectDetectorPlugin` interface (§[09](09-perception-architecture.md),
   [12](12-plugin-architecture.md)) — the pipeline (acquire → detect → geometry → TF →
   result) is real; the model behind step 2 is a placeholder.

## Explicitly Out of Scope for MVP

- Manipulation/MoveIt, docking, mapping/SLAM control.
- Multi-robot (§[14](14-multi-robot.md)) — single `robot_id`, single process.
- Authentication/authorization beyond the OS process boundary of local stdio
  (§[10](10-safety-and-trust.md) Security Layer, MVP note).
- Raw ROS tools (`ros.*`) — defined in the contract, `disabled` by default, not exercised
  by MVP acceptance tests.
- `NavigateThroughPoses`, Nav2 planner/controller/recovery introspection, `robot.diagnose`.
- Ambient event-subscription tool / push notifications beyond in-flight `navigate`
  progress (§[11](11-context-and-streaming.md)).
- HITL approval UI/channel (the `AWAITING_APPROVAL` state and `policy_based` distance
  rule are implemented; the actual approval-collection transport is not).
- OpenTelemetry export (structured logs + in-process metrics counters suffice for MVP,
  §[15](15-observability-and-failure-handling.md)).

## Build Order (each step independently runnable/testable)

1. **Discovery** — `DiscoveryEngine` against a live or absent ROS graph; unit-testable
   with a mocked `rclpy` node graph.
2. **State / Capabilities** — `CapabilityInferenceEngine` + `CapabilityRegistry` +
   `robot.get_state`/`robot.get_capabilities` tool handlers, `Subscription Manager` for
   `/odom`.
3. **Safety** — `ValidationEngine` + `SafetyPolicyEngine` wired into the (not-yet-adapter-
   connected) Execution Manager skeleton; unit tests assert rejection paths in isolation.
4. **Move / Stop** — `CmdVelMotionPlugin` implementing `MotionBackend`, closed loop per
   [07](07-motion-architecture.md), wired through Execution Manager + Safety Engine.
5. **Scan / Camera** — `PerceptionAdapter` laser-scan summary + camera acquisition
   (no detection yet).
6. **Navigate** — `Nav2NavigationPlugin` implementing `NavigationBackend`.
7. **Detect stub** — `StubDetectorPlugin` implementing `ObjectDetectorPlugin`, wired into
   `robot.detect_objects`.

## Acceptance Criteria (frozen; implementation must satisfy all)

- [ ] `python -m pytest tests/ -q` passes; unit tests cover message conversion, safety
      rejection, scan processing, and the command state machine, with ROS mocked (no live
      ROS required to run this suite).
- [ ] `python -m ros_mcp.server --config config/robot.yaml` starts and serves over stdio
      with no robot present, reporting zero capabilities rather than crashing.
- [ ] With `turtlebot3_gazebo` running: `get_state` returns a pose; `get_laser_scan`
      returns structured ranges with nearest-obstacle geometry; `move(forward, 1.0)`
      stops within 10 cm of target; `stop` halts immediately; `navigate` reaches a valid
      goal and reports final pose.
- [ ] A `move` request exceeding the configured distance or velocity limit returns
      `SAFETY_REJECTED` and publishes nothing to `/cmd_vel` — asserted by a test that
      spies on the publisher.
- [ ] `examples/claude_desktop_config.json` connects Claude Desktop to the server; README
      documents the five-line setup.
- [ ] No robot-side ROS package is modified anywhere in the repository or the
      implementation process.

## Relationship to This Document Set

This file is the scope contract for the MVP implementation pass. Every "why" behind these
requirements lives in the numbered docs it references — implementers should treat those
as the specification and this file as the checklist.

# 20 — Implementation Roadmap

## Phase 1 — MVP Foundation (frozen scope: [17-mvp.md](17-mvp.md))

- **Deliverables**: discovery, capability registry, 8 MCP tools, 3 resources, closed-loop
  `/cmd_vel` motion, Nav2 navigation, laser/camera perception, stub detector, safety
  engine, Claude Desktop integration.
- **Dependencies**: ROS 2 Humble, `turtlebot3_gazebo`, Nav2 (optional at runtime).
- **Tests**: unit suite (mocked ROS) + sim acceptance run (§[18](18-testing-strategy.md)).
- **Acceptance**: [17-mvp.md](17-mvp.md) checklist, all boxes checked with pasted
  evidence.

## Phase 2 — Navigation Depth

- **Deliverables**: `NavigateThroughPoses`, Nav2 planner/controller/recovery
  introspection surfaced via `robot.diagnose`, richer TF diagnostics
  (`show me the current TF tree` as a resource), localization-health resource.
- **Dependencies**: Phase 1 complete.
- **Tests**: sim tests for multi-waypoint navigation, recovery-behavior triggering
  (blocked path scenario).
- **Acceptance**: multi-waypoint nav succeeds; `robot.diagnose` correctly identifies an
  injected planner failure.

## Phase 3 — Perception Depth

- **Deliverables**: real `ObjectDetectorPlugin` (e.g. YOLO via a local/remote inference
  plane), depth-image-based geometric estimation (replacing scan-correlation fallback
  when a depth source exists), point-cloud summary tool, richer laser-scan free-space
  reasoning.
- **Dependencies**: Phase 1; a GPU-capable inference host (local or remote) for the real
  detector.
- **Tests**: detection-accuracy smoke tests against known sim objects; TF-grounding
  accuracy test (known object placed at a known pose, detected position within
  tolerance).
- **Acceptance**: `detect_objects` returns real labels/positions for standard sim
  objects at ≥ a documented baseline accuracy.

## Phase 4 — Advanced Robotics

- **Deliverables**: `ManipulationBackend`/MoveIt integration, `DockingBackend`, SLAM
  control (start/stop mapping, save map), Prompt-layer workflows (`navigation_assistant`,
  `inspection_workflow`, `object_search`).
- **Dependencies**: Phases 1–3; a manipulator-equipped robot for manipulation testing.
- **Tests**: MoveIt planning/execution integration tests; docking success-rate sim tests.
- **Acceptance**: `robot.move_arm`/`robot.grasp` succeed on a MoveIt-enabled sim robot;
  `object_search` prompt workflow (Flow E, §[26 in earlier draft]) completes end-to-end.

## Phase 5 — Production

- **Deliverables**: multi-robot router (§[14](14-multi-robot.md)), token-based
  authentication/RBAC, OpenTelemetry export, plugin marketplace/SDK packaging,
  networked (non-stdio) MCP transport, DDS-Security deployment guide.
- **Dependencies**: Phases 1–4; a concrete multi-robot deployment target to validate
  against.
- **Tests**: multi-robot isolation tests (one robot's failure doesn't affect another);
  authZ negative tests (denied session cannot invoke MOTION tools); load test on the
  router.
- **Acceptance**: two simulated robots independently controllable through one router with
  documented per-robot policy isolation.

Every phase preserves [13-contracts.md](13-contracts.md) as written for Phase 1's scope —
new contracts are additive (new Protocols, new dataclass subtypes), and any change to an
existing frozen signature requires a new ADR explaining the break.

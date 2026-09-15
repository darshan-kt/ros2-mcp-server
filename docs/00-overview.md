# 00 — Overview

## Status

**FROZEN.** This document set is the architecture specification for ROS-MCP-Server.
Once merged, changes require a new ADR (see [adr/](adr/README.md)) — not silent edits.
Implementation must conform to [13-contracts.md](13-contracts.md); if an implementation
need conflicts with a contract, the fix is a new ADR, not a workaround in code.

## Mission

> Any MCP-compatible LLM should be able to safely understand and operate any ROS 2 robot
> through a stable semantic interface, without modifying the robot's existing software.

ROS 2 is the execution substrate. MCP is the AI integration protocol. ROS-MCP-Server is the
intelligent interoperability layer between them.

```text
LLM (Claude)
 ↓ MCP
ROS-MCP-Server            ← this system
 ↓ rclpy / ROS 2 client APIs
ROS 2 graph (unmodified)
 ↓
Robot / Simulation
```

## Design Principles

1. **Semantic API, not topic proxy.** The LLM calls `robot.move`, `robot.navigate`,
   `robot.detect_objects` — never `ros.publish("/cmd_vel", ...)` as its primary interface.
   Raw ROS access exists but is a deliberately separate, gated surface (`ros.*` tools).
2. **Zero modification.** The robot's ROS 2 packages are inspected and interacted with
   through standard ROS 2 client APIs only (`rclpy`, `ros2 node/topic/service/action`
   introspection, TF2, parameters). No robot source is patched, no custom messages are
   required on the robot side.
3. **Deterministic safety outside the LLM.** The LLM proposes; it never has a code path
   that reaches an actuator without passing through the safety/validation engine
   ([10-safety-and-trust.md](10-safety-and-trust.md)). This is enforced structurally, not
   by prompting.
4. **ROS 2 stays real-time-adjacent; the LLM never is.** No LLM call sits inside a control
   loop. The boundary is: LLM → semantic command → deterministic controller → ROS 2 loop.
5. **Discover, don't assume.** Capabilities are inferred from the live ROS graph at
   startup and kept fresh; nothing about `/cmd_vel` semantics, Nav2 presence, or sensor
   availability is hardcoded per-robot. Explicit config overrides discovery
   ([16-configuration.md](16-configuration.md)).
6. **Small, ranked tool surface.** The MCP tool list is capability-derived and bounded —
   not one tool per ROS interface. See [03-capability-discovery.md](03-capability-discovery.md)
   and [04-mcp-surface.md](04-mcp-surface.md).
7. **Sensor data is untrusted data, never instructions.** Perception output crossing back
   into LLM context is treated the same as any other untrusted external input. See the
   trust-boundary section of [10-safety-and-trust.md](10-safety-and-trust.md).
8. **Structured results, structured errors, always.** No bare strings, no swallowed
   exceptions at the MCP boundary. See [13-contracts.md](13-contracts.md) §Result/Error
   models.
9. **Simulation and hardware are the same code path.** The adapters talk to whatever ROS
   graph exists; Gazebo/Isaac Sim vs. real hardware is invisible above the adapter layer.

## Document Map

| # | Document | Covers |
|---|----------|--------|
| 00 | [Overview](00-overview.md) | This file |
| 01 | [System Architecture](01-system-architecture.md) | Components, top-level diagrams |
| 02 | [ROS 2 Architecture](02-ros2-architecture.md) | Discovery, generic message handling, adapters |
| 03 | [Capability Discovery](03-capability-discovery.md) | Ontology, inference, confidence, registry |
| 04 | [MCP Surface](04-mcp-surface.md) | Tools, resources, prompts, schemas |
| 05 | [Semantic Command Model](05-semantic-command-model.md) | Command data model |
| 06 | [Execution](06-execution.md) | State machine, execution manager, idempotency |
| 07 | [Motion Architecture](07-motion-architecture.md) | Closed-loop `/cmd_vel` control |
| 08 | [Navigation Architecture](08-navigation-architecture.md) | Nav2 integration |
| 09 | [Perception Architecture](09-perception-architecture.md) | Laser, camera, TF, plugin detectors |
| 10 | [Safety & Trust](10-safety-and-trust.md) | Policy engine, HITL, security, prompt injection |
| 11 | [Context & Streaming](11-context-and-streaming.md) | Context strategy, event/notification mapping |
| 12 | [Plugin Architecture](12-plugin-architecture.md) | Plugin SDK, lifecycle |
| 13 | [Contracts](13-contracts.md) | Frozen interfaces between every module |
| 14 | [Multi-Robot](14-multi-robot.md) | Identity, namespaces, isolation (post-MVP) |
| 15 | [Observability & Failure Handling](15-observability-and-failure-handling.md) | Telemetry, failure matrix |
| 16 | [Configuration](16-configuration.md) | `robot.yaml` schema, precedence |
| 17 | [MVP](17-mvp.md) | Frozen MVP scope and acceptance criteria |
| 18 | [Testing Strategy](18-testing-strategy.md) | Unit/integration/sim/e2e/fault injection |
| 19 | [Deployment & Scalability](19-deployment-and-scalability.md) | Planes, repo layout, tech choices |
| 20 | [Roadmap](20-roadmap.md) | Phase 1–5 |
| — | [ADRs](adr/README.md) | 15 key decisions with alternatives/tradeoffs |

## Glossary

- **Semantic command** — a validated, typed operation (e.g. `NAVIGATE`) produced from an
  MCP tool call, independent of which ROS backend executes it.
- **Adapter** — a module translating one semantic command family to one ROS backend
  (Nav2 adapter, cmd_vel/motion adapter, perception adapter, …).
- **Capability** — an inferred or configured semantic ability of the robot
  (`differential_drive_motion`, `autonomous_navigation`, `visual_observation`, …).
- **Backend** — the concrete ROS mechanism used to realize a capability (an action, a
  topic pair, a service).
- **Trust boundary** — a point in the pipeline where data changes from trusted
  (user instruction, validated command) to untrusted (sensor/perception output) or back.

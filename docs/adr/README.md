# Architecture Decision Records

Frozen decisions for ROS-MCP-Server. Each ADR follows: Decision → Why → Alternatives →
Tradeoffs → Failure Modes → Recommendation. Changing a decision here requires a new ADR
(status: superseded on the old one), not a silent code change.

| ADR | Title |
|---|---|
| [001](ADR-001-semantic-tool-surface.md) | Semantic tool surface, not raw topic proxy |
| [002](ADR-002-safety-engine-outside-llm.md) | Deterministic safety engine outside the LLM's reach |
| [003](ADR-003-capability-inference-with-confidence.md) | Capability inference with confidence levels + config override |
| [004](ADR-004-semantic-command-and-state-machine.md) | Semantic command model + explicit state machine (not a workflow engine) |
| [005](ADR-005-cmdvel-closed-loop-vs-nav2.md) | Closed-loop `/cmd_vel` for relative motion; Nav2 for absolute goals |
| [006](ADR-006-nav2-first-class-clean-fallback.md) | Nav2 as first-class navigation backend with clean `CAPABILITY_UNAVAILABLE`, no silent fallback |
| [007](ADR-007-generic-message-introspection.md) | Generic ROS message handling via `rosidl` introspection |
| [008](ADR-008-subscription-pooling.md) | Subscription pooling with last-value cache, not per-request subscriptions |
| [009](ADR-009-binary-data-handling.md) | Binary/large data never inlined raw into MCP results |
| [010](ADR-010-perception-plugin-interface.md) | Perception detection behind a plugin interface from day one |
| [011](ADR-011-async-boundary.md) | Two-runtime process: dedicated rclpy executor thread + asyncio MCP loop |
| [012](ADR-012-sensor-data-is-untrusted.md) | Sensor/perception data is untrusted data, never an instruction source |
| [013](ADR-013-config-overrides-discovery.md) | Config overrides discovery; no environment-variable safety overrides |
| [014](ADR-014-mcp-transport-and-robot-identity.md) | stdio transport for MVP; robot identity modeled from day one |
| [015](ADR-015-structured-error-model.md) | Structured, closed-enum error model at every tool boundary |

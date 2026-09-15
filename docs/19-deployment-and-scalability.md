# 19 — Deployment, Scalability & Technology Choices

## Control / Data / AI-Inference Planes

| Plane | Contents | Where it runs |
|---|---|---|
| **Control plane** | MCP protocol handling, Validation, Safety, Planner, Execution Manager, state machine | Same process, MCP/asyncio runtime — must stay low-latency and resource-light; this is the plane every command passes through. |
| **Data plane** | Discovery, Subscription Manager, TF Adapter, Motion/Nav2 adapters — direct ROS I/O | Same process, rclpy runtime (dedicated thread) — isolated so control-plane/AI latency never delays a `/cmd_vel` publish or a subscription callback. |
| **AI-inference plane** | `ObjectDetectorPlugin` implementations beyond the MVP stub (YOLO, VLM, remote inference) | Explicitly **out-of-process** for anything GPU-heavy — a plugin implementation is free to be a thin RPC client to a separate inference service/container; the perception adapter's `async def detect()` boundary makes this transparent. The MVP `StubDetectorPlugin` runs in-process because it does no real inference. |

This separation is why the MVP runs comfortably on a resource-constrained robot computer
(e.g. a Jetson or a NUC): the control and data planes are lightweight Python/rclpy work;
anything GPU-bound is pushed to the inference plane, which may live on a more capable
machine.

## Technology Choices

| Concern | Choice | Why |
|---|---|---|
| Language (MVP) | Python (`rclpy`) throughout | The MVP's bottleneck is I/O (ROS calls, MCP messages), not CPU; `rclpy` + `asyncio` is sufficient and keeps one language across the whole server. `rclcpp`/C++ is a future option only for a specific hot path proven to need it (e.g. a very high-rate control loop) — not adopted speculatively. |
| MCP SDK | Official Python `mcp` SDK, stdio transport for MVP | Matches Claude Desktop/Code's native local-server integration; HTTP+SSE transport is additive later without changing tool logic. |
| Async architecture | `asyncio` event loop (MCP/control plane) + dedicated `rclpy` executor thread (data plane), bridged per §[13-contracts.md](13-contracts.md) §13 | Keeps ROS callbacks non-blocking and MCP tool handling non-blocking of each other; avoids the anti-pattern of spinning `rclpy` on the same thread as `asyncio.run()`. |
| Executor | `SingleThreadedExecutor` for MVP's modest callback count; documented upgrade path to `MultiThreadedExecutor` + callback groups if/when concurrent long-running callbacks (e.g. simultaneous multi-topic high-rate subscriptions) show contention | Simpler to reason about and debug for MVP scale; premature multi-threading inside rclpy adds lock-discipline risk for no measured benefit yet. |
| Configuration | YAML (`robot.yaml`) + `pydantic` models + narrow env var overrides | Human-reviewable safety limits in a committed file (§[16](16-configuration.md)); `pydantic` gives validation at load time for free. |
| Serialization | `rosidl` runtime introspection (see [02](02-ros2-architecture.md)) for ROS↔dict; stdlib `json` for MCP wire content | No hand-maintained per-message-type serializers. |
| Logging | stdlib `logging` + a project-owned structured (JSON) formatter | No new dependency; sufficient for MVP; OpenTelemetry logging bridge is additive later. |
| Metrics | In-process counters/histograms behind the `MetricsSink` interface, logged periodically for MVP | Avoids adding `opentelemetry-sdk` before there's an operator actually consuming traces/metrics. |
| Testing | `pytest` | Approved dependency; mocking `rclpy` at the API boundary rather than requiring a live ROS graph for unit tests (§[18](18-testing-strategy.md)). |
| Containerization | Docker image bundling ROS 2 Humble + the server, used for the sim-first dev/test environment | Reproducible dev/CI environment; not required to run the server (a normally-sourced ROS workspace works too). |
| Orchestration | **No Kubernetes in MVP or the near-term roadmap.** | A single robot runs one server process on one machine — there is nothing to orchestrate. Kubernetes becomes relevant only for the Phase-5 fleet-management router (§[14](14-multi-robot.md)) managing many worker processes across machines, and even then only if the deployment target is cloud/edge-cluster rather than "one process per physical robot" (which is the more common robotics deployment shape and needs no orchestrator at all). |

## Repository Structure

```text
ros-mcp-server/
├── README.md
├── LICENSE
├── pyproject.toml
├── docker/
│   └── sim.Dockerfile            # ROS 2 Humble + turtlebot3_gazebo + server, sim-first dev/test
├── docs/                         # this document set (frozen architecture)
├── examples/
│   ├── claude_desktop_config.json
│   └── flows/                    # example end-to-end transcripts
├── config/
│   └── robot.yaml                # reference TurtleBot3 config
├── tests/
│   ├── unit/
│   └── integration/
└── src/
    └── ros_mcp/
        ├── server.py              # entrypoint: python -m ros_mcp.server
        ├── mcp/                   # transport, session manager, tool/resource/prompt provider
        ├── ros/                   # rclpy node, RosBridge, TopicAdapter/ServiceAdapter/ActionAdapter
        ├── discovery/              # DiscoveryEngine, GraphSnapshot
        ├── capabilities/           # ontology, CapabilityInferenceEngine, CapabilityRegistry
        ├── commands/               # SemanticCommand, SemanticCommandFactory
        ├── execution/              # ExecutionManager, CommandPlanner, state machine
        ├── safety/                 # ValidationEngine, SafetyPolicyEngine
        ├── adapters/
        │   ├── motion/              # CmdVelMotionPlugin (MotionBackend)
        │   ├── navigation/          # Nav2NavigationPlugin (NavigationBackend)
        │   └── perception/          # PerceptionAdapter, StubDetectorPlugin, TFAdapter
        ├── subscriptions/          # SubscriptionManager
        ├── codec/                  # MessageCodec
        ├── plugins/                 # PluginManager, entry-point discovery
        ├── context/                 # Context Manager (resource assembly, compression)
        ├── security/                 # audit log, (future) authN/authZ
        ├── telemetry/                # structured logging setup, MetricsSink
        └── config/                   # ConfigProvider, pydantic models
```

Each package boundary matches a contract group in [13-contracts.md](13-contracts.md);
`adapters/*` is the only tree plugins add to without touching other packages
(§[12-plugin-architecture.md](12-plugin-architecture.md)).

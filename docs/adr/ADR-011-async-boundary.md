# ADR-011: Two-Runtime Process — Dedicated `rclpy` Executor Thread + `asyncio` MCP Loop

**Status**: Accepted

## Decision

The server process runs a dedicated OS thread spinning an `rclpy` executor (all ROS
callbacks execute there) alongside the main-thread `asyncio` event loop (all MCP/control-
plane code runs there), bridged only through the two whitelisted crossing functions in
`RosBridge` (§[13-contracts.md](../13-contracts.md) §13).

## Why

Neither runtime may block on the other: an `rclpy` subscription callback awaiting MCP/LLM
work would stall ROS message processing (violating "ROS callbacks never block on MCP
work," an explicit MVP requirement); conversely, `asyncio` code calling a blocking
`rclpy` API directly would stall tool-call handling for unrelated commands. A single
shared thread running both `rclpy.spin()` and `asyncio.run()` cooperatively is fragile
(both want to own "the" event loop) and error-prone to get right.

## Alternatives Considered

- **Run everything on one thread, interleaving `rclpy.spin_once()` calls inside the
  asyncio loop via a periodic callback.** Rejected: couples ROS message-processing
  latency to asyncio scheduling fairness; a slow tool handler delays ROS callback
  processing, defeating the goal directly.
  Considered again with `rclpy`'s experimental asyncio-compatible executor: not mature/
  stable enough across ROS 2 Humble to depend on for MVP; revisit if it matures.
- **Two separate OS processes (ROS side, MCP side) communicating over IPC.** Rejected for
  MVP: adds a serialization/IPC layer and a second process-lifecycle to manage for no
  present benefit at single-robot scale; the multi-robot router (§[14](../14-multi-robot.md))
  is where a process boundary earns its keep, not within a single robot's worker.

## Tradeoffs

- Pro: clean isolation; a stuck tool handler cannot starve ROS callback processing and
  vice versa; matches the "LLM must never be part of a hard real-time loop" principle
  structurally.
  Con: every cross-boundary call has bridging overhead (thread-safe future/queue hop) —
  small and acceptable relative to ROS/network latency; requires discipline (enforced by
  the contract) that no code takes a shortcut around the bridge.

## Failure Modes

- A developer calls a blocking `rclpy` API directly from an `async def` tool handler,
  stalling the asyncio loop for all sessions — mitigated by code review against the
  contract and, longer-term, a lint rule flagging direct `rclpy` imports outside
  `ros_mcp.ros.*`/`ros_mcp.adapters.*`.
- An `rclpy` callback awaits or performs long synchronous work — mitigated the same way,
  contract-enforced.

## Recommendation

Treat `RosBridge` as the only legal crossing point permanently; do not adopt an
asyncio-native `rclpy` executor until it is stable and proven for the target ROS 2
distributions this project supports.

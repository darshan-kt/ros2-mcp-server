# ADR-001: Semantic Tool Surface, Not Raw Topic Proxy

**Status**: Accepted

## Decision

The LLM-facing MCP tool set is a small, semantic robotics API (`robot.move`,
`robot.navigate`, `robot.get_state`, …) derived from inferred capabilities, capped at the
handful of tools listed in [04-mcp-surface.md](../04-mcp-surface.md). Raw ROS access
(`ros.publish`, `ros.call_service`, …) exists as a separate, disabled-by-default,
individually-gated tool family for advanced/debugging use — never the default or primary
interface.

## Why

An LLM reasoning over "move to the kitchen" should not need to know whether that robot
uses `/cmd_vel`, a custom `DriveTo` action, or Nav2. A 1:1 topic-to-tool mapping also
does not scale: a robot with hundreds of topics would produce hundreds of tools, blowing
the LLM's context and giving it far more surface area to misuse than the task requires.

## Alternatives Considered

- **Expose every discovered topic/service/action as its own MCP tool.** Rejected: no
  bound on tool count, no semantic grouping, forces the LLM to understand raw ROS
  message shapes to do anything, and multiplies the surface an adversarial or confused
  prompt could exploit.
- **Single generic `ros.do(intent: str)` tool that free-form parses intent server-side.**
  Rejected: reintroduces natural-language parsing ambiguity into the server instead of
  leaving it to the LLM (which is what it's good at) and gives no structured schema for
  the MCP client to validate against.

## Tradeoffs

- Pro: bounded, predictable tool list; stable across robots with different backends;
  safety/validation logic centralizes naturally around a small verb set.
  Con: any capability the ontology doesn't yet model (§[03](../03-capability-discovery.md))
  has no semantic tool until the ontology is extended — the raw-tool escape hatch exists
  precisely for this gap, at the cost of being less safe/structured when used.

## Failure Modes

- Ontology too coarse for a genuinely novel robot capability → mitigated by raw tools +
  the plugin architecture (§[12](../12-plugin-architecture.md)) as the extension path,
  not by growing the semantic tool list ad hoc per robot.
- Capability inference wrongly excludes something real → surfaced (not silently dropped)
  via `AMBIGUOUS` confidence in `robot.get_capabilities`, correctable via config.

## Recommendation

Keep the semantic surface frozen at the MVP's eight tools until Phase 4 (manipulation)
genuinely requires new verbs; extend the ontology and tool set only when a real capability
class is missing, not per-robot.

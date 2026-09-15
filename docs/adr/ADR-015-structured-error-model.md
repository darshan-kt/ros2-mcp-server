# ADR-015: Structured, Closed-Enum Error Model at Every Tool Boundary

**Status**: Accepted

## Decision

Every tool result is a typed `ToolResult` subclass; every failure carries a `ToolError`
with a `code` drawn from a closed `ErrorCode` enum (§[13-contracts.md](../13-contracts.md)
§2). No tool handler may return a bare string or let an unhandled exception cross the
MCP boundary — the outermost dispatch layer converts any uncaught exception to
`ErrorCode.INTERNAL_ERROR` with a sanitized message.

## Why

An LLM reasoning about "why did this fail, what should I try next" needs a
machine-readable signal, not prose to re-parse. A closed enum lets the LLM (and any
future automated retry/recovery logic) branch reliably: `CAPABILITY_UNAVAILABLE` means
"try a different tool," `SAFETY_REJECTED` means "don't retry with the same
parameters," `SENSOR_STALE` means "the robot's perception is degraded right now."
A bare `"error: something went wrong"` string gives none of that.

## Alternatives Considered

- **Return free-form error strings, let the LLM interpret them.** Rejected: unreliable
  parsing, no stable contract for testing ("does this correctly report
  SAFETY_REJECTED" becomes a substring-matching guess instead of an enum equality
  check), and no way for the audit log to aggregate failure reasons meaningfully.
- **Use HTTP-style numeric status codes.** Rejected: robotics-specific failure modes
  (`TF_UNAVAILABLE`, `SENSOR_STALE`, `CAPABILITY_UNAVAILABLE`) don't map cleanly onto
  generic HTTP semantics, and a named enum is more self-documenting in logs/tests than a
  numeric code needing a lookup table.
- **Let exceptions propagate to the MCP transport layer and let the SDK format them.**
  Rejected: loses the structured `details` payload (e.g. which frame was invalid, what
  frames *are* known) that helps the LLM self-correct, and risks leaking internal
  stack-trace detail into the result.

## Tradeoffs

- Pro: testable ("assert error.code == SAFETY_REJECTED"), aggregable in telemetry,
  self-documenting, gives the LLM actionable signal.
  Con: adding a genuinely new failure mode requires extending the enum (a small,
  deliberate change) rather than just writing a new string — this friction is a feature,
  keeping the error space closed and reviewed.

## Failure Modes

- A new adapter introduces a failure mode not covered by the existing enum and the
  implementer is tempted to overload an unrelated code (e.g. using
  `ROS_INTERFACE_ERROR` for what's really a new distinct case) — mitigated by code review
  against this ADR; extending the enum is cheap and should be preferred over overloading.

## Recommendation

Keep `ErrorCode` a single, centrally-defined enum in `ros_mcp.contracts`; never let an
adapter define its own ad hoc error strings that leak into `ToolError.code`.

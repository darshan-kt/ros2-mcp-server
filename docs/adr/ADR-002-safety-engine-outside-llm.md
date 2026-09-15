# ADR-002: Deterministic Safety Engine Outside the LLM's Reach

**Status**: Accepted

## Decision

Every `MOTION`/`HIGH_RISK` semantic command passes through a deterministic
`SafetyPolicyEngine` (§[10](../10-safety-and-trust.md), §[13](../13-contracts.md)) that
the MCP tool-call path cannot bypass by construction — adapters are unreachable from tool
handlers except via the Execution Manager, which is contractually required to call the
Safety Engine first.

## Why

LLM output is probabilistic; a robot actuator command must not be. Prompting the model to
"be careful" is not a safety mechanism — a single bad completion, a jailbreak, or an
ambiguous instruction must not be able to reach `/cmd_vel` or Nav2 unfiltered.

## Alternatives Considered

- **Rely on the LLM's own judgment / system prompt instructions for limits.** Rejected:
  not deterministic, not auditable, not testable, and not defensible as "safety" in any
  serious sense — this is exactly the anti-pattern called out in the source design brief.
- **Safety checks embedded inside each adapter individually.** Rejected: duplicated
  logic, easy to miss when adding a new adapter, no single place to audit "what limits
  apply to every motion command."
- **Clamp out-of-range commands to the limit and execute anyway.** Rejected (see
  [10-safety-and-trust.md](../10-safety-and-trust.md)): silently altering the command
  means the LLM/user believes a different action occurred than what ran; a hard reject
  with a clear reason is safer and more honest.

## Tradeoffs

- Pro: one auditable, testable choke point; safety limits are a config file a human
  reviews, not a prompt.
  Con: adds latency (one extra check) to every motion command — negligible in practice
  (microseconds of Python logic) relative to control-loop/network latency.

## Failure Modes

- A future adapter added without going through the Execution Manager would bypass safety
  — mitigated structurally by import-boundary rules (adapters import nothing from
  `ros_mcp.mcp.*`, §[13](../13-contracts.md) Non-Bypass Rule) and should be caught in
  code review / a lint rule enforcing the import boundary.
- Overly conservative limits block legitimate operation → mitigated by config being
  operator-editable per robot, not hardcoded.

## Recommendation

Never weaken the Non-Bypass Rule for convenience (e.g. a "fast path" tool that skips
validation). Any future HIGH_RISK capability (manipulation, docking) must integrate at
this same choke point, not a parallel one.

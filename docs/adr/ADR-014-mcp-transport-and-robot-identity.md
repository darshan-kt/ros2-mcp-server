# ADR-014: stdio Transport for MVP; Robot Identity Modeled From Day One

**Status**: Accepted

## Decision

The MVP server communicates with Claude Desktop/Code exclusively over MCP stdio (local
process, no network socket). Every `SemanticCommand` carries a `robot_id` field
(§[05-semantic-command-model.md](../05-semantic-command-model.md)) even though the MVP
only ever runs a single robot.

## Why

**Transport**: stdio is what Claude Desktop/Code natively supports for local MCP
servers, requires no auth infrastructure (the OS process boundary is the trust boundary,
§[10](../10-safety-and-trust.md)), and matches the MVP's single-operator, single-robot,
simulation-first scope exactly — building an HTTP+SSE transport with real auth for the
MVP would be speculative infrastructure for a need that doesn't exist yet.

**Robot identity**: adding `robot_id` now costs nothing (it's one field, defaulted
trivially when there's one robot) and avoids a breaking schema change to
`SemanticCommand`/`ToolResult`/the audit log format when multi-robot
(§[14-multi-robot.md](../14-multi-robot.md)) is built later.

## Alternatives Considered

- **Build networked transport + auth now, "since production will need it eventually."**
  Rejected: violates the MVP's own scope (§[17-mvp.md](../17-mvp.md) explicitly excludes
  auth); speculative infrastructure with no current consumer, and the shape of "right"
  auth depends on deployment details (single-tenant vs. fleet-operator) not yet decided.
- **Omit `robot_id` from the MVP data model, add it in Phase 5.** Rejected: this *would*
  be a breaking schema/contract change touching `SemanticCommand`, every `ToolResult`,
  the audit log, and the Capability Registry — cheap to avoid now, expensive to retrofit
  later, so it's included from the start even though it's a no-op in the MVP's single-
  robot reality.

## Tradeoffs

- Pro: MVP ships faster with zero auth-infrastructure risk; multi-robot extension is
  additive to the data model, not a breaking migration.
  Con: MVP genuinely has no meaningful access control beyond "who can run this local
  process" — explicitly documented as a limitation, not hidden.

## Failure Modes

- A user exposing the MVP server over a network transport without adding the Phase-5
  auth layer would have no access control — mitigated by documentation stating stdio/
  local-only is the supported MVP transport; a networked transport is out of scope until
  auth ships alongside it.

## Recommendation

Do not add a networked transport without also shipping the corresponding authN/authZ
layer from [14-multi-robot.md](../14-multi-robot.md) in the same change — the two must
land together, never transport-first.

# ADR-004: Semantic Command Model + Explicit State Machine (Not a Workflow Engine)

**Status**: Accepted

## Decision

Every actuation-relevant tool call becomes one `SemanticCommand`
(§[05](../05-semantic-command-model.md)) tracked through a small, explicit
`ExecutionState` state machine (§[06](../06-execution.md)) owned solely by the Execution
Manager. No general-purpose workflow/orchestration engine is introduced.

## Why

Commands are short-lived, single-entity, and the terminal-state set is small and fully
known ahead of time (`SUCCEEDED`, `FAILED`, `TIMEOUT`, `CANCELLED`, `SAFETY_STOP`,
`REJECTED`, `SAFETY_REJECTED`, `CAPABILITY_UNAVAILABLE`). A hand-rolled but explicit state
machine gives auditability (one place answers "what is command X doing right now") and
cancellation semantics without persistence, DAG scheduling, or retry-policy machinery a
workflow engine would bring.

## Alternatives Considered

- **Bare `async def` per tool with ad hoc try/except/cancellation.** Rejected: scatters
  cancellation/timeout/audit logic per tool, makes "what is the robot doing" an ad hoc
  query instead of a registry read, and makes it easy to forget a case (e.g. an adapter
  that doesn't publish a final zero-velocity on one exit path).
- **A general workflow/orchestration engine (e.g. a DAG-based task runner).** Rejected
  for MVP/near-term: massive overkill for single-command execution; adds a persistence
  and scheduling dependency for no present need. Revisit only if/when multi-step composed
  workflows (Phase 4 Prompts) genuinely require durable multi-step orchestration across
  process restarts.

## Tradeoffs

- Pro: simple, fully unit-testable without infrastructure, matches the actual shape of
  the problem (one command, one lifecycle).
  Con: does not natively support durable/resumable multi-step plans; a Phase-4 workflow
  layer will need to compose multiple `SemanticCommand`s at a layer above this one rather
  than get durability for free from the state machine itself.

## Failure Modes

- A future feature needing durable, restart-survivable multi-step execution would strain
  this model — explicitly deferred, not solved prematurely.
- An adapter that doesn't honor `cancellation_token` polling promptly breaks the
  responsiveness of `robot.stop` — mitigated by the contract requirement (§[13](../13-contracts.md)
  Adapter contracts: poll at ≤ one control-loop period) and tested per adapter.

## Recommendation

Do not introduce a workflow engine until Phase 4's Prompt workflows demonstrably need
durable multi-step state; keep the state machine as the unit of execution beneath any
future workflow layer.

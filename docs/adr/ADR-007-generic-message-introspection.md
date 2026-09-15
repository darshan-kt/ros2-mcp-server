# ADR-007: Generic ROS Message Handling via `rosidl` Runtime Introspection

**Status**: Accepted

## Decision

All ROS message/service/action (de)serialization and schema generation goes through one
`MessageCodec` seam built on `rosidl_runtime_py` introspection utilities
(§[02](../02-ros2-architecture.md), §[13](../13-contracts.md) §10), rather than
hand-written per-type converters.

## Why

The system must work with unknown/custom message types (robot-specific messages,
services, actions) without a code change per type — that is a hard zero-modification/
generality requirement. `rosidl_runtime_py` already provides exactly this capability
(`message_to_ordereddict`, type resolution via `get_message`/`get_service`/`get_action`)
as part of the standard ROS 2 tooling.

## Alternatives Considered

- **Hand-write a converter per message type as needed.** Rejected: does not scale, breaks
  the zero-modification/generality promise the moment a robot uses any message type not
  yet hand-coded.
- **Use `.msg`/`.srv`/`.action` IDL files directly and write a custom parser.** Rejected:
  duplicates what `rosidl_parser`/`rosidl_runtime_py` already does correctly and
  maintains for every ROS distribution; higher maintenance burden, higher bug surface.

## Tradeoffs

- Pro: zero per-message-type code; automatically supports any interface package installed
  on the server host, custom or standard.
  Con: still requires the interface package to be importable on the server's ROS
  workspace (documented limitation, §[02](../02-ros2-architecture.md)) — this is a
  deployment requirement, not a source modification, but is a real constraint worth
  stating plainly rather than glossing over.
  Con: introspection has a nonzero cost per unique type on first use — mitigated by
  process-lifetime schema caching.

## Failure Modes

- Interface package not installed on the server host → that type's capability/topic is
  excluded with a logged `ROS_INTERFACE_ERROR`-class warning, not a crash.
- Extremely deep/recursive nested custom messages → schema walker has a defensive
  recursion-depth cap (documented, configurable) to avoid pathological stack depth on a
  malformed or adversarially large custom type.

## Recommendation

Keep `MessageCodec` as the single seam; if performance ever requires precompiled per-type
codecs, generate them from the same introspection data at discovery time rather than
reverting to hand-written converters — see the note in
[02-ros2-architecture.md](../02-ros2-architecture.md).

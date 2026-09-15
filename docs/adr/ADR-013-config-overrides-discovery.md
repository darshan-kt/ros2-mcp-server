# ADR-013: Config Overrides Discovery; No Environment-Variable Safety Overrides

**Status**: Accepted

## Decision

Precedence is `defaults < discovery/inference < robot.yaml < env vars`, **except** safety
limits, which env vars may never override — only `robot.yaml` sets them
(§[16-configuration.md](../16-configuration.md)).

## Why

Automatic discovery is convenient but must not have final say over safety-relevant
values (limits, geofence, approval policy) — those must be human-reviewable in a
committed file. Environment variables are the right override for deployment-specific,
non-safety concerns (log level, config path) precisely because they're *not* meant to be
reviewed the way a committed YAML file is; letting them silently change a velocity limit
in one deployment environment vs. another is a foot-gun (e.g. a CI/staging env var
leaking into a value that reaches a real robot).

## Alternatives Considered

- **Let discovery fully determine capabilities/limits with no override mechanism.**
  Rejected: §41 of the source brief explicitly requires config to resolve ambiguity;
  discovery alone cannot know a robot's true safe velocity limits (that's a property of
  the physical platform, not observable from the ROS graph).
  Additionally, this would violate the zero-modification design if it were the only means
  of narrowing scope — config exists precisely so the robot itself need not change.
- **Allow environment variables to override everything, including safety limits, for
  deployment flexibility.** Rejected: safety limits belong in a reviewable, versioned
  file; env-var overrides of safety values are exactly the kind of change that could slip
  through unreviewed in a deployment script.

## Tradeoffs

- Pro: safety limits are always traceable to a specific committed config; deployment
  flexibility is preserved for genuinely deployment-scoped settings.
  Con: changing a safety limit requires a file edit + (ideally) review/redeploy rather
  than a quick env var tweak — this friction is intentional.

## Failure Modes

- A deployment script or container orchestrator that tries to inject
  `ROS_MCP_SAFETY_MAX_LINEAR_MPS` would simply have no effect (no such variable is read)
  — fails safe by being ignored, not by silently applying.

## Recommendation

Never add a `ROS_MCP_SAFETY_*` environment variable, even under deployment-convenience
pressure; if a deployment truly needs different limits per environment, maintain
per-environment `robot.yaml` files instead.

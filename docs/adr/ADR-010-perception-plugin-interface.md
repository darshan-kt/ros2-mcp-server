# ADR-010: Perception Detection Behind a Plugin Interface From Day One

**Status**: Accepted

## Decision

`robot.detect_objects` is implemented entirely against the `ObjectDetectorPlugin`
interface (§[13-contracts.md](../13-contracts.md) §7); the MVP ships `StubDetectorPlugin`
as the only implementation, but the perception adapter has zero knowledge of it being a
stub.

## Why

Object detection models (YOLO, Grounding DINO, SAM, CLIP, VLMs, Isaac perception, remote
inference services) are numerous, evolve fast, and have wildly different runtime/dependency
footprints (some need a GPU, some call out to a remote service). Building the MVP's
detection pipeline (acquire → detect → geometry → TF → structured result) against a real
interface — even with a trivial implementation — proves the seam works and means swapping
in a real model later touches only the plugin, not the adapter, the TF integration, or the
MCP tool schema.

## Alternatives Considered

- **Skip `robot.detect_objects` entirely in the MVP, add it only once a real detector
  exists.** Rejected: leaves the geometry/TF-grounding pipeline — arguably the harder,
  more architecturally significant part — unbuilt and untested until a heavier dependency
  (a real model) is available; also leaves an MVP acceptance-criteria gap
  (§[17-mvp.md](../17-mvp.md) requires this tool).
- **Hardcode a specific detector library into the perception adapter for MVP, refactor to
  a plugin later.** Rejected: "refactor to a plugin later" reliably doesn't happen cleanly
  once callers depend on the concrete shape; building the interface first costs little
  extra and avoids the rewrite.

## Tradeoffs

- Pro: the plugin boundary is proven by a real, if simple, implementation; adding a real
  detector later is additive.
  Con: `StubDetectorPlugin`'s output is not meaningfully accurate — mitigated by
  `detector_plugin_id` always being present in results so callers know the provenance
  and accuracy expectation.

## Failure Modes

- A future real detector plugin with heavy startup cost (loading model weights) must
  implement `health_check()` honestly (`DEGRADED` while loading) rather than blocking
  plugin registration — contract requirement, §[12](../12-plugin-architecture.md).

## Recommendation

Do not special-case `StubDetectorPlugin` anywhere in the adapter; if the adapter needs a
special case to make the stub work, that's a sign the interface is wrong and needs fixing
before more plugins are built against it.

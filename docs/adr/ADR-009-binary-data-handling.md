# ADR-009: Binary/Large Data Never Inlined Raw Into MCP Results

**Status**: Accepted

## Decision

Images are re-encoded to downsized JPEG thumbnails; large arrays above a configurable
element threshold are truncated with metadata instead of inlined; every tool result is
capped at `max_result_bytes` centrally (§[09-perception-architecture.md](../09-perception-architecture.md)
Binary Data Policy).

## Why

MCP messages travel over the same channel as the LLM's reasoning context; a raw camera
frame (`sensor_msgs/Image`, uncompressed, easily several MB) or a `PointCloud2` would
dominate context/bandwidth for no benefit — the LLM cannot usefully reason over a raw
byte array of pixel values anyway; it needs either a compact visual (JPEG, which the
model can actually view) or a pre-computed structured summary (nearest object, detected
labels).

## Alternatives Considered

- **Send raw message bytes base64-encoded in JSON.** Rejected: multi-MB base64 strings in
  a tool result are both wasteful and useless to the model; also risks silently exceeding
  MCP/transport message-size limits.
- **Provide only structured summaries, never actual image content.** Rejected for the
  camera tool specifically: some queries ("what do you see") genuinely benefit from the
  LLM viewing an actual (small) image, which MCP explicitly supports as an image content
  block — throwing that away in favor of a text-only summary would lose real capability.
- **Store large data in an external object store, return only a reference URI.** Deferred,
  not rejected — noted in [09](../09-perception-architecture.md) as the natural extension
  for point clouds/maps post-MVP; not needed for the MVP's JPEG-thumbnail-sized camera
  results.

## Tradeoffs

- Pro: bounded, predictable result sizes; images remain genuinely useful to a
  vision-capable LLM; arrays too large to reason over are clearly flagged rather than
  silently dropped.
  Con: thumbnailing loses resolution/detail — acceptable for "what's in front of the
  robot" style queries; a future high-resolution-on-demand path can be added without
  breaking this default.

## Failure Modes

- A detector plugin needing full-resolution input internally must operate on the raw
  image *before* thumbnailing (the perception pipeline order in
  [09](../09-perception-architecture.md) step 4 runs detection on the full-res cached
  image; only the tool *result* image is downsized) — this ordering is a contract detail
  implementers must preserve.

## Recommendation

Keep the size ceiling centrally enforced (not per-adapter) so no future adapter can
accidentally reintroduce an unbounded result.

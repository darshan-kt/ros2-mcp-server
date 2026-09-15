"""StubDetectorPlugin — structurally satisfies
ros_mcp.contracts.adapters.ObjectDetectorPlugin and ros_mcp.contracts.plugins.Plugin
(docs/13-contracts.md §7 §12, docs/09-perception-architecture.md, ADR-010).

MVP's only ObjectDetectorPlugin implementation: a deterministic, data-dependent (not
hardcoded/random) heuristic — the darkest connected region of the frame above a
brightness-deviation threshold — proving the full pipeline (acquire -> detect ->
geometry -> TF -> structured result) without depending on a real model or GPU. Its
`detector_plugin_id` always appears in results so callers know not to treat this as
production-accurate (04-mcp-surface.md robot.detect_objects description).
"""
from __future__ import annotations

import numpy as np

from ros_mcp.contracts.adapters import DetectedObject2D, RawImage
from ros_mcp.contracts.plugins import PluginHealth, PluginMetadata

PLUGIN_ID = "stub_detector"


def _grayscale_array(image: RawImage) -> np.ndarray:
    arr = np.frombuffer(image.data, dtype=np.uint8)
    if image.encoding in ("rgb8", "bgr8", "rgba8", "bgra8"):
        channels = 4 if image.encoding in ("rgba8", "bgra8") else 3
        pixel_count = image.height_px * image.width_px
        arr = arr[: pixel_count * channels].reshape((image.height_px, image.width_px, channels))
        return arr[:, :, :3].astype(np.float32).mean(axis=2)
    # mono8 or anything else: best-effort single-channel reshape.
    pixel_count = image.height_px * image.width_px
    return arr[:pixel_count].reshape((image.height_px, image.width_px)).astype(np.float32)


class StubDetectorPlugin:
    """Structurally satisfies ros_mcp.contracts.adapters.ObjectDetectorPlugin and
    ros_mcp.contracts.plugins.Plugin."""

    def __init__(self, *, confidence: float = 0.3) -> None:
        self.metadata = PluginMetadata(
            plugin_id=PLUGIN_ID,
            api_version="1.0.0",
            provides_capabilities=("object_detection",),
            requires=(),
        )
        self._confidence = confidence

    async def health_check(self) -> PluginHealth:
        return PluginHealth.HEALTHY

    async def detect(self, image: RawImage) -> tuple[DetectedObject2D, ...]:
        if image.height_px <= 0 or image.width_px <= 0:
            return ()
        gray = _grayscale_array(image)
        threshold = gray.mean() - gray.std()
        mask = gray < threshold
        if not mask.any():
            return ()
        ys, xs = np.nonzero(mask)
        bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
        return (DetectedObject2D(label="object", confidence=self._confidence, bbox_px=bbox),)

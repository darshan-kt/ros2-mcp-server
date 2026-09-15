"""Unit tests for StubDetectorPlugin — a deterministic, data-dependent heuristic, not a
hardcoded or random result (ADR-010)."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone

import numpy as np

from ros_mcp.contracts.adapters import RawImage
from ros_mcp.adapters.perception.stub_detector import StubDetectorPlugin


def _raw_image(arr: np.ndarray, encoding: str = "rgb8") -> RawImage:
    return RawImage(
        width_px=arr.shape[1], height_px=arr.shape[0], encoding=encoding,
        data=arr.astype("uint8").tobytes(), frame_id="camera_link",
        stamp=datetime.now(timezone.utc),
    )


def test_uniform_image_yields_no_detection():
    arr = np.full((20, 20, 3), 128, dtype="uint8")
    image = _raw_image(arr)
    result = asyncio.run(StubDetectorPlugin().detect(image))
    assert result == ()


def test_dark_patch_is_detected_with_matching_bbox():
    arr = np.full((20, 30, 3), 200, dtype="uint8")
    arr[5:10, 10:20, :] = 0  # a clearly darker rectangular patch
    image = _raw_image(arr)

    result = asyncio.run(StubDetectorPlugin().detect(image))
    assert len(result) == 1
    obj = result[0]
    assert obj.label == "object"
    assert 0.0 < obj.confidence <= 1.0
    x_min, y_min, x_max, y_max = obj.bbox_px
    assert 8 <= x_min <= 12
    assert 3 <= y_min <= 7
    assert 18 <= x_max <= 22
    assert 8 <= y_max <= 12


def test_zero_size_image_yields_no_detection():
    image = RawImage(width_px=0, height_px=0, encoding="rgb8", data=b"", frame_id="",
                      stamp=datetime.now(timezone.utc))
    result = asyncio.run(StubDetectorPlugin().detect(image))
    assert result == ()


def test_metadata_and_health():
    plugin = StubDetectorPlugin()
    assert plugin.metadata.plugin_id == "stub_detector"
    health = asyncio.run(plugin.health_check())
    assert health.value == "healthy"

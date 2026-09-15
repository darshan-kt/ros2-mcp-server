"""Raw sensor_msgs/Image -> downsized JPEG thumbnail (docs/09-perception-architecture.md
Binary Data Policy, ADR-009). The only module that imports Pillow."""
from __future__ import annotations

import io
from typing import Any

import numpy as np
from PIL import Image

_SUPPORTED_ENCODINGS = {"rgb8", "bgr8", "mono8", "rgba8", "bgra8"}


def _raw_image_to_pil(raw_msg: Any) -> Image.Image:
    height, width, encoding = raw_msg.height, raw_msg.width, raw_msg.encoding
    if encoding not in _SUPPORTED_ENCODINGS:
        raise ValueError(f"unsupported sensor_msgs/Image encoding: {encoding!r}")

    data = bytes(raw_msg.data)
    arr = np.frombuffer(data, dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        arr = arr.reshape((height, width, 3))
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return Image.fromarray(arr, mode="RGB")
    if encoding == "mono8":
        arr = arr.reshape((height, width))
        return Image.fromarray(arr, mode="L").convert("RGB")
    # rgba8 / bgra8
    arr = arr.reshape((height, width, 4))
    if encoding == "bgra8":
        arr = arr[:, :, [2, 1, 0, 3]]
    return Image.fromarray(arr, mode="RGBA").convert("RGB")


def _compressed_image_to_pil(raw_msg: Any) -> Image.Image:
    data = bytes(raw_msg.data)
    return Image.open(io.BytesIO(data)).convert("RGB")


def to_pil_image(raw_msg: Any, *, is_compressed: bool) -> Image.Image:
    return _compressed_image_to_pil(raw_msg) if is_compressed else _raw_image_to_pil(raw_msg)


def encode_jpeg_thumbnail(
    raw_msg: Any, *, max_width_px: int, is_compressed: bool = False, jpeg_quality: int = 85
) -> tuple[bytes, int, int, int, int]:
    """Returns (jpeg_bytes, width_px, height_px, original_width_px, original_height_px)."""
    image = to_pil_image(raw_msg, is_compressed=is_compressed)
    original_width, original_height = image.size

    if original_width > max_width_px:
        scale = max_width_px / original_width
        new_size = (max_width_px, max(1, round(original_height * scale)))
        image = image.resize(new_size, Image.BILINEAR)

    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=jpeg_quality)
    return buffer.getvalue(), image.size[0], image.size[1], original_width, original_height

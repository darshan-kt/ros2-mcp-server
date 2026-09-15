"""ToolResult -> MCP content-block serialization. `CameraImageResult` gets a real MCP
image content block (docs/04-mcp-surface.md: "MCP image content block"); every other
result is a single JSON text block — never a bare string (docs/13-contracts.md §2)."""
from __future__ import annotations

import base64
import dataclasses
import json
from datetime import datetime
from enum import Enum
from typing import Any

from ros_mcp.contracts.errors import ToolResult
from ros_mcp.contracts.results import CameraImageResult


def to_json_safe(value: Any) -> Any:
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {f.name: to_json_safe(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, bytes):
        return base64.b64encode(value).decode("ascii")
    if isinstance(value, (list, tuple)):
        return [to_json_safe(v) for v in value]
    if isinstance(value, (frozenset, set)):
        return sorted(to_json_safe(v) for v in value)
    if isinstance(value, dict):
        return {str(k): to_json_safe(v) for k, v in value.items()}
    return value


def tool_result_to_mcp_content(result: ToolResult) -> list[dict[str, Any]]:
    """Returns plain dicts shaped like mcp.types content blocks
    ({"type": "text", "text": ...} / {"type": "image", "data": ..., "mimeType": ...}) —
    the server module (the only place that imports the `mcp` package itself) converts
    these into the real SDK types."""
    if isinstance(result, CameraImageResult) and result.status == "succeeded":
        image_b64 = base64.b64encode(result.image_jpeg_bytes).decode("ascii")
        metadata = to_json_safe(dataclasses.replace(result, image_jpeg_bytes=b""))
        return [
            {"type": "text", "text": json.dumps(metadata)},
            {"type": "image", "data": image_b64, "mimeType": "image/jpeg"},
        ]
    return [{"type": "text", "text": json.dumps(to_json_safe(result))}]

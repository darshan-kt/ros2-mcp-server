"""Message codec contract — frozen by docs/13-contracts.md §10."""
from __future__ import annotations

from typing import Any, Protocol


class MessageCodec(Protocol):
    def schema_for(self, type_name: str) -> dict[str, Any]:
        """JSON-Schema-shaped dict for a ROS type string. Cached per type_name for
        process lifetime (02-ros2-architecture.md)."""
        ...

    def to_dict(self, message: Any, *, element_limit: int = 4096) -> dict[str, Any]:
        """Recursive conversion; arrays longer than element_limit are truncated per the
        Binary Data Policy (09-perception-architecture.md), never silently included
        whole."""
        ...

    def from_dict(self, type_name: str, data: dict[str, Any]) -> Any:
        """Inverse of to_dict, used by ros.publish / ros.call_service / ros.send_action_goal."""
        ...

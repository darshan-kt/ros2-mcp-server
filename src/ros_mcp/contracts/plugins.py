"""Plugin contract — frozen by docs/13-contracts.md §12."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


@dataclass(frozen=True)
class PluginMetadata:
    plugin_id: str
    api_version: str
    provides_capabilities: tuple[str, ...]
    requires: tuple[str, ...]


class PluginHealth(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class Plugin(Protocol):
    metadata: PluginMetadata

    async def health_check(self) -> PluginHealth: ...

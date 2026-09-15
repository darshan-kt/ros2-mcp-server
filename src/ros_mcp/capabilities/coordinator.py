"""CapabilityCoordinator — glue wiring DiscoveryEngine snapshots into the
CapabilityRegistry via CapabilityInferenceEngine. Not a frozen contract type itself; it
composes three frozen ones (docs/13-contracts.md §3)."""
from __future__ import annotations

from typing import Callable

from ros_mcp.contracts.capabilities import CapabilityInferenceEngine
from ros_mcp.contracts.config import CapabilityConfig
from ros_mcp.contracts.discovery import DiscoveryEngine, GraphSnapshot
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry


class CapabilityCoordinator:
    def __init__(
        self,
        discovery_engine: DiscoveryEngine,
        inference_engine: CapabilityInferenceEngine,
        registry: InMemoryCapabilityRegistry,
        capability_config_provider: Callable[[], CapabilityConfig],
    ) -> None:
        self._discovery_engine = discovery_engine
        self._inference_engine = inference_engine
        self._registry = registry
        self._capability_config_provider = capability_config_provider
        self._discovery_engine.on_snapshot_changed(self._on_snapshot_changed)

    def _on_snapshot_changed(self, snapshot: GraphSnapshot) -> None:
        overrides = self._capability_config_provider()
        entries = self._inference_engine.infer(snapshot, overrides)
        self._registry.update(entries)

    async def refresh_from_current_snapshot(self) -> None:
        """Force one inference pass against whatever snapshot discovery currently
        holds — used at startup after DiscoveryEngine.start() so capabilities are
        populated even though the first snapshot may not have "changed" from the
        registry's still-empty initial state via the on_changed path alone."""
        snapshot = await self._discovery_engine.get_current_snapshot()
        self._on_snapshot_changed(snapshot)

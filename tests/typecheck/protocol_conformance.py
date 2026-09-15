"""Static Protocol-conformance check, verified with `mypy` (not runtime isinstance,
since only `CancellationToken` among the frozen Protocols in docs/13-contracts.md is
declared `@runtime_checkable` — the others are checked structurally by mypy instead).

This file is not executed by pytest; it is type-checked by mypy as part of the test
suite's acceptance evidence. Each line assigns a concrete implementation instance to a
variable annotated with the frozen Protocol type it must satisfy — mypy errors if the
concrete class's methods don't match the Protocol's signatures.

Run: python -m mypy tests/typecheck/protocol_conformance.py
"""
from __future__ import annotations

from ros_mcp.contracts.discovery import DiscoveryEngine
from ros_mcp.discovery.engine import RclpyDiscoveryEngine

_discovery_engine: DiscoveryEngine = RclpyDiscoveryEngine(ros_bridge=None, node=None)

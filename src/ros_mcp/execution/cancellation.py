"""SimpleCancellationToken — structurally satisfies
ros_mcp.contracts.adapters.CancellationToken (docs/13-contracts.md §7, `@runtime_checkable`).

Adapters poll `is_cancelled()`/`deadline_exceeded()` at <= one control-loop period, per
the MotionBackend/NavigationBackend contract note."""
from __future__ import annotations

import time


class SimpleCancellationToken:
    """Structurally (and, since CancellationToken is @runtime_checkable, at runtime via
    isinstance) satisfies ros_mcp.contracts.adapters.CancellationToken."""

    def __init__(self, deadline_monotonic: float) -> None:
        self._cancelled = False
        self._deadline_monotonic = deadline_monotonic

    def cancel(self) -> None:
        self._cancelled = True

    def is_cancelled(self) -> bool:
        return self._cancelled

    def deadline_exceeded(self) -> bool:
        return time.monotonic() >= self._deadline_monotonic

"""TF contract — frozen by docs/13-contracts.md §8."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Literal, Protocol


@dataclass(frozen=True)
class TransformResult:
    ok: bool
    x: float | None
    y: float | None
    yaw: float | None
    error: Literal["frame_unknown", "not_connected", "extrapolation", "timeout"] | None


class TFAdapter(Protocol):
    async def lookup_transform(
        self, target_frame: str, source_frame: str, stamp: datetime, timeout_s: float
    ) -> TransformResult:
        """Never raises tf2 exceptions across this boundary; all failure modes are
        values in TransformResult.error."""
        ...

    def known_frames(self) -> frozenset[str]: ...

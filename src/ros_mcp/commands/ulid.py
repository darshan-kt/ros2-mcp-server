"""Minimal ULID generator (pure stdlib — no new dependency). A ULID is a 48-bit
millisecond timestamp followed by 80 bits of randomness, both Crockford base32 encoded,
giving a lexically-time-sortable command_id (docs/05-semantic-command-model.md field
rationale)."""
from __future__ import annotations

import os
import time

_CROCKFORD_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def _encode_base32(data: bytes) -> str:
    # 128 bits (16 bytes) -> 26 Crockford base32 characters, matching the ULID spec.
    value = int.from_bytes(data, byteorder="big")
    chars = []
    for _ in range(26):
        value, remainder = divmod(value, 32)
        chars.append(_CROCKFORD_ALPHABET[remainder])
    return "".join(reversed(chars))


def new_ulid() -> str:
    timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFFFFFF  # 48 bits
    timestamp_bytes = timestamp_ms.to_bytes(6, byteorder="big")
    randomness = os.urandom(10)  # 80 bits
    return _encode_base32(timestamp_bytes + randomness)

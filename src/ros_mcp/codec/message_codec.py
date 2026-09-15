"""RosidlMessageCodec — structurally satisfies ros_mcp.contracts.codec.MessageCodec
(docs/13-contracts.md §10, ADR-007, docs/02-ros2-architecture.md Generic Message
Handling).

Conversion is built entirely on `rosidl_runtime_py` runtime introspection: no message
type (standard or custom) is hand-coded. `to_dict`/`from_dict` share the same
introspection data, so there is one source of truth for the ROS<->dict mapping.
"""
from __future__ import annotations

import re
import threading
from typing import Any

from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message

_ARRAY_PATTERN = re.compile(r"^(?P<base>[a-zA-Z0-9_/]+)\[(?P<bound>\d*)\]$")
_SEQUENCE_PATTERN = re.compile(r"^sequence<(?P<base>[a-zA-Z0-9_/]+)(?:,\s*(?P<bound>\d+))?>$")

_PRIMITIVE_JSON_TYPES: dict[str, str] = {
    "boolean": "boolean",
    "bool": "boolean",
    "byte": "integer",
    "char": "integer",
    "float": "number",
    "float32": "number",
    "float64": "number",
    "double": "number",
    "int8": "integer",
    "uint8": "integer",
    "int16": "integer",
    "uint16": "integer",
    "int32": "integer",
    "uint32": "integer",
    "int64": "integer",
    "uint64": "integer",
    "string": "string",
    "wstring": "string",
}

_TRUNCATION_NOTE = "use a dedicated perception tool or raise element_limit explicitly"


def _to_message_type_name(field_type: str) -> str:
    """rosidl's get_fields_and_field_types() reports nested message fields as
    'pkg/Type' (e.g. 'std_msgs/Header'); get_message() needs 'pkg/msg/Type'."""
    if "/" in field_type and "/msg/" not in field_type:
        pkg, cls = field_type.split("/", 1)
        return f"{pkg}/msg/{cls}"
    return field_type


def _is_message_type(field_type: str) -> bool:
    return "/" in field_type


class RosidlMessageCodec:
    """Structurally satisfies ros_mcp.contracts.codec.MessageCodec."""

    def __init__(self) -> None:
        self._schema_cache: dict[str, dict[str, Any]] = {}
        self._lock = threading.Lock()

    def schema_for(self, type_name: str) -> dict[str, Any]:
        with self._lock:
            cached = self._schema_cache.get(type_name)
            if cached is not None:
                return cached
        schema = self._build_schema(type_name, depth=0)
        with self._lock:
            self._schema_cache[type_name] = schema
        return schema

    def _build_schema(self, type_name: str, *, depth: int) -> dict[str, Any]:
        if depth > 16:  # defensive recursion-depth cap (02-ros2-architecture.md)
            return {"type": "object", "note": "max recursion depth reached"}
        msg_class = get_message(type_name)
        fields = msg_class.get_fields_and_field_types()
        properties: dict[str, Any] = {}
        for field_name, field_type in fields.items():
            properties[field_name] = self._schema_for_field_type(field_type, depth=depth)
        return {"type": "object", "typeName": type_name, "properties": properties}

    def _schema_for_field_type(self, field_type: str, *, depth: int) -> dict[str, Any]:
        array_match = _ARRAY_PATTERN.match(field_type)
        seq_match = _SEQUENCE_PATTERN.match(field_type)
        if array_match or seq_match:
            match = array_match or seq_match
            base = match.group("base")  # type: ignore[union-attr]
            bound = match.group("bound")  # type: ignore[union-attr]
            item_schema = self._scalar_or_message_schema(base, depth=depth)
            array_schema: dict[str, Any] = {"type": "array", "items": item_schema}
            if bound:
                array_schema["maxItems"] = int(bound)
            return array_schema
        return self._scalar_or_message_schema(field_type, depth=depth)

    def _scalar_or_message_schema(self, base_type: str, *, depth: int) -> dict[str, Any]:
        if _is_message_type(base_type):
            if base_type in ("octet", "uint8[]"):
                return {"type": "string", "format": "binary"}
            return self._build_schema(_to_message_type_name(base_type), depth=depth + 1)
        json_type = _PRIMITIVE_JSON_TYPES.get(base_type, "string")
        return {"type": json_type}

    def to_dict(self, message: Any, *, element_limit: int = 4096) -> dict[str, Any]:
        raw = message_to_ordereddict(message)
        converted = _truncate_large_values(raw, element_limit)
        assert isinstance(converted, dict)
        return converted

    def from_dict(self, type_name: str, data: dict[str, Any]) -> Any:
        msg_class = get_message(type_name)
        instance = msg_class()
        set_message_fields(instance, data)
        return instance


def _truncate_large_values(value: Any, element_limit: int) -> Any:
    """Binary Data Policy (docs/09-perception-architecture.md): any array-valued field
    longer than element_limit is replaced with a `{truncated, length, note}` marker
    rather than inlined whole. Applied recursively over the message_to_ordereddict
    output (which is already a plain-Python nested dict/list/OrderedDict tree)."""
    if isinstance(value, dict):
        return {k: _truncate_large_values(v, element_limit) for k, v in value.items()}
    if isinstance(value, bytes):
        if len(value) > element_limit:
            return {"truncated": True, "length": len(value), "note": _TRUNCATION_NOTE}
        return value.hex()
    if isinstance(value, (list, tuple)):
        if len(value) > element_limit:
            return {"truncated": True, "length": len(value), "note": _TRUNCATION_NOTE}
        return [_truncate_large_values(v, element_limit) for v in value]
    if hasattr(value, "tolist"):  # numpy arrays some rosidl generators return
        as_list = value.tolist()
        return _truncate_large_values(as_list, element_limit)
    return value

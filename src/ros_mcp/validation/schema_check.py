"""Minimal, dependency-free JSON-Schema subset checker sufficient for the MVP's flat
tool input schemas (object / string / number / integer / boolean / array-of-primitive,
enum, minimum, maximum, maxItems, required, additionalProperties). Intentionally not a
general-purpose validator — the MVP's own schemas (docs/04-mcp-surface.md) are all this
shallow, and adding the `jsonschema` package was not on the approved dependency list."""
from __future__ import annotations

from typing import Any

_TYPE_CHECKERS: dict[str, Any] = {
    "string": lambda v: isinstance(v, str),
    "number": lambda v: isinstance(v, (int, float)) and not isinstance(v, bool),
    "integer": lambda v: isinstance(v, int) and not isinstance(v, bool),
    "boolean": lambda v: isinstance(v, bool),
    "array": lambda v: isinstance(v, list),
    "object": lambda v: isinstance(v, dict),
}


def check_schema(schema: dict[str, Any], value: Any) -> str | None:
    """Returns None if `value` conforms to `schema`, else a human-readable error."""
    return _check(schema, value, path="$")


def _check(schema: dict[str, Any], value: Any, *, path: str) -> str | None:
    expected_type = schema.get("type")
    if expected_type is not None:
        checker = _TYPE_CHECKERS.get(expected_type)
        if checker is not None and not checker(value):
            return f"{path}: expected type {expected_type}, got {type(value).__name__}"

    if expected_type == "object":
        properties = schema.get("properties", {})
        required = schema.get("required", [])
        additional_allowed = schema.get("additionalProperties", True)
        for key in required:
            if key not in value:
                return f"{path}: missing required field '{key}'"
        for key, sub_value in value.items():
            if key in properties:
                error = _check(properties[key], sub_value, path=f"{path}.{key}")
                if error:
                    return error
            elif not additional_allowed:
                return f"{path}: unexpected field '{key}'"

    if expected_type == "array":
        items_schema = schema.get("items")
        max_items = schema.get("maxItems")
        if max_items is not None and len(value) > max_items:
            return f"{path}: array longer than maxItems={max_items}"
        if items_schema is not None:
            for i, item in enumerate(value):
                error = _check(items_schema, item, path=f"{path}[{i}]")
                if error:
                    return error

    if "enum" in schema and value not in schema["enum"]:
        return f"{path}: {value!r} not in allowed values {schema['enum']}"

    if expected_type in ("number", "integer"):
        minimum = schema.get("minimum")
        maximum = schema.get("maximum")
        if minimum is not None and value < minimum:
            return f"{path}: {value} < minimum {minimum}"
        if maximum is not None and value > maximum:
            return f"{path}: {value} > maximum {maximum}"

    return None

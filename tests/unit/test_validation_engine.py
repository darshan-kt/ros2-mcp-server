"""Unit tests for DefaultValidationEngine."""
from __future__ import annotations

from ros_mcp.contracts.errors import ErrorCode
from ros_mcp.validation.engine import DefaultValidationEngine


def test_unknown_tool_is_invalid():
    outcome = DefaultValidationEngine().validate("robot.frobnicate", {})
    assert outcome.ok is False
    assert outcome.error.code == ErrorCode.INVALID_ARGUMENT


def test_get_capabilities_requires_no_arguments():
    outcome = DefaultValidationEngine().validate("robot.get_capabilities", {})
    assert outcome.ok is True


def test_get_capabilities_rejects_unexpected_field():
    outcome = DefaultValidationEngine().validate("robot.get_capabilities", {"foo": 1})
    assert outcome.ok is False


def test_move_forward_requires_distance_m():
    outcome = DefaultValidationEngine().validate("robot.move", {"direction": "forward"})
    assert outcome.ok is False
    assert "distance_m" in outcome.error.message


def test_move_forward_with_distance_is_valid():
    outcome = DefaultValidationEngine().validate(
        "robot.move", {"direction": "forward", "distance_m": 1.0}
    )
    assert outcome.ok is True


def test_move_rotate_requires_angle_deg():
    outcome = DefaultValidationEngine().validate("robot.move", {"direction": "rotate_left"})
    assert outcome.ok is False
    assert "angle_deg" in outcome.error.message


def test_move_rejects_out_of_range_angle():
    outcome = DefaultValidationEngine().validate(
        "robot.move", {"direction": "rotate_left", "angle_deg": 400}
    )
    assert outcome.ok is False


def test_move_rejects_invalid_direction_enum():
    outcome = DefaultValidationEngine().validate(
        "robot.move", {"direction": "sideways", "distance_m": 1.0}
    )
    assert outcome.ok is False


def test_navigate_requires_x_and_y():
    outcome = DefaultValidationEngine().validate("robot.navigate", {"x": 1.0})
    assert outcome.ok is False


def test_navigate_valid_with_x_y():
    outcome = DefaultValidationEngine().validate("robot.navigate", {"x": 1.0, "y": 2.0})
    assert outcome.ok is True


def test_get_camera_image_rejects_width_over_max():
    outcome = DefaultValidationEngine().validate(
        "robot.get_camera_image", {"max_width_px": 5000}
    )
    assert outcome.ok is False


def test_detect_objects_labels_must_be_array_of_strings():
    outcome = DefaultValidationEngine().validate(
        "robot.detect_objects", {"labels": ["chair", 5]}
    )
    assert outcome.ok is False

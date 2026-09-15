"""Unit tests for YamlConfigProvider."""
from __future__ import annotations

import pytest

from ros_mcp.config.provider import ConfigLoadError, YamlConfigProvider


def test_loads_reference_robot_yaml():
    provider = YamlConfigProvider("config/robot.yaml")
    assert provider.robot().id == "turtlebot3_waffle"
    assert provider.safety().max_linear_mps == 0.5
    assert provider.capabilities_overrides().motion.cmd_vel_topic == "/cmd_vel"
    assert provider.raw_ros_access().enabled is False
    assert provider.timeouts().move_default_s == 30.0
    assert provider.perception().camera_max_width_px_default == 640
    assert provider.logging_config().format == "json"


def test_missing_file_raises_config_load_error():
    with pytest.raises(ConfigLoadError):
        YamlConfigProvider("config/does-not-exist.yaml")


def test_invalid_yaml_raises_config_load_error(tmp_path):
    bad = tmp_path / "robot.yaml"
    bad.write_text("robot:\n  id: [unterminated\n")
    with pytest.raises(ConfigLoadError):
        YamlConfigProvider(bad)


def test_validation_error_on_bad_field(tmp_path):
    bad = tmp_path / "robot.yaml"
    bad.write_text(
        "robot:\n  id: x\n  name: y\nsafety:\n  max_linear_mps: -1.0\n"
    )
    with pytest.raises(ConfigLoadError):
        YamlConfigProvider(bad)


def test_geofence_validator_rejects_inverted_bounds(tmp_path):
    bad = tmp_path / "robot.yaml"
    bad.write_text(
        "robot:\n  id: x\n  name: y\n"
        "safety:\n  geofence:\n    min_x: 5.0\n    max_x: 1.0\n"
    )
    with pytest.raises(ConfigLoadError):
        YamlConfigProvider(bad)


def test_reload_picks_up_file_changes(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("robot:\n  id: bot1\n  name: n1\n")
    provider = YamlConfigProvider(path)
    assert provider.robot().id == "bot1"

    path.write_text("robot:\n  id: bot2\n  name: n2\n")
    provider.reload()
    assert provider.robot().id == "bot2"


def test_env_overrides_logging_not_safety(tmp_path, monkeypatch):
    path = tmp_path / "robot.yaml"
    path.write_text("robot:\n  id: x\n  name: y\nsafety:\n  max_linear_mps: 0.5\n")
    monkeypatch.setenv("ROS_MCP_LOG_LEVEL", "DEBUG")
    provider = YamlConfigProvider(path)
    assert provider.logging_config().level == "DEBUG"
    # No env var path exists for safety values at all (ADR-013).
    assert provider.safety().max_linear_mps == 0.5

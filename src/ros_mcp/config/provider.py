"""YamlConfigProvider — structurally satisfies ros_mcp.contracts.config.ConfigProvider
(docs/13-contracts.md §11, docs/16-configuration.md).

Precedence: defaults (hardcoded in the pydantic models) < robot.yaml < env vars for the
non-safety keys ADR-013 names. There is deliberately no environment-variable path for
any `safety.*` value.
"""
from __future__ import annotations

import os
import threading
from pathlib import Path

import yaml
from pydantic import ValidationError

from ros_mcp.contracts.config import (
    CapabilityConfig,
    LoggingConfig,
    PerceptionConfig,
    RawRosAccessConfig,
    RobotConfig,
    RootConfig,
    SafetyConfig,
    TimeoutConfig,
)

ENV_CONFIG_PATH = "ROS_MCP_CONFIG_PATH"
ENV_LOG_LEVEL = "ROS_MCP_LOG_LEVEL"
ENV_LOG_FORMAT = "ROS_MCP_LOG_FORMAT"


class ConfigLoadError(RuntimeError):
    """Raised when robot.yaml is missing or fails pydantic validation. Server startup
    must fail fast on this (16-configuration.md Typed Loader), not at first tool use."""


class YamlConfigProvider:
    """Structurally satisfies ros_mcp.contracts.config.ConfigProvider."""

    def __init__(self, config_path: str | Path) -> None:
        self._config_path = Path(config_path)
        self._lock = threading.Lock()
        self._root: RootConfig = self._load()

    @classmethod
    def from_env_or_default(
        cls, default_path: str = "config/robot.yaml"
    ) -> "YamlConfigProvider":
        path = os.environ.get(ENV_CONFIG_PATH, default_path)
        return cls(path)

    def _load(self) -> RootConfig:
        if not self._config_path.is_file():
            raise ConfigLoadError(f"config file not found: {self._config_path}")
        try:
            raw_text = self._config_path.read_text()
        except OSError as exc:
            raise ConfigLoadError(f"could not read config file {self._config_path}: {exc}") from exc
        try:
            raw = yaml.safe_load(raw_text) or {}
        except yaml.YAMLError as exc:
            raise ConfigLoadError(f"invalid YAML in {self._config_path}: {exc}") from exc
        try:
            root = RootConfig.model_validate(raw)
        except ValidationError as exc:
            raise ConfigLoadError(f"invalid config in {self._config_path}:\n{exc}") from exc

        # Non-safety env overrides only (ADR-013): logging.
        log_level = os.environ.get(ENV_LOG_LEVEL)
        log_format = os.environ.get(ENV_LOG_FORMAT)
        if log_level or log_format:
            updated_logging = root.logging.model_copy(
                update={
                    **({"level": log_level} if log_level else {}),
                    **({"format": log_format} if log_format in ("json", "text") else {}),
                }
            )
            root = root.model_copy(update={"logging": updated_logging})
        return root

    def robot(self) -> RobotConfig:
        with self._lock:
            return self._root.robot

    def safety(self) -> SafetyConfig:
        with self._lock:
            return self._root.safety

    def capabilities_overrides(self) -> CapabilityConfig:
        with self._lock:
            return self._root.capabilities

    def raw_ros_access(self) -> RawRosAccessConfig:
        with self._lock:
            return self._root.raw_ros_access

    def timeouts(self) -> TimeoutConfig:
        with self._lock:
            return self._root.timeouts

    def perception(self) -> PerceptionConfig:
        with self._lock:
            return self._root.perception

    def logging_config(self) -> LoggingConfig:
        with self._lock:
            return self._root.logging

    def reload(self) -> None:
        new_root = self._load()
        with self._lock:
            self._root = new_root

"""Config contract — frozen by docs/13-contracts.md §11 + field shapes from
docs/16-configuration.md's robot.yaml schema.

Typed config models are `pydantic.BaseModel` subclasses — "the one place pydantic is
used, per the approved dependency list" (13-contracts.md §11). This is the only module
in the codebase that imports pydantic.

`ConfigProvider` exposes the four accessor methods 13-contracts.md §11 names
(`robot`, `safety`, `capabilities_overrides`, `raw_ros_access`) plus `reload`. 16-
configuration.md additionally names `TimeoutConfig`, `PerceptionConfig`, and
`LoggingConfig` as models this system produces, but does not add accessor methods for
them to the frozen Protocol. Rather than widen a frozen signature, `ConfigProvider`
below carries `timeouts()`, `perception()`, and `logging_config()` as *additional*
methods beyond the Protocol's required minimum — Python Protocols are structural: a
conforming class may implement more than a Protocol requires without breaking
conformance to it. Consumers that only need the frozen four continue to type against
the `ConfigProvider` Protocol unchanged.
"""
from __future__ import annotations

from typing import Literal, Protocol

from pydantic import BaseModel, Field, model_validator


class RobotConfig(BaseModel):
    id: str
    name: str


class DiscoveryConfig(BaseModel):
    poll_interval_s: float = Field(default=5.0, gt=0)


class MotionCapabilityConfig(BaseModel):
    enabled: bool = True
    backend: str = "cmd_vel"
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"


class NavigationCapabilityConfig(BaseModel):
    enabled: bool = True
    backend: str = "nav2"
    action_name: str = "/navigate_to_pose"


class LidarCapabilityConfig(BaseModel):
    topic: str = "/scan"


class CameraCapabilityConfig(BaseModel):
    image_topic: str = "/camera/image_raw"
    info_topic: str = "/camera/camera_info"


class ObjectDetectionCapabilityConfig(BaseModel):
    backend: str = "stub_detector"


class CapabilityConfig(BaseModel):
    motion: MotionCapabilityConfig = Field(default_factory=MotionCapabilityConfig)
    navigation: NavigationCapabilityConfig = Field(default_factory=NavigationCapabilityConfig)
    lidar: LidarCapabilityConfig = Field(default_factory=LidarCapabilityConfig)
    camera: CameraCapabilityConfig = Field(default_factory=CameraCapabilityConfig)
    object_detection: ObjectDetectionCapabilityConfig = Field(
        default_factory=ObjectDetectionCapabilityConfig
    )


class GeofenceConfig(BaseModel):
    enabled: bool = False
    frame: str = "map"
    min_x: float = -10.0
    max_x: float = 10.0
    min_y: float = -10.0
    max_y: float = 10.0

    @model_validator(mode="after")
    def _check_bounds(self) -> "GeofenceConfig":
        if self.min_x >= self.max_x:
            raise ValueError("geofence.min_x must be < geofence.max_x")
        if self.min_y >= self.max_y:
            raise ValueError("geofence.min_y must be < geofence.max_y")
        return self


class HumanApprovalConfig(BaseModel):
    required_above_distance_m: float = Field(default=5.0, ge=0)
    timeout_s: float = Field(default=30.0, gt=0)
    on_timeout: Literal["deny", "allow"] = "deny"


class SafetyConfig(BaseModel):
    max_linear_mps: float = Field(default=0.5, gt=0)
    max_angular_rps: float = Field(default=1.0, gt=0)
    max_linear_acceleration_mps2: float = Field(default=0.5, gt=0)
    max_angular_acceleration_rps2: float = Field(default=1.0, gt=0)
    max_move_distance_m: float = Field(default=5.0, gt=0)
    max_navigation_distance_m: float = Field(default=10.0, gt=0)
    obstacle_stop_distance_m: float = Field(default=0.3, ge=0)
    odom_stale_s: float = Field(default=0.5, gt=0)
    tf_stale_s: float = Field(default=1.0, gt=0)
    sensor_stale_s: float = Field(default=1.0, gt=0)
    cmd_vel_watchdog_s: float = Field(default=1.0, gt=0)
    rate_limit_rps: float = Field(default=2.0, gt=0)
    geofence: GeofenceConfig = Field(default_factory=GeofenceConfig)
    approval_policy: Literal["automatic", "policy_based", "always_human"] = "automatic"
    human_approval: HumanApprovalConfig = Field(default_factory=HumanApprovalConfig)


class TimeoutConfig(BaseModel):
    move_default_s: float = Field(default=30.0, gt=0)
    move_max_s: float = Field(default=60.0, gt=0)
    navigate_default_s: float = Field(default=120.0, gt=0)
    navigate_max_s: float = Field(default=300.0, gt=0)

    @model_validator(mode="after")
    def _check_defaults_within_max(self) -> "TimeoutConfig":
        if self.move_default_s > self.move_max_s:
            raise ValueError("timeouts.move_default_s must be <= timeouts.move_max_s")
        if self.navigate_default_s > self.navigate_max_s:
            raise ValueError("timeouts.navigate_default_s must be <= timeouts.navigate_max_s")
        return self


class PerceptionConfig(BaseModel):
    max_result_bytes: int = Field(default=1_048_576, gt=0)
    camera_max_width_px_default: int = Field(default=640, gt=0)
    camera_max_width_px_cap: int = Field(default=1280, gt=0)
    array_truncate_element_limit: int = Field(default=4096, gt=0)

    @model_validator(mode="after")
    def _check_camera_widths(self) -> "PerceptionConfig":
        if self.camera_max_width_px_default > self.camera_max_width_px_cap:
            raise ValueError(
                "perception.camera_max_width_px_default must be <= camera_max_width_px_cap"
            )
        return self


class RawRosAccessConfig(BaseModel):
    enabled: bool = False
    allow_publish: bool = False
    allow_param_write: bool = False
    topic_allowlist: list[str] = Field(default_factory=list)


class LoggingConfig(BaseModel):
    level: str = "INFO"
    format: Literal["json", "text"] = "json"


class RootConfig(BaseModel):
    """The full parsed shape of robot.yaml (docs/16-configuration.md)."""

    robot: RobotConfig
    discovery: DiscoveryConfig = Field(default_factory=DiscoveryConfig)
    capabilities: CapabilityConfig = Field(default_factory=CapabilityConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    timeouts: TimeoutConfig = Field(default_factory=TimeoutConfig)
    perception: PerceptionConfig = Field(default_factory=PerceptionConfig)
    raw_ros_access: RawRosAccessConfig = Field(default_factory=RawRosAccessConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)


class ConfigProvider(Protocol):
    def robot(self) -> RobotConfig: ...
    def safety(self) -> SafetyConfig: ...
    def capabilities_overrides(self) -> CapabilityConfig: ...
    def raw_ros_access(self) -> RawRosAccessConfig: ...
    def reload(self) -> None:
        """Re-reads robot.yaml from disk; does NOT retroactively affect
        safety_constraints already resolved into in-flight SemanticCommands
        (05-semantic-command-model.md)."""
        ...

    # Additional accessors beyond the frozen four (see module docstring).
    def timeouts(self) -> TimeoutConfig: ...
    def perception(self) -> PerceptionConfig: ...
    def logging_config(self) -> LoggingConfig: ...

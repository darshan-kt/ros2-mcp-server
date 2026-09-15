# 16 — Configuration

## Precedence

`robot.yaml` (explicit) **always** overrides automatic discovery/inference; automatic
discovery fills in everything not explicitly configured. Environment variables override
`robot.yaml` values that are deployment-specific (paths, log level) but never override
safety limits (safety limits must be reviewable in a committed file, not silently changed
by an environment variable — see [ADR-013](adr/ADR-013-config-overrides-discovery.md)).

```text
defaults (hardcoded, conservative) < discovery/inference < robot.yaml < env vars (non-safety keys only)
```

## `robot.yaml` Schema (MVP fields)

```yaml
robot:
  id: turtlebot3_waffle          # stable robot_id, multi-robot-ready (14-multi-robot.md)
  name: "TurtleBot3 Waffle (sim)"

discovery:
  poll_interval_s: 5.0

capabilities:
  # Explicit overrides win over inference at any confidence level (03-capability-discovery.md).
  motion:
    enabled: true
    backend: cmd_vel
    cmd_vel_topic: /cmd_vel
    odom_topic: /odom
  navigation:
    enabled: true
    backend: nav2
    action_name: /navigate_to_pose
  lidar:
    topic: /scan
  camera:
    image_topic: /camera/image_raw
    info_topic: /camera/camera_info
  object_detection:
    backend: stub_detector

safety:
  max_linear_mps: 0.5
  max_angular_rps: 1.0
  max_linear_acceleration_mps2: 0.5
  max_angular_acceleration_rps2: 1.0
  max_move_distance_m: 5.0
  max_navigation_distance_m: 10.0
  obstacle_stop_distance_m: 0.3
  odom_stale_s: 0.5
  tf_stale_s: 1.0
  sensor_stale_s: 1.0
  cmd_vel_watchdog_s: 1.0
  rate_limit_rps: 2.0
  geofence:
    enabled: false
    frame: map
    min_x: -10.0
    max_x: 10.0
    min_y: -10.0
    max_y: 10.0
  approval_policy: automatic     # automatic | policy_based | always_human
  human_approval:
    required_above_distance_m: 5.0
    timeout_s: 30
    on_timeout: deny

timeouts:
  move_default_s: 30.0
  move_max_s: 60.0
  navigate_default_s: 120.0
  navigate_max_s: 300.0

perception:
  max_result_bytes: 1048576
  camera_max_width_px_default: 640
  camera_max_width_px_cap: 1280
  array_truncate_element_limit: 4096

raw_ros_access:
  enabled: false
  allow_publish: false
  allow_param_write: false
  topic_allowlist: []

logging:
  level: INFO
  format: json                   # json | text
```

## Typed Loader

`ConfigProvider` (§[13-contracts.md](13-contracts.md)) loads this file into `pydantic`
models (`RobotConfig`, `SafetyConfig`, `CapabilityConfig`, `RawRosAccessConfig`,
`TimeoutConfig`, `PerceptionConfig`, `LoggingConfig`) with field validation (e.g.
`max_linear_mps > 0`, `geofence.min_x < geofence.max_x`) at load time — a malformed
`robot.yaml` fails the server at startup with a clear validation error, not at first use.

## Environment Variable Overrides (non-safety only)

```text
ROS_MCP_CONFIG_PATH      # path to robot.yaml, default ./config/robot.yaml
ROS_MCP_LOG_LEVEL        # overrides logging.level
ROS_MCP_LOG_FORMAT       # overrides logging.format
```

No `ROS_MCP_SAFETY_*` environment variables exist — this is deliberate (see
[ADR-013](adr/ADR-013-config-overrides-discovery.md)).

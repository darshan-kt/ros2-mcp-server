"""The fixed capability ontology (docs/03-capability-discovery.md). Each `_infer_*`
function is a pure function of (GraphSnapshot, CapabilityConfig, SafetyConfig) ->
CapabilityEntry | None. Only the capabilities the MVP's 8 tools actually need are
implemented — manipulation/docking/diagnostics are post-MVP (17-mvp.md exclusions).

Non-speculation rule (03-capability-discovery.md, ADR-003): a capability at
Confidence.AMBIGUOUS is still registered (visible via robot.get_capabilities) but the
Tool Provider must exclude it from the live tool list — enforced in
ros_mcp.mcp.tool_provider, not here.
"""
from __future__ import annotations

from ros_mcp.contracts.capabilities import CapabilityEntry
from ros_mcp.contracts.config import CapabilityConfig, SafetyConfig
from ros_mcp.contracts.core import Confidence
from ros_mcp.contracts.discovery import GraphSnapshot, TopicInfo

_TWIST_TYPES = frozenset({"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"})
_ODOM_TYPE = "nav_msgs/msg/Odometry"
_LASER_SCAN_TYPE = "sensor_msgs/msg/LaserScan"
_IMAGE_TYPES = frozenset({"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"})
_POSE_WITH_COV_TYPE = "geometry_msgs/msg/PoseWithCovarianceStamped"
_NAVIGATE_TO_POSE_TYPE = "nav2_msgs/action/NavigateToPose"


def _topics_of_type(snapshot: GraphSnapshot, type_name: str) -> tuple[TopicInfo, ...]:
    return tuple(t for t in snapshot.topics if t.type_name == type_name)


def _topics_of_types(snapshot: GraphSnapshot, type_names: frozenset[str]) -> tuple[TopicInfo, ...]:
    return tuple(t for t in snapshot.topics if t.type_name in type_names)


def _resolve_single_topic_capability(
    *,
    capability_id: str,
    backend_id: str,
    configured_name: str,
    candidates: tuple[TopicInfo, ...],
    resolved_key: str,
    extra_limits: dict[str, float] | None = None,
) -> CapabilityEntry | None:
    if not candidates:
        return None
    exact = next((t for t in candidates if t.name == configured_name), None)
    if exact is not None:
        return CapabilityEntry(
            capability_id=capability_id,
            confidence=Confidence.CONFIRMED,
            backend_id=backend_id,
            resolved_topics={resolved_key: exact.name},
            limits=extra_limits or {},
        )
    if len(candidates) == 1:
        return CapabilityEntry(
            capability_id=capability_id,
            confidence=Confidence.LIKELY,
            backend_id=backend_id,
            resolved_topics={resolved_key: candidates[0].name},
            limits=extra_limits or {},
        )
    return CapabilityEntry(
        capability_id=capability_id,
        confidence=Confidence.AMBIGUOUS,
        backend_id=backend_id,
        resolved_topics={},
        limits=extra_limits or {},
    )


def infer_differential_drive_motion(
    snapshot: GraphSnapshot, overrides: CapabilityConfig, safety: SafetyConfig
) -> CapabilityEntry | None:
    cfg = overrides.motion
    if not cfg.enabled:
        return None
    twist_candidates = _topics_of_types(snapshot, _TWIST_TYPES)
    odom_candidates = _topics_of_type(snapshot, _ODOM_TYPE)
    if not twist_candidates or not odom_candidates:
        return None

    cmd_vel_exact = next((t for t in twist_candidates if t.name == cfg.cmd_vel_topic), None)
    odom_exact = next((t for t in odom_candidates if t.name == cfg.odom_topic), None)

    if cmd_vel_exact is not None and odom_exact is not None:
        confidence = Confidence.CONFIRMED
        cmd_vel_name, odom_name = cmd_vel_exact.name, odom_exact.name
    elif len(twist_candidates) == 1 and len(odom_candidates) == 1:
        confidence = Confidence.LIKELY
        cmd_vel_name, odom_name = twist_candidates[0].name, odom_candidates[0].name
    else:
        confidence = Confidence.AMBIGUOUS
        cmd_vel_name = odom_name = ""

    resolved_topics = {"cmd_vel": cmd_vel_name, "odom": odom_name} if confidence != Confidence.AMBIGUOUS else {}
    return CapabilityEntry(
        capability_id="differential_drive_motion",
        confidence=confidence,
        backend_id="cmd_vel",
        resolved_topics=resolved_topics,
        limits={
            "max_linear_mps": safety.max_linear_mps,
            "max_angular_rps": safety.max_angular_rps,
            "max_move_distance_m": safety.max_move_distance_m,
        },
    )


def infer_autonomous_navigation(
    snapshot: GraphSnapshot, overrides: CapabilityConfig, safety: SafetyConfig
) -> CapabilityEntry | None:
    cfg = overrides.navigation
    if not cfg.enabled:
        return None
    matching = [a for a in snapshot.actions if a.type_name == _NAVIGATE_TO_POSE_TYPE]
    if not matching:
        return None
    exact = next((a for a in matching if a.name == cfg.action_name), None)
    if exact is not None:
        confidence = Confidence.CONFIRMED
        action_name = exact.name
    elif len(matching) == 1:
        confidence = Confidence.LIKELY
        action_name = matching[0].name
    else:
        confidence = Confidence.AMBIGUOUS
        action_name = ""
    return CapabilityEntry(
        capability_id="autonomous_navigation",
        confidence=confidence,
        backend_id="nav2",
        resolved_topics={"navigate_to_pose_action": action_name} if action_name else {},
        limits={"max_navigation_distance_m": safety.max_navigation_distance_m},
    )


def infer_localization(snapshot: GraphSnapshot, overrides: CapabilityConfig) -> CapabilityEntry | None:
    amcl_candidates = _topics_of_type(snapshot, _POSE_WITH_COV_TYPE)
    amcl_exact = next((t for t in amcl_candidates if t.name == "/amcl_pose"), None)
    if amcl_exact is not None:
        return CapabilityEntry(
            capability_id="localization",
            confidence=Confidence.CONFIRMED,
            backend_id="amcl",
            resolved_topics={"amcl_pose": amcl_exact.name},
        )
    if "map" in snapshot.tf_frames and len(snapshot.tf_frames) > 1:
        return CapabilityEntry(
            capability_id="localization",
            confidence=Confidence.LIKELY,
            backend_id="tf",
            resolved_topics={},
        )
    return None


def infer_range_sensing(snapshot: GraphSnapshot, overrides: CapabilityConfig) -> CapabilityEntry | None:
    candidates = _topics_of_type(snapshot, _LASER_SCAN_TYPE)
    return _resolve_single_topic_capability(
        capability_id="range_sensing",
        backend_id="laser_scan",
        configured_name=overrides.lidar.topic,
        candidates=candidates,
        resolved_key="scan",
    )


def infer_visual_observation(snapshot: GraphSnapshot, overrides: CapabilityConfig) -> CapabilityEntry | None:
    candidates = _topics_of_types(snapshot, _IMAGE_TYPES)
    return _resolve_single_topic_capability(
        capability_id="visual_observation",
        backend_id="camera",
        configured_name=overrides.camera.image_topic,
        candidates=candidates,
        resolved_key="image",
    )


def infer_object_detection(
    visual_observation: CapabilityEntry | None, overrides: CapabilityConfig
) -> CapabilityEntry | None:
    if visual_observation is None:
        return None
    return CapabilityEntry(
        capability_id="object_detection",
        confidence=visual_observation.confidence,
        backend_id=overrides.object_detection.backend,
        resolved_topics=dict(visual_observation.resolved_topics),
    )

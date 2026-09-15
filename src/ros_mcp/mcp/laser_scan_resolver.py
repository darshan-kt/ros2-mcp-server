"""Builds LaserScanResult from the CapabilityRegistry + SubscriptionManager
(docs/04-mcp-surface.md robot.get_laser_scan, docs/09-perception-architecture.md)."""
from __future__ import annotations

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.errors import ErrorCode, ToolError
from ros_mcp.contracts.results import LaserScanResult
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.execution.error_results import build_error_result
from ros_mcp.perception.laser_scan import analyze_laser_scan


def resolve_laser_scan(
    *,
    robot_id: str,
    command_id: str,
    duration_sec: float,
    registry: CapabilityRegistry,
    subscriptions: SubscriptionManager,
    include_raw_ranges: bool,
    sensor_stale_s: float,
) -> LaserScanResult:
    cap = registry.get("range_sensing")
    if cap is None or "scan" not in cap.resolved_topics:
        result = build_error_result(
            LaserScanResult,
            command_id=command_id,
            robot_id=robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.CAPABILITY_UNAVAILABLE, message="no laser scan sensor available"),
        )
        assert isinstance(result, LaserScanResult)
        return result

    topic = cap.resolved_topics["scan"]
    cached = subscriptions.get_latest(topic)
    if cached is None:
        result = build_error_result(
            LaserScanResult,
            command_id=command_id,
            robot_id=robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.SENSOR_STALE, message=f"no laser scan ever received on {topic}"),
        )
        assert isinstance(result, LaserScanResult)
        return result

    analysis = analyze_laser_scan(cached.raw)
    stale = cached.age_s > sensor_stale_s

    return LaserScanResult(
        status="succeeded",
        command_id=command_id,
        robot_id=robot_id,
        duration_sec=duration_sec,
        stamp=cached.stamp,
        frame=analysis.frame_id,
        nearest=analysis.nearest,
        farthest=analysis.farthest,
        sectors=dict(analysis.sectors),
        raw_ranges=analysis.raw_ranges if include_raw_ranges else None,
        data_age_s=cached.age_s,
        stale=stale,
        invalid_beam_count=analysis.invalid_beam_count,
    )

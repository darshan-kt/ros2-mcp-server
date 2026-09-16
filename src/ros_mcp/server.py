"""ROS-MCP-Server entrypoint: `python -m ros_mcp.server --config config/robot.yaml`.

Wires every component built in docs/17-mvp.md's seven steps into one running process:
rclpy executor thread + ThreadSafeRosBridge (ADR-011) on one side, the MCP stdio server
on the other, joined only through the frozen contracts (docs/13-contracts.md). Starts
and serves even with no robot present, reporting zero capabilities rather than crashing
(17-mvp.md acceptance criterion).
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import re
import sys
from typing import Any

logger = logging.getLogger("ros_mcp.server")


def _sanitize_node_name(robot_id: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_]", "_", robot_id)
    if not cleaned or not (cleaned[0].isalpha() or cleaned[0] == "_"):
        cleaned = f"_{cleaned}"
    return f"ros_mcp_{cleaned}"


async def _amain(config_path: str) -> None:
    # Imports deferred past logging setup / argument parsing so `--help` and config
    # validation errors don't require rclpy to already be importable.
    import rclpy
    import tf2_ros

    from ros_mcp.adapters.motion.cmd_vel_plugin import CmdVelMotionPlugin
    from ros_mcp.adapters.navigation.nav2_plugin import Nav2NavigationPlugin
    from ros_mcp.adapters.perception.perception_adapter import DefaultPerceptionAdapter
    from ros_mcp.adapters.perception.stub_detector import StubDetectorPlugin
    from ros_mcp.adapters.perception.tf_adapter import RclpyTFAdapter
    from ros_mcp.capabilities.coordinator import CapabilityCoordinator
    from ros_mcp.capabilities.inference import DefaultCapabilityInferenceEngine
    from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
    from ros_mcp.codec.message_codec import RosidlMessageCodec
    from ros_mcp.commands.factory import SemanticCommandFactory
    from ros_mcp.config.provider import YamlConfigProvider
    from ros_mcp.contracts.core import Pose2D
    from ros_mcp.discovery.engine import RclpyDiscoveryEngine
    from ros_mcp.execution.manager import DefaultExecutionManager
    from ros_mcp.execution.planner import DefaultCommandPlanner
    from ros_mcp.mcp.read_handlers import build_read_handlers
    from ros_mcp.mcp.resource_provider import DefaultResourceProvider
    from ros_mcp.mcp.tool_provider import DefaultToolProvider
    from ros_mcp.robot_state import resolve_current_pose
    from ros_mcp.ros.bridge import ThreadSafeRosBridge
    from ros_mcp.ros.executor_thread import RclpyExecutorThread
    from ros_mcp.safety.policy_engine import DefaultSafetyPolicyEngine
    from ros_mcp.telemetry.logging_setup import configure_logging
    from ros_mcp.validation.engine import DefaultValidationEngine

    config_provider = YamlConfigProvider(config_path)
    logging_cfg = config_provider.logging_config()
    configure_logging(level=logging_cfg.level, format_=logging_cfg.format)
    logger.info("loaded config from %s", config_path)

    robot_id = config_provider.robot().id
    node_name = _sanitize_node_name(robot_id)

    rclpy.init(args=None)
    node = rclpy.create_node(node_name)
    executor_thread = RclpyExecutorThread(node)
    executor_thread.start()

    loop = asyncio.get_running_loop()
    bridge = ThreadSafeRosBridge(executor_thread, loop)

    tf_buffer, _tf_listener = await bridge.call_ros_from_asyncio(
        lambda: (lambda buf: (buf, tf2_ros.TransformListener(buf, node)))(tf2_ros.Buffer())
    )

    codec = RosidlMessageCodec()
    subscriptions = _make_subscription_manager(bridge, node, codec)

    registry = InMemoryCapabilityRegistry()
    discovery_engine = RclpyDiscoveryEngine(
        ros_bridge=bridge, node=node, tf_buffer=tf_buffer,
        poll_interval_s=config_provider.discovery().poll_interval_s,
    )
    inference_engine = DefaultCapabilityInferenceEngine(config_provider.safety())
    CapabilityCoordinator(
        discovery_engine, inference_engine, registry, config_provider.capabilities_overrides
    )

    tf_adapter = RclpyTFAdapter(
        ros_bridge=bridge, tf_buffer=tf_buffer,
        graph_snapshot_provider=discovery_engine.latest_snapshot_sync,
    )

    motion_cfg = config_provider.capabilities_overrides().motion
    lidar_cfg = config_provider.capabilities_overrides().lidar
    motion_backend = CmdVelMotionPlugin(
        ros_bridge=bridge, node=node, subscriptions=subscriptions, robot_id=robot_id,
        cmd_vel_topic=motion_cfg.cmd_vel_topic, odom_topic=motion_cfg.odom_topic,
        laser_scan_topic=lidar_cfg.topic, safety_config_provider=config_provider.safety,
    )

    # The robot-base TF frame varies by platform (TurtleBot3's TF tree uses
    # base_footprint, not the generic base_link the docs use as an example — confirmed
    # against the live reference platform). Not a robot.yaml field (16-configuration.md's
    # schema doesn't have one); ROS_MCP_BASE_FRAME is a plain deployment-specific env
    # var in the same spirit as ROS_MCP_LOG_LEVEL (ADR-013), defaulting to base_link.
    base_frame = os.environ.get("ROS_MCP_BASE_FRAME", "base_link")

    progress_sink_box: dict[str, Any] = {"fn": None}
    navigation_backend = Nav2NavigationPlugin(
        ros_bridge=bridge, node=node, tf_adapter=tf_adapter, capability_registry=registry,
        action_name=config_provider.capabilities_overrides().navigation.action_name,
        base_frame=base_frame,
        progress_sink=lambda cid, payload: progress_sink_box["fn"] and progress_sink_box["fn"](cid, payload),
    )

    detector_plugin = StubDetectorPlugin()
    camera_cfg = config_provider.capabilities_overrides().camera
    perception_adapter = DefaultPerceptionAdapter(
        robot_id=robot_id, registry=registry, subscriptions=subscriptions, tf_adapter=tf_adapter,
        detector_plugin=detector_plugin, safety_config_provider=config_provider.safety,
        perception_config_provider=config_provider.perception, base_frame=base_frame,
        image_topic=camera_cfg.image_topic, scan_topic=lidar_cfg.topic,
    )

    planner = DefaultCommandPlanner(motion_backend=motion_backend, navigation_backend=navigation_backend)
    safety_engine = DefaultSafetyPolicyEngine(config_provider.safety)

    async def current_pose_provider() -> Pose2D | None:
        return await resolve_current_pose(
            registry=registry, subscriptions=subscriptions,
            tf_adapter=tf_adapter, base_frame=base_frame,
        )

    execution_manager = DefaultExecutionManager(
        registry=registry, command_planner=planner, safety_policy_engine=safety_engine,
        current_pose_provider=current_pose_provider, read_handlers={},
        motion_backend=motion_backend, navigation_backend=navigation_backend,
    )
    execution_manager.set_read_handlers(
        build_read_handlers(
            registry=registry, subscriptions=subscriptions, perception_adapter=perception_adapter,
            execution_manager=execution_manager, perception_config_provider=config_provider.perception,
            tf_adapter=tf_adapter, base_frame=base_frame,
        )
    )
    progress_sink_box["fn"] = execution_manager.emit_progress

    validation_engine = DefaultValidationEngine()
    command_factory = SemanticCommandFactory(robot_id=robot_id)
    tool_provider = DefaultToolProvider(
        registry=registry, validation_engine=validation_engine, execution_manager=execution_manager,
        command_factory=command_factory, config_provider=config_provider,
    )
    resource_provider = DefaultResourceProvider(
        registry=registry, subscriptions=subscriptions, discovery_engine=discovery_engine,
        execution_manager=execution_manager, config_provider=config_provider,
        tf_adapter=tf_adapter, base_frame=base_frame,
    )

    await discovery_engine.start()
    logger.info(
        "discovery complete: %d capabilities registered",
        len(registry.current()),
    )

    try:
        await _serve_mcp_stdio(tool_provider, resource_provider, robot_id)
    finally:
        await discovery_engine.stop()
        executor_thread.stop()
        rclpy.shutdown()


def _make_subscription_manager(bridge: Any, node: Any, codec: Any) -> Any:
    from ros_mcp.subscriptions.manager import PooledSubscriptionManager

    return PooledSubscriptionManager(ros_bridge=bridge, node=node, codec=codec)


async def _serve_mcp_stdio(tool_provider: Any, resource_provider: Any, robot_id: str) -> None:
    import mcp.types as types
    from mcp.server.lowlevel import Server
    from mcp.server.lowlevel.server import ServerRequestContext
    from mcp.server.stdio import stdio_server

    from ros_mcp.mcp.serialization import tool_result_to_mcp_content

    session_id = f"stdio-{robot_id}"

    def _content_block(raw: dict[str, Any]) -> Any:
        if raw["type"] == "image":
            return types.ImageContent(type="image", data=raw["data"], mime_type=raw["mimeType"])
        return types.TextContent(type="text", text=raw["text"])

    async def on_list_tools(ctx: ServerRequestContext, params: Any) -> types.ListToolsResult:
        specs = tool_provider.current_tools()
        tools = [
            types.Tool(name=s.name, description=s.description, input_schema=s.input_schema)
            for s in specs
        ]
        return types.ListToolsResult(tools=tools)

    async def on_call_tool(ctx: ServerRequestContext, params: types.CallToolRequestParams) -> types.CallToolResult:
        result = await tool_provider.handle_call(params.name, dict(params.arguments or {}), session_id)
        content = [_content_block(b) for b in tool_result_to_mcp_content(result)]
        return types.CallToolResult(content=content, is_error=(result.status == "failed"))

    async def on_list_resources(ctx: ServerRequestContext, params: Any) -> types.ListResourcesResult:
        resources = [
            types.Resource(name=uri, uri=uri, description=f"ROS-MCP-Server resource: {uri}",
                            mime_type="application/json")
            for uri in resource_provider.current_resources()
        ]
        return types.ListResourcesResult(resources=resources)

    async def on_read_resource(
        ctx: ServerRequestContext, params: types.ReadResourceRequestParams
    ) -> types.ReadResourceResult:
        import json

        data = await resource_provider.read(str(params.uri))
        return types.ReadResourceResult(
            contents=[types.TextResourceContents(uri=params.uri, mime_type="application/json", text=json.dumps(data))]
        )

    server: Any = Server(
        "ros-mcp-server",
        version="0.1.0",
        description="Semantic robotics MCP surface over an unmodified ROS 2 graph.",
        on_list_tools=on_list_tools,
        on_call_tool=on_call_tool,
        on_list_resources=on_list_resources,
        on_read_resource=on_read_resource,
    )

    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="python -m ros_mcp.server")
    parser.add_argument("--config", default="config/robot.yaml", help="Path to robot.yaml")
    args = parser.parse_args(argv)

    try:
        asyncio.run(_amain(args.config))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(sys.argv[1:])

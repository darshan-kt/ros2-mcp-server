"""Static Protocol-conformance check, verified with `mypy` (not runtime isinstance,
since only `CancellationToken` among the frozen Protocols in docs/13-contracts.md is
declared `@runtime_checkable` — the others are checked structurally by mypy instead).

This file is not executed by pytest; it is type-checked by mypy as part of the test
suite's acceptance evidence. Each line assigns a concrete implementation instance to a
variable annotated with the frozen Protocol type it must satisfy — mypy errors if the
concrete class's methods don't match the Protocol's signatures.

Run: python -m mypy tests/typecheck/protocol_conformance.py
"""
from __future__ import annotations

from ros_mcp.capabilities.inference import DefaultCapabilityInferenceEngine
from ros_mcp.capabilities.registry import InMemoryCapabilityRegistry
from ros_mcp.codec.message_codec import RosidlMessageCodec
from ros_mcp.config.provider import YamlConfigProvider
from ros_mcp.contracts.capabilities import CapabilityInferenceEngine, CapabilityRegistry
from ros_mcp.contracts.codec import MessageCodec
from ros_mcp.contracts.config import ConfigProvider, PerceptionConfig, SafetyConfig
from ros_mcp.contracts.discovery import DiscoveryEngine
from ros_mcp.contracts.adapters import CancellationToken, MotionBackend, NavigationBackend
from ros_mcp.contracts.execution import CommandPlanner, ExecutionManager
from ros_mcp.contracts.tf import TFAdapter
from ros_mcp.contracts.safety import SafetyPolicyEngine, ValidationEngine
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.commands.factory import SemanticCommandFactory
from ros_mcp.discovery.engine import RclpyDiscoveryEngine
from ros_mcp.adapters.motion.cmd_vel_plugin import CmdVelMotionPlugin
from ros_mcp.adapters.navigation.nav2_plugin import Nav2NavigationPlugin
from ros_mcp.adapters.perception.perception_adapter import DefaultPerceptionAdapter
from ros_mcp.adapters.perception.stub_detector import StubDetectorPlugin
from ros_mcp.adapters.perception.tf_adapter import RclpyTFAdapter
from ros_mcp.contracts.adapters import ObjectDetectorPlugin, PerceptionAdapter
from ros_mcp.contracts.mcp_surface import ResourceProvider, ToolProvider
from ros_mcp.execution.cancellation import SimpleCancellationToken
from ros_mcp.mcp.resource_provider import DefaultResourceProvider
from ros_mcp.mcp.tool_provider import DefaultToolProvider
from ros_mcp.execution.manager import DefaultExecutionManager
from ros_mcp.execution.planner import DefaultCommandPlanner
from ros_mcp.safety.policy_engine import DefaultSafetyPolicyEngine
from ros_mcp.subscriptions.manager import PooledSubscriptionManager
from ros_mcp.validation.engine import DefaultValidationEngine

_discovery_engine: DiscoveryEngine = RclpyDiscoveryEngine(ros_bridge=None, node=None)
_capability_registry: CapabilityRegistry = InMemoryCapabilityRegistry()
_capability_inference: CapabilityInferenceEngine = DefaultCapabilityInferenceEngine(SafetyConfig())
_message_codec: MessageCodec = RosidlMessageCodec()
_subscription_manager: SubscriptionManager = PooledSubscriptionManager(ros_bridge=None, node=None)
_config_provider: ConfigProvider = YamlConfigProvider("config/robot.yaml")
_validation_engine: ValidationEngine = DefaultValidationEngine()
_safety_policy_engine: SafetyPolicyEngine = DefaultSafetyPolicyEngine(lambda: SafetyConfig())
_cancellation_token: CancellationToken = SimpleCancellationToken(deadline_monotonic=0.0)
_motion_backend: MotionBackend = CmdVelMotionPlugin(
    ros_bridge=None,
    node=None,
    subscriptions=PooledSubscriptionManager(ros_bridge=None, node=None),
    robot_id="r1",
    cmd_vel_topic="/cmd_vel",
    odom_topic="/odom",
    laser_scan_topic=None,
    safety_config_provider=lambda: SafetyConfig(),
)
_command_planner: CommandPlanner = DefaultCommandPlanner(motion_backend=None, navigation_backend=None)
_tf_adapter: TFAdapter = RclpyTFAdapter(ros_bridge=None, tf_buffer=None)
_navigation_backend: NavigationBackend = Nav2NavigationPlugin(
    ros_bridge=None,
    node=None,
    tf_adapter=_tf_adapter,
    capability_registry=_capability_registry,
    action_name="/navigate_to_pose",
)


async def _current_pose_provider():
    return None


_execution_manager: ExecutionManager = DefaultExecutionManager(
    registry=_capability_registry,
    command_planner=_command_planner,
    safety_policy_engine=_safety_policy_engine,
    current_pose_provider=_current_pose_provider,
    read_handlers={},
)
_detector_plugin: ObjectDetectorPlugin = StubDetectorPlugin()
_perception_adapter: PerceptionAdapter = DefaultPerceptionAdapter(
    robot_id="r1",
    registry=_capability_registry,
    subscriptions=_subscription_manager,
    tf_adapter=_tf_adapter,
    detector_plugin=_detector_plugin,
    safety_config_provider=lambda: SafetyConfig(),
    perception_config_provider=lambda: PerceptionConfig(),
)
_tool_provider: ToolProvider = DefaultToolProvider(
    registry=_capability_registry,
    validation_engine=_validation_engine,
    execution_manager=_execution_manager,
    command_factory=SemanticCommandFactory(robot_id="r1"),
    config_provider=_config_provider,
)
_resource_provider: ResourceProvider = DefaultResourceProvider(
    registry=_capability_registry,
    subscriptions=_subscription_manager,
    discovery_engine=_discovery_engine,
    execution_manager=_execution_manager,
    config_provider=_config_provider,
)

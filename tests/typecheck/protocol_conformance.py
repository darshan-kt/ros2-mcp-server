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
from ros_mcp.contracts.config import ConfigProvider, SafetyConfig
from ros_mcp.contracts.discovery import DiscoveryEngine
from ros_mcp.contracts.safety import SafetyPolicyEngine, ValidationEngine
from ros_mcp.contracts.subscriptions import SubscriptionManager
from ros_mcp.discovery.engine import RclpyDiscoveryEngine
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

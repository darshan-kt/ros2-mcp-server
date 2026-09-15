"""The mandatory safety choke point (docs/10-safety-and-trust.md, ADR-002). Every
MOTION/HIGH_RISK SemanticCommand passes through SafetyPolicyEngine.check() before any
adapter is dereferenced — enforced structurally by ros_mcp.execution.manager, not by
convention."""

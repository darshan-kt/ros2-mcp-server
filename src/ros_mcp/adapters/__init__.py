"""ROS 2 backend adapters (docs/13-contracts.md §7, Non-Bypass Rule).

Hard module-boundary rule (frozen, docs/13-contracts.md §5 Non-Bypass Rule): packages
under `ros_mcp.adapters.*` import nothing from `ros_mcp.mcp.*`. Adapters receive a
SemanticCommand that has already been validated and safety-checked; they never see raw
MCP tool arguments and have no path back into command construction or execution-state
mutation.
"""

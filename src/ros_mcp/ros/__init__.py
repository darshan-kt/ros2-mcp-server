"""The only package allowed to import `rclpy` directly (alongside `ros_mcp.adapters.*`,
which imports rclpy message/action types but never spins an executor itself).

Owns the async/threading boundary (docs/13-contracts.md §13, ADR-011): a dedicated
executor thread plus the `RosBridge` implementation that is the sole legal crossing
point between it and the MCP-side asyncio loop.
"""

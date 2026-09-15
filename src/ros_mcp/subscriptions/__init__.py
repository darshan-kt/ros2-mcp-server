"""Subscription pooling with a last-value cache (docs/13-contracts.md §9, ADR-008).
Exactly one rclpy subscription per (topic, type) for the process lifetime; no adapter
creates its own."""

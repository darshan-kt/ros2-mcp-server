"""Small shared geometry helpers used across the state resolver, motion adapter, and
navigation adapter. Not part of any frozen contract — plain utility functions."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


def quaternion_to_yaw(q: Quaternion) -> float:
    """Standard yaw-from-quaternion extraction (Z-axis rotation of a 2D-planar robot)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw / 2.0
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def normalize_angle(angle_rad: float) -> float:
    """Wrap to (-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def angular_difference(target_rad: float, current_rad: float) -> float:
    """Shortest signed angular distance from current to target, in (-pi, pi]."""
    return normalize_angle(target_rad - current_rad)


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)

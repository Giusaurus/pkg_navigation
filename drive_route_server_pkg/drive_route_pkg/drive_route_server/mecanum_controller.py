"""
mecanum_controller.py

"""

from __future__ import annotations

import math
from dataclasses import dataclass


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wraps an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Velocity2D:

    vx: float
    vy: float
    omega: float


@dataclass
class ControllerGains:

    k_linear: float = 1.0
    k_angular: float = 2.0

    max_linear_speed: float = 0.4   # m/s
    max_angular_speed: float = 1.5  # rad/s

    # Waypoint is reached in this radius

    waypoint_tolerance_m: float = 0.1

    # smaller tolerances for last waypoint
    final_position_tolerance_m: float = 0.03
    final_heading_tolerance_rad: float = 0.15


def compute_velocity_command(
    current: Pose2D,
    target_x: float,
    target_y: float,
    gains: ControllerGains,
    is_final_waypoint: bool = False,
) -> tuple[Velocity2D, bool]:

    dx_world = target_x - current.x
    dy_world = target_y - current.y
    distance = math.hypot(dx_world, dy_world)

    position_tolerance = gains.final_position_tolerance_m if is_final_waypoint else gains.waypoint_tolerance_m
    reached = distance <= position_tolerance

    if reached and not is_final_waypoint:

        return Velocity2D(0.0, 0.0, 0.0), True


    if distance > 1e-3:
        desired_yaw = math.atan2(dy_world, dx_world)
    else:
        desired_yaw = current.yaw

    heading_error = normalize_angle(desired_yaw - current.yaw)


    cos_yaw = math.cos(current.yaw)
    sin_yaw = math.sin(current.yaw)
    dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
    dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world

    linear_scale = gains.k_linear
    brake_zone = gains.final_position_tolerance_m * 5
    if is_final_waypoint and distance < brake_zone:
        linear_scale *= max(distance / brake_zone, 0.15)

    vx = _clamp(dx_robot * linear_scale, gains.max_linear_speed)
    vy = _clamp(dy_robot * linear_scale, gains.max_linear_speed)
    omega = _clamp(heading_error * gains.k_angular, gains.max_angular_speed)

    if is_final_waypoint and reached:

        if abs(heading_error) <= gains.final_heading_tolerance_rad:
            return Velocity2D(0.0, 0.0, 0.0), True
        else:
            return Velocity2D(0.0, 0.0, omega), False

    return Velocity2D(vx, vy, omega), False


def _clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return 0.0
    return max(-limit, min(limit, value))

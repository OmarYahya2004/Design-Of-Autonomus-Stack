"""Shared math helpers for the localization nodes (noise + EKF)."""
import math


def euler_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw (rad, ZYX order) to a quaternion [x, y, z, w]."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def quaternion_to_euler(qx, qy, qz, qw):
    """Convert a quaternion [x, y, z, w] to roll, pitch, yaw (rad, ZYX order)."""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sin_pitch = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

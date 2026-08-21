"""Inverse kinematics for a compound arm mechanism.

Two-stage IK solver based on:
  R.L. Williams II, "The Delta Parallel Robot: Kinematics Solutions", 2016.
  https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf

Mechanism topology (per arm):
  Servo -> [servo_upper_arm, servo_lower_arm] -> arm_base
  arm_base -> [arm_joint_offset] -> arm_joint -> [upper_arm, lower_arm] -> end_effector

Stage 1: Given target position (relative to arm base), compute main arm angle theta
          using the E·cos(θ) + F·sin(θ) + G = 0 loop-closure constraint.
Stage 2: Given theta, compute required arm_base position, then solve the servo
          two-circle intersection to find the servo angle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class ArmMechConfig:
    """Geometry for one arm of the mechanism."""

    # Servo linkage: connects servo to arm_base
    servo_upper_arm_len: float  # servo -> elbow
    servo_lower_arm_len: float  # elbow -> arm_base

    # Positions in workspace frame
    servo_pos: Tuple[float, float, float]  # servo pivot position
    platform_joint_pos: Tuple[float, float, float]  # target / platform joint

    # Arm geometry
    servo_to_arm_base_len: float  # horizontal offset: servo pivot to arm_base
    arm_joint_len: float  # arm_base -> arm_joint offset length
    upper_arm_len: float  # arm_joint -> elbow (main arm)
    lower_arm_len: float  # elbow -> end_effector (main arm)


def _solve_arm_ik_stage1(
    target: np.ndarray,
    arm_joint_len: float,
    upper_arm_len: float,
    lower_arm_len: float,
) -> float:
    """Compute main arm angle theta from target position (relative to arm base).

    Uses the loop-closure constraint for a 2-link planar arm:
        E * cos(theta) + F * sin(theta) + G = 0
    solved via tangent half-angle substitution (Williams, 2016, p.12).

    The arm joint is at offset (arm_joint_len, 0) from the arm base.
    End effector = arm_joint + (upper_arm_len * cos(theta), lower_arm_len * sin(theta)).

    Returns:
        theta in radians (angle of upper arm from horizontal).

    Raises:
        ValueError: if target is unreachable (negative discriminant).
    """
    tx, ty = target[0], target[1]

    # Arm joint offset from arm base (along x-axis in arm frame)
    jx = arm_joint_len

    # Target relative to arm joint
    dx = tx - jx
    dy = ty

    # E, F, G from expanding |end_effector - target|^2 = 0
    # where end_effector = (jx + u*cos(theta), l*sin(theta))
    E = 2.0 * upper_arm_len * dx
    F = 2.0 * lower_arm_len * dy
    G = dx * dx + dy * dy + upper_arm_len * upper_arm_len - lower_arm_len * lower_arm_len

    # Tangent half-angle substitution: t = tan(theta/2)
    # Quadratic: (G - E)*t^2 + 2*F*t + (G + E) = 0
    discriminant = E * E + F * F - G * G
    if discriminant < 0:
        raise ValueError(
            f"Target {target} unreachable: discriminant = {discriminant:.6f} < 0. "
            f"Target may be outside the workspace."
        )

    sqrt_disc = math.sqrt(discriminant)

    # Choose the '+' solution (knee-out configuration, typical for delta robots)
    t = (-F + sqrt_disc) / (G - E) if abs(G - E) > 1e-12 else (-F - sqrt_disc) / (G - E)

    theta = 2.0 * math.atan(t)
    return theta


def _solve_servo_ik_stage2(
    theta: float,
    base_len: float,
    joint_len: float,
    u_len: float,
    l_len: float,
) -> float:
    """Compute servo angle from main arm angle theta.

    Given theta, the desired arm_base position is:
        p = (base_len + joint_len * cos(theta), joint_len * sin(theta))

    The servo arm (upper=u_len, lower=l_len) must reach p from the origin.
    This is solved as the intersection of two circles:
        Circle 1: center (0,0), radius u_len
        Circle 2: center p,      radius l_len

    Returns:
        Servo angle in radians.

    Raises:
        ValueError: if circles don't intersect (servo cannot reach arm_base).
    """
    # Desired arm_base position in servo frame
    px = base_len + joint_len * math.cos(theta)
    py = joint_len * math.sin(theta)

    # Distance between circle centers (origin and p)
    dist = math.sqrt(px * px + py * py)

    if dist < 1e-12:
        raise ValueError("Servo and arm_base coincide; mechanism is singular.")

    # Circle-circle intersection (two circles: origin@u_len, p@l_len)
    # a = distance from origin along the line to the chord midpoint
    a = (u_len * u_len + dist * dist - l_len * l_len) / (2.0 * dist)
    h_sq = u_len * u_len - a * a
    if h_sq < 0:
        raise ValueError(
            f"Servo circles don't intersect: h^2 = {h_sq:.6f} < 0. "
            f"Check servo_upper_arm_len={u_len} and servo_lower_arm_len={l_len}."
        )
    h = math.sqrt(h_sq)

    # Unit vector from origin to p
    ux, uy = px / dist, py / dist

    # Perpendicular unit vector (rotated 90 degrees CCW)
    nx, ny = -uy, ux

    # Chord midpoint
    mx, my = a * ux, a * uy

    # Two intersection points; choose the one with larger y (knee-out)
    tar_ctrl_x = mx + h * nx
    tar_ctrl_y = my + h * ny

    # Servo angle from origin to control point
    servo_angle = math.atan2(tar_ctrl_y, tar_ctrl_x)
    return servo_angle


def arm_compute_ik(
    tar_pos: np.ndarray,
    mech_config: Sequence[ArmMechConfig],
) -> Sequence[float]:
    """Compute inverse kinematics for all arms.

    For each arm, solves:
        Stage 1: target position -> main arm angle theta
        Stage 2: theta -> servo angle

    Args:
        tar_pos: Target end-effector position (x, y, z), relative to each arm base.
        mech_config: Sequence of ArmMechConfig, one per arm.

    Returns:
        List of servo angles in radians, one per arm.
    """
    results: list[float] = []
    tar_pos = np.asarray(tar_pos, dtype=float)

    for config in mech_config:
        # --- Stage 1: Arm IK ---
        # Solve for main arm angle theta from target position
        theta = _solve_arm_ik_stage1(
            target=tar_pos,
            arm_joint_len=config.arm_joint_len,
            upper_arm_len=config.upper_arm_len,
            lower_arm_len=config.lower_arm_len,
        )

        # --- Stage 2: Servo IK ---
        # Convert theta to servo angle via two-circle intersection
        servo_angle = _solve_servo_ik_stage2(
            theta=theta,
            base_len=config.servo_to_arm_base_len,
            joint_len=config.arm_joint_len,
            u_len=config.servo_upper_arm_len,
            l_len=config.servo_lower_arm_len,
        )

        results.append(servo_angle)

    return results

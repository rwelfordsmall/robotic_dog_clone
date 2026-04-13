"""
kinematics.py
-------------

Shared kinematics utilities for the active 8-DOF quadruped control path.

Canonical conventions:
- Cartesian leg target frame:
    +X = forward
    +Y = left / outward
    +Z = up in body-frame convention
- Active sagittal IK path uses:
    foot_x : forward-positive target (mm)
    foot_z : downward-positive leg depth (mm)
- /joint_angles publishes motor-space commands
- Gazebo / ros2_control consume geometric / URDF joint angles

This module owns:
- sagittal inverse kinematics
- sagittal forward kinematics
- motor-space <-> geometric-space conversions
- all-leg conversion into the 8-value /joint_angles vector

If Gazebo motion disagrees with the Cartesian convention above,
fix the shared math here rather than compensating in test nodes.
"""

import math
from dog.robot_config import (
    HIP_LENGTH,
    UPPER_LENGTH,
    LOWER_LENGTH,
    JOINT_DIRECTION,
    JOINT_OFFSETS,
    JOINT_ANGLE_MIN,
    JOINT_ANGLE_MAX,
)


class IKError(Exception):
    """Raised when a foot position is outside the reachable workspace."""
    pass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ik_sagittal_geometric(foot_x: float, foot_z: float) -> tuple[float, float]:
    """
    Compute sagittal shoulder and knee angles in geometric / URDF space.

    Parameters
    ----------
    foot_x : float
        Forward-positive foot target in mm.
    foot_z : float
        Downward-positive foot depth below the shoulder pivot in mm.

    Returns
    -------
    (shoulder_deg, knee_deg)
        Shoulder and knee in DEGREES in geometric / IK space:
        - shoulder_deg: 0 = upper leg vertical
        - knee_deg: 0 = straight leg, negative = bent
    """
    vertical_offset = foot_z
    reach = math.sqrt(foot_x ** 2 + vertical_offset ** 2)

    max_reach = UPPER_LENGTH + LOWER_LENGTH
    min_reach = abs(UPPER_LENGTH - LOWER_LENGTH)

    if reach > max_reach:
        raise IKError(
            f"Foot ({foot_x:.1f}, {foot_z:.1f}) out of reach "
            f"— distance {reach:.1f} mm > max {max_reach:.1f} mm"
        )
    if reach < min_reach + 1e-3:
        raise IKError(
            f"Foot too close to shoulder — distance {reach:.1f} mm < min {min_reach:.1f} mm"
        )

    cos_knee = (
        UPPER_LENGTH ** 2 + LOWER_LENGTH ** 2 - reach ** 2
    ) / (2.0 * UPPER_LENGTH * LOWER_LENGTH)
    cos_knee = clamp(cos_knee, -1.0, 1.0)
    knee_deg = math.degrees(math.acos(cos_knee)) - 180.0

    cos_sho = (
        UPPER_LENGTH ** 2 + reach ** 2 - LOWER_LENGTH ** 2
    ) / (2.0 * UPPER_LENGTH * reach)
    cos_sho = clamp(cos_sho, -1.0, 1.0)
    alpha = math.degrees(math.acos(cos_sho))

    # Canonical rule: +foot_x means forward.
    # Shared IK must map that rule to the actual URDF/sim sign convention.
    beta = math.degrees(math.atan2(-foot_x, vertical_offset))
    shoulder_deg = alpha + beta

    return shoulder_deg, knee_deg

def fk_sagittal_geometric(shoulder_deg: float, knee_deg: float) -> tuple[float, float]:
    """
    Forward kinematics for the active sagittal leg model.

    Parameters
    ----------
    shoulder_deg : float
        Shoulder angle in geometric / IK space.
    knee_deg : float
        Knee angle in geometric / IK space.

    Returns
    -------
    (foot_x, foot_z)
        Forward-positive X and downward-positive Z in mm.
    """
    shoulder_rad = math.radians(shoulder_deg)
    knee_rad = math.radians(knee_deg)

    upper_x = -UPPER_LENGTH * math.sin(shoulder_rad)
    upper_z =  UPPER_LENGTH * math.cos(shoulder_rad)

    lower_abs = shoulder_rad + knee_rad
    lower_x = -LOWER_LENGTH * math.sin(lower_abs)
    lower_z =  LOWER_LENGTH * math.cos(lower_abs)

    foot_x = upper_x + lower_x
    foot_z = upper_z + lower_z
    return foot_x, foot_z

def motor_from_geometric(shoulder_deg: float, knee_deg: float, leg_index: int) -> tuple[float, float]:
    """
    Convert geometric shoulder/knee angles in degrees into motor-space commands in radians.
    """
    def to_rad(ik_deg: float, leg: int, joint: int) -> float:
        direction = JOINT_DIRECTION.get((leg, joint), 1)
        offset_rad = math.radians(JOINT_OFFSETS.get((leg, joint), 0.0))
        cmd = direction * math.radians(ik_deg) + offset_rad
        return clamp(cmd, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX)

    knee_coupled = shoulder_deg + knee_deg
    return (
        to_rad(shoulder_deg, leg_index, 1),
        to_rad(knee_coupled, leg_index, 2),
    )

def geometric_from_motor(shoulder_motor_rad: float, knee_motor_rad: float, leg_index: int) -> tuple[float, float]:
    """
    Convert motor-space commands in radians back into geometric / IK-space angles in degrees.
    Useful for debugging and FK validation.
    """
    shoulder_dir = JOINT_DIRECTION.get((leg_index, 1), 1)
    shoulder_off = math.radians(JOINT_OFFSETS.get((leg_index, 1), 0.0))

    knee_dir = JOINT_DIRECTION.get((leg_index, 2), 1)
    knee_off = math.radians(JOINT_OFFSETS.get((leg_index, 2), 0.0))

    shoulder_deg = math.degrees((shoulder_motor_rad - shoulder_off) / shoulder_dir)
    knee_coupled_deg = math.degrees((knee_motor_rad - knee_off) / knee_dir)
    knee_deg = knee_coupled_deg - shoulder_deg

    return shoulder_deg, knee_deg

def solve_leg_ik(foot_x: float, foot_y: float, foot_z: float, leg_side: int = 1) -> tuple[float, float, float]:
    """
    Compatibility wrapper for older callers.

    Active 8-DOF control path uses sagittal-only IK:
    - hip angle is returned as 0.0
    - foot_y and leg_side are ignored except for compatibility
    """
    shoulder_deg, knee_deg = ik_sagittal_geometric(foot_x, foot_z)
    hip_deg = 0.0
    return hip_deg, shoulder_deg, knee_deg

def ik_to_joint_angles(shoulder_ik: float, knee_ik: float, leg_index: int) -> tuple[float, float]:
    """
    Backward-compatible wrapper around motor_from_geometric().
    """
    return motor_from_geometric(shoulder_ik, knee_ik, leg_index)


def compute_all_legs(foot_positions: list[tuple[float, float, float]]) -> list[float]:
    """
    Run sagittal IK for all four legs and return 8 motor-space commands in radians.

    Input foot_positions are ordered [FR, FL, RR, RL] and use:
    - x = forward-positive
    - z = downward-positive leg depth
    - y is currently ignored in the active 8-DOF path
    """
    angles = []

    for i, (x, y, z) in enumerate(foot_positions):
        try:
            shoulder_deg, knee_deg = ik_sagittal_geometric(x, z)
            s_rad, k_rad = motor_from_geometric(shoulder_deg, knee_deg, i)
        except IKError:
            from dog.robot_config import NEUTRAL_ANGLES
            s_rad = NEUTRAL_ANGLES[i * 2]
            k_rad = NEUTRAL_ANGLES[i * 2 + 1]

        angles += [s_rad, k_rad]

    return angles


if __name__ == "__main__":
    test_points = [
        (0.0, 396.2),
        (50.0, 396.2),
        (-50.0, 396.2),
    ]

    for x, z in test_points:
        sho, kne = ik_sagittal_geometric(x, z)
        rx, rz = fk_sagittal_geometric(sho, kne)
        print(
            f"target=({x:.1f}, {z:.1f}) "
            f"ik=(sho={sho:.2f} deg, kne={kne:.2f} deg) "
            f"fk=({rx:.1f}, {rz:.1f})"
        )
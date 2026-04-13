#!/usr/bin/env python3
"""
step_test_node_claude.py
------------------------

Shared-IK validation node for robotic_dog_clone (8-DOF).

This node validates:
1. shared Cartesian sign conventions
2. shared sagittal IK/FK behavior
3. motor-space to geometric-space knee conversion through sim_bridge_node
4. single-leg step-cycle behavior in Gazebo

Published output:
- /joint_angles: 8 motor-space joint commands in radians
  Order: FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne

Important conventions:
- +X = forward
- -X = rearward
- Z is downward-positive foot depth in the shared sagittal model
- published values are MOTOR-FRAME commands intended for /joint_angles
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from dog.robot_config import NEUTRAL_ANGLES, STAND_HEIGHT
from dog.kinematics import (
    ik_sagittal_geometric,
    fk_sagittal_geometric,
    motor_from_geometric,
    geometric_from_motor,
    IKError,
)

# Leg indices used throughout the repo
FR = 0
FL = 1
RR = 2
RL = 3

LEG_NAMES = {
    FR: "FR",
    FL: "FL",
    RR: "RR",
    RL: "RL",
}

# ---------------------------------------------------------------------------
# Validation parameters
# ---------------------------------------------------------------------------

# Active test leg
TEST_LEG = FR

# Cartesian validation targets (meters)
X_REAR_M = -0.050
X_MID_M = 0.000
X_FWD_M = 0.050

# Single-step validation targets (meters)
X_STEP_M = 0.060
Z_STAND_M = STAND_HEIGHT / 1000.0
Z_LIFT_M = (STAND_HEIGHT - 55.0) / 1000.0  # smaller z => foot lifted upward

# Hold durations (seconds)
T_INITIAL_SETTLE = 4.0
T_POSE_HOLD = 3.0
T_SWEEP_HOLD = 3.0
T_STEP_SHIFT = 2.5
T_STEP_LIFT = 2.5
T_STEP_SWING = 3.0
T_STEP_PLANT = 2.0
T_STEP_RECOVER = 3.0

# ---------------------------------------------------------------------------
# Shared-IK wrappers
# ---------------------------------------------------------------------------


def ik_motor_frame(x_fwd_m: float, z_down_m: float, leg_index: int = TEST_LEG) -> tuple[float, float]:
    """
    Convert a shared Cartesian sagittal target into motor-frame commands.

    Parameters
    ----------
    x_fwd_m : float
        Forward-positive x target in meters.
    z_down_m : float
        Downward-positive z depth in meters.

    Returns
    -------
    (shoulder_motor_rad, knee_motor_rad)
        Motor-frame commands suitable for /joint_angles.
    """
    x_mm = x_fwd_m * 1000.0
    z_mm = z_down_m * 1000.0
    shoulder_deg, knee_deg = ik_sagittal_geometric(x_mm, z_mm)
    return motor_from_geometric(shoulder_deg, knee_deg, leg_index)


def describe_motor_pose(label: str, shoulder_motor: float, knee_motor: float, leg_index: int = TEST_LEG) -> str:
    """
    Return a diagnostic string describing a motor-frame pose in shared geometric/FK terms.
    """
    shoulder_deg, knee_deg = geometric_from_motor(shoulder_motor, knee_motor, leg_index)
    x_mm, z_mm = fk_sagittal_geometric(shoulder_deg, knee_deg)
    return (
        f"{label}: "
        f"motor=({shoulder_motor:.4f}, {knee_motor:.4f}) rad | "
        f"geo=({shoulder_deg:.2f}, {knee_deg:.2f}) deg | "
        f"fk=({x_mm:.1f} mm, {z_mm:.1f} mm)"
    )


@dataclass(frozen=True)
class LegPose:
    label: str
    shoulder_motor: float
    knee_motor: float


def make_pose(label: str, x_m: float, z_m: float, leg_index: int = TEST_LEG) -> LegPose:
    """
    Build a motor-frame pose from a shared Cartesian target.
    """
    sho, kne = ik_motor_frame(x_m, z_m, leg_index)
    return LegPose(label=label, shoulder_motor=sho, knee_motor=kne)


# ---------------------------------------------------------------------------
# Precomputed validation poses (all in MOTOR FRAME for /joint_angles)
# ---------------------------------------------------------------------------

# Basic Cartesian sign sweep
POSE_REAR = make_pose("rear", X_REAR_M, Z_STAND_M)
POSE_MID = make_pose("mid", X_MID_M, Z_STAND_M)
POSE_FWD = make_pose("forward", X_FWD_M, Z_STAND_M)

# Single step sequence
POSE_STEP_SHIFT = make_pose("shift/rear_support", -0.025, Z_STAND_M)
POSE_STEP_LIFT = make_pose("lift", -X_STEP_M, Z_LIFT_M)
POSE_STEP_SWING = make_pose("swing_forward", +X_STEP_M, Z_LIFT_M)
POSE_STEP_PLANT = make_pose("plant_forward", +X_STEP_M, Z_STAND_M)
POSE_STEP_RECOVER = make_pose("recover_mid", 0.0, Z_STAND_M)


class IKValidationNode(Node):
    """
    Shared-IK validation node for a single leg.
    """

    def __init__(self) -> None:
        super().__init__("ik_validation_node")

        self.pub = self.create_publisher(Float32MultiArray, "/joint_angles", 10)
        self.state_pub = self.create_publisher(String, "/robot_state", 10)

        self.get_logger().info("IK validation node starting")
        self.get_logger().info(f"Test leg: {LEG_NAMES[TEST_LEG]}")
        self.get_logger().info("Using shared kinematics helpers from dog.kinematics")
        self.get_logger().info("Canonical convention: +X = forward")

        self._log_pose_diagnostics()

    # ------------------------------------------------------------------
    # Diagnostics / publishing helpers
    # ------------------------------------------------------------------

    def _log_pose_diagnostics(self) -> None:
        """
        Log shared-IK / FK diagnostics for all precomputed poses.
        """
        poses = [
            POSE_REAR,
            POSE_MID,
            POSE_FWD,
            POSE_STEP_SHIFT,
            POSE_STEP_LIFT,
            POSE_STEP_SWING,
            POSE_STEP_PLANT,
            POSE_STEP_RECOVER,
        ]

        self.get_logger().info("Precomputed motor-frame pose diagnostics:")
        for pose in poses:
            self.get_logger().info(
                describe_motor_pose(
                    pose.label,
                    pose.shoulder_motor,
                    pose.knee_motor,
                    TEST_LEG,
                )
            )

    def _publish_robot_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Published /robot_state = {state}")

    def _build_joint_vector(self, pose: LegPose) -> list[float]:
        """
        Build the full 8-value motor-frame command vector, keeping all non-test legs at neutral.
        """
        cmd = list(NEUTRAL_ANGLES)
        base = TEST_LEG * 2
        cmd[base] = pose.shoulder_motor
        cmd[base + 1] = pose.knee_motor
        return [float(x) for x in cmd]

    def _publish_pose(self, pose: LegPose, log: bool = False) -> None:
        msg = Float32MultiArray()
        msg.data = self._build_joint_vector(pose)
        self.pub.publish(msg)

        if log:
            self.get_logger().info(
                f"Pose '{pose.label}' | "
                + describe_motor_pose(
                    pose.label,
                    pose.shoulder_motor,
                    pose.knee_motor,
                    TEST_LEG,
                )
            )

    def _hold_pose(self, pose: LegPose, duration_s: float, rate_hz: float = 20.0) -> None:
        """
        Re-publish the given pose for a fixed duration so Gazebo has a stable command stream.
        Only logs once at the start of the hold.
        """
        self.get_logger().info(f"Holding '{pose.label}' for {duration_s:.1f}s")
        self._publish_pose(pose, log=False)

        end_time = self.get_clock().now().nanoseconds / 1e9 + duration_s
        period = 1.0 / rate_hz

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= end_time:
                break
            self._publish_pose(pose, log=False)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._sleep(period)

    def _sleep(self, seconds: float) -> None:
        """
        Sleep while still allowing ROS callbacks to process.
        """
        end_time = self.get_clock().now().nanoseconds / 1e9 + seconds
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= end_time:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, end_time - now))

    # ------------------------------------------------------------------
    # Validation sequences
    # ------------------------------------------------------------------

    def run(self) -> None:
        """
        Full validation sequence:
        1. command STANDING
        2. settle in neutral
        3. Cartesian X sign sweep
        4. single-leg step sequence
        5. return to neutral
        """
        self._publish_robot_state("STANDING")
        self.get_logger().info("Initial settle at neutral stance")
        self._hold_pose(POSE_MID, T_INITIAL_SETTLE)

        self.get_logger().info("Starting Cartesian X sign sweep")
        self._hold_pose(POSE_REAR, T_SWEEP_HOLD)
        self._hold_pose(POSE_MID, T_SWEEP_HOLD)
        self._hold_pose(POSE_FWD, T_SWEEP_HOLD)
        self._hold_pose(POSE_MID, T_SWEEP_HOLD)

        self.get_logger().info("Starting single-step validation sequence")
        self._hold_pose(POSE_STEP_SHIFT, T_STEP_SHIFT)
        self._hold_pose(POSE_STEP_LIFT, T_STEP_LIFT)
        self._hold_pose(POSE_STEP_SWING, T_STEP_SWING)
        self._hold_pose(POSE_STEP_PLANT, T_STEP_PLANT)
        self._hold_pose(POSE_STEP_RECOVER, T_STEP_RECOVER)

        self.get_logger().info("Returning to neutral")
        self._hold_pose(POSE_MID, T_POSE_HOLD)
        self.get_logger().info("Validation sequence complete")

    # ------------------------------------------------------------------
    # Static validation helpers
    # ------------------------------------------------------------------

    @staticmethod
    def validate_precomputed_poses() -> None:
        """
        Offline sanity check for shared IK/FK consistency.
        """
        poses = [
            POSE_REAR,
            POSE_MID,
            POSE_FWD,
            POSE_STEP_SHIFT,
            POSE_STEP_LIFT,
            POSE_STEP_SWING,
            POSE_STEP_PLANT,
            POSE_STEP_RECOVER,
        ]

        print("Shared-IK validation poses:")
        for pose in poses:
            print(
                describe_motor_pose(
                    pose.label,
                    pose.shoulder_motor,
                    pose.knee_motor,
                    TEST_LEG,
                )
            )


def main(args: list[str] | None = None) -> None:
    """
    Entry point for the shared-IK validation node.
    """
    try:
        IKValidationNode.validate_precomputed_poses()
    except IKError as exc:
        print(f"Precompute IK validation failed: {exc}")
        raise

    rclpy.init(args=args)
    node = IKValidationNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except IKError as exc:
        node.get_logger().error(f"IK validation failed: {exc}")
        raise
    finally:
        # Return to neutral once before shutdown
        try:
            node._publish_pose(POSE_MID)
            node._sleep(0.25)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
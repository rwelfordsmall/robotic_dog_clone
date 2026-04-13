#!/usr/bin/env python3
"""
step_test_node.py  ── Open-loop step validation for robotic_dog_clone (8-DOF)
Publishes a Float32MultiArray on /joint_angles with 8 values in the order:
    [FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne]

This node is INTENTIONALLY self-contained: it does not call gait_node.py,
gait_generator.py, or any higher-level stack.  Its sole job is to validate
a single clean step cycle in Gazebo so we can confirm:
  1. The belt-drive knee frame conversion is correct.
  2. IK-derived angles match the URDF geometry.
  3. The lateral-sequence crawl order is statically stable.
Once this passes visual inspection in Gazebo, the approach can be promoted
into gait_generator.py.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

# All geometry constants from the single source of truth.
from dog.robot_config import (
    UPPER_LENGTH,   # mm
    LOWER_LENGTH,   # mm
    STAND_HEIGHT,   # mm
    STEP_HEIGHT,    # mm
    STEP_LENGTH,    # mm
    JOINT_ANGLE_MIN,
    JOINT_ANGLE_MAX,
    NEUTRAL_ANGLES,  # see NOTE below about frame
)

# ── Unit conversion ───────────────────────────────────────────────────────────
# robot_config stores lengths in mm; all IK math here uses metres.
L_U = UPPER_LENGTH / 1000.0   # upper leg length, metres
L_L = LOWER_LENGTH / 1000.0   # lower leg length, metres
Z_STAND  = STAND_HEIGHT  / 1000.0   # nominal foot depth below shoulder pivot, m
Z_LIFT   = (STAND_HEIGHT - STEP_HEIGHT) / 1000.0  # depth when foot is raised, m
X_STEP   = (STEP_LENGTH / 2.0) / 1000.0            # half-step forward offset, m

# ── Leg index constants (position in the Float32MultiArray) ───────────────────
FR, FL, RR, RL = 0, 1, 2, 3   # logical leg indices used throughout

# ── Hold times (seconds) ─────────────────────────────────────────────────────
# Long phases allow frame-by-frame inspection in Gazebo and real-time verification
# of the robot's posture before the next command.  Do not shorten below 0.5s
# until the sequence is visually confirmed stable.
T_SETTLE   = 5.0   # initial stand-up hold — let Gazebo physics converge
T_SHIFT    = 1.5   # weight-shift hold before each lift
T_LIFT     = 1.0   # lift-apex hold
T_SWING    = 3.0   # swing / step hold
T_PLANT    = 1.5   # place and re-load hold


# ─────────────────────────────────────────────────────────────────────────────
#  IMPORTANT FRAME NOTE — READ BEFORE TOUCHING JOINT ANGLES
# ─────────────────────────────────────────────────────────────────────────────
# This robot has a belt-drive knee: the knee motor encodes the lower-leg angle
# in the BODY frame, not relative to the upper leg.
#   real-robot CAN bus command:  motor_knee = shoulder_ik + knee_ik_geometric
# In Gazebo / ign_ros2_control the knee joint is a standard URDF revolute joint:
#   Gazebo receives:  knee_urdf = angle RELATIVE TO upper_leg_link
#   body-frame lower-leg angle = shoulder + knee_urdf
#
# Therefore the conversion between the two frames is:
#   motor_knee (real robot) = shoulder + knee_ik_geo      (coupling IN)
#   knee_urdf  (Gazebo)     = knee_ik_geo = motor_knee - shoulder  (coupling OUT)
#
# robot_config.NEUTRAL_ANGLES stores motor-frame values (real robot convention).
# step_test_node must UNDO the coupling before publishing to Gazebo:
#   knee_cmd_to_Gazebo = NEUTRAL_ANGLES[knee] - NEUTRAL_ANGLES[shoulder]
#
# Concretely:  NEUTRAL_ANGLES = [0.60, -0.60, ...]
#   motor_knee = -0.60 rad  → real robot sees body-frame lower-leg = -0.60
#   knee_urdf  = motor_knee - shoulder = -0.60 - 0.60 = -1.20 rad (verified ✓)
#   FK at (shoulder=0.60, knee_urdf=-1.20): foot at x≈0, z≈396mm ≈ STAND_HEIGHT ✓
#
# If you publish -0.60 directly to Gazebo WITHOUT undoing the coupling,
# Gazebo computes: body-frame lower-leg = 0.60 + (-0.60) = 0.0  (straight down)
# → foot at z≈438mm, 43mm lower than intended, legs nearly straight → collapse.
# ─────────────────────────────────────────────────────────────────────────────


def _clamp(angle: float) -> float:
    """Clamp a joint angle to the hardware limits defined in robot_config.py."""
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, angle))


def _ik_motor_frame(x_fwd: float, z_down: float) -> tuple[float, float]:
    """
    Returns (shoulder_cmd, knee_motor_cmd) in MOTOR FRAME —
    i.e., what to publish to /joint_angles so sim_bridge_node
    correctly decouples it before forwarding to Gazebo.

    motor_knee = shoulder + knee_geometric  (belt-drive coupling)
    sim_bridge_node then does: geo_knee = motor_knee - shoulder
    which recovers knee_geometric for the URDF controller.
    """
    d_sq = x_fwd**2 + z_down**2
    d = math.sqrt(d_sq)
    cos_phi = (d_sq - L_U**2 - L_L**2) / (2.0 * L_U * L_L)
    cos_phi = max(-1.0, min(1.0, cos_phi))
    phi_mag = math.acos(cos_phi)
    knee_geo = -phi_mag                          # geometric knee (URDF-relative)

    gamma = math.atan2(x_fwd, z_down)
    sin_psi = L_L * math.sin(phi_mag) / d
    psi = math.asin(max(-1.0, min(1.0, sin_psi)))
    shoulder = gamma + psi

    motor_knee = shoulder + knee_geo             # apply belt-drive coupling
    return shoulder, motor_knee

# Cartesian diagnostic points for one-leg X sweep
X_REAR = -0.050   # -50 mm
X_MID  =  0.000   #   0 mm
X_FWD  =  0.050   # +50 mm

_SHO_REAR, _KNE_REAR = _ik_motor_frame(X_REAR, Z_STAND)
_SHO_MID,  _KNE_MID  = _ik_motor_frame(X_MID,  Z_STAND)
_SHO_FWD,  _KNE_FWD  = _ik_motor_frame(X_FWD,  Z_STAND)

def _fr_only_pose(fr_sho: float, fr_kne: float) -> list[float]:
    # Keep all other legs at neutral
    return [
        _clamp(fr_sho), _clamp(fr_kne),             # FR
        NEUTRAL_ANGLES[2], NEUTRAL_ANGLES[3],       # FL
        NEUTRAL_ANGLES[4], NEUTRAL_ANGLES[5],       # RR
        NEUTRAL_ANGLES[6], NEUTRAL_ANGLES[7],       # RL
    ]

def _make_full_pose(
    sho: tuple[float, float, float, float],
    kne: tuple[float, float, float, float],
) -> list[float]:
    """
    Assemble an 8-element list matching the Float32MultiArray contract:
        [FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne]

    Clamps all values to [JOINT_ANGLE_MIN, JOINT_ANGLE_MAX].
    """
    return [
        _clamp(sho[FR]), _clamp(kne[FR]),
        _clamp(sho[FL]), _clamp(kne[FL]),
        _clamp(sho[RR]), _clamp(kne[RR]),
        _clamp(sho[RL]), _clamp(kne[RL]),
    ]


# ── Pre-compute canonical poses once at import time ───────────────────────────
# All values are in Gazebo / URDF-relative frame.  Each is verified by FK.

# Neutral standing pose: foot directly below shoulder at STAND_HEIGHT depth.
_SHO_N, _KNE_N = _ik_motor_frame(0.0, Z_STAND)

# Lift apex: foot raised STEP_HEIGHT above the neutral ground plane.
# Z_LIFT < Z_STAND means the foot is physically higher (closer to the body).
_SHO_L, _KNE_L = _ik_motor_frame(0.0, Z_LIFT)

# Forward placement: foot is X_STEP = STEP_LENGTH/2 forward of neutral.
_SHO_FWD, _KNE_FWD = _ik_motor_frame(-X_STEP, Z_STAND)

# Precompute the missing pose: foot forward AND high
_SHO_FWD_HIGH, _KNE_FWD_HIGH = _ik_motor_frame(-X_STEP, Z_LIFT)

# Rearward position (used when a support leg must push back to drive body forward).
_SHO_BCK, _KNE_BCK = _ik_motor_frame(-X_STEP, Z_STAND)

# Weight-shift pose for support legs.
# Without hip abduction, lateral CoM shift is impossible.  We instead shift
# CoM FORWARD by tilting the support legs backward in the shoulder frame,
# which is kinematically equivalent to the body leaning forward by COM_SHIFT_RAD.
# Before lifting a swing leg, the three support legs adopt this pose so the
# CoM projection moves toward the interior of the three-leg support triangle.
# COM_SHIFT_RAD = 0.05 rad → CoM shifts ~20mm forward (= STAND_HEIGHT * sin(0.05)).
# This is the maximum sagittal CoM shift available without hip abduction.
COM_SHIFT_RAD = 0.15  # rad; increase cautiously — too large risks a forward tip
# COM_SHIFT_RAD_STRONG = 0.35  # rad; increase cautiously — too large risks a forward tip
_X_WS = -Z_STAND * math.sin(COM_SHIFT_RAD)   # foot shifts backward in shoulder frame
_SHO_WS, _KNE_WS = _ik_motor_frame(_X_WS, Z_STAND)
# _X_WS_STRONG = -Z_STAND * math.sin(COM_SHIFT_RAD_STRONG)   # foot shifts backward in shoulder frame
# _SHO_WS_STRONG, _KNE_WS_STRONG = _ik_motor_frame(_X_WS_STRONG, Z_STAND)


class StepTestNode(Node):
    """Open-loop step-test node for robotic_dog_clone (8-DOF, no hip abduction)."""

    def __init__(self):
        super().__init__("step_test_node")
        self._pub = self.create_publisher(Float32MultiArray, "/joint_angles", 10)
        # Brief delay before sending the first command gives Gazebo time to
        # register the joint_group_position_controller subscriber.
        self.create_timer(0.5, self._start_sequence_once)
        self._started = False
        self.get_logger().info("StepTestNode ready.  Sequence begins in 0.5s.")

    def _start_sequence_once(self):
        if self._started:
            return
        self._started = True
        self.run_sequence()

    # ── Low-level helpers ─────────────────────────────────────────────────────

    def _publish(self, angles: list[float]):
        msg = Float32MultiArray()
        msg.data = angles
        self._pub.publish(msg)

    def _hold(self, angles: list[float], duration: float, label: str):
        """
        Publish `angles` repeatedly at 10 Hz for `duration` seconds.
        Continuous publishing is required: ign_ros2_control will hold the last
        received command, but re-publishing prevents any stale-command timeouts
        and keeps the log timestamps useful for post-analysis.
        """
        self.get_logger().info(f"  [{label}]  {duration:.1f}s")
        rate_hz = 10.0
        steps = max(1, int(duration * rate_hz))
        dt = duration / steps
        self.get_logger().info(
            f"    cmd: sho={angles[0]:.3f} kne={angles[1]:.3f} "
            f"(all legs symmetric)"
        )
        for _ in range(steps):
            self._publish(angles)
            rclpy.spin_once(self, timeout_sec=dt)

    # ── Pose builders ─────────────────────────────────────────────────────────

    def _pose_all_neutral(self) -> list[float]:
        """All four legs at IK-derived neutral standing posture."""
        return _make_full_pose(
            (_SHO_N, _SHO_N, _SHO_N, _SHO_N),
            (_KNE_N, _KNE_N, _KNE_N, _KNE_N),
        )

    def _pose_with_leg(
        self,
        swing: int,
        sho_sw: float,
        kne_sw: float,
        support_override: dict[int, tuple[float, float]] | None = None,
    ) -> list[float]:
        """
        Build a full 8-DOF pose with one custom leg and optional support overrides.
        All legs not listed in support_override default to neutral.
        """
        sho = [_SHO_N] * 4
        kne = [_KNE_N] * 4
        sho[swing] = sho_sw
        kne[swing] = kne_sw
        if support_override:
            for idx, (s, k) in support_override.items():
                sho[idx] = s
                kne[idx] = k
        return _make_full_pose(tuple(sho), tuple(kne))

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run_sequence(self):
        """
        One complete lateral-sequence crawl cycle (4 steps).

        Confirmed parameters from validation:
        - COM_SHIFT_RAD = 0.15  (0.05 was insufficient, 0.15 confirmed stable in Test 3)
        - Negative X = forward  (confirmed in Test 4)
        - Via-point swing: forward-at-height then descend (prevents ground drag)
        - Support legs held in weight-shift pose through entire swing phase
        - Support legs restored to neutral only after FR is planted
        - Closed-loop controller (p=200, d=5) confirmed reaching targets in Test 4
        """

        # Phase 0: settle — let physics converge from xacro initial pose
        self.get_logger().info("Phase 0: settle")
        self._hold(self._pose_all_neutral(), T_SETTLE, "stand-settle")

        # Lateral crawl: FR → RR → FL → RL
        # This sequence maximises the support triangle area at each swing phase.
        crawl_order = [FR, RR, FL, RL]
        names = {FR: "FR", FL: "FL", RR: "RR", RL: "RL"}

        for step_i, swing in enumerate(crawl_order):
            supports = [l for l in crawl_order if l != swing]
            self.get_logger().info(
                f"── Step {step_i+1}/4: swing {names[swing]}, "
                f"support {[names[l] for l in supports]} ──"
            )
            sup_ws = {l: (_SHO_WS, _KNE_WS) for l in supports}

            # Phase A: weight shift
            # Support legs lean backward (COM_SHIFT_RAD=0.15, ~59mm forward shift).
            # Swing leg stays at neutral — still bearing load during shift.
            self._hold(
                self._pose_with_leg(swing, _SHO_N, _KNE_N, sup_ws),
                T_SHIFT, f"{names[swing]} weight-shift",
            )

            # Phase B: lift
            # Raise swing leg STEP_HEIGHT above ground.
            # Support legs remain in weight-shift pose throughout swing.
            self._hold(
                self._pose_with_leg(swing, _SHO_L, _KNE_L, sup_ws),
                T_LIFT, f"{names[swing]} lift",
            )

            # Phase C1: swing forward at lift height
            # Move foot forward (-X) while still high, keeping clearance.
            # Doing this as a separate phase prevents the foot dragging
            # along the ground mid-arc if we went directly to planted position.
            self._hold(
                self._pose_with_leg(swing, _SHO_FWD_HIGH, _KNE_FWD_HIGH, sup_ws),
                T_SWING, f"{names[swing]} swing-fwd-high",
            )

            # Phase C2: descend to plant
            # Lower foot to ground at forward position.
            self._hold(
                self._pose_with_leg(swing, _SHO_FWD, _KNE_FWD, sup_ws),
                T_SWING, f"{names[swing]} swing-descend",
            )

            # Phase D: plant and restore support legs to neutral
            # Swing leg stays at forward position.
            # Support legs return to neutral — body stabilises.
            self._hold(
                self._pose_with_leg(swing, _SHO_FWD, _KNE_FWD),
                T_PLANT, f"{names[swing]} plant+restore",
            )

        self.get_logger().info("Sequence complete — holding final pose for inspection.")
        final = self._pose_all_neutral()
        while rclpy.ok():
            self._publish(final)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = StepTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""
robot_config.py
---------------

Central configuration for the active quadruped control stack.

This file separates:
- physical leg geometry assumptions
- active gait tuning parameters
- neutral stance definitions
- motor/geometric conversion conventions
- joint limits, CAN protocol limits, and controller values

Important conventions
---------------------
The active robot path is currently modeled as an 8-DOF sagittal-leg system:
- 4 shoulder joints
- 4 knee joints
- no active hip ab/adduction in the current end-to-end gait/IK path

Joint spaces used in this repo:
1. Geometric / URDF space
   - used by IK/FK and Gazebo joint controllers

2. Motor command space
   - published on /joint_angles
   - includes belt-drive knee coupling

Knee coupling rule in motor space:
    shoulder_motor = shoulder_geometric
    knee_motor     = shoulder_geometric + knee_geometric

Therefore:
    knee_geometric = knee_motor - shoulder_motor

All joint command constants in this file are in radians unless otherwise noted.
The IK solver may work internally in degrees, but outputs are converted to radians.
"""

# ---------------------------------------------------------------------------
# 1. Physical leg geometry (mm)
# ---------------------------------------------------------------------------

# NOTE:
# The URDF/xacro remains the authoritative source for simulated joint layout
# and link origins. These values are the simplified dimensions used by the
# active sagittal control model and should be kept aligned with Gazebo behavior.

# Active kinematic model: sagittal 2-link leg for the current 8DOF robot path.
# The original software stack assumed an additional hip ab/adduction DOF, but
# the active gait/IK pipeline does not command that DOF.
HIP_LENGTH = 0.0

# Shared IK/FK link lengths.
# Keep these aligned with the effective URDF leg geometry as closely as possible.
UPPER_LENGTH = 218.25
LOWER_LENGTH = 240.0

# Body dimensions between shoulder pivots, used by higher-level pose/gait logic.
BODY_LENGTH = 648.0   # shoulder pivot x: +0.324 to -0.324 m
BODY_WIDTH  = 233.0   # shoulder pivot y: +0.1165 to -0.1165 m

# ---------------------------------------------------------------------------
# 2. Active gait tuning
# ---------------------------------------------------------------------------

# Neutral downward-positive foot depth used by the active sagittal gait path.
# This should remain consistent with the chosen neutral stance.
STAND_HEIGHT = 396.2

# Swing-foot lift height used by the gait generator, in mm.
# After Patch 2, this value is intended to match the actual generated swing apex.
STEP_HEIGHT = 70.0

# Full fore-aft swing distance used by the gait generator, in mm.
# After Patch 2, this value is intended to match the actual generated sagittal sweep.
STEP_LENGTH = 200.0

# Nominal step duration used by gait/state logic.
STEP_DURATION = 0.45

# ---------------------------------------------------------------------------
# 3. Body pose limits (degrees)
# Used inside gait/state nodes
# ---------------------------------------------------------------------------

MAX_BODY_ROLL  = 15.0
MAX_BODY_PITCH = 15.0
MAX_BODY_YAW   = 20.0

# ---------------------------------------------------------------------------
# 4. Neutral stance definition
# ---------------------------------------------------------------------------

# Canonical neutral Cartesian target for the active sagittal model:
# x = 0 mm, z = STAND_HEIGHT mm (downward-positive)
NEUTRAL_CARTESIAN = (0.0, STAND_HEIGHT)

# NEUTRAL_ANGLES are MOTOR-SPACE commands published on /joint_angles.
# Order:
#   [FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne]
#
# Coupling reminder:
#   shoulder_motor = shoulder_geometric
#   knee_motor     = shoulder_geometric + knee_geometric
#
# Therefore, for Gazebo / URDF geometric space:
#   knee_geometric = motor_knee - motor_shoulder
#
# Example:
#   shoulder_motor =  0.60 rad
#   knee_motor     = -0.60 rad
#   knee_geometric = -0.60 - 0.60 = -1.20 rad
#
# Do NOT publish NEUTRAL_ANGLES directly to the Gazebo controller without
# first decoupling the knee.
NEUTRAL_ANGLES = [
     0.60, -0.60,   # FR
     0.60, -0.60,   # FL
     0.60, -0.60,   # RR
     0.60, -0.60,   # RL
]

SIT_ANGLES = [
     1.1346, -1.1346,   # FR
     1.1346, -1.1346,   # FL
     1.1346, -1.1346,   # RR
     1.1346, -1.1346,   # RL
]

# Deep sit: shoulder tilts backward, knee bends moderately.
# Body rests low; tune shoulder/knee magnitudes to match physical geometry.
#   shoulder < 0  -> upper leg tilts rearward
#   knee_motor    -> shoulder_geometric + knee_geometric
DEEP_SIT_ANGLES = [
    -0.6, -1.0,   # FR
    -0.6, -1.0,   # FL
    -0.6, -1.0,   # RR
    -0.6, -1.0,   # RL
]

# ---------------------------------------------------------------------------
# 5. Motor/geometric joint conversion
# ---------------------------------------------------------------------------

# Tuple key format:
#   (leg_index, joint_index)
# where:
#   leg_index:  0=FR, 1=FL, 2=RR, 3=RL
#   joint_index: 1=shoulder, 2=knee

# JOINT_DIRECTION maps geometric / IK angles into motor command direction.
#
# Verification convention:
#   Shoulder: positive command should move the leg in the positive shoulder
#             direction defined by the active geometric model.
#   Knee:     verify only after shoulder direction is confirmed, since the
#             knee motor is belt-coupled.
JOINT_DIRECTION = {
    (0, 1):  1, (0, 2):  1,   # FR shoulder, knee
    (1, 1):  1, (1, 2):  1,   # FL shoulder, knee
    (2, 1):  1, (2, 2):  1,   # RR shoulder, knee
    (3, 1):  1, (3, 2):  1,   # RL shoulder, knee
}

# JOINT_OFFSETS are fine trim offsets applied after motor zeroing.
# Formula:
#   motor_cmd_rad = direction * geometric_angle_rad + radians(offset_deg)
JOINT_OFFSETS = {
    (0, 1): 0.0, (0, 2): 0.0,   # FR shoulder, knee
    (1, 1): 0.0, (1, 2): 0.0,   # FL shoulder, knee
    (2, 1): 0.0, (2, 2): 0.0,   # RR shoulder, knee
    (3, 1): 0.0, (3, 2): 0.0,   # RL shoulder, knee
}

# ---------------------------------------------------------------------------
# 6. CAN motor configuration
# ---------------------------------------------------------------------------

# CAN bus -> Teensy 4.1 pins:
#   CAN1: TX=22, RX=23  -> front legs (FR + FL)
#   CAN3: TX=31, RX=30  -> rear legs  (RR + RL)
#
# Each motor needs a unique CAN ID (1-127).
# Set IDs using the CubeMars R-Link USB adapter + CubeMars debugging software.
#
# CAN_ID_MAP[(leg_index, joint_index)] = (can_bus, motor_id)
#   leg_index:  0=FR, 1=FL, 2=RR, 3=RL
#   joint_index: 1=shoulder, 2=knee
CAN_ID_MAP = {
    # Front Right — CAN1
    (0, 1): (1, 0x79),  # FR Shoulder (121)
    (0, 2): (1, 0x7A),  # FR Knee     (122)

    # Front Left — CAN1
    (1, 1): (1, 0x75),  # FL Shoulder (117)
    (1, 2): (1, 0x78),  # FL Knee     (120)

    # Rear Right — CAN3
    (2, 1): (3, 0x73),  # RR Shoulder (115)
    (2, 2): (3, 0x74),  # RR Knee     (116)

    # Rear Left — CAN3
    (3, 1): (3, 0x76),  # RL Shoulder (118)
    (3, 2): (3, 0x77),  # RL Knee     (119)
}

# Flat list: index = leg*2 + (joint-1) -> motor_id
# Order matches the 8-value /joint_angles vector:
#   FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne
MOTOR_IDS = [
    CAN_ID_MAP[(leg, joint)][1]
    for leg in range(4)
    for joint in range(1, 3)
]

# ---------------------------------------------------------------------------
# 7. MIT Mini Cheetah CAN protocol limits
# AK45-36 operating ranges
# ---------------------------------------------------------------------------

CAN_POS_MIN = -12.5    # rad
CAN_POS_MAX =  12.5    # rad
CAN_VEL_MIN = -45.0    # rad/s
CAN_VEL_MAX =  45.0    # rad/s
CAN_KP_MIN  =   0.0    # N·m/rad
CAN_KP_MAX  = 500.0    # N·m/rad
CAN_KD_MIN  =   0.0    # N·m·s/rad
CAN_KD_MAX  =   5.0    # N·m·s/rad
CAN_TRQ_MIN = -18.0    # N·m
CAN_TRQ_MAX =  18.0    # N·m

# Default impedance gains for position control
CAN_KP_DEFAULT = 150.0
CAN_KD_DEFAULT =   2.0

# ---------------------------------------------------------------------------
# 8. Joint limits / controller values
# ---------------------------------------------------------------------------

# Soft limits applied before sending CAN commands.
# Tune to match physical joint stops.
JOINT_ANGLE_MIN = -2.618   # -150 deg
JOINT_ANGLE_MAX =  2.618   #  150 deg

# ---------------------------------------------------------------------------
# 9. Self-righting poses (motor frame, same convention as NEUTRAL_ANGLES)
# ---------------------------------------------------------------------------

# RIGHTING_TUCK_ANGLES:
# all legs folded close to the body to reduce rotational inertia and avoid
# ground interference at the start/end of the righting sequence.
RIGHTING_TUCK_ANGLES = [
     1.30, -1.1346,   # FR
     1.30, -1.1346,   # FL
     1.30, -1.1346,   # RR
     1.30, -1.1346,   # RL
]

# ---------------------------------------------------------------------------
# 10. IMU / GPS
# ---------------------------------------------------------------------------

# BNO085 9-DOF IMU (connected directly to Jetson I2C)
BNO085_ADDRESS   = 0x4A
IMU_PUBLISH_RATE = 50    # Hz

# u-blox SAM-M10Q GPS (connected to Jetson UART)
GPS_PORT      = '/dev/ttyUSB0'
GPS_BAUD_RATE = 9600

# ---------------------------------------------------------------------------
# 11. Controller axes / buttons
# ---------------------------------------------------------------------------

# Controller axes (identical for Xbox and PS4 for the common mappings below)
AXIS_LEFT_X  = 0   # Strafe left/right
AXIS_LEFT_Y  = 1   # Forward / backward

# PS4 (ds4drv/hid-sony): axes 2=RX, 3=RY, 4=L2, 5=R2
# Xbox / PS4 (xpadneo):  axes 2=L2, 3=RX, 4=RY, 5=R2
# Verify with: ros2 topic echo /joy
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_LT      = 2
AXIS_RT      = 5

# Xbox controller buttons
BTN_A     = 0   # Stand / sit toggle
BTN_B     = 1   # Reserved
BTN_X     = 2   # Change gait
BTN_Y     = 3   # Reserved
BTN_LB    = 4   # Deadman enable
BTN_RB    = 5   # Turbo speed
BTN_BACK  = 6   # E-stop
BTN_START = 7   # Reset E-stop

# PS4 DualShock 4 button layout
PS4_BTN_SQUARE = 0   # Stand / sit toggle
PS4_BTN_CROSS  = 1   # Reserved
PS4_BTN_CIRCLE = 2   # Change gait
PS4_BTN_TRI    = 3   # Reserved
PS4_BTN_L1     = 4   # Deadman enable
PS4_BTN_R1     = 5   # Turbo speed
PS4_BTN_BACK   = 8   # E-stop
PS4_BTN_START  = 9   # Reset E-stop

DEADMAN_BUTTON    = BTN_LB
JOYSTICK_SCALE    = 0.7
TURN_SCALE        = 1.5
TURBO_MULTIPLIER  = 2.0
JOYSTICK_DEADZONE = 0.1

# ---------------------------------------------------------------------------
# 12. Jump keyframe angles (motor frame)
# ---------------------------------------------------------------------------

# JUMP_CROUCH_ANGLES:
# Front legs crouch deeper than rear to bias the body nose-down before launch.
JUMP_CROUCH_ANGLES = [
     1.30, -1.30,   # FR
     1.30, -1.30,   # FL
     1.00, -1.00,   # RR
     1.00, -1.00,   # RL
]

# JUMP_LAUNCH_ANGLES:
# Straight-leg launch in motor frame.
# Since geometric_knee = motor_knee - motor_shoulder, matching shoulder and knee
# yields geometric_knee ~= 0.0 (straight leg).
JUMP_LAUNCH_ANGLES = [
     0.80,  0.80,   # FR
     0.80,  0.80,   # FL
     0.80,  0.80,   # RR
     0.80,  0.80,   # RL
]

JUMP_TUCK_ANGLES = [
     1.25, -1.25,   # FR
     1.25, -1.25,   # FL
     1.25, -1.25,   # RR
     1.25, -1.25,   # RL
]

JUMP_LAND_ANGLES = [
     1.20, -1.20,   # FR
     1.20, -1.20,   # FL
     0.50, -0.50,   # RR
     0.50, -0.50,   # RL
]

# ---------------------------------------------------------------------------
# 13. Backflip keyframe angles (motor frame)
# ---------------------------------------------------------------------------

BACKFLIP_CROUCH_ANGLES = [
     1.40, -1.40,   # FR
     1.40, -1.40,   # FL
     1.40, -1.40,   # RR
     1.40, -1.40,   # RL
]

# Phase 1: front lifts while rear remains planted to create a pivot.
BACKFLIP_FRONT_LIFT_ANGLES = [
     0.80,  0.80,   # FR
     0.80,  0.80,   # FL
     1.40, -1.40,   # RR
     1.40, -1.40,   # RL
]

# Phase 2: rear launches while front tucks.
BACKFLIP_LAUNCH_ANGLES = [
     1.50, -1.50,   # FR
     1.50, -1.50,   # FL
     0.80,  0.80,   # RR
     0.80,  0.80,   # RL
]

BACKFLIP_TUCK_ANGLES = [
     1.50, -1.50,   # FR
     1.50, -1.50,   # FL
     1.50, -1.50,   # RR
     1.50, -1.50,   # RL
]

BACKFLIP_LAND_ANGLES = [
     0.70, -0.70,   # FR
     0.70, -0.70,   # FL
     0.70, -0.70,   # RR
     0.70, -0.70,   # RL
]
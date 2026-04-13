# Kinematics and Command Conventions

This document defines the canonical coordinate, joint, and command conventions for the quadruped software stack.

These conventions are the source of truth for:
- gait generation
- inverse kinematics
- validation/test nodes
- sim bridge conversion
- Gazebo controller commands

---

## 1. Robot body / leg target frame

For the active 8-DOF robot configuration, the software uses the following body-relative leg target convention:

- **+X = forward**
- **+Y = left**
- **+Z = up**

When specifying a desired foot target relative to a leg/shoulder frame:

- `foot_x > 0` means the foot is placed **forward**
- `foot_x < 0` means the foot is placed **rearward**
- `foot_z > 0` means the foot is **lower** than the shoulder pivot in the current sagittal-leg model
- `foot_y` is not actively used in the current 8-DOF sagittal gait path

Notes:
- The active gait/IK path currently supports sagittal `(x, z)` motion only.
- Lateral `y` foot placement is not part of the current end-to-end 8-DOF control path.

---

## 2. Joint-space conventions

Two different joint spaces exist in this repo and must not be confused.

### A. Geometric / URDF joint space
This is the joint-angle space used by:
- the URDF
- Gazebo
- ros2_control joint position controller

In this document, this is called **geometric space** or **URDF space**.

### B. Motor command space
This is the command space published on `/joint_angles`.

In this document, this is called **motor space**.

---

## 3. Knee coupling convention

The robot uses a coupled knee command convention in motor space.

For each leg:

- `shoulder_motor = shoulder_geometric`
- `knee_motor = shoulder_geometric + knee_geometric`

Therefore, converting from motor space back to URDF geometric space is:

- `shoulder_geometric = shoulder_motor`
- `knee_geometric = knee_motor - shoulder_motor`

This means:

- `/joint_angles` publishes **motor-space** commands
- Gazebo controller commands must be **geometric-space** joint angles

---

## 4. Topic conventions

### `/joint_angles`
- Message type: `Float64MultiArray`
- Contents: 8 motor-space joint commands
- Order:
  - front-right shoulder
  - front-right knee
  - front-left shoulder
  - front-left knee
  - rear-right shoulder
  - rear-right knee
  - rear-left shoulder
  - rear-left knee

### `/joint_group_position_controller/commands`
- Message type: `Float64MultiArray`
- Contents: 8 geometric / URDF joint commands
- Same joint ordering as above

---

## 5. Active robot model

The active robot configuration is treated as an **8-DOF sagittal-leg robot**:

- 4 shoulder joints
- 4 knee joints
- no active hip ab/adduction in the current gait/IK path

Implications:
- gait generation should not assume lateral foot motion is preserved end-to-end
- IK validation should use the same sagittal model as the active control stack
- test nodes must not invent a separate forward/sign convention

---

## 6. Forward-motion rule

The canonical rule for this repo is:

> A positive commanded foot X target means forward.

If Gazebo motion does not obey that rule, the shared IK/conversion code must be fixed.
Individual test nodes must not silently compensate by flipping the X sign locally.

---

## 7. Neutral stance rule

Neutral stance must be defined consistently across:
- robot config
- IK/FK utilities
- test nodes
- URDF startup joint values

The repo should not maintain multiple independent definitions of neutral stance.

---

## 8. Practical debugging rule

When debugging leg motion, always state which space is being discussed:

- Cartesian target space `(x, z)`
- geometric / URDF joint space
- motor joint command space

Never use “joint angle” without stating which of the above is meant.
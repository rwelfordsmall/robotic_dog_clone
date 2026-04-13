"""
gait_node.py
------------

ROS2 node that runs the active gait generator and inverse kinematics.

Published output:
- /joint_angles: 8 motor-space position commands in radians
  Order: FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne

Active robot mode:
- 8DOF sagittal gait path
- forward/reverse stepping supported
- lateral foot placement is not part of the active end-to-end path
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Vector3

from dog.gait_generator import GaitGenerator, GaitType
from dog.kinematics import compute_all_legs
from dog.state_manager import RobotState
from dog.robot_config import NEUTRAL_ANGLES

KINEMATIC_MODE = "8dof_sagittal"

CONTROL_RATE_HZ = 50

# PD gains per gait.  Softer gains during dynamic gaits reduce impact loads on
# the PLA frame and let the 36:1 motors swing freely without fighting themselves.
# Stiffer gains during stand/turtle give solid position hold.
_GAIT_GAINS = {
    GaitType.STAND:  (45.0, 2.0),
    GaitType.TURTLE: (40.0, 2.0),
    GaitType.CRAWL:  (35.0, 1.5),
    GaitType.WALK:   (30.0, 1.3),
    GaitType.TROT:   (22.0, 1.2),
    GaitType.GALLOP: (18.0, 1.0),
}


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')

        self.gait       = GaitGenerator()
        self.vx         = 0.0
        self.vy         = 0.0
        self.yaw        = 0.0
        self.roll       = 0.0
        self.pitch      = 0.0
        self.robot_state     = RobotState.SITTING
        self.walk_gait_type  = GaitType.WALK

        self.create_subscription(Twist,  'gait_command', self._cmd_callback,       10)
        self.create_subscription(Vector3,'body_pose',    self._pose_callback,      10)
        self.create_subscription(String, 'robot_state',  self._state_callback,     10)
        self.create_subscription(String, 'gait_type',    self._gait_type_callback, 10)

        self.joint_pub = self.create_publisher(
            Float32MultiArray, 'joint_angles', 10
        )
        self.gains_pub = self.create_publisher(
            Float32MultiArray, 'joint_gains', 10
        )

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)

        self.get_logger().info(f'Gait node running at {CONTROL_RATE_HZ} Hz')
        self.get_logger().info(f'Kinematic mode: {KINEMATIC_MODE}')
        self._publish_gains(GaitType.STAND)

    # ────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Twist):
        # Active 8DOF path uses vx only; vy/yaw are currently accepted for interface
        # compatibility but not preserved as lateral foot placement.
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.yaw = msg.angular.z

    def _pose_callback(self, msg: Vector3):
        # Active path supports pitch-oriented sagittal adjustment.
        # Roll is currently not preserved through lateral foot placement.
        self.roll = msg.x
        self.pitch = msg.y

    def _gait_type_callback(self, msg: String):
        try:
            gt = GaitType[msg.data]
        except KeyError:
            self.get_logger().warn(f'Unknown gait type: {msg.data}')
            return
        self.walk_gait_type = gt
        if self.robot_state in (RobotState.WALKING, RobotState.AUTONOMOUS):
            self.gait.set_gait(gt)
            self._publish_gains(gt)
            self.get_logger().info(f'Switched to {gt.name} gait')

    def _state_callback(self, msg: String):
        new_state = msg.data
        if new_state == self.robot_state:
            return
        self.robot_state = new_state

        if new_state == RobotState.STANDING:
            self.gait.set_gait(GaitType.STAND)
            self._publish_gains(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0
        elif new_state in (RobotState.WALKING, RobotState.AUTONOMOUS):
            self.gait.set_gait(self.walk_gait_type)
            self._publish_gains(self.walk_gait_type)
        elif new_state in (RobotState.SITTING, RobotState.ESTOP,
                           RobotState.IDLE, RobotState.RIGHTING,
                           RobotState.JUMPING, RobotState.BACKFLIP):
            self.gait.set_gait(GaitType.STAND)
            self._publish_gains(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0

    def _publish_gains(self, gait_type: GaitType):
        kp, kd = _GAIT_GAINS.get(gait_type, (35.0, 1.5))
        msg = Float32MultiArray()
        msg.data = [kp, kd]
        self.gains_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────
    def _control_loop(self):
        if self.robot_state in (
            RobotState.POSITIONING,
            RobotState.SITTING,
            RobotState.ESTOP,
            RobotState.IDLE,
            RobotState.RIGHTING,
            RobotState.JUMPING,
            RobotState.BACKFLIP,
        ):
            return

        if self.robot_state == RobotState.STANDING:
            no_move = (
                abs(self.vx) < 0.001 and 
                abs(self.vy) < 0.001 and 
                abs(self.yaw) < 0.001
                )
            no_tilt = abs(self.roll) < 0.5 and abs(self.pitch) < 0.5
            if no_move and no_tilt:
                return

        self._warn_if_unsupported_motion_requested()

        if KINEMATIC_MODE == "8dof_sagittal":
            foot_positions = self.gait.update_sagittal_8dof(
                vx=self.vx,
                body_pitch=self.pitch,
            )
        else:
            foot_positions = self.gait.update(
                vx=self.vx,
                vy=self.vy,
                yaw=self.yaw,
                body_roll=self.roll,
                body_pitch=self.pitch,
            )

        try:
            angles = compute_all_legs(foot_positions)
        except Exception as e:
            self.get_logger().error(
                f"IK failed: {e}; requested foot positions={foot_positions}"
            )
            angles = list(NEUTRAL_ANGLES)

        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.joint_pub.publish(msg)

    def _warn_if_unsupported_motion_requested(self):
        if KINEMATIC_MODE != "8dof_sagittal":
            return

        unsupported = []
        if abs(self.vy) > 1e-3:
            unsupported.append(f"vy={self.vy:.3f}")
        if abs(self.yaw) > 1e-3:
            unsupported.append(f"yaw={self.yaw:.3f}")
        if abs(self.roll) > 0.5:
            unsupported.append(f"roll={self.roll:.2f}")

        if unsupported:
            self.get_logger().warn(
                "Ignoring unsupported lateral/roll command inputs in 8DOF sagittal mode: "
                + ", ".join(unsupported),
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

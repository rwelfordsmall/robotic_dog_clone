"""
step_test_node.py
-----------------
Very simple open-loop stepping validator for the migrated 8-DOF robot.

Purpose:
  - Validate that the robot can execute clear lift / swing / place motions
    one leg at a time from a stable neutral pose.
  - Isolate URDF + controller + joint convention issues from the full gait stack.

Published topics:
  /joint_angles  (std_msgs/Float32MultiArray)  — 8 motor commands in radians
  /can_enable    (std_msgs/Bool)               — enable motor mode

Joint order:
  [FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne]
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from dog.robot_config import NEUTRAL_ANGLES, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX


def clamp(v: float) -> float:
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, v))


class StepTestNode(Node):
    def __init__(self):
        super().__init__('step_test_node')
        self.joint_pub = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self.enable_pub = self.create_publisher(Bool, 'can_enable', 10)
        self.get_logger().info('step_test_node ready')

    def enable(self, value: bool):
        msg = Bool()
        msg.data = value
        self.enable_pub.publish(msg)
        time.sleep(0.1 if value else 0.05)

    def send(self, angles, hold_s: float = 0.6):
        msg = Float32MultiArray()
        msg.data = [float(clamp(a)) for a in angles]
        self.joint_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(hold_s)

    def leg_pose(self, base, leg: int, shoulder: float, knee: float):
        out = list(base)
        i = leg * 2
        out[i] = clamp(shoulder)
        out[i + 1] = clamp(knee)
        return out


def run_sequence(node: StepTestNode):
    neutral = list(NEUTRAL_ANGLES)
    base = list(neutral)

    # Slow crawl order
    leg_order = [0, 3, 1, 2]
    names = ['FR', 'FL', 'RR', 'RL']

    opposite_side_support = {
        0: [1, 3],  # lift FR -> bias FL, RL
        1: [0, 2],  # lift FL -> bias FR, RR
        2: [1, 3],  # lift RR -> bias FL, RL
        3: [0, 2],  # lift RL -> bias FR, RR
    }

    def set_leg(pose_base, leg, shoulder, knee):
        out = list(pose_base)
        i = leg * 2
        out[i] = clamp(shoulder)
        out[i + 1] = clamp(knee)
        return out

    def set_many(pose_base, legs, shoulder, knee):
        out = list(pose_base)
        for leg in legs:
            i = leg * 2
            out[i] = clamp(shoulder)
            out[i + 1] = clamp(knee)
        return out

    node.get_logger().info('Holding neutral before forward lean-step test')
    node.send(base, hold_s=1.0)

    for cycle in range(2):
        node.get_logger().info(f'Forward lean-step cycle {cycle + 1}/2')

        for leg in leg_order:
            node.get_logger().info(f'Stepping {names[leg]}')
            support_legs = opposite_side_support[leg]
            other_legs = [i for i in range(4) if i != leg and i not in support_legs]

            # 1) Mild weight shift only
            shifted = list(base)
            shifted = set_many(shifted, support_legs, shoulder=0.55, knee=-0.72)
            shifted = set_many(shifted, other_legs,   shoulder=0.60, knee=-0.64)
            shifted = set_leg(shifted, leg,           shoulder=0.62, knee=-0.58)
            node.send(shifted, hold_s=0.7)

            # 2) Lift target leg a little more
            lifted = list(shifted)
            lifted = set_leg(lifted, leg, shoulder=0.36, knee=-1.12)
            node.send(lifted, hold_s=0.7)

            # 3) Swing clearly forward while lifted
            swung = list(lifted)
            swung = set_leg(swung, leg, shoulder=0.00, knee=-1.00)
            node.send(swung, hold_s=0.8)

            # 4) Place it down forward and KEEP that forward placement
            placed = list(base)
            placed = set_leg(placed, leg, shoulder=0.18, knee=-0.84)
            node.send(placed, hold_s=0.9)

            # 5) Small support re-bias, but do not drag the moved leg back to neutral
            advanced = list(placed)
            advanced = set_many(advanced, support_legs, shoulder=0.56, knee=-0.70)
            advanced = set_many(advanced, other_legs,   shoulder=0.60, knee=-0.64)
            advanced = set_leg(advanced, leg, shoulder=0.18, knee=-0.84)
            node.send(advanced, hold_s=0.9)

            # Update base so the robot keeps the newly advanced leg position
            base = list(advanced)

    node.get_logger().info('Forward lean-step test complete; settling to neutral')
    node.send(list(NEUTRAL_ANGLES), hold_s=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = StepTestNode()
    try:
        node.enable(True)
        run_sequence(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(list(NEUTRAL_ANGLES), hold_s=0.4)
            node.enable(False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
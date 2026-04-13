"""
open_loop_smoke_test.py
-----------------

Open-loop motor-command smoke test for the migrated 8-DOF robot.

This node is intentionally simple:
- it publishes hand-authored motor-frame poses to /joint_angles
- it is useful for checking actuator response, sequencing, and controller behavior
- it is NOT the canonical validator for Cartesian IK, foot-frame signs, or URDF geometry

For kinematic validation, use the shared-IK validation node instead.
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

    # Diagonal-pair crawl:
    #   pair A = FR -> RL
    #   body advance
    #   pair B = FL -> RR
    #   body advance
    pair_a = [0, 3]   # FR, RL
    pair_b = [1, 2]   # FL, RR
    names = ['FR', 'FL', 'RR', 'RL']

    # These are hand-tuned MOTOR-FRAME poses.
    # They are not derived from shared Cartesian IK targets and should not be used
    # as proof of geometric correctness.
    
    # Slightly stronger support bias than before
    support_shift = (0.53, -0.74)
    center_support = (0.60, -0.64)

    lift_pose  = (0.34, -1.16)
    swing_pose = (-0.04, -1.02)
    place_pose = (0.14, -0.86)

    # Small body-catch-up pose applied after each diagonal pair
    body_advance_pose = (0.44, -0.72)

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

    def step_one_leg(current_base, leg):
        """Lift, swing, place one leg while biasing load to the other three."""
        support_legs = [i for i in range(4) if i != leg]

        # 1) Pre-shift weight to support legs
        shifted = list(current_base)
        shifted = set_many(shifted, support_legs, *support_shift)
        shifted = set_leg(shifted, leg, 0.62, -0.58)
        node.send(shifted, hold_s=1.2)

        # 2) Lift target leg
        lifted = list(shifted)
        lifted = set_leg(lifted, leg, *lift_pose)
        node.send(lifted, hold_s=1.0)

        # 3) Swing target leg forward
        swung = list(lifted)
        swung = set_leg(swung, leg, *swing_pose)
        node.send(swung, hold_s=1.2)

        # 4) Place target leg down forward
        placed = list(current_base)
        placed = set_leg(placed, leg, *place_pose)
        node.send(placed, hold_s=1.2)

        return list(placed)

    def body_advance(current_base):
        """
        Try to move the body forward relative to planted feet.

        The idea is:
        - keep the most recently advanced feet farther forward
        - pull the support pattern slightly backward under the body
        - rely on planted contact to translate the chassis forward

        This is still open-loop, so it will never be perfect, but it should be
        more directional than the previous symmetric catch-up phase.
        """
        advanced = list(current_base)

        for leg in range(4):
            i = leg * 2

            # Only pull back legs that are still "behind" the newly placed footholds.
            # Legs already forward stay mostly where they are.
            if advanced[i] > 0.20:
                # keep forward-placed leg more forward
                advanced[i] = clamp(advanced[i] - 0.04)
                advanced[i + 1] = clamp(-0.82)
            else:
                # pull support pattern backward a bit more aggressively
                advanced[i] = clamp(body_advance_pose[0])
                advanced[i + 1] = clamp(body_advance_pose[1])

        node.send(advanced, hold_s=1.5)
        return list(advanced)

    node.get_logger().info('Holding neutral before diagonal-pair crawl test')
    node.send(base, hold_s=1.0)

    for cycle in range(2):
        node.get_logger().info(f'Diagonal-pair crawl cycle {cycle + 1}/2')

        # Pair A: FR -> RL
        for leg in pair_a:
            node.get_logger().info(f'Step {names[leg]}')
            base = step_one_leg(base, leg)

        # Body catch-up phase
        node.get_logger().info('Body advance after FR/RL')
        base = body_advance(base)

        # Pair B: FL -> RR
        for leg in pair_b:
            node.get_logger().info(f'Step {names[leg]}')
            base = step_one_leg(base, leg)

        # Body catch-up phase
        node.get_logger().info('Body advance after FL/RR')
        base = body_advance(base)

        # Brief settle in the progressed stance
        settled = set_many(base, [0, 1, 2, 3], *center_support)
        node.send(settled, hold_s=1.5)
        base = list(settled)

    node.get_logger().info('Diagonal-pair crawl test complete; settling to neutral')
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
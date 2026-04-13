"""
sim_bridge_node.py
------------------

Bridge between the repo's motor-space joint command topic and Gazebo's
geometric / URDF joint controller topic.

Input:
- /joint_angles
- 8 motor-space joint commands in radians
- order: [FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne]

Output:
- /joint_group_position_controller/commands
- 8 geometric / URDF joint commands in radians
- same ordering

Knee decoupling rule:
    shoulder_geometric = shoulder_motor
    knee_geometric     = knee_motor - shoulder_motor

This node should preserve the shared convention used by the gait stack and
provide a single observation point for validating motor-space -> URDF conversion.
"""

import math
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Empty
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

# Ignition Gazebo world name (must match worlds/empty.sdf)
_WORLD_NAME = 'dog_world'


# Joint order expected by /joint_group_position_controller/commands
# Must match joint order in ros2_controllers.yaml (hip joints removed, 8DOF)
JOINT_NAMES = [
    'fr_shoulder_joint', 'fr_knee_joint',
    'fl_shoulder_joint', 'fl_knee_joint',
    'rr_shoulder_joint', 'rr_knee_joint',
    'rl_shoulder_joint', 'rl_knee_joint',
]
NUM_JOINTS = len(JOINT_NAMES)

# ── Hardware → geometric joint conversion ──────────────────────────────────
# The real robot uses a belt-drive on the knee: the motor encoder measures
#   motor_knee = geometric_shoulder + geometric_knee   (coupled)
# so to recover the geometric knee angle for the Gazebo position controller:
#   geometric_knee = motor_knee_cmd - motor_shoulder_cmd
#
# Indices within the 8-element command vector:
#   [FR_sho(0), FR_kne(1), FL_sho(2), FL_kne(3),
#    RR_sho(4), RR_kne(5), RL_sho(6), RL_kne(7)]

FR_SHO = 0
FR_KNE = 1
FL_SHO = 2
FL_KNE = 3
RR_SHO = 4
RR_KNE = 5
RL_SHO = 6
RL_KNE = 7

EXPECTED_LEN = 8
LEG_PAIRS = [
    ("FR", FR_SHO, FR_KNE),
    ("FL", FL_SHO, FL_KNE),
    ("RR", RR_SHO, RR_KNE),
    ("RL", RL_SHO, RL_KNE),
]


def _quat_to_roll_pitch_deg(q):
    """Convert geometry_msgs/Quaternion to (roll_deg, pitch_deg)."""
    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (rotation around x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    return math.degrees(roll_rad), math.degrees(pitch_rad)


class SimBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_bridge_node')

        # Publish joint commands to ign_ros2_control position controller
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10,
        )

        # Publish euler angles to match Teensy /imu/euler output (degrees)
        self._euler_pub = self.create_publisher(
            Vector3,
            '/imu/euler',
            10,
        )

        # Publish sim joint feedback in a separate topic so state_manager can
        # track live positions without colliding with the hardware /joint_states type.
        self._joint_state_sim_pub = self.create_publisher(
            JointState,
            '/joint_states_sim',
            10,
        )

        # Last received joint command — republished at fixed rate so the
        # Gazebo controller catches it even if it activates after the command
        # was first sent (state_manager only publishes on state transitions).
        self._last_cmd: Float64MultiArray | None = None

        # Subscribe to joint angle commands from gait_node / state_manager
        self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self._joint_angles_cb,
            10,
        )

        # Subscribe to Gazebo IMU (bridged via ros_gz_bridge)
        self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_cb,
            10,
        )

        # Subscribe to sim reset trigger from keyboard_node ('r' key)
        self.create_subscription(
            Empty,
            '/sim_reset',
            self._sim_reset_cb,
            10,
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_cb,
            10,
        )

        # Broadcast odom→base_link TF from Gazebo ground-truth odometry.
        # This is only published when autonomous:=true (gz_odom_bridge runs).
        # SLAM Toolbox and Nav2 require this TF to function.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10,
        )

        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_angles_geometric_debug",
            10
        )
                
        self.declare_parameter("debug_joint_conversion", False)
        self.debug_joint_conversion = bool(
            self.get_parameter("debug_joint_conversion").value
        )

        self._last_debug_log_time = 0.0
        self._debug_log_period_sec = 1.0
        # Republish the last command at 20 Hz so the position controller
        # receives it as soon as it becomes active.
        self.create_timer(0.05, self._republish_cmd)

        self.get_logger().info('sim_bridge_node started — bridging to Gazebo Fortress')
        self.get_logger().info(
            f"sim_bridge_node ready | debug_joint_conversion={self.debug_joint_conversion}"
        )
    # ─────────────────────────────────────────────────────────────────
    def _joint_angles_cb(self, msg: Float32MultiArray):
        """Cache and forward /joint_angles (Float32[8]) -> geometric controller commands (Float64[8])."""
        motor_positions = [float(v) for v in msg.data]

        try:
            geo_positions = self._motor_to_geometric(motor_positions)
        except ValueError as exc:
            self.get_logger().error(f"Bad /joint_angles message: {exc}")
            return

        self._log_conversion(motor_positions, geo_positions)

        cmd = Float64MultiArray()
        cmd.data = geo_positions
        self._last_cmd = cmd
        self._cmd_pub.publish(cmd)

        if self.debug_joint_conversion:
            dbg = Float64MultiArray()
            dbg.data = geo_positions
            self.debug_pub.publish(dbg)

    def _republish_cmd(self):
        """Republish last joint command so controller picks it up once active."""
        if self._last_cmd is not None:
            self._cmd_pub.publish(self._last_cmd)

    def _sim_reset_cb(self, _msg: Empty):
        """Reset the Gazebo world (resets model poses and sim time)."""
        self.get_logger().info('Resetting Gazebo world…')
        self._last_cmd = None  # stop republishing stale commands
        subprocess.Popen([
            'ign', 'service',
            '-s', f'/world/{_WORLD_NAME}/control',
            '--reqtype',  'ignition.msgs.WorldControl',
            '--reptype',  'ignition.msgs.Boolean',
            '--timeout',  '2000',
            '--req',      'reset: {model_only: true}',
        ])

    def _odom_cb(self, msg: Odometry):
        """Broadcast odom→base_link TF from Gazebo ground-truth odometry.

        Stamps the transform with the node's current clock (sim time when
        use_sim_time:=true) rather than msg.header.stamp.  This prevents
        TF_OLD_DATA warnings caused by DDS reordering /odom messages slightly
        out of order — the node clock is always monotonically increasing.
        """
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()  # monotonic sim clock
        t.header.frame_id = msg.header.frame_id              # 'odom'
        t.child_frame_id  = msg.child_frame_id               # 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _imu_cb(self, msg: Imu):
        """Convert Imu orientation quaternion → /imu/euler (degrees, matches Teensy)."""
        roll_deg, pitch_deg = _quat_to_roll_pitch_deg(msg.orientation)

        euler = Vector3()
        euler.x = roll_deg
        euler.y = pitch_deg
        euler.z = 0.0  # yaw not used by state_manager fall-detection
        self._euler_pub.publish(euler)
    
    def _joint_states_cb(self, msg: JointState):
        """Republish Gazebo joint states on /joint_states_sim for state_manager."""
        self._joint_state_sim_pub.publish(msg)
    
    def _motor_to_geometric(self, motor_positions: list[float]) -> list[float]:
        """
        Convert 8 motor-space joint commands into 8 geometric / URDF joint commands.
        """
        if len(motor_positions) != EXPECTED_LEN:
            raise ValueError(
                f"Expected {EXPECTED_LEN} motor commands, got {len(motor_positions)}"
            )

        geo = list(motor_positions)
        for _, sho_idx, kne_idx in LEG_PAIRS:
            geo[kne_idx] = motor_positions[kne_idx] - motor_positions[sho_idx]
        return geo
    
    def _log_conversion(self, motor_positions: list[float], geo_positions: list[float]) -> None:
        if not self.debug_joint_conversion:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_debug_log_time) < self._debug_log_period_sec:
            return
        self._last_debug_log_time = now
        
        self.get_logger().info(
            "Motor -> geometric conversion:\n"
            + "\n".join(
                (
                    f"  {leg}: "
                    f"motor(sho={motor_positions[sho_idx]: .4f}, kne={motor_positions[kne_idx]: .4f}) -> "
                    f"geo(sho={geo_positions[sho_idx]: .4f}, kne={geo_positions[kne_idx]: .4f})"
                )
                for leg, sho_idx, kne_idx in LEG_PAIRS
            )
        )
        for _, sho_idx, kne_idx in LEG_PAIRS:
            expected = motor_positions[kne_idx] - motor_positions[sho_idx]
            actual = geo_positions[kne_idx]
            if abs(expected - actual) > 1e-9:
                self.get_logger().error(
                    f"Bridge conversion mismatch: expected {expected}, got {actual}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

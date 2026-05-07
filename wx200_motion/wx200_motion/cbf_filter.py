#!/usr/bin/env python3
import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CBFFilter(Node):

    def __init__(self):
        super().__init__('cbf_filter')

        # Workspace square bounds in the world XY plane [metres]
        self.declare_parameter('x_min', 0.15)
        self.declare_parameter('x_max',  0.45)
        self.declare_parameter('y_min', -0.15)
        self.declare_parameter('y_max',  0.15)

        # Desired EE height and P-gain to hold it
        self.declare_parameter('z_target', 0.10)
        self.declare_parameter('z_gain', 1.0)

        # CBF decay rate α: higher values brake harder near boundaries
        self.declare_parameter('cbf_alpha', 1.0)

        # P-gain for orientation correction toward downward EE orientation
        self.declare_parameter('orient_gain', 1.0)

        # Frames
        self.declare_parameter('ee_frame', 'wx200/ee_gripper_link')
        self.declare_parameter('reference_frame', 'world')

        self._mode = 'velocity'

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(TwistStamped, '/servo/delta_twist_cmds', 10)
        self._cmd_sub = self.create_subscription(
            TwistStamped, 'cmd_vel', self._cmd_cb, 10)
        self._mode_sub = self.create_subscription(
            String, '/position_server/mode', self._mode_cb, 10)

        self._switch_cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self._switch_timer = self.create_timer(1.0, self._set_command_type)

        self.get_logger().info('CBF filter ready')

    # ── Servo init ────────────────────────────────────────────────────────────

    def _set_command_type(self):
        if not self._switch_cli.service_is_ready():
            return
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self._switch_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            'Servo command type set to TWIST' if f.result() and f.result().success
            else f'Failed to set servo command type: {f.result()}'
        ))
        self._switch_timer.cancel()

    # ── TF ────────────────────────────────────────────────────────────────────

    def _get_ee_transform(self):
        """Return (position, rotation_matrix) in reference_frame, or (None, None)."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.get_parameter('reference_frame').value,
                self.get_parameter('ee_frame').value,
                rclpy.time.Time(),
            )
            R = _quat_to_matrix(t.transform.rotation)
            return t.transform.translation, R
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None, None

    # ── CBF position filter ───────────────────────────────────────────────────

    def _apply_cbf(self, twist: TwistStamped, pos) -> TwistStamped:
        x, y, z = pos.x, pos.y, pos.z
        a = self.get_parameter('cbf_alpha').value

        # XY square: CBF clamping — allowed velocity toward a boundary shrinks to
        # zero as the EE approaches it, preventing constraint violation.
        twist.twist.linear.x = _clamp(
            twist.twist.linear.x,
            -a * (x - self.get_parameter('x_min').value),
             a * (self.get_parameter('x_max').value - x),
        )
        twist.twist.linear.y = _clamp(
            twist.twist.linear.y,
            -a * (y - self.get_parameter('y_min').value),
             a * (self.get_parameter('y_max').value - y),
        )

        # Z: ignore raw vz; P-controller holds EE at z_target
        # twist.twist.linear.z = (
        #     self.get_parameter('z_gain').value
        #     * (self.get_parameter('z_target').value - z)
        # )

        return twist

    # ── Orientation correction ────────────────────────────────────────────────

    def _check_orientation(self, twist: TwistStamped, R: np.ndarray) -> TwistStamped:
        """Add corrective angular velocity (world frame) to maintain downward EE Z-axis."""
        z_ee  = R @ np.array([0.0, 0.0, 1.0])  # EE z-axis expressed in world frame
        z_des = np.array([0.0, 0.0, -1.0])       # desired: pointing straight down

        cross = np.cross(z_ee, z_des)
        sin_angle = np.linalg.norm(cross)
        cos_angle = float(np.dot(z_ee, z_des))

        if sin_angle > 1e-6:
            axis  = cross / sin_angle
            angle = np.arctan2(sin_angle, cos_angle)
            if angle > 0.05:
                omega = self.get_parameter('orient_gain').value * angle * axis
                twist.twist.angular.x += float(omega[0])
                twist.twist.angular.y += float(omega[1])
                twist.twist.angular.z += float(omega[2])

        return twist

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        self._mode = msg.data

    def _cmd_cb(self, msg: TwistStamped):
        if self._mode != 'velocity':
            return

        pos, R = self._get_ee_transform()
        if pos is None:
            return

        msg = self._apply_cbf(msg, pos)
        # msg = self._check_orientation(msg, R)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('reference_frame').value
        self._cmd_pub.publish(msg)

def main(args=None):
    """Entrypoint for pick_node."""
    rclpy.init(args=args)
    node = CBFFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
import rclpy.time
from rclpy.node import Node
from realtime_servo.msg import RelativeMove
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


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

        # CBF decay rate α: higher values brake harder near boundaries
        self.declare_parameter('cbf_alpha', 1.0)

        # Frames
        self.declare_parameter('ee_frame', 'wx200/ee_gripper_link')
        self.declare_parameter('reference_frame', 'world')

        # Topics
        # Output is wx200-namespaced; downstream jacobian node should subscribe
        # to /wx200/cmd_vel (set via its velocity_command_topic parameter).
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/wx200/cmd_vel')

        self._mode = 'velocity'

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(
            RelativeMove, self.get_parameter('output_topic').value, 10)
        self._cmd_sub = self.create_subscription(
            RelativeMove, self.get_parameter('input_topic').value, self._cmd_cb, 10)
        self._mode_sub = self.create_subscription(
            String, '/position_server/mode', self._mode_cb, 10)

        self.get_logger().info('CBF filter ready')


    def _get_ee_position(self):
        """Return EE translation in reference_frame, or None on failure."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.get_parameter('reference_frame').value,
                self.get_parameter('ee_frame').value,
                rclpy.time.Time(),
            )
            return t.transform.translation
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None


    def _apply_cbf(self, msg: RelativeMove, pos) -> RelativeMove:
        x, y = pos.x, pos.y
        a = self.get_parameter('cbf_alpha').value

        msg.dx = _clamp(
            msg.dx,
            -a * (x - self.get_parameter('x_min').value),
             a * (self.get_parameter('x_max').value - x),
        )
        msg.dy = _clamp(
            msg.dy,
            -a * (y - self.get_parameter('y_min').value),
             a * (self.get_parameter('y_max').value - y),
        )
        return msg


    def _mode_cb(self, msg: String):
        self._mode = msg.data

    def _cmd_cb(self, msg: RelativeMove):
        if self._mode != 'velocity':
            return

        pos = self._get_ee_position()
        if pos is None:
            return

        msg = self._apply_cbf(msg, pos)
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CBFFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

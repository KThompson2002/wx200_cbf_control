#!/usr/bin/env python3
import rclpy
import rclpy.time
from geometry_msgs.msg import Point
from rclpy.node import Node
from realtime_servo.msg import RelativeMove
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CBFFilter(Node):

    def __init__(self):
        super().__init__('cbf_filter')

        # Workspace square bounds in the world XY plane [metres]
        self.declare_parameter('x_min', 0.12)
        self.declare_parameter('x_max',  0.35)
        self.declare_parameter('y_min', -0.17)
        self.declare_parameter('y_max',  0.17)

        # CBF decay rate α: higher values brake harder near boundaries
        self.declare_parameter('cbf_alpha', 1.1)

        # Frames
        self.declare_parameter('ee_frame', 'wx200/ee_gripper_link')
        self.declare_parameter('reference_frame', 'world')

        # Topics
        # Output is wx200-namespaced; downstream jacobian node should subscribe
        # to /wx200/cmd_vel (set via its velocity_command_topic parameter).
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/wx200/cmd_vel')

        #   z_kp: 1/s          (dz contribution per metre of z error)
        #   z_ki: 1/s^2        (set to 0 for P-only)
        #   z_integral_clamp:  metres·s, anti-windup saturation
        #   z_correction_max:  m/s, hard saturation on the injected dz
        #   dz_motion_threshold: m/s, |dz| above this is treated as user-driven
        #                        Z motion; the target re-latches to current z
        self.declare_parameter('enable_z_hold', True)
        self.declare_parameter('z_kp', 2.0)
        self.declare_parameter('z_ki', 0.1)
        self.declare_parameter('z_integral_clamp', 0.05)
        self.declare_parameter('z_correction_max', 0.05)
        self.declare_parameter('dz_motion_threshold', 1e-1)

        self._mode = 'velocity'
        self._z_target = None
        self._z_integral = 0.0
        self._last_cmd_time = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(
            RelativeMove, self.get_parameter('output_topic').value, 10)
        self._cmd_sub = self.create_subscription(
            RelativeMove, self.get_parameter('input_topic').value, self._cmd_cb, 10)
        self._mode_sub = self.create_subscription(
            String, '/position_server/mode', self._mode_cb, 10)

        self._marker_pub = self.create_publisher(MarkerArray, '~/bounds_marker', 1)
        self._marker_timer = self.create_timer(1.0, self._publish_bounds_marker)

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


    def _apply_z_hold(self, msg: RelativeMove, pos) -> RelativeMove:
        if not self.get_parameter('enable_z_hold').value:
            return msg

        kp = self.get_parameter('z_kp').value
        ki = self.get_parameter('z_ki').value
        i_clamp = self.get_parameter('z_integral_clamp').value
        corr_max = self.get_parameter('z_correction_max').value
        motion_thresh = self.get_parameter('dz_motion_threshold').value

        # User is actively commanding Z motion: pass it through and re-latch the
        # plane to the current z. The next "hold" cycle will start from here.
        if abs(msg.dz) > motion_thresh:
            self._z_target = pos.z
            self._z_integral = 0.0
            return msg

        if self._z_target is None:
            self._z_target = pos.z

        error = self._z_target - pos.z

        now = self.get_clock().now()
        if ki != 0.0 and self._last_cmd_time is not None:
            dt = (now - self._last_cmd_time).nanoseconds * 1e-9
            # Reject stale dt (mode-change or long gap) — don't integrate over it.
            if 0.0 < dt < 0.2:
                self._z_integral = _clamp(
                    self._z_integral + error * dt, -i_clamp, i_clamp)
        self._last_cmd_time = now

        correction = kp * error + ki * self._z_integral
        msg.dz = _clamp(correction, -corr_max, corr_max)
        return msg


    def _publish_bounds_marker(self):
        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        frame = self.get_parameter('reference_frame').value

        # Draw the rectangle at the latched z plane if available; otherwise at
        # the current EE z; otherwise at z=0 as a last resort.
        if self._z_target is not None:
            z = self._z_target
        else:
            pos = self._get_ee_position()
            z = pos.z if pos is not None else 0.0

        now = self.get_clock().now().to_msg()

        outline = Marker()
        outline.header.frame_id = frame
        outline.header.stamp = now
        outline.ns = 'cbf_bounds'
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.005
        outline.color.r = 0.1
        outline.color.g = 0.9
        outline.color.b = 0.2
        outline.color.a = 1.0
        outline.pose.orientation.w = 1.0
        corners = [
            (x_min, y_min), (x_max, y_min),
            (x_max, y_max), (x_min, y_max),
            (x_min, y_min),
        ]
        outline.points = [Point(x=float(cx), y=float(cy), z=float(z)) for cx, cy in corners]

        plane = Marker()
        plane.header.frame_id = frame
        plane.header.stamp = now
        plane.ns = 'cbf_bounds'
        plane.id = 1
        plane.type = Marker.CUBE
        plane.action = Marker.ADD
        plane.pose.position.x = 0.5 * (x_min + x_max)
        plane.pose.position.y = 0.5 * (y_min + y_max)
        plane.pose.position.z = float(z)
        plane.pose.orientation.w = 1.0
        plane.scale.x = max(1e-3, x_max - x_min)
        plane.scale.y = max(1e-3, y_max - y_min)
        plane.scale.z = 0.001
        plane.color.r = 0.1
        plane.color.g = 0.6
        plane.color.b = 1.0
        plane.color.a = 0.15

        self._marker_pub.publish(MarkerArray(markers=[outline, plane]))


    def _mode_cb(self, msg: String):
        # On any mode change, drop the latched target and integral so MoveIt's
        # motion doesn't get fought by stale state when velocity mode resumes.
        if msg.data != self._mode:
            self._z_target = None
            self._z_integral = 0.0
            self._last_cmd_time = None
        self._mode = msg.data

    def _cmd_cb(self, msg: RelativeMove):
        if self._mode != 'velocity':
            return

        pos = self._get_ee_position()
        if pos is None:
            return

        # msg = self._apply_cbf(msg, pos)
        # msg = self._apply_z_hold(msg, pos)
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CBFFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

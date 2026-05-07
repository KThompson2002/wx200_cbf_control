#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class JointStateSanitizer(Node):
    """Republishes joint states with NaN values replaced by zero."""

    def __init__(self):
        super().__init__('joint_state_sanitizer')
        self.declare_parameter('input_topic', '/wx200/joint_states')
        self.declare_parameter('output_topic', '/wx200/joint_states_clean')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(JointState, out_topic, qos)
        self._sub = self.create_subscription(JointState, in_topic, self._cb, qos)
        self.get_logger().info(f'Sanitizing {in_topic} → {out_topic}')

    def _cb(self, msg: JointState):
        msg.position = [0.0 if math.isnan(p) else p for p in msg.position]
        msg.velocity = [0.0 if math.isnan(v) else v for v in msg.velocity]
        msg.effort   = [0.0 if math.isnan(e) else e for e in msg.effort]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

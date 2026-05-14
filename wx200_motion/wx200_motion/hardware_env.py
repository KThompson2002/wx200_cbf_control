#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from realtime_servo.msg import RelativeMove
from sensor_msgs.msg import JointState
# from intera_core_msgs.msg import EndpointState
from std_msgs.msg import String
import cv2
import numpy as np
import time
import torch
from copy import deepcopy
from wx200_motion_interface.action import MoveToPose


class HardwareEnv(Node):
    """
    Placeholder class for real-world hardware environments on the Sawyer robot.
    The action space will always only be continuous, with controls being [dx, dy] of the end-effector.
    """
    def __init__(self):
        super().__init__('hardware_env')

        self.save_path = "/home/kyle-thompson/FinalProject/ws/src/runs/imgs"  # TODO: change this path as needed
        self.idx = 0

        # Action space: [dx, dy] end-effector velocity - force only 2D planar
        self.action_space = type('', (), {})()
        self.action_space.low = np.array([-0.1, -0.1], dtype=np.float32)
        self.action_space.high = np.array([0.1, 0.1], dtype=np.float32)
        self.action_space.sample = lambda: np.random.uniform(self.action_space.low, self.action_space.high).astype(np.float32)

        # Publishers
        self.rel_move_pub = self.create_publisher(RelativeMove, '/velocity_pub/vel_command', 1)  # CHANGED VEL PUBLISHER
        self.reset_pub = self.create_publisher(String, '/wx200/reset', 10)
        self.comms = self.create_publisher(String, '/wx200/comms', 10)

        self._pos_client = ActionClient(self, MoveToPose, '/position_server/move_to_pose')


        # Subscribers
        self.last_joint_state = None
        self.last_endpoint_state = None
        self.create_subscription(JointState, '/wx200/joint_states', self._joint_state_callback, 10)
        # self.create_subscription(EndpointState, '/robot/limb/right/endpoint_state', self._endpoint_state_callback, 10) ### ????

        self.done_reset = False
        self.control_freq = 10.0  # Hz

        # Publish initial reset
        msg = String()
        msg.data = ""
        self.reset_pub.publish(msg)

        # Timer
        self.timer = self.create_timer(1.0 / self.control_freq, self.timer_callback)

        #Setup Camera
        self.camera = cv2.VideoCapture('/dev/video2')
        if not self.camera.isOpened():
            self.get_logger().error("Failed to open Camera")

    def timer_callback(self):
        self.get_logger().info('main', once=True)
        if self.idx < 10:
            self.step(np.array([0.1, 0.0]))
            self.idx += 1
        if self.idx == 10:
            self.step(np.array([0.0, 0.0]))
            # self.reset()
            self.idx += 1

    def shutdown(self):
        self.get_logger().info("Shutting down")
        self.step(np.array([0.0, 0.0]))
        if hasattr(self, "timer") and self.timer:
            self.timer.cancel()
        if hasattr(self, "camera") and self.camera:
            if self.camera.isOpened():
                self.camera.release()
        cv2.destroyAllWindows()
        self.get_logger().info("HardwareEnv shutdown: camera released and timer stopped")

    def _joint_state_callback(self, msg):
        self.last_joint_state = msg

    # def _endpoint_state_callback(self, msg):
    #     self.last_endpoint_state = deepcopy(msg)

    def process_image(self, image):
        """Process raw image from camera if needed. [H, W, C] -> [64, 64, C] tensor"""
        processed = cv2.resize(image, (64, 64))
        return processed

    def reset(self):
        """Reset arm to home position"""
        try:
            msg = String()
            msg.data = "reset"
            self.reset_pub.publish(msg)
            time.sleep(0.2)
            msg.data = ""
            self.reset_pub.publish(msg)
            obs = self._get_observation()
            self.done_reset = False
            return obs, {}
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
            raise

    def _wait_for_arm_stopped(self, vel_threshold=0.01, timeout_sec=5.0):
        """Spin until all joint velocities drop below threshold or timeout."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_joint_state is not None:
                if all(abs(v) < vel_threshold for v in self.last_joint_state.velocity):
                    return True
        self.get_logger().warn("Arm did not stop within timeout, proceeding anyway")
        return False

    def reset(self):
        # Zero velocity and let the arm controller finish before switching to position mode.
        stop = RelativeMove()
        stop.dx = 0.0
        stop.dy = 0.0
        self.rel_move_pub.publish(stop)
        # time.sleep(0.5)
        self._wait_for_arm_stopped()

        goal = MoveToPose.Goal()
        goal.named_target = 'Start'

        send_future = self._pos_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Reset goal rejected by position server")
            return self._get_observation(), {'reset_failed': True}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        # self.done_reset = False
        msg = String()
        msg.data = "reset"
        self.reset_pub.publish(msg)
        time.sleep(0.2)
        msg.data = ""
        self.reset_pub.publish(msg)
        obs = self._get_observation()
        self.done_reset = False
        return obs, {}
        return self._get_observation(), {}
        
    def render(self):
        """Read from camera, and save to path."""
        self.get_logger().info("render called")
        ret, frame = self.camera.read()
        if ret:
            # Ayush/Jared Code
            # filename = "captured_image_{:04d}.jpg".format(self.idx)
            # ok = cv2.imwrite(filename, self.process_image(frame))
            # if ok:
            #     self.get_logger().info("Image saved to {}".format(filename))
            #     self.idx += 1
            # else:
            #     self.get_logger().warn("Failed to write to {}".format(filename))

            # Kyle Code
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return self.process_image(frame_rgb,)
            # tensor = torch.from_numpy(frame_64).float() / 255.0 # converts to 0 - 1 and (C, H, W)
            # return tensor.permute(2, 0, 1)
        else:
            self.get_logger().warn("Failed to read frame from camera")

    def step(self, action):
        """
        Publish action to /relative_move topic.
        Args:
            action: [dx, dy] end-effector velocity
        Returns:
            obs, reward, done, truncated, info
        """
        self.get_logger().info(f"step {action}")
        action = np.clip(action, self.action_space.low, self.action_space.high)

        rel_move = RelativeMove()
        rel_move.dx = float(action[0])
        # self.get_logger().info(f"shape: {action.shape}")
        rel_move.dy = float(action[1])
        self.get_logger().info(str(rel_move))
        self.rel_move_pub.publish(rel_move)

        # Small delay to let command execute
        # time.sleep(0.1)

        obs = self._get_observation()

        reward = 0.0
        done = False
        truncated = False
        info = {}

        return obs, reward, done, truncated, info

    def _get_observation(self):
        """
        Extract observation from robot state.
        Could be endpoint pose, joint angles, or rendered image.
        """
        # if self.last_endpoint_state is not None:
        #     pose = self.last_endpoint_state.pose.position
        #     return np.array([pose.x, pose.y, pose.z])
        # else:
        return np.zeros(3)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareEnv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
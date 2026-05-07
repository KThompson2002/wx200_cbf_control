#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Pose as GeometryPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from wx200_motion_interface.action import MoveToPose

# Joint values for each SRDF named state; mirrors config/named_states.yaml.
NAMED_STATES = {
    'Home':    {'waist': 0.0, 'shoulder':  0.0,  'elbow':  0.0,    'wrist_angle': 0.0,     'wrist_rotate': 0.0},
    'Upright': {'waist': 0.0, 'shoulder':  0.0,  'elbow': -1.5708, 'wrist_angle': 0.0,     'wrist_rotate': 0.0},
    'Sleep':   {'waist': 0.0, 'shoulder': -1.88, 'elbow':  1.55,   'wrist_angle': 0.8,     'wrist_rotate': 0.0},
    'Start':   {'waist': 0.0, 'shoulder': -0.3,  'elbow': 0.5,    'wrist_angle':  1.37,   'wrist_rotate': 0.0},
    'Servo':   {'waist': 0.0, 'shoulder': -0.3,  'elbow': 0.5,    'wrist_angle':  1.1,    'wrist_rotate': 0.0},
}


def _joint_constraints(joint_positions: dict, tol: float = 0.01) -> Constraints:
    c = Constraints()
    for name, value in joint_positions.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(value)
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def _pose_constraints(pose_stamped, ee_link: str) -> Constraints:
    c = Constraints()

    pc = PositionConstraint()
    pc.header = pose_stamped.header
    pc.link_name = ee_link
    bv = BoundingVolume()
    bv.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
    box_pose = GeometryPose()
    box_pose.position = pose_stamped.pose.position
    box_pose.orientation.w = 1.0
    bv.primitive_poses.append(box_pose)
    pc.constraint_region = bv
    pc.weight = 1.0
    c.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header = pose_stamped.header
    oc.link_name = ee_link
    oc.orientation = pose_stamped.pose.orientation
    oc.absolute_x_axis_tolerance = 3.14159
    oc.absolute_y_axis_tolerance = 3.14159
    oc.absolute_z_axis_tolerance = 3.14159
    oc.weight = 1.0
    c.orientation_constraints.append(oc)

    return c


class PositionServer(Node):

    def __init__(self):
        super().__init__('position_server')

        self.declare_parameter('planning_group', 'interbotix_arm')
        self.declare_parameter('ee_link', 'wx200/ee_gripper_link')
        self.declare_parameter('default_velocity_scaling', 0.5)
        self.declare_parameter('default_acceleration_scaling', 0.5)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('move_group_action', '/move_action')

        # Mode topic: CBF filter subscribes to know when to pause velocity commands.
        self._mode_pub = self.create_publisher(String, '~/mode', 10)
        self._set_mode('velocity')

        # Separate callback groups: server group ensures one active goal at a time;
        # client group allows move_group responses to fire while execute_cb awaits.
        server_group = MutuallyExclusiveCallbackGroup()
        client_group = MutuallyExclusiveCallbackGroup()

        self._mg_client = ActionClient(
            self,
            MoveGroup,
            self.get_parameter('move_group_action').value,
            callback_group=client_group,
        )

        self._action_server = ActionServer(
            self,
            MoveToPose,
            '~/move_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=server_group,
        )

        self.get_logger().info('Waiting for /move_action ...')
        self._mg_client.wait_for_server()
        self.get_logger().info('Position server ready')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _build_mg_goal(self, req) -> MoveGroup.Goal:
        vel = req.velocity_scaling if req.velocity_scaling > 0.0 \
            else self.get_parameter('default_velocity_scaling').value
        acc = req.acceleration_scaling if req.acceleration_scaling > 0.0 \
            else self.get_parameter('default_acceleration_scaling').value

        if req.named_target:
            goal_constraints = [_joint_constraints(NAMED_STATES[req.named_target])]
        else:
            goal_constraints = [_pose_constraints(
                req.target_pose,
                self.get_parameter('ee_link').value,
            )]

        plan_req = MotionPlanRequest()
        plan_req.group_name = self.get_parameter('planning_group').value
        plan_req.goal_constraints = goal_constraints
        plan_req.num_planning_attempts = 5
        plan_req.allowed_planning_time = self.get_parameter('planning_time').value
        plan_req.max_velocity_scaling_factor = float(vel)
        plan_req.max_acceleration_scaling_factor = float(acc)

        mg_goal = MoveGroup.Goal()
        mg_goal.request = plan_req
        mg_goal.planning_options.plan_only = False
        mg_goal.planning_options.replan = False
        return mg_goal

    # ── action server callbacks ───────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        has_named = bool(goal_request.named_target)
        has_pose = bool(goal_request.target_pose.header.frame_id)

        if not has_named and not has_pose:
            self.get_logger().warn('Rejected: set named_target or target_pose.header.frame_id')
            return GoalResponse.REJECT
        if has_named and goal_request.named_target not in NAMED_STATES:
            self.get_logger().warn(f'Rejected: unknown named_target "{goal_request.named_target}"')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        result = MoveToPose.Result()
        feedback = MoveToPose.Feedback()

        # Signal CBF to pause velocity commands; move_group and any active servo
        # trajectory need a moment to settle before we send a new plan.
        self._set_mode('position')

        try:
            mg_goal = self._build_mg_goal(goal_handle.request)

            feedback.status = 'planning'
            goal_handle.publish_feedback(feedback)

            # Await goal acceptance by move_group.
            mg_goal_handle = await self._mg_client.send_goal_async(mg_goal)
            if not mg_goal_handle.accepted:
                result.success = False
                result.message = 'move_group rejected the goal'
                goal_handle.abort()
                return result

            feedback.status = 'executing'
            goal_handle.publish_feedback(feedback)

            # Await trajectory execution.  Cancellation is forwarded to move_group.
            if goal_handle.is_cancel_requested:
                await mg_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            wrapped = await mg_goal_handle.get_result_async()
            mg_result = wrapped.result

            if mg_result.error_code.val == MoveItErrorCodes.SUCCESS:
                result.success = True
                result.message = 'Motion completed'
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f'MoveIt error code {mg_result.error_code.val}'
                goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f'Exception in execute_cb: {e}')
            result.success = False
            result.message = str(e)
            goal_handle.abort()

        finally:
            self._set_mode('velocity')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PositionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
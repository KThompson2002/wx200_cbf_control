#!/usr/bin/env python3
"""
Effort + motor-status watchdog for the wx200 Interbotix arm.

Watches /wx200/joint_states for joint-effort violations and polls the XS
driver's Hardware_Error_Status register for motor faults. On violation it
floods zero RelativeMove commands at both the CBF input and the velocity
controller input so the arm stops regardless of who is publishing, and
publishes a status string for downstream UIs.

The reboot path is opt-in (auto_reboot param) — by default the node halts
and waits for human intervention, since auto-rebooting under sustained
load can mask the underlying problem.
"""

import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from realtime_servo.msg import RelativeMove
from interbotix_xs_msgs.srv import Reboot, RegisterValues, TorqueEnable


# Dynamixel X-series Hardware_Error_Status bit map (control table 70).
HW_ERROR_BITS = {
    0: 'input_voltage',
    2: 'overheating',
    3: 'motor_encoder',
    4: 'electrical_shock',
    5: 'overload',
}


def _decode_hw_error(code: int) -> List[str]:
    return [name for bit, name in HW_ERROR_BITS.items() if code & (1 << bit)]


class EffortMotorWatchdog(Node):

    def __init__(self):
        super().__init__('effort_motor_watchdog')

        # ── Topics / services ────────────────────────────────────────────
        self.declare_parameter('joint_states_topic', '/wx200/joint_states')
        self.declare_parameter('cmd_vel_input_topic', '/velocity_pub/vel_command')
        self.declare_parameter('cmd_vel_output_topic', '/wx200/cmd_vel')
        self.declare_parameter('status_topic', '~/status')
        self.declare_parameter('reboot_service', '/wx200/reboot_motors')
        self.declare_parameter('get_registers_service', '/wx200/get_motor_registers')
        self.declare_parameter('torque_enable_service', '/wx200/torque_enable')
        self.declare_parameter('group_name', 'arm')

        # ── Effort thresholds (per-joint, in driver units). Defaults are
        #    rail-berkeley's wx250s numbers minus the dual shoulder, padded
        #    by 1.7×; tune these on real hardware before relying on them. ─
        self.declare_parameter('joint_names', [
            'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate',
        ])
        self.declare_parameter('effort_thresholds', [
            800.0 * 1.7,
            1000.0 * 1.7,
            600.0 * 1.7,
            600.0 * 1.7,
            600.0 * 1.7,
        ])
        self.declare_parameter('effort_violation_window_sec', 0.1)

        # ── Rates ────────────────────────────────────────────────────────
        self.declare_parameter('effort_check_rate_hz', 50.0)
        self.declare_parameter('motor_status_check_rate_hz', 1.0)
        self.declare_parameter('zero_publish_rate_hz', 30.0)

        # ── Recovery policy ──────────────────────────────────────────────
        self.declare_parameter('auto_reboot', False)
        self.declare_parameter('reboot_cooldown_sec', 5.0)
        self.declare_parameter('reset_torque_after_reboot', True)

        self._joint_names: List[str] = list(self.get_parameter('joint_names').value)
        thresholds = list(self.get_parameter('effort_thresholds').value)
        if len(thresholds) != len(self._joint_names):
            raise RuntimeError(
                f'effort_thresholds length ({len(thresholds)}) must match '
                f'joint_names length ({len(self._joint_names)})')
        self._thresholds: Dict[str, float] = dict(zip(self._joint_names, thresholds))
        self._effort_window = float(self.get_parameter('effort_violation_window_sec').value)

        self._state_lock = threading.Lock()
        self._latest_state: Optional[JointState] = None
        self._latched_fault: Optional[str] = None
        self._latched_fault_detail: str = ''
        self._last_reboot_time: Optional[rclpy.time.Time] = None
        self._effort_violation_start: Optional[rclpy.time.Time] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        sensor_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = ReentrantCallbackGroup()

        self._js_sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._joint_state_cb,
            qos,
            callback_group=sensor_group,
        )

        # Publish zeros on both the CBF input and the post-CBF output so an
        # operator publishing through hardware_env *and* a planner that has
        # bypassed the CBF both get overridden.
        self._cmd_in_pub = self.create_publisher(
            RelativeMove, self.get_parameter('cmd_vel_input_topic').value, 10)
        self._cmd_out_pub = self.create_publisher(
            RelativeMove, self.get_parameter('cmd_vel_output_topic').value, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        self._reboot_client = self.create_client(
            Reboot,
            self.get_parameter('reboot_service').value,
            callback_group=service_group,
        )
        self._get_regs_client = self.create_client(
            RegisterValues,
            self.get_parameter('get_registers_service').value,
            callback_group=service_group,
        )
        self._torque_client = self.create_client(
            TorqueEnable,
            self.get_parameter('torque_enable_service').value,
            callback_group=service_group,
        )

        effort_period = 1.0 / max(1.0, float(self.get_parameter('effort_check_rate_hz').value))
        motor_period = 1.0 / max(0.1, float(self.get_parameter('motor_status_check_rate_hz').value))
        zero_period = 1.0 / max(1.0, float(self.get_parameter('zero_publish_rate_hz').value))

        self._effort_timer = self.create_timer(
            effort_period, self._check_effort, callback_group=timer_group)
        self._motor_timer = self.create_timer(
            motor_period, self._check_motor_status, callback_group=timer_group)
        # Zero-publish timer is always on once a fault is latched; it is a
        # cheap message at 30 Hz and ensures any new publisher gets overridden.
        self._zero_timer = self.create_timer(
            zero_period, self._publish_zero_if_faulted, callback_group=timer_group)

        self._publish_status('ok')
        self.get_logger().info(
            f'Watchdog ready. effort thresholds: {self._thresholds}, '
            f"auto_reboot={self.get_parameter('auto_reboot').value}"
        )

    # ── Subscriptions ────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        with self._state_lock:
            self._latest_state = msg

    # ── Effort check ─────────────────────────────────────────────────────

    def _check_effort(self):
        with self._state_lock:
            state = self._latest_state

        if state is None or not state.effort:
            return

        # Sustain-window: a single noisy sample shouldn't trip the e-stop.
        # Require continuous violation for effort_violation_window_sec.
        violated_joint, value, threshold = self._scan_effort(state)
        now = self.get_clock().now()
        if violated_joint is None:
            self._effort_violation_start = None
            return

        if self._effort_violation_start is None:
            self._effort_violation_start = now
            return

        elapsed = (now - self._effort_violation_start).nanoseconds * 1e-9
        if elapsed < self._effort_window:
            return

        self._latch_fault(
            'effort_violation',
            f'{violated_joint} effort={value:.1f} > {threshold:.1f}',
        )

    def _scan_effort(self, state: JointState):
        for name, effort in zip(state.name, state.effort):
            threshold = self._thresholds.get(name)
            if threshold is None:
                continue
            if abs(effort) > threshold:
                return name, abs(effort), threshold
        return None, 0.0, 0.0

    # ── Motor hardware status poll ───────────────────────────────────────

    def _check_motor_status(self):
        if not self._get_regs_client.service_is_ready():
            self._get_regs_client.wait_for_service(timeout_sec=0.0)
            return

        req = RegisterValues.Request()
        req.cmd_type = 'group'
        req.name = self.get_parameter('group_name').value
        req.reg = 'Hardware_Error_Status'

        future = self._get_regs_client.call_async(req)
        future.add_done_callback(self._motor_status_done)

    def _motor_status_done(self, future):
        try:
            res = future.result()
        except Exception as ex:
            self.get_logger().warn(f'get_motor_registers failed: {ex}', throttle_duration_sec=5.0)
            return

        faults = []
        for idx, code in enumerate(res.values):
            if code:
                joint = self._joint_names[idx] if idx < len(self._joint_names) else f'idx_{idx}'
                faults.append(f'{joint}={_decode_hw_error(int(code))}')

        if not faults:
            # Clear a latched motor fault if the registers are clean again
            # after a reboot. Effort violations do *not* auto-clear — the
            # operator has to intervene, since clearing them would just let
            # the same command continue stalling the motor.
            if self._latched_fault == 'motor_fault':
                self.get_logger().info('Motor hardware error cleared')
                self._clear_fault()
            return

        detail = ', '.join(faults)
        self._latch_fault('motor_fault', detail)

        if self.get_parameter('auto_reboot').value:
            self._maybe_reboot()

    # ── Fault latching and zero-publish ──────────────────────────────────

    def _latch_fault(self, kind: str, detail: str):
        if self._latched_fault == kind and self._latched_fault_detail == detail:
            return
        self._latched_fault = kind
        self._latched_fault_detail = detail
        self.get_logger().error(f'FAULT {kind}: {detail}')
        self._publish_status(f'{kind}: {detail}')
        # Flush a zero immediately rather than waiting for the timer.
        self._publish_zero()

    def _clear_fault(self):
        self._latched_fault = None
        self._latched_fault_detail = ''
        self._effort_violation_start = None
        self._publish_status('ok')

    def _publish_zero_if_faulted(self):
        if self._latched_fault is not None:
            self._publish_zero()

    def _publish_zero(self):
        zero = RelativeMove()
        zero.dx = 0.0
        zero.dy = 0.0
        zero.dz = 0.0
        zero.dtheta = 0.0
        self._cmd_in_pub.publish(zero)
        self._cmd_out_pub.publish(zero)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    # ── Reboot path ──────────────────────────────────────────────────────

    def _maybe_reboot(self):
        now = self.get_clock().now()
        cooldown = float(self.get_parameter('reboot_cooldown_sec').value)
        if self._last_reboot_time is not None:
            elapsed = (now - self._last_reboot_time).nanoseconds * 1e-9
            if elapsed < cooldown:
                return

        if not self._reboot_client.service_is_ready():
            self.get_logger().warn('reboot service not ready, skipping auto-reboot',
                                   throttle_duration_sec=5.0)
            return

        self._last_reboot_time = now
        self._publish_status('rebooting')
        group = self.get_parameter('group_name').value
        self.get_logger().warn(f'Issuing smart_reboot on group "{group}"')

        req = Reboot.Request()
        req.cmd_type = 'group'
        req.name = self.get_parameter('group_name').value
        # smart_reboot only resets motors that are in an error state, so it
        # is safe to call on the full arm group when only one joint faults.
        req.smart_reboot = True
        req.enable = bool(self.get_parameter('reset_torque_after_reboot').value)

        future = self._reboot_client.call_async(req)
        future.add_done_callback(self._reboot_done)

    def _reboot_done(self, future):
        try:
            future.result()
        except Exception as ex:
            self.get_logger().error(f'reboot service call failed: {ex}')
            return
        self.get_logger().info('Reboot service returned; motor status check will verify')


def main(args=None):
    rclpy.init(args=args)
    node = EffortMotorWatchdog()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

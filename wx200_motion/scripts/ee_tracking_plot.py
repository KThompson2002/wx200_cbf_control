#!/usr/bin/env python3
"""
ee_tracking_plot.py — visualize Cartesian end-effector tracking error.

Records three things over a run and plots them:
  1. Inputted Cartesian velocity command (RelativeMove dx/dy/dz).
  2. PREDICTED EE position — the command integrated over time, seeded at the
     real EE pose at t0. This is "where you told it to go" (the RViz/ideal path).
  3. ACTUAL EE position — read from the TF tree (base_link -> ee_gripper_link),
     which is the FK of the measured joint angles, i.e. what the real arm did.

The gap between (2) and (3) is the Cartesian tracking error — the gravity /
PID lag we've been chasing, shown in task space instead of per-joint.

Run (workspace must be sourced):
    python3 ee_tracking_plot.py --ros-args \
        -p cmd_topic:=/wx200/cmd_vel \
        -p duration_sec:=0.0

Output defaults to the package img/ folder
(.../wx200_motion/img/ee_tracking.png and .csv); override with -p output:=...

Notes:
  * cmd_topic default is /wx200/cmd_vel (what the controller actually receives,
    after the CBF filter). Use /velocity_pub/vel_command to integrate the raw
    teleop input instead.
  * Stop with Ctrl-C (or set duration_sec > 0) to trigger the plot + CSV dump.
  * Prediction honors cmd_timeout_sec (default 0.4, matching the controller's
    command_timeout_sec) so a latched-but-stale command stops integrating when
    the controller would have stopped — keeping the comparison fair.
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from realtime_servo.msg import RelativeMove


def _stamp_path(path, stamp):
    """Insert `_<stamp>` before the file extension of `path`."""
    root, ext = os.path.splitext(path)
    return f'{root}_{stamp}{ext}'


class EETrackingPlot(Node):
    def __init__(self):
        super().__init__('ee_tracking_plot')

        self.declare_parameter('cmd_topic', '/wx200/cmd_vel')
        self.declare_parameter('base_frame', 'wx200/base_link')
        self.declare_parameter('ee_frame', 'wx200/ee_gripper_link')
        self.declare_parameter('sample_rate_hz', 50.0)
        self.declare_parameter('duration_sec', 0.0)        # 0 = until Ctrl-C
        self.declare_parameter('cmd_timeout_sec', 0.4)     # match controller
        self.declare_parameter(
            'output',
            '/home/kyle-thompson/FinalProject/ws/src/wx200_cbf_control/wx200_motion/img/ee_tracking.png')
        self.declare_parameter(
            'csv_output',
            '/home/kyle-thompson/FinalProject/ws/src/wx200_cbf_control/wx200_motion/img/ee_tracking.csv')

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.sample_rate = float(self.get_parameter('sample_rate_hz').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        # Insert a shared run timestamp before the extension so successive runs
        # don't clobber each other: ee_tracking_20260520_175200.png / .csv
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output = _stamp_path(self.get_parameter('output').value, stamp)
        self.csv_output = _stamp_path(self.get_parameter('csv_output').value, stamp)

        # Latest commanded Cartesian velocity (base frame) + arrival time.
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.last_cmd_time = None

        # Recorded samples.
        self.t = []                      # seconds since first sample
        self.vx, self.vy, self.vz = [], [], []
        self.px, self.py, self.pz = [], [], []   # predicted
        self.ax, self.ay, self.az = [], [], []   # actual (TF)

        self.predicted = None            # seeded from first actual TF reading
        self.t0 = None
        self.last_sample_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(RelativeMove, self.cmd_topic, self._cmd_cb, 50)
        self.timer = self.create_timer(1.0 / self.sample_rate, self._sample)

        self.get_logger().info(
            f"Recording: cmd='{self.cmd_topic}', TF '{self.base_frame}'->"
            f"'{self.ee_frame}' @ {self.sample_rate:.0f} Hz. Ctrl-C to plot.")

    def _cmd_cb(self, msg: RelativeMove):
        self.cmd_vel = (msg.dx, msg.dy, msg.dz)
        self.last_cmd_time = self.get_clock().now()

    def _lookup_ee(self):
        """Return (x, y, z) of ee_frame in base_frame, or None if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, Time())
            tr = tf.transform.translation
            return (tr.x, tr.y, tr.z)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF lookup {self.base_frame}->{self.ee_frame} failed: {e}",
                throttle_duration_sec=2.0)
            return None

    def _sample(self):
        actual = self._lookup_ee()
        if actual is None:
            return  # wait until TF is available before seeding

        now = self.get_clock().now()

        # Seed prediction at the real EE pose on the first good sample.
        if self.predicted is None:
            self.predicted = list(actual)
            self.t0 = now
            self.last_sample_time = now
            return

        dt = (now - self.last_sample_time).nanoseconds * 1e-9
        self.last_sample_time = now

        # Use the commanded velocity, but treat it as zero if the command has
        # gone stale (mirrors the controller's command_timeout_sec).
        vx, vy, vz = self.cmd_vel
        if self.last_cmd_time is None or \
           (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            vx, vy, vz = 0.0, 0.0, 0.0

        # Integrate predicted EE position.
        self.predicted[0] += vx * dt
        self.predicted[1] += vy * dt
        self.predicted[2] += vz * dt

        t_rel = (now - self.t0).nanoseconds * 1e-9
        self.t.append(t_rel)
        self.vx.append(vx); self.vy.append(vy); self.vz.append(vz)
        self.px.append(self.predicted[0]); self.py.append(self.predicted[1]); self.pz.append(self.predicted[2])
        self.ax.append(actual[0]); self.ay.append(actual[1]); self.az.append(actual[2])

        if self.duration_sec > 0.0 and t_rel >= self.duration_sec:
            self.get_logger().info("Duration reached; plotting.")
            self.finish()
            rclpy.shutdown()

    def finish(self):
        if not self.t:
            self.get_logger().error("No samples recorded — nothing to plot.")
            return
        self._dump_csv()
        self._plot()

    def _dump_csv(self):
        try:
            with open(self.csv_output, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'vx', 'vy', 'vz',
                            'pred_x', 'pred_y', 'pred_z',
                            'act_x', 'act_y', 'act_z',
                            'err_x', 'err_y', 'err_z'])
                for i in range(len(self.t)):
                    ex = self.px[i] - self.ax[i]
                    ey = self.py[i] - self.ay[i]
                    ez = self.pz[i] - self.az[i]
                    w.writerow([self.t[i], self.vx[i], self.vy[i], self.vz[i],
                                self.px[i], self.py[i], self.pz[i],
                                self.ax[i], self.ay[i], self.az[i],
                                ex, ey, ez])
            self.get_logger().info(f"Wrote data: {self.csv_output}")
        except OSError as e:
            self.get_logger().error(f"CSV write failed: {e}")

    def _plot(self):
        try:
            import matplotlib
            matplotlib.use('Agg')  # headless-safe; PNG always written
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().error(
                "matplotlib not installed (pip install matplotlib). "
                f"CSV is still at {self.csv_output}.")
            return

        ex = [p - a for p, a in zip(self.px, self.ax)]
        ey = [p - a for p, a in zip(self.py, self.ay)]
        ez = [p - a for p, a in zip(self.pz, self.az)]
        emag = [math.sqrt(a*a + b*b + c*c) for a, b, c in zip(ex, ey, ez)]

        fig, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=True)
        fig.suptitle('EE tracking: predicted (command) vs actual (TF)', fontsize=14)

        for ax, pred, act, lbl in (
                (axes[0], self.px, self.ax, 'X'),
                (axes[1], self.py, self.ay, 'Y'),
                (axes[2], self.pz, self.az, 'Z')):
            ax.plot(self.t, pred, '--', label=f'predicted {lbl}', color='tab:blue')
            ax.plot(self.t, act, '-', label=f'actual {lbl}', color='tab:red')
            ax.set_ylabel(f'{lbl} (m)')
            ax.legend(loc='upper left')
            ax.grid(True, alpha=0.3)

        axes[3].plot(self.t, ex, label='err X', color='tab:blue')
        axes[3].plot(self.t, ey, label='err Y', color='tab:green')
        axes[3].plot(self.t, ez, label='err Z', color='tab:red')
        axes[3].plot(self.t, emag, label='|err|', color='black', linewidth=2)
        axes[3].set_ylabel('error (m)')
        axes[3].legend(loc='upper left')
        axes[3].grid(True, alpha=0.3)

        axes[4].plot(self.t, self.vx, label='cmd vx', color='tab:blue')
        axes[4].plot(self.t, self.vy, label='cmd vy', color='tab:green')
        axes[4].plot(self.t, self.vz, label='cmd vz', color='tab:red')
        axes[4].set_ylabel('cmd vel (m/s)')
        axes[4].set_xlabel('time (s)')
        axes[4].legend(loc='upper left')
        axes[4].grid(True, alpha=0.3)

        fig.tight_layout(rect=[0, 0, 1, 0.98])
        try:
            fig.savefig(self.output, dpi=120)
            self.get_logger().info(f"Wrote plot: {self.output}  (max |err| = "
                                   f"{max(emag):.4f} m)")
        except OSError as e:
            self.get_logger().error(f"Plot save failed: {e}")


def main():
    rclpy.init()
    node = EETrackingPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C — plotting recorded data.")
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

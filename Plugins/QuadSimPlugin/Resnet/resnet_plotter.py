#!/usr/bin/env python3
import threading
from collections import deque

import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except Exception as e:  # pragma: no cover
    print("matplotlib is required for resnet_plotter. Try: pip install matplotlib")
    raise

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray


class ResNetPlotter(Node):
    def __init__(self):
        super().__init__('resnet_plotter')

        self.lock = threading.Lock()

        self.maxlen = 1000
        self.t = deque(maxlen=self.maxlen)

        # Velocities
        self.ref_v = deque(maxlen=self.maxlen)
        self.odom_v = deque(maxlen=self.maxlen)
        self.err_v = deque(maxlen=self.maxlen)

        # Residuals dx, dy, dz
        self.residual = deque(maxlen=self.maxlen)

        # Diagnostics: [step, avg_err, std_err, avg_out, std_out, train_active, ||W||, lr]
        self.diagnostics = deque(maxlen=self.maxlen)

        # Subs
        self.create_subscription(Odometry, '/quadsim/odom', self.cb_odom, 10)
        self.create_subscription(TwistStamped, '/ref/vel_local', self.cb_ref, 10)
        self.create_subscription(Float32MultiArray, '/resnet/cmd_vel_residual_vec', self.cb_residual, 10)
        self.create_subscription(Float32MultiArray, '/resnet/diagnostics', self.cb_diag, 10)

        self.current_ref = np.zeros(3)
        self.current_odom = np.zeros(3)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def cb_ref(self, msg: TwistStamped):
        with self.lock:
            self.current_ref = np.array([
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ], dtype=float)

    def cb_odom(self, msg: Odometry):
        with self.lock:
            self.current_odom = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ], dtype=float)
            t = self.now_sec()
            vref = self.current_ref.copy()
            vod = self.current_odom.copy()
            verr = vref - vod
            self.t.append(t)
            self.ref_v.append(vref)
            self.odom_v.append(vod)
            self.err_v.append(verr)

    def cb_residual(self, msg: Float32MultiArray):
        with self.lock:
            data = np.array(msg.data, dtype=float)
            if data.shape[0] >= 3:
                self.residual.append(data[:3])

    def cb_diag(self, msg: Float32MultiArray):
        with self.lock:
            data = np.array(msg.data, dtype=float)
            self.diagnostics.append(data)


def launch_plot(node: ResNetPlotter):
    fig = plt.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    ax1.set_title('Velocity Tracking (m/s)')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('vel')

    ax2.set_title('Error Magnitude and Per-Axis')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('error')

    ax3.set_title('Residual Correction (dx, dy, dz)')
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('m/s')

    ax4.set_title('Diagnostics (avg_err, avg_out, lr, train)')
    ax4.set_xlabel('time (s)')

    # Lines
    (r_x_line,) = ax1.plot([], [], 'r--', label='ref_x')
    (r_y_line,) = ax1.plot([], [], 'g--', label='ref_y')
    (r_z_line,) = ax1.plot([], [], 'b--', label='ref_z')
    (o_x_line,) = ax1.plot([], [], 'r-', label='odom_x')
    (o_y_line,) = ax1.plot([], [], 'g-', label='odom_y')
    (o_z_line,) = ax1.plot([], [], 'b-', label='odom_z')
    ax1.legend(loc='upper right')

    (e_mag_line,) = ax2.plot([], [], 'k-', label='|err|')
    (e_x_line,) = ax2.plot([], [], 'r-', label='ex')
    (e_y_line,) = ax2.plot([], [], 'g-', label='ey')
    (e_z_line,) = ax2.plot([], [], 'b-', label='ez')
    ax2.legend(loc='upper right')

    (dx_line,) = ax3.plot([], [], 'r-', label='dx')
    (dy_line,) = ax3.plot([], [], 'g-', label='dy')
    (dz_line,) = ax3.plot([], [], 'b-', label='dz')
    ax3.legend(loc='upper right')

    (avg_err_line,) = ax4.plot([], [], 'm-', label='avg_err')
    (avg_out_line,) = ax4.plot([], [], 'c-', label='avg_out')
    (lr_line,) = ax4.plot([], [], 'y-', label='lr')
    ax4_yr = ax4.twinx()
    (train_line,) = ax4_yr.plot([], [], 'k--', label='train_active')
    ax4.legend(loc='upper left')
    ax4_yr.legend(loc='upper right')

    def update(_):
        with node.lock:
            if len(node.t) < 5:
                return (
                    r_x_line,
                    r_y_line,
                    r_z_line,
                    o_x_line,
                    o_y_line,
                    o_z_line,
                    e_mag_line,
                    e_x_line,
                    e_y_line,
                    e_z_line,
                    dx_line,
                    dy_line,
                    dz_line,
                    avg_err_line,
                    avg_out_line,
                    lr_line,
                    train_line,
                )

            t = np.array(node.t)
            t = t - t[0]
            ref = np.array(node.ref_v)
            od = np.array(node.odom_v)
            err = np.array(node.err_v)
            res = np.array(node.residual) if len(node.residual) > 0 else np.zeros((0, 3))
            diag = np.array(node.diagnostics) if len(node.diagnostics) > 0 else np.zeros((0, 8))

        # Axis 1: velocities
        r_x_line.set_data(t[: len(ref)], ref[:, 0])
        r_y_line.set_data(t[: len(ref)], ref[:, 1])
        r_z_line.set_data(t[: len(ref)], ref[:, 2])
        o_x_line.set_data(t[: len(od)], od[:, 0])
        o_y_line.set_data(t[: len(od)], od[:, 1])
        o_z_line.set_data(t[: len(od)], od[:, 2])
        if len(ref) > 0:
            ax1.set_xlim(t[0], t[min(len(ref), len(od)) - 1])
            all_vals = np.concatenate([ref, od], axis=0)
            ymin, ymax = np.min(all_vals) - 0.1, np.max(all_vals) + 0.1
            if ymin == ymax:
                ymin -= 0.1
                ymax += 0.1
            ax1.set_ylim(ymin, ymax)

        # Axis 2: error
        if len(err) > 0:
            emag = np.linalg.norm(err, axis=1)
            e_mag_line.set_data(t[: len(err)], emag)
            e_x_line.set_data(t[: len(err)], err[:, 0])
            e_y_line.set_data(t[: len(err)], err[:, 1])
            e_z_line.set_data(t[: len(err)], err[:, 2])
            ax2.set_xlim(t[0], t[len(err) - 1])
            vals = np.concatenate([err, emag.reshape(-1, 1)], axis=1)
            ax2.set_ylim(np.min(vals) - 0.05, np.max(vals) + 0.05)

        # Axis 3: residual
        if len(res) > 0:
            n = len(res)
            tt = t[:n]
            dx_line.set_data(tt, res[:, 0])
            dy_line.set_data(tt, res[:, 1])
            dz_line.set_data(tt, res[:, 2])
            ax3.set_xlim(tt[0], tt[-1])
            ax3.set_ylim(np.min(res) - 0.05, np.max(res) + 0.05)

        # Axis 4: diagnostics
        if len(diag) > 0:
            n = len(diag)
            tt = t[:n]
            avg_err_line.set_data(tt, diag[:, 1])
            avg_out_line.set_data(tt, diag[:, 3])
            lr_line.set_data(tt, diag[:, 7])
            train_line.set_data(tt, diag[:, 5])
            ax4.set_xlim(tt[0], tt[-1])
            ax4.set_ylim(min(np.min(diag[:, 1]), np.min(diag[:, 3])) - 0.05,
                         max(np.max(diag[:, 1]), np.max(diag[:, 3])) + 0.05)

        return (
            r_x_line,
            r_y_line,
            r_z_line,
            o_x_line,
            o_y_line,
            o_z_line,
            e_mag_line,
            e_x_line,
            e_y_line,
            e_z_line,
            dx_line,
            dy_line,
            dz_line,
            avg_err_line,
            avg_out_line,
            lr_line,
            train_line,
        )

    ani = FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()


def main():  # pragma: no cover
    rclpy.init()
    node = ResNetPlotter()

    exec = rclpy.executors.MultiThreadedExecutor()
    exec.add_node(node)

    spin_thread = threading.Thread(target=exec.spin, daemon=True)
    spin_thread.start()

    try:
        launch_plot(node)
    finally:
        exec.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()


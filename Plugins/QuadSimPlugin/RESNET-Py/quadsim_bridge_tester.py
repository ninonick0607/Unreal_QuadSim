#!/usr/bin/env python3
# quadsim_bridge_tester.py
#
# A single-node ROS 2 (rclpy) harness to validate Unreal's pub/sub/service map.
# - Subscribes: /clock, /tf, /tf_static, /quadsim/odom, /quadsim/imu
# - Publishes:  /resnet/cmd_rate (geometry_msgs/TwistStamped)
# - Services:   /quadsim/pause (std_srvs/SetBool), /quadsim/reset (std_srvs/Trigger), /quadsim/step (std_srvs/Trigger)
#
# Usage (ROS 2 Humble+ recommended):
#   chmod +x quadsim_bridge_tester.py
#   ros2 run (via a package) OR: ./quadsim_bridge_tester.py --help
#
# Quick direct run (no package):
#   python3 quadsim_bridge_tester.py --hz 50 --pub --seq
#
# Notes:
#   - QoS is set to match Unreal's expectations: sensors use BestEffort depth=1,
#     tf_static subscription uses TRANSIENT_LOCAL durability to receive latched transforms.
#   - The publisher for /resnet/cmd_rate uses RELIABLE depth=1 to match Unreal's subscriber.
#
import math
import signal
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Messages
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

# Services
from std_srvs.srv import SetBool, Trigger

@dataclass
class Counters:
    clock: int = 0
    tf: int = 0
    tf_static: int = 0
    odom: int = 0
    imu: int = 0
    sent_cmd: int = 0
    step_calls: int = 0
    reset_calls: int = 0
    pause_calls: int = 0

class QuadSimTester(Node):
    def __init__(self, hz_cmd: float = 50.0, do_pub: bool = True, do_seq: bool = False):
        super().__init__("quadsim_bridge_tester")

        # QoS profiles
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latched
        )
        self.reliable1_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers (Unreal publishers)
        self.counters = Counters()
        self.last_log_time = self.get_clock().now()

        self.sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.sub_tf = self.create_subscription(TFMessage, "/tf", self._on_tf, self.tf_qos)
        self.sub_tf_static = self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, self.tf_static_qos)
        self.sub_odom = self.create_subscription(Odometry, "/quadsim/odom", self._on_odom, self.sensor_qos)
        self.sub_imu = self.create_subscription(Imu, "/quadsim/imu", self._on_imu, self.sensor_qos)

        # Publisher (Unreal subscriber)
        self.do_pub = do_pub
        if self.do_pub:
            self.cmd_pub = self.create_publisher(TwistStamped, "/resnet/cmd_rate", self.reliable1_qos)
            period = 1.0 / max(1.0, float(hz_cmd))
            self.pub_timer = self.create_timer(period, self._pub_cmd)
            self.get_logger().info(f"Publishing /resnet/cmd_rate at {1.0/period:.1f} Hz (reliable, depth=1)")

        # Service clients (Unreal service servers)
        self.cli_pause = self.create_client(SetBool, "/quadsim/pause")
        self.cli_reset = self.create_client(Trigger, "/quadsim/reset")
        self.cli_step  = self.create_client(Trigger, "/quadsim/step")

        self.do_seq = do_seq
        if self.do_seq:
            # simple sequence timer
            self.seq_stage = 0
            self.seq_timer = self.create_timer(1.0, self._sequence_tick)

        # periodic status log
        self.status_timer = self.create_timer(2.0, self._print_status)

    # ---- Sub callbacks ----
    def _on_clock(self, msg: Clock):
        self.counters.clock += 1

    def _on_tf(self, msg: TFMessage):
        self.counters.tf += 1

    def _on_tf_static(self, msg: TFMessage):
        self.counters.tf_static += 1

    def _on_odom(self, msg: Odometry):
        self.counters.odom += 1
        # throttle a small sample print
        if self.counters.odom % 100 == 0:
            p = msg.pose.pose.position
            self.get_logger().info(f"odom sample x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")

    def _on_imu(self, msg: Imu):
        self.counters.imu += 1
        # throttle
        if self.counters.imu % 200 == 0:
            wz = msg.angular_velocity.z
            self.get_logger().info(f"imu sample wz={wz:.3f} rad/s")

    # ---- Publisher ----
    def _pub_cmd(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # simple command: small yaw rate chirp and zero thrust by default
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.5 * math.sin(0.5 * t)  # rad/s
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0  # set if you use thrust here (e.g., 0..1)

        self.cmd_pub.publish(msg)
        self.counters.sent_cmd += 1

    # ---- Simple test sequence (optional) ----
    def _sequence_tick(self):
        # seq: unpause -> send commands -> reset -> step -> pause
        if self.seq_stage == 0:
            self._call_pause(False)  # unpause
            self.seq_stage = 1
        elif self.seq_stage == 1:
            # let publisher run for a few seconds
            if self.counters.sent_cmd > 200:
                self.seq_stage = 2
        elif self.seq_stage == 2:
            self._call_reset()
            self.seq_stage = 3
        elif self.seq_stage == 3:
            # try a few lockstep steps
            self._call_step()
            if self.counters.step_calls >= 5:
                self.seq_stage = 4
        elif self.seq_stage == 4:
            self._call_pause(True)  # pause
            self.get_logger().info("Sequence complete. You can Ctrl-C to exit.")
            self.seq_stage = 5
        else:
            pass

    # ---- Service helpers ----
    def _wait_for(self, client, name: str, timeout_s: float = 2.0):
        if client.wait_for_service(timeout_s):
            return True
        self.get_logger().warn(f"Service {name} not available (timeout {timeout_s:.1f}s).")
        return False

    def _call_pause(self, data: bool):
        if not self._wait_for(self.cli_pause, "/quadsim/pause"):
            return
        req = SetBool.Request()
        req.data = bool(data)
        fut = self.cli_pause.call_async(req)
        fut.add_done_callback(lambda f: self._on_pause_done(f, data))

    def _on_pause_done(self, fut, data):
        self.counters.pause_calls += 1
        try:
            res = fut.result()
            self.get_logger().info(f"pause({data}) -> success={res.success} msg='{res.message}'")
        except Exception as e:
            self.get_logger().error(f"pause({data}) call failed: {e!r}")

    def _call_reset(self):
        if not self._wait_for(self.cli_reset, "/quadsim/reset"):
            return
        req = Trigger.Request()
        fut = self.cli_reset.call_async(req)
        fut.add_done_callback(self._on_reset_done)

    def _on_reset_done(self, fut):
        self.counters.reset_calls += 1
        try:
            res = fut.result()
            self.get_logger().info(f"reset() -> success={res.success} msg='{res.message}'")
        except Exception as e:
            self.get_logger().error(f"reset() call failed: {e!r}")

    def _call_step(self):
        if not self._wait_for(self.cli_step, "/quadsim/step"):
            return
        req = Trigger.Request()
        fut = self.cli_step.call_async(req)
        fut.add_done_callback(self._on_step_done)

    def _on_step_done(self, fut):
        self.counters.step_calls += 1
        try:
            res = fut.result()
            self.get_logger().info(f"step() -> success={res.success} msg='{res.message}'")
        except Exception as e:
            self.get_logger().error(f"step() call failed: {e!r}")

    # ---- Status ----
    def _print_status(self):
        now = self.get_clock().now()
        dt = (now - self.last_log_time).nanoseconds * 1e-9
        self.last_log_time = now
        self.get_logger().info(
            "rx{clk=%d tf=%d tf_static=%d odom=%d imu=%d}  tx{cmd=%d}  svc{pause=%d reset=%d step=%d} (last %.1fs)"
            % (self.counters.clock, self.counters.tf, self.counters.tf_static,
               self.counters.odom, self.counters.imu,
               self.counters.sent_cmd,
               self.counters.pause_calls, self.counters.reset_calls, self.counters.step_calls, dt)
        )

def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="QuadSim Unreal pub/sub/service integration tester")
    parser.add_argument("--hz", type=float, default=50.0, help="Publish rate for /resnet/cmd_rate (Hz)")
    parser.add_argument("--pub", action="store_true", help="Publish /resnet/cmd_rate commands")
    parser.add_argument("--seq", action="store_true", help="Run a small service sequence (unpause->cmds->reset->step->pause)")
    args = parser.parse_args()

    rclpy.init(args=None)

    node = QuadSimTester(hz_cmd=args.hz, do_pub=args.pub, do_seq=args.seq)

    # Graceful Ctrl-C
    def _sigint_handler(sig, frame):
        node.get_logger().info("Shutting down...")
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

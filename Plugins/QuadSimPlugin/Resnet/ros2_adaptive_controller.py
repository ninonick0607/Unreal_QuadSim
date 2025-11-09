#!/usr/bin/env python3
"""
ROS2 Adaptive ResNet Controller for QuadSim
Online training for adaptive velocity control corrections
"""

import numpy as np
from typing import Optional, Dict, Any
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Message imports
from geometry_msgs.msg import TwistStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32MultiArray, Header
from rosgraph_msgs.msg import Clock

# Service imports
from std_srvs.srv import SetBool, Trigger

# Local imports
from src.core.neural_network import NeuralNetwork


class AdaptiveResNetController(Node):
    """
    ROS2 node for online adaptive control using ResNet
    Subscribes to drone odometry and publishes velocity corrections
    """

    def __init__(self):
        super().__init__('adaptive_resnet_controller')

        # Initialize state variables
        self.current_odom: Optional[Odometry] = None
        self.current_imu: Optional[Imu] = None
        self.reference_velocity: Optional[TwistStamped] = None
        self.baseline_cmd: Optional[TwistStamped] = None
        self.last_odom_time = 0.0
        self.sim_time = 0.0  # Simulation time from /clock
        self.time_step = 0
        self.training_active = False
        self.controller_enabled = True

        # Thread safety (re-entrant to avoid deadlocks with NN input_func)
        self.state_lock = threading.RLock()

        # Declare and get parameters
        self.setup_parameters()
        self.load_parameters()

        # Initialize neural network
        self.initialize_neural_network()

        # Setup QoS profiles (matching your C++ side)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # rclUE in this project publishes /clock as RELIABLE + VOLATILE.
        # Match publisher (RELIABLE, VOLATILE) to avoid durability mismatch.
        qos_clock_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create subscribers (matching your AROS2Controller topics)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/quadsim/odom',
            self.odom_callback,
            qos_best_effort  # Use best_effort to match Unreal Engine publisher
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/quadsim/imu',
            self.imu_callback,
            qos_best_effort
        )

        self.ref_vel_sub = self.create_subscription(
            TwistStamped,
            '/ref/vel_local',
            self.ref_vel_callback,
            qos_reliable
        )

        # Optional: Subscribe to baseline controller for enhancement
        self.baseline_cmd_sub = self.create_subscription(
            TwistStamped,
            '/baseline/cmd_vel',
            self.baseline_cmd_callback,
            qos_reliable
        )

        # Clock subscription for simulation time
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            qos_clock_reliable
        )

        # Create publishers
        self.resnet_cmd_pub = self.create_publisher(
            TwistStamped,
            '/resnet/cmd_vel_residual',
            qos_reliable
        )

        # Convenience mirror for easy echoing of corrections as a vector
        self.resnet_cmd_vec_pub = self.create_publisher(
            Float32MultiArray,
            '/resnet/cmd_vel_residual_vec',
            qos_reliable
        )

        # Publish desired velocity (matching UpdateDesiredVel from C++)
        self.desired_vel_pub = self.create_publisher(
            TwistStamped,
            '/ref/vel_local',  # Same topic we subscribe to - can publish reference if needed
            qos_reliable
        )

        # Make diagnostics/status RELIABLE so `ros2 topic echo` sees them by default
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray,
            '/resnet/diagnostics',
            qos_reliable
        )

        self.status_pub = self.create_publisher(
            String,
            '/resnet/status',
            qos_reliable
        )

        # Create services
        self.training_srv = self.create_service(
            SetBool,
            '/resnet/enable_training',
            self.handle_training_toggle
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/resnet/reset_weights',
            self.handle_reset_weights
        )

        # Additional services to integrate with simulation control
        self.sim_pause_client = self.create_client(SetBool, '/pause')
        self.sim_reset_client = self.create_client(Trigger, '/reset')
        self.sim_step_client = self.create_client(Trigger, '/step')

        # Create main control timer (100Hz to match your odom rate)
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_and_training_callback
        )

        # Create diagnostics timer (10Hz)
        self.diagnostics_timer = self.create_timer(
            0.1,
            self.publish_diagnostics
        )

        # Debug/QoS visibility: dump endpoint info once after startup and then every 5s
        self._dbg_qos_logged_once = False
        self.qos_debug_timer = self.create_timer(
            5.0,
            self._qos_debug_tick
        )

        # Training metrics
        self.tracking_errors = []
        self.nn_outputs = []
        self.training_loss_history = []
        self.prev_residual = np.zeros(3)

        self.get_logger().info('Adaptive ResNet Controller initialized')
        self.publish_status('Initialized - Waiting for data')
        self._state_logged_valid = False
        self._publish_count = 0
        # Periodic status heartbeat (1 Hz)
        self.status_heartbeat_timer = self.create_timer(
            1.0,
            lambda: self.publish_status(f'Heartbeat: pubs={self._publish_count}')
        )

    def setup_parameters(self):
        """Declare all ROS2 parameters"""
        # Neural Network Architecture
        self.declare_parameter('resnet.num_blocks', 2)
        self.declare_parameter('resnet.num_layers', 3)
        self.declare_parameter('resnet.num_neurons', 20)
        self.declare_parameter('resnet.input_size', 15)  # pos(3) + vel(3) + ref_vel(3) + error(3) + angles(3)
        self.declare_parameter('resnet.output_size', 3)  # velocity corrections [vx, vy, vz]

        # Activation functions
        self.declare_parameter('resnet.inner_activation', 'tanh')
        self.declare_parameter('resnet.output_activation', 'identity')
        self.declare_parameter('resnet.shortcut_activation', 'identity')

        # Learning parameters
        self.declare_parameter('learning.initial_rate', 0.001)
        self.declare_parameter('learning.min_singular_value', 0.1)
        self.declare_parameter('learning.max_singular_value', 10.0)
        self.declare_parameter('learning.weight_bounds', 2.0)

        # Control parameters
        self.declare_parameter('control.rate_hz', 100.0)
        self.declare_parameter('control.max_velocity_correction', 0.5)  # m/s (XY cap)
        self.declare_parameter('control.max_velocity_correction_z', 0.3)  # m/s (Z cap)
        self.declare_parameter('control.baseline_gain', 1.0)
        self.declare_parameter('control.mix_source', 'baseline')  # 'reference' or 'baseline'
        self.declare_parameter('control.residual_gain', 0.3)
        self.declare_parameter('control.residual_smoothing_alpha', 0.2)  # 0..1
        self.declare_parameter('control.output_ramp_steps', 300)  # gradual ramp-in for residuals

        # Training control
        self.declare_parameter('training.enabled_on_start', False)
        self.declare_parameter('training.warmup_steps', 100)
        self.declare_parameter('training.min_error_threshold', 0.01)
        self.declare_parameter('training.max_error_threshold', 2.0)
        self.declare_parameter('training.learning_decay_rate', 0.999)

        # Safety parameters
        self.declare_parameter('safety.max_output_magnitude', 1.0)
        self.declare_parameter('safety.emergency_stop_error', 5.0)

        # Note: use_sim_time is automatically declared by ROS2 during node initialization

    def load_parameters(self):
        """Load parameters into instance variables"""
        self.control_rate_hz = self.get_parameter('control.rate_hz').value
        self.max_velocity_correction = self.get_parameter('control.max_velocity_correction').value
        self.max_velocity_correction_z = self.get_parameter('control.max_velocity_correction_z').value
        self.baseline_gain = self.get_parameter('control.baseline_gain').value
        self.mix_source = str(self.get_parameter('control.mix_source').value).lower()
        self.residual_gain = float(self.get_parameter('control.residual_gain').value)
        self.residual_smoothing_alpha = float(self.get_parameter('control.residual_smoothing_alpha').value)
        self.output_ramp_steps = int(self.get_parameter('control.output_ramp_steps').value)

        self.training_enabled_on_start = self.get_parameter('training.enabled_on_start').value
        self.warmup_steps = self.get_parameter('training.warmup_steps').value
        self.min_error_threshold = self.get_parameter('training.min_error_threshold').value
        self.max_error_threshold = self.get_parameter('training.max_error_threshold').value
        self.learning_decay_rate = self.get_parameter('training.learning_decay_rate').value

        self.max_output_magnitude = self.get_parameter('safety.max_output_magnitude').value
        self.emergency_stop_error = self.get_parameter('safety.emergency_stop_error').value

        self.use_sim_time = self.get_parameter('use_sim_time').value

    def initialize_neural_network(self):
        """Initialize the ResNet with ROS2 parameters"""
        # Create configuration for neural network
        config = {
            'num_blocks': self.get_parameter('resnet.num_blocks').value,
            'num_layers': self.get_parameter('resnet.num_layers').value,
            'num_neurons': self.get_parameter('resnet.num_neurons').value,
            'output_size': self.get_parameter('resnet.output_size').value,
            'inner_activation': self.get_parameter('resnet.inner_activation').value,
            'output_activation': self.get_parameter('resnet.output_activation').value,
            'shortcut_activation': self.get_parameter('resnet.shortcut_activation').value,
            'initial_learning_rate': self.get_parameter('learning.initial_rate').value,
            'minimum_singular_value': self.get_parameter('learning.min_singular_value').value,
            'maximum_singular_value': self.get_parameter('learning.max_singular_value').value,
            'weight_bounds': self.get_parameter('learning.weight_bounds').value,
            'time_step_delta': 1.0 / self.control_rate_hz,
            'final_time': 2.0,  # 2 seconds = 200 steps at 100Hz (circular buffer wraps around indefinitely)
            'seed': 42
        }

        # Define input function for neural network
        def nn_input_func(step: int) -> np.ndarray:
            """Create neural network input from current state"""
            with self.state_lock:
                if not self.has_valid_state():
                    return np.zeros(self.get_parameter('resnet.input_size').value)

                # Extract position
                pos = np.array([
                    self.current_odom.pose.pose.position.x,
                    self.current_odom.pose.pose.position.y,
                    self.current_odom.pose.pose.position.z
                ])

                # Extract velocity
                vel = np.array([
                    self.current_odom.twist.twist.linear.x,
                    self.current_odom.twist.twist.linear.y,
                    self.current_odom.twist.twist.linear.z
                ])

                # Extract reference velocity
                if self.reference_velocity:
                    ref_vel = np.array([
                        self.reference_velocity.twist.linear.x,
                        self.reference_velocity.twist.linear.y,
                        self.reference_velocity.twist.linear.z
                    ])
                else:
                    ref_vel = np.zeros(3)

                # Compute velocity error
                vel_error = ref_vel - vel

                # Extract orientation (roll, pitch, yaw from quaternion)
                if self.current_imu:
                    # Simplified - you might want proper quaternion to euler conversion
                    orientation = np.array([
                        self.current_imu.orientation.x,
                        self.current_imu.orientation.y,
                        self.current_imu.orientation.z
                    ])
                else:
                    orientation = np.zeros(3)

                # Concatenate all inputs
                return np.concatenate([pos, vel, ref_vel, vel_error, orientation])

        # Initialize neural network
        self.neural_network = NeuralNetwork(nn_input_func, config)
        self.get_logger().info(f'Neural network initialized with {np.size(self.neural_network.weights)} parameters')
        self.get_logger().info(f'Buffer allocated for {self.neural_network.time_steps} timesteps ({config["final_time"]:.1f} seconds at {self.control_rate_hz} Hz)')

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        if self.time_step < 5:
            self.get_logger().info('Received odometry message')
        with self.state_lock:
            self.current_odom = msg
            # Use simulation time if available, otherwise wall time
            if self.use_sim_time and self.sim_time > 0:
                self.last_odom_time = self.sim_time
            else:
                self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg: Imu):
        """Handle IMU updates"""
        if self.time_step < 5:
            self.get_logger().info('Received IMU message')
        with self.state_lock:
            self.current_imu = msg

    def ref_vel_callback(self, msg: TwistStamped):
        """Handle reference velocity updates"""
        if self.time_step < 5:
            self.get_logger().info('Received reference velocity message')
        with self.state_lock:
            self.reference_velocity = msg

    def baseline_cmd_callback(self, msg: TwistStamped):
        """Handle baseline controller commands"""
        with self.state_lock:
            self.baseline_cmd = msg

    def clock_callback(self, msg: Clock):
        """Handle simulation clock updates"""
        if self.use_sim_time:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _qos_debug_tick(self):
        """Periodically log QoS/endpoint connection info for key topics"""
        try:
            odom_pubs = self.get_publishers_info_by_topic('/quadsim/odom')
            imu_pubs = self.get_publishers_info_by_topic('/quadsim/imu')
            ref_pubs = self.get_publishers_info_by_topic('/ref/vel_local')
            clk_pubs = self.get_publishers_info_by_topic('/clock')
            resnet_subs = self.get_subscriptions_info_by_topic('/resnet/cmd_vel_residual')
            diag_subs = self.get_subscriptions_info_by_topic('/resnet/diagnostics')
            status_subs = self.get_subscriptions_info_by_topic('/resnet/status')

            self.get_logger().info(
                f"[QoS] pubs odom/imu/ref/clock: {len(odom_pubs)}/{len(imu_pubs)}/{len(ref_pubs)}/{len(clk_pubs)}; "
                f"subs resnet/diag/status: {len(resnet_subs)}/{len(diag_subs)}/{len(status_subs)}"
            )

            # Log one-time detailed info early after startup
            if not self._dbg_qos_logged_once:
                def fmt_endpoints(endpoints):
                    return [
                        f"{e.node_name}({e.node_namespace}) rel={e.qos_profile.reliability.name} dur={e.qos_profile.durability.name}"
                        for e in endpoints
                    ]
                self.get_logger().info(f"[QoS Detail] /clock pubs: {fmt_endpoints(clk_pubs)}")
                self.get_logger().info(f"[QoS Detail] /quadsim/odom pubs: {fmt_endpoints(odom_pubs)}")
                self.get_logger().info(f"[QoS Detail] /quadsim/imu pubs: {fmt_endpoints(imu_pubs)}")
                self.get_logger().info(f"[QoS Detail] /ref/vel_local pubs: {fmt_endpoints(ref_pubs)}")
                self.get_logger().info(f"[QoS Detail] /resnet/cmd_vel_residual subs: {fmt_endpoints(resnet_subs)}")
                self._dbg_qos_logged_once = True
        except Exception as e:
            self.get_logger().warn(f"QoS debug failed: {e}")

    def has_valid_state(self) -> bool:
        """Check if we have valid state for control"""
        # Basic requirement: must have received odometry and reference velocity
        if self.current_odom is None or self.reference_velocity is None:
            return False

        # Check data freshness (max 0.1 seconds old)
        # Skip freshness check if we haven't started yet (both times are 0)
        if self.last_odom_time == 0:
            return True

        if self.use_sim_time:
            current_time = self.sim_time
        else:
            current_time = self.get_clock().now().nanoseconds / 1e9

        data_age = current_time - self.last_odom_time
        ok = data_age < 0.1
        if ok and not self._state_logged_valid:
            self.get_logger().info('State is valid: control loop active')
            self._state_logged_valid = True
        return ok

    def compute_velocity_error(self) -> np.ndarray:
        """Compute velocity tracking error"""
        if not self.has_valid_state():
            return np.zeros(3)

        current_vel = np.array([
            self.current_odom.twist.twist.linear.x,
            self.current_odom.twist.twist.linear.y,
            self.current_odom.twist.twist.linear.z
        ])

        ref_vel = np.array([
            self.reference_velocity.twist.linear.x,
            self.reference_velocity.twist.linear.y,
            self.reference_velocity.twist.linear.z
        ])

        return ref_vel - current_vel

    def should_train(self, error_magnitude: float) -> bool:
        """Determine if training should be active"""
        # Check warmup period
        if self.time_step < self.warmup_steps:
            return False

        # Check if training is enabled
        if not self.training_active:
            return False

        # Safety check - disable if error too large
        if error_magnitude > self.max_error_threshold:
            self.get_logger().warn(f'Training disabled: error {error_magnitude:.3f} > max {self.max_error_threshold}')
            return False

        # Don't train if already tracking well
        if error_magnitude < self.min_error_threshold:
            return False

        return True

    def apply_safety_limits(self, correction: np.ndarray) -> np.ndarray:
        """Apply safety limits to ResNet output"""
        # Per-axis caps (tighter Z)
        correction[0] = float(np.clip(correction[0], -self.max_velocity_correction, self.max_velocity_correction))
        correction[1] = float(np.clip(correction[1], -self.max_velocity_correction, self.max_velocity_correction))
        correction[2] = float(np.clip(correction[2], -self.max_velocity_correction_z, self.max_velocity_correction_z))

        # Limit total magnitude
        magnitude = np.linalg.norm(correction)
        if magnitude > self.max_output_magnitude:
            correction = correction * (self.max_output_magnitude / magnitude)

        return correction

    def control_and_training_callback(self):
        """Main control loop with online training"""
        with self.state_lock:
            if not self.has_valid_state():
                # Debug: log why state is invalid (only first 10 times)
                if self.time_step < 10:
                    self.get_logger().warn(
                        f'Invalid state: odom={self.current_odom is not None}, '
                        f'imu={self.current_imu is not None}, '
                        f'ref_vel={self.reference_velocity is not None}'
                    )
                return

            if not self.controller_enabled:
                # Failsafe: keep Unreal's control loop alive by pass-through publishing
                base_x = (self.baseline_cmd.twist.linear.x if (self.mix_source == 'baseline' and self.baseline_cmd)
                          else self.reference_velocity.twist.linear.x)
                base_y = (self.baseline_cmd.twist.linear.y if (self.mix_source == 'baseline' and self.baseline_cmd)
                          else self.reference_velocity.twist.linear.y)
                base_z = (self.baseline_cmd.twist.linear.z if (self.mix_source == 'baseline' and self.baseline_cmd)
                          else self.reference_velocity.twist.linear.z)

                cmd_msg = TwistStamped()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.header.frame_id = 'base_link'
                cmd_msg.twist.linear.x = base_x
                cmd_msg.twist.linear.y = base_y
                cmd_msg.twist.linear.z = base_z
                self.resnet_cmd_pub.publish(cmd_msg)
                # Update vector mirror as zero residual
                vec = Float32MultiArray(); vec.data = [0.0, 0.0, 0.0]
                self.resnet_cmd_vec_pub.publish(vec)
                if (self.time_step % 50) == 0:
                    self.get_logger().warn('Controller disabled; publishing pass-through base command')
                return

            # Increment time step
            self.time_step += 1

            # Compute velocity error
            velocity_error = self.compute_velocity_error()
            error_magnitude = np.linalg.norm(velocity_error)

            # Check for emergency stop
            if error_magnitude > self.emergency_stop_error:
                self.get_logger().error(f'Emergency stop: error {error_magnitude:.3f} > limit {self.emergency_stop_error}')
                self.controller_enabled = False
                self.publish_status(f'EMERGENCY STOP - Error: {error_magnitude:.3f}')
                return

            # Determine if we should train
            train_this_step = self.should_train(error_magnitude)

            # Neural network forward pass (with or without training)
            if train_this_step:
                # Training step - update weights
                # Use modulo to wrap around buffer (circular buffer approach)
                buffer_step = self.time_step % len(self.neural_network.learning_rate)
                nn_output = self.neural_network.train_step(
                    buffer_step,
                    velocity_error.reshape(-1, 1)
                )
                self.training_loss_history.append(error_magnitude)

                # Apply learning rate decay
                if self.time_step % 100 == 0:
                    current_lr = self.neural_network.learning_rate[buffer_step, 0, 0]
                    new_lr = current_lr * self.learning_decay_rate
                    # Decay all learning rates in buffer
                    self.neural_network.learning_rate *= self.learning_decay_rate
            else:
                # Inference only - no weight updates
                # Use the network's internal input_func; predict requires a step index
                buffer_step = self.time_step % self.neural_network.time_steps
                nn_output = self.neural_network.predict(buffer_step)

            # Reshape output and apply shaping
            resnet_correction = nn_output.flatten()[:3]  # Ensure we get 3 values
            # Overall residual gain
            resnet_correction = self.residual_gain * resnet_correction

            # Error-based gating: suppress when too small/large
            if error_magnitude < self.min_error_threshold:
                gate = 0.0
            elif error_magnitude > self.max_error_threshold:
                if error_magnitude >= self.emergency_stop_error:
                    gate = 0.0
                else:
                    gate = max(0.0, 1.0 - (error_magnitude - self.max_error_threshold) /
                               max(1e-6, (self.emergency_stop_error - self.max_error_threshold)))
            else:
                gate = 1.0
            resnet_correction *= gate

            # Apply output ramp-in during initial steps
            ramp = 1.0 if self.time_step >= self.output_ramp_steps else max(0.0, float(self.time_step) / max(1, self.output_ramp_steps))
            resnet_correction = resnet_correction * ramp

            # Low-pass filter on residual to reduce oscillations
            a = float(np.clip(self.residual_smoothing_alpha, 0.0, 1.0))
            resnet_correction = a * self.prev_residual + (1.0 - a) * resnet_correction
            self.prev_residual = resnet_correction.copy()

            # Apply safety limits
            resnet_correction = self.apply_safety_limits(resnet_correction)

            # Store metrics
            self.tracking_errors.append(error_magnitude)
            self.nn_outputs.append(np.linalg.norm(resnet_correction))

            # Create and publish command
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'

            # Choose mix source according to parameter
            use_baseline = (self.mix_source == 'baseline') and (self.baseline_cmd is not None)
            if use_baseline:
                base_x = self.baseline_cmd.twist.linear.x
                base_y = self.baseline_cmd.twist.linear.y
                base_z = self.baseline_cmd.twist.linear.z
            else:
                base_x = self.reference_velocity.twist.linear.x
                base_y = self.reference_velocity.twist.linear.y
                base_z = self.reference_velocity.twist.linear.z

            cmd_msg.twist.linear.x = base_x + resnet_correction[0]
            cmd_msg.twist.linear.y = base_y + resnet_correction[1]
            cmd_msg.twist.linear.z = base_z + resnet_correction[2]

            # Copy angular velocities if available
            if self.baseline_cmd:
                cmd_msg.twist.angular = self.baseline_cmd.twist.angular

            # Publish command
            self.resnet_cmd_pub.publish(cmd_msg)
            self._publish_count += 1
            # Publish vector mirror for easy CLI echo
            vec = Float32MultiArray()
            if use_baseline:
                src_x, src_y, src_z = (
                    self.baseline_cmd.twist.linear.x,
                    self.baseline_cmd.twist.linear.y,
                    self.baseline_cmd.twist.linear.z,
                )
            else:
                src_x, src_y, src_z = (
                    self.reference_velocity.twist.linear.x,
                    self.reference_velocity.twist.linear.y,
                    self.reference_velocity.twist.linear.z,
                )
            vec.data = [
                float(cmd_msg.twist.linear.x - src_x),
                float(cmd_msg.twist.linear.y - src_y),
                float(cmd_msg.twist.linear.z - src_z),
            ]
            self.resnet_cmd_vec_pub.publish(vec)
            if self._publish_count <= 5 or (self._publish_count % 100) == 0:
                self.get_logger().info(
                    f"Published /resnet/cmd_vel_residual [#{self._publish_count}] v=({cmd_msg.twist.linear.x:.3f},"
                    f" {cmd_msg.twist.linear.y:.3f}, {cmd_msg.twist.linear.z:.3f})"
                )

            # Log progress every 100 steps
            if self.time_step % 100 == 0:
                avg_error = np.mean(self.tracking_errors[-100:]) if len(self.tracking_errors) >= 100 else error_magnitude
                avg_output = np.mean(self.nn_outputs[-100:]) if len(self.nn_outputs) >= 100 else np.linalg.norm(resnet_correction)
                self.get_logger().info(
                    f'Step {self.time_step}: Avg Error: {avg_error:.4f}, '
                    f'Avg Output: {avg_output:.4f}, Training: {train_this_step}'
                )

    def publish_diagnostics(self):
        """Publish diagnostics information"""
        if len(self.tracking_errors) == 0:
            return

        # Prepare diagnostics message
        diag_msg = Float32MultiArray()

        # Calculate metrics
        recent_errors = self.tracking_errors[-100:] if len(self.tracking_errors) >= 100 else self.tracking_errors
        recent_outputs = self.nn_outputs[-100:] if len(self.nn_outputs) >= 100 else self.nn_outputs

        # Use modulo to safely index into circular buffer
        buffer_step = self.time_step % len(self.neural_network.learning_rate)

        diagnostics_data = [
            float(self.time_step),                    # 0: Current time step
            float(np.mean(recent_errors)),            # 1: Average tracking error
            float(np.std(recent_errors)),             # 2: Std dev of tracking error
            float(np.mean(recent_outputs)),           # 3: Average NN output magnitude
            float(np.std(recent_outputs)),            # 4: Std dev of NN output
            float(self.training_active),              # 5: Training active flag
            float(np.linalg.norm(self.neural_network.weights)),  # 6: Weights norm
            float(self.neural_network.learning_rate[buffer_step, 0, 0]),  # 7: Current learning rate
        ]

        diag_msg.data = diagnostics_data
        self.diagnostics_pub.publish(diag_msg)

    def publish_status(self, status: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[Step {self.time_step}] {status}'
        self.status_pub.publish(msg)

    def handle_training_toggle(self, request, response):
        """Service handler for enabling/disabling training"""
        self.training_active = request.data

        if self.training_active:
            self.get_logger().info('Training ENABLED')
            self.publish_status('Training enabled')
            # Reset training metrics
            if len(self.training_loss_history) > 1000:
                self.training_loss_history = self.training_loss_history[-1000:]
        else:
            self.get_logger().info('Training DISABLED')
            self.publish_status('Training disabled')

        response.success = True
        response.message = f'Training {"enabled" if self.training_active else "disabled"}'
        return response

    def handle_reset_weights(self, request, response):
        """Service handler for resetting neural network weights"""
        self.get_logger().info('Resetting neural network weights')

        # Reinitialize the neural network
        self.initialize_neural_network()

        # Reset metrics
        self.time_step = 0
        self.tracking_errors = []
        self.nn_outputs = []
        self.training_loss_history = []

        response.success = True
        response.message = 'Neural network weights reset successfully'
        self.publish_status('Weights reset')

        return response

    async def reset_simulation(self):
        """Call the simulation reset service"""
        if not self.sim_reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Simulation reset service not available')
            return False

        request = Trigger.Request()
        future = self.sim_reset_client.call_async(request)

        try:
            response = await future
            if response.success:
                self.get_logger().info('Simulation reset successful')
                # Reset our internal state too
                self.time_step = 0
                with self.state_lock:
                    self.current_odom = None
                    self.current_imu = None
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to reset simulation: {e}')

        return False


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = AdaptiveResNetController()

        # Enable training if specified
        if node.training_enabled_on_start:
            node.training_active = True
            node.get_logger().info('Training enabled on startup')

        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down Adaptive ResNet Controller')
        finally:
            node.destroy_node()

    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

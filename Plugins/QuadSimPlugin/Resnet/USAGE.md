# Adaptive ResNet Controller — Quick Start & Tuning Cheatsheet

This is a runnable crib sheet with the exact commands to install, run, plot, and tune the controller alongside QuadSim. Copy/paste as needed.

## 1) Environment & Install

- Source your ROS 2 (Humble) environment first.
- From this package folder (`Plugins/QuadSimPlugin/Resnet`):

```
# Python deps for the controller
pip install -r requirements.txt

# Optional (for plotting UI)
pip install matplotlib

# Build the ROS 2 package and expose console entry points
colcon build --packages-select adaptive_resnet_controller
source install/setup.bash
```

## 2) Launch the Controller (training OFF; safe defaults)

```
ros2 launch adaptive_resnet_controller adaptive_controller.launch.py \
  enable_training:=false
```

Tips:
- Safe defaults are in `share/adaptive_resnet_controller/config/adaptive_controller_params.yaml` (source file under `config/adaptive_controller_params.yaml`).
- The controller publishes the enhanced command to `/resnet/cmd_vel_residual` (baseline or reference + residual).
- For pure residual visualization, see `/resnet/cmd_vel_residual_vec`.

## 3) Launch the Live Plotter (optional)

Use either of these (in a separate terminal):

```
# Direct entry point
ros2 run adaptive_resnet_controller resnet_plotter

# Or via launch
ros2 launch adaptive_resnet_controller resnet_plotter.launch.py
```

The plotter shows:
- Ref vs odom velocities (x,y,z)
- Error magnitude and per‑axis error
- Residual corrections (dx,dy,dz)
- Diagnostics (avg_err, avg_residual, learning rate, training active)

## 4) Observe Topics (sanity checks)

```
# Data sources from Unreal
ros2 topic hz  /quadsim/odom
ros2 topic hz  /ref/vel_local

# Enhanced command consumed by Unreal (baseline/ref + residual)
ros2 topic hz  /resnet/cmd_vel_residual

# Pure residual vector (easy to echo)
ros2 topic echo /resnet/cmd_vel_residual_vec

# Status / diagnostics
ros2 topic echo /resnet/status
ros2 topic echo /resnet/diagnostics
```

## 5) Live Tuning (with training OFF)

Start with conservative residual shaping; keep training off while you dial in “feel”. In another terminal, set parameters live on the node `adaptive_resnet_controller`:

```
# Mix residual with baseline PID output
ros2 param set /adaptive_resnet_controller control.mix_source baseline

# Residual shaping
ros2 param set /adaptive_resnet_controller control.residual_gain 0.10
ros2 param set /adaptive_resnet_controller control.residual_smoothing_alpha 0.05
ros2 param set /adaptive_resnet_controller control.output_ramp_steps 75

# Residual caps (XY and Z)
ros2 param set /adaptive_resnet_controller control.max_velocity_correction 0.20
ros2 param set /adaptive_resnet_controller control.max_velocity_correction_z 0.05

# Total residual magnitude clamp
ros2 param set /adaptive_resnet_controller safety.max_output_magnitude 0.30
```

Guidance:
- If sluggish: lower `residual_smoothing_alpha` (e.g., 0.02), shorten `output_ramp_steps` (50), then (optionally) raise `residual_gain` gradually (0.12–0.15).
- If overshoot: lower `residual_gain`, raise `residual_smoothing_alpha` (e.g., 0.10), or tighten caps.

To make these permanent, edit `config/adaptive_controller_params.yaml` and rebuild.

## 6) Enable Training (after tuning feels good)

Only enable training when the system is close to the setpoint.

```
# Conservative learning & training window
ros2 param set /adaptive_resnet_controller learning.initial_rate 0.0001
ros2 param set /adaptive_resnet_controller learning.learning_decay_rate 0.99
ros2 param set /adaptive_resnet_controller learning.weight_bounds 0.3
ros2 param set /adaptive_resnet_controller training.warmup_steps 1000
ros2 param set /adaptive_resnet_controller training.max_error_threshold 1.5

# Optional: allow larger error before emergency during tests
ros2 param set /adaptive_resnet_controller safety.emergency_stop_error 8.0

# Start training
ros2 service call /resnet/enable_training std_srvs/srv/SetBool '{"data": true}'
```

Recommended workflow:
- Make a small slider change, let tracking settle (watch the plot).
- Enable training; observe diagnostics (avg_err should trend down slowly).
- For large step changes, disable training first, step, then re‑enable when error < `training.max_error_threshold`.

Disable or reset:
```
# Freeze learned weights
ros2 service call /resnet/enable_training std_srvs/srv/SetBool '{"data": false}'

# Reset network weights (cold start)
ros2 service call /resnet/reset_weights std_srvs/srv/Trigger {}
```

## 7) Useful Debug/Inspection

```
# Who is publishing/subscribing
ros2 topic info /resnet/cmd_vel_residual --verbose
ros2 node info  /adaptive_resnet_controller

# Quick QoS note
# - /quadsim/odom and /quadsim/imu use BEST_EFFORT (SensorData);
# - diagnostics/status are RELIABLE; no special echo flags needed.
```

## 8) Common Symptoms & Fixes

- “Emergency stop” during big steps:
  - Reduce step size from sliders or temporarily raise `safety.emergency_stop_error` while testing.
  - Keep training OFF during big steps; re‑enable when near setpoint.
- Slider feels delayed when training ON:
  - Training is heavy; prefer enabling training only near the setpoint, with `learning.initial_rate` low and `training` window tight (`training.max_error_threshold ~ 1.5–2.0`).
- Residual spikes:
  - Lower `residual_gain`, increase `residual_smoothing_alpha`, and/or tighten `max_velocity_correction` caps.

## 9) File Paths Reference

- Controller entry: `ros2_adaptive_controller.py`
- Config (edit defaults): `config/adaptive_controller_params.yaml`
- Plotter: `resnet_plotter.py`
- Launch files:
  - Controller: `launch/adaptive_controller.launch.py`
  - Plotter: `launch/resnet_plotter.launch.py`

---

If you want a combined launch that starts the controller and plotter together and forwards tuning args, say the word and we’ll add it.


# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Unreal Engine 5.5 quadcopter simulation project integrating ROS2 communication and ROSFlight compatibility. The project simulates realistic quadcopter physics, sensor data, and provides comprehensive robotics integration for research and development.

## Build Commands

### Project Generation and Building
```bash
# Generate project files and build editor (interactive menu)
./generate_and_run.sh

# Build editor only
make Unreal_QuadSimEditor

# Build standalone game
make Unreal_QuadSim

# Generate project files only
make configure
```

### ROS2 Integration
```bash
# Build ROS2 configuration package
cd unreal_sim_config
colcon build

# Launch ROSFlight simulation stack
ros2 launch unreal_sim_config unreal_sim.launch.py
```

### Alternative Build Methods
```bash
# Build editor directly with UE tools
Engine/Build/BatchFiles/Linux/Build.sh Unreal_QuadSimEditor Linux Development -project="Unreal_QuadSim.uproject"

# Build standalone game
Engine/Build/BatchFiles/Linux/Build.sh Unreal_QuadSim Linux Development -project="Unreal_QuadSim.uproject"
```

## Architecture Overview

### Core Plugin Structure

**QuadSimPlugin** - Main simulation plugin
- **AQuadPawn**: Primary quadcopter actor with thruster physics, cameras, and ROS2 integration
- **UQuadDroneController**: Flight control system with PID controllers and multiple flight modes
- **ADroneManager**: Manages multiple drones and swarm scenarios
- **AROS2Controller**: ROS2 communication hub for odometry, commands, and sensor data

**rclUE** - ROS2 communication framework
- **UROS2NodeComponent**: Central ROS2 node for publishers, subscribers, services, and actions
- Comprehensive ROS2 message type implementations
- Linux-only support with ROS2 Foxy+ compatibility

**UnrealRosFlight** - ROSFlight protocol implementation
- **UROSFlightComponent**: Realistic flight controller simulation with sensor noise modeling
- ROSFlight-specific message types and UDP networking
- Integration with external ROSFlight tools

### Key Integration Points

1. **QuadPawn** contains both `UROS2NodeComponent` and `UROSFlightComponent`
2. **UQuadDroneController** implements `IROSFlightControllerSource` interface
3. Communication flows: External ROS2 → rclUE → QuadSimPlugin Controllers
4. ROSFlight integration: External ROSFlight → UnrealRosFlight → QuadSimPlugin Physics

## Important Configuration Files

- `Plugins/QuadSimPlugin/Config/DroneConfig.json` - Flight parameters and controller settings
- `Plugins/QuadSimPlugin/PIDGains.csv` - PID controller tuning parameters
- `unreal_sim_config/params/multirotor_dynamics.yaml` - Physical rotor and motor parameters
- `unreal_sim_config/params/custom_multirotor_firmware.yaml` - ROSFlight firmware configuration

## Development Workflow

1. **Project Setup**: Use `./generate_and_run.sh` for initial setup and builds
2. **ROS2 Development**: Build `unreal_sim_config` package and launch ROSFlight stack
3. **Plugin Development**: Modify plugins in `Plugins/` directory, rebuild with make commands
4. **Testing**: Use the launch script's interactive menu to test different configurations

## Key Dependencies

- Unreal Engine 5.5 (Linux build from source required)
- ROS2 Foxy+ distribution
- ROSFlight simulation stack
- Linux development environment (Ubuntu 20.04+)

## Flight Controller Integration

The simulation supports both direct gamepad/keyboard control and ROSFlight-based control:
- **Direct Control**: Uses UQuadDroneController with various flight modes
- **ROSFlight Control**: Integrates with external ROSFlight SIL (Software-in-the-Loop) simulation
- **Hybrid Mode**: Can switch between control methods during runtime

## Sensor Simulation

Comprehensive sensor suite including:
- IMU with configurable noise and bias
- Barometer, magnetometer, and GPS
- Multiple camera views (FPV, third-person, ground tracking)
- Battery status and system telemetry
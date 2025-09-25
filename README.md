# QuadSimPlugin — In-Depth Guide

This plugin provides a modular, deterministic quadrotor simulation stack for Unreal Engine, with:
- A core flight stack (QuadSimCore) including the quad pawn, controllers, and managers
- A simulation framework (SimulationCore) for fixed-step time control and modes
- A sensor suite and robot abstractions (RobotCore)
- An ImGui-based control panel and HUD (SimHUD)
- Optional external control integration (PX4/MAVLink)

This README explains the structure, data flow, and key classes so you can navigate, extend, and debug the system efficiently.

## Overview: Actors, Components, and Modules

- `QuadSimCore` (game logic)
  - `AQuadPawn` — the quadcopter pawn actor with mesh, thrusters, cameras, sensors, and controller reference
  - `UQuadDroneController` — the core flight controller (velocity/angle/rate joystick modes; PID cascades; hover)
  - `ADroneManager` — spawns/manages drones, tracks selection, updates control each fixed step, and can broadcast flight modes
  - `UPX4Component` — optional external controller (PX4 HIL style); streams state and receives motor commands
  - `UDroneJSONConfig` — loads/saves flight and controller parameters (JSON)
  - `UThrusterComponent` — applies forces/torques per motor
  - `AQuadSimGameMode`, `AQuadSimPlayerController` — runtime glue (input routing, ImGui toggle, default possession)

- `RobotCore` (sensors, utilities)
  - `USensorManagerComponent` — owns sensors and provides fused `FSensorData`
  - `UGPSSensor`, `UIMUSensor`, `UMagSensor`, `UBaroSensor` — key sensor components
  - `CoordinateTransform` utilities

- `SimulationCore` (time + stepping)
  - `ASimulationManager` — fixed-step scheduling, simulation modes (Realtime, Paused, FastForward, Lockstep), robot registry
  - `UTimeController` — accumulator-based fixed timestep (default 100 Hz) and time scaling
  - `ISimulatable` — interface for actors to receive fixed updates

- `SimHUD` (ImGui-based HUD)
  - `USimHUDTaskbarSubsystem` + `ImGuiBootstrapSubsystem` — orchestrate HUD windows and input
  - `ControlPanelUI.cpp/.h` — flight modes, setpoints, queue builder, PX4 controls
  - `SettingsUI.cpp` — runtime settings (time scale, spawn toggles, etc.)

- `ThirdParty/MAVLink` (vendored) — generated headers for MAVLink; do not modify

## Data and Control Flow

High level loop:
1. `ASimulationManager` ticks (pre-physics), accumulates `DeltaTime` in `UTimeController`, and executes one or more fixed steps (100 Hz default) per frame.
2. Each fixed step calls `SimulationUpdate` on registered robots. `ADroneManager` is a registered robot and updates all `AQuadPawn` instances:
   - PX4 component update (if active)
   - `AQuadPawn::UpdateControl(FixedDeltaTime)` — which ultimately calls `UQuadDroneController::Update(...)` with current `FSensorData`.
3. `UQuadDroneController` computes desired thrust/torques via PID cascades depending on the flight mode, and `UThrusterComponent` applies forces/torques to the pawn.
4. `SimHUD` draws runtime UI for flight controls, configuration, PX4, debug visuals, and navigation queues.

### Simulation timing and modes
- `UTimeController`: accumulator, `FixedTimestep` (0.01 s default), `ShouldStep()`, `ConsumeTime()`
- `ASimulationManager::Tick`:
  - Realtime and FastForward use accumulator stepping; max steps-per-frame guarded by `MaxStepsPerFrame` (warns if falling behind)
  - Paused and Lockstep use single-step semantics (respecting `RequestSimulationStep`)
  - Time scaling is applied via world time dilation in manager; `UTimeController` remains at 1x to avoid double scaling

### Drone spawning and selection (`ADroneManager`)
- On `BeginPlay`, registers to actor-spawn events; finds existing `AQuadPawn`s
- `SpawnDrone(...)`: spawns near the currently possessed pawn or at `APlayerStart`; auto-possesses and switches camera
- `SelectDroneByIndex(...)`: sets current selection and can possess/switch camera
- Swarm mode: tracks if UI should render only for leader vs. all
- Simulation update: for each drone
  - If PX4 active, run `UPX4Component::SimulationUpdate`
  - Call `AQuadPawn::UpdateControl(FixedDeltaTime)`

### Quad pawn (`AQuadPawn`)
- Components:
  - `UStaticMeshComponent* DroneBody`
  - Sensors: `USensorManagerComponent`, which owns `UIMUSensor`, `UGPSSensor`, `UMagSensor`, `UBaroSensor`
  - Cameras: spring arm + third-person, FPV, ground-track cameras (switchable)
  - Thrusters: `TArray<UThrusterComponent*>`
  - External control: `UPX4Component* PX4Component`
- Input handling (gamepad): binds axes to `FGamepadInputs` (Throttle/Yaw/Pitch/Roll)
- Control update: pushes external commands or sensor-driven updates to `UQuadDroneController`
- Reset helpers: rotation/position reset, camera switching

### Controller (`UQuadDroneController`)
- Flight modes:
  - `AutoWaypoint`: position-follow using `UNavigationComponent` setpoints
  - `VelocityControl`: desired body-frame velocity
  - `AngleControl`, `RateControl` (raw angle/rate targets)
  - `JoyStickAngleControl`, `JoyStickAcroControl` (gamepad-driven variants)
- State and PIDs:
  - Position PIDs (X/Y/Z) ? angle targets
  - Angle PIDs (Roll/Pitch) ? angle rates
  - Yaw rate PID ? torque
  - Altitude PID (`AltitudePID`) cascaded to Z velocity
- Key processing paths:
  - `Update(...)` selects `GamepadController` vs `FlightController` and stores sensor data
  - `GamepadController(...)`:
    - Integrates `hoverTargetAltitude` via gamepad throttle using configured altitude rate (m/s), clamps to min altitude
    - Computes `altVelSetpoint = PID(hoverTargetAltitude, currentAltitude)`
    - Computes `zEffort = ZPID(altVelSetpoint, localVelZ) * 100` (cm-scale)
    - Reads sticks ? `desiredRoll/Pitch/YawRate`, runs P cascades for roll/pitch, `YawRateControl` for yaw, `ThrustMixer(...)`
  - `FlightController(...)`:
    - If `AutoWaypoint`: updates nav, sets `setPoint`, computes local pos error, targets max velocity along that direction
    - Hover overlay: clamps Z velocity via altitude PID
    - Velocity PIDs ? desired angles; angle PIDs ? roll/pitch rate outputs; yaw rate PID
    - `ThrustMixer(...)` mixes base hover thrust with zOut and torque-like terms
- `ThrustMixer(...)`:
  - Computes `hoverThrust` from body mass and gravity
  - Adds Z effort (scaled) and compensates for commanded tilt
  - Distributes to 4 motors and clamps to `maxThrust` per motor; applies yaw virtual torque
- External control and ROS hooks:
  - `SetUseExternalController(bool)` toggles PX4 mode; resets PIDs on switch
  - `ApplyMotorCommands(TArray<float>)`: maps 0..1 motor commands to Newtons, slews, converts to UE4 force units, updates HUD thrust cache
- Debug drawing:
  - `DrawDebugVisuals(...)`: line from current pos to `setPoint` (yellow sphere marker also drawn)
  - `DrawDebugVisualsVel(...)`: draws desired velocity axes and motor labels

### Sensors (`RobotCore`)
- `USensorManagerComponent` initializes and updates sensors, reporting a compact `FSensorData` with IMU attitude/rates, GPS position, baro altitude, etc.
- Each sensor has initialization, Update, and noise modeling. `UMagSensor` uses declination model if a `AGeoReferencingSystem` is present.

### HUD and control panel (`SimHUD`)
- `ImGuiBootstrapSubsystem.cpp` orchestrates ImGui initialization and delegates per-frame draws (taskbar, settings, control panel)
- `ControlPanelUI.cpp`:
  - Mode selection: AutoWaypoint, Velocity, Angle, Rate, Joystick variants
  - AutoWaypoint:
    - Manual queue builder for setpoints in meters (X/Y/Z), applies to `UNavigationComponent`
    - Displays current setpoint; controller also draws debug line + marker
  - Velocity/Angle/Rate sliders (with session overrides for max angle/vel/rate)
  - Hover mode controls (activate/deactivate, altitude slider in meters), flows to `SetHoverMode` in controller
  - PX4 connect/enable and status widgets
- `SettingsUI.cpp`:
  - Time scaling (shows `ASimulationManager` settings), spawn toggles, camera modes

### Configuration (`UDroneJSONConfig`)
- Loads defaults from `Plugins/QuadSimPlugin/Config/DroneConfig.json` (or project config fallback)
- Key groups:
  - `flight_parameters`: max velocity, angle, angle rate, PID output caps, thrust, altitude thresholds, min local altitude, acceptable distance
  - `controller`: altitude_rate (cm/s), yaw_rate (deg/s), min_velocity_for_yaw (m/s)
- Provides `SaveConfig()` to persist runtime edits from HUD

## File-by-File Highlights

QuadSimCore:
- `Public/Controllers/QuadDroneController.h` / `Private/Controllers/QuadDroneController.cpp`
  - Defines controller state, PIDs, modes, update paths, thrust mixing, yaw control
- `Public/Pawns/QuadPawn.h` / `Private/Pawns/QuadPawn.cpp`
  - Drone actor, components, input binding, camera management, control update entry point
- `Public/Core/DroneManager.h` / `Private/Core/DroneManager.cpp`
  - Spawn/select drones, simulation integration, obstacle manager toggling
- `Public/Core/DroneJSONConfig.h` / `Private/Core/DroneJSONConfig.cpp`
  - Config schema and JSON load/save across flight and controller parameters
- `Public/Core/ThrusterComponent.h` / `Private/Core/ThrusterComponent.cpp`
  - Per-motor force and torque application
- `Private/Controllers/PX4Component.cpp` (+ header)
  - TCP server, MAVLink HIL messages, heartbeat/state senders, motor command parsing
- `QuadSimGameMode*.h/.cpp`, `QuadSimPlayerController*.h/.cpp`
  - Input routes (ImGui toggle), default possession, mouse/UI focus helpers

RobotCore:
- `Public/Sensors/*` and `Private/Sensors/*`
  - Sensor components and `USensorManagerComponent`
- `Private/CoordinateTransform.cpp`
  - Frame conversions used by PX4 and sensors

SimulationCore:
- `Public/Core/SimulationManager.h` / `Private/Core/SimulationManager.cpp`
  - Simulation loop, modes, robot registry
- `Public/Core/TimeController.h` / `Private/Core/TimeController.cpp`
  - Fixed-step accumulator, time scale, pause
- `Public/Interfaces/ISimulatable.h`
  - Executed by SimulationManager per fixed step

SimHUD:
- `Private/ImGuiHud/ImGuiBootstrapSubsystem.cpp`
  - Sets up ImGui context, registers draw delegates, handles taskbar window and error-boundary style drawing
- `Private/ImGuiHud/ControlPanelUI.cpp` / `Public/ImGuiHud/ControlPanelUI.h`
  - All end-user flight controls and nav tools, plus PX4 knobs
- `Private/ImGuiHud/SettingsUI.cpp`
  - Time, spawn, and UI theme options

ThirdParty:
- `ThirdParty/MAVLink/*` — generated MAVLink message definitions, tests, and headers (do not edit)

## Typical Runtime Flow

1. Editor starts level with `ASimulationManager` and `ADroneManager` placed.
2. `ADroneManager` finds/spawns `AQuadPawn` and `AQuadPawn` owns sensors and controller.
3. Sensors update under `USensorManagerComponent`; controller pulls `FSensorData` each step.
4. User selects flight mode via SimHUD (or presses gamepad toggle for joystick mode).
5. Controller computes desired outputs, calls `ThrustMixer`, thrusters apply forces/torques.
6. If PX4 is enabled and connected, PX4 dictates motor commands; controller updates HUD thrust cache.

## Key Tunables
- JSON config (`Plugins/QuadSimPlugin/Config/DroneConfig.json`):
  - `flight_parameters.max_velocity`, `max_angle`, `max_angle_rate`, `max_pid_output`, `max_thrust`
  - `flight_parameters.min_altitude_local` (cm), `acceptable_distance` (cm)
  - `controller.altitude_rate` (cm/s), `yaw_rate` (deg/s), `min_velocity_for_yaw` (m/s)
- HUD sliders — session-only overrides for angle/rate/velocity
- `ASimulationManager`: `MaxStepsPerFrame`, mode, and world time dilation

## Extending the System
- New flight mode: add enum to `EFlightMode`, branch in `UQuadDroneController::FlightController`, and expose in HUD
- New sensor: add a component in `RobotCore`, initialize in `USensorManagerComponent`, extend `FSensorData`
- New navigation pattern: add to `AQuadPawn::GenerateFigureEightWaypoints` or implement alternate planner component
- External controller: implement your own component (similar to `UPX4Component`) and drive `ApplyMotorCommands`

## Troubleshooting
- ImGui crashes: usually due to formatting with null/invalid pointers. Check `ImGuiBootstrapSubsystem.cpp` around draw calls and guard pointers and strings.
- Vertical control oddities: ensure Z PID outputs are scaled to cm when mixing thrust; verify gravity compensation uses commanded tilt.
- No movement: confirm sensors are valid (`FSensorData` flags), PIDs not saturated (check HUD), and thruster forces below `max_thrust`.
- PX4 integration: verify TCP server port and state streaming are active; watch logs for heartbeat and HIL messages.

## Build and Run
- Generate project files:
  `"<UE_Install>\Engine\Build\BatchFiles\GenerateProjectFiles.bat" -project="%CD%\Unreal_QuadSim.uproject" -game -engine`
- Build:
  `msbuild Unreal_QuadSim.sln /p:Configuration=Development /m`
- Launch editor:
  `"<UE_Install>\Engine\Binaries\Win64\UnrealEditor.exe" .\Unreal_QuadSim.uproject`
- Run tests (headless example):
  `UnrealEditor-Cmd.exe .\Unreal_QuadSim.uproject -ExecCmds="Automation RunTests QuadSim.*; Quit" -nullrhi -nop4 -Unattended`

## Repository Conventions
- Follow Unreal C++ style; prefer UE containers/types
- Don’t modify generated MAVLink headers; wrap them if needed
- Keep secrets out of source; use `Default*.ini`/JSON config patterns
- Commit messages: Conventional Commits preferred (`feat:`, `fix:`, `refactor:`)

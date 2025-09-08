# Repository Guidelines

## Project Structure & Modules
- Plugin root: `Plugins/QuadSimPlugin/`
- Code: `Source/` split into modules `QuadSimCore`, `RobotCore`, `SimulationCore`, `SimHUD` with `Public/` headers and `Private/` sources. Module rules in `*.Build.cs`.
- Assets: `Content/` (Blueprints, UI, models). Config in `Config/` (e.g., `DroneConfig.json`).
- Build outputs in `Binaries/`; generated files in `Intermediate/` — do not edit either.

## Build, Test, and Dev Commands
- Build plugin (packaged): run `RunUAT.bat BuildPlugin -Plugin="Plugins/QuadSimPlugin/QuadSimPlugin.uplugin" -Package="Packages/QuadSimPlugin" -TargetPlatforms=Win64` from the Unreal Engine root.
- Open locally: launch the project `.uproject` in Unreal Editor, enable Live Coding, and compile modules (`Ctrl+Alt+F11`).
- Run automation tests (example): `UnrealEditor.exe <Project>.uproject -ExecCmds="Automation RunTests QuadSimCore" -unattended -nullrhi -nop4 -testexit="Automation Test Queue Empty"`.

## Coding Style & Naming
- Follow Unreal C++ style: 4‑space indent, braces on new lines, `#pragma once` in headers.
- Class/file names match and use UE prefixes (`U` UObject, `A` Actor, `F` struct, `I` interface). Example: `AQuadPawn` in `QuadPawn.h/cpp`.
- Place public API in `Public/`, internals in `Private/`. Keep module boundaries clean (include via public headers).
- Use UE logging categories per module; avoid `std::` where UE types exist (`FString`, `TArray`).

## Testing Guidelines
- Prefer Unreal Automation/Spec tests under `Source/<Module>/Private/Tests/*Spec.cpp`.
- Name tests by feature: `DroneManager_SpawningSpec.cpp`. Aim for coverage on controllers, managers, and UI glue.
- Run from Session Frontend (Editor) or CLI (see above).

## Commit & Pull Requests
- Use clear, scoped commits. Conventional Commits encouraged: `feat(QuadSimCore): add PID tuning UI`.
- PRs must include: concise description, linked issues, affected modules, and screenshots/GIFs for UI.
- Exclude `Binaries/` and `Intermediate/` changes. Large assets should use Git LFS.

## Notes & Tips
- Configuration: keep `Config/*.ini` and `DroneConfig.json` in sync with defaults.
- Don’t hardcode asset paths; use soft references where possible.
- Agent hint: when editing code, respect module `*.Build.cs` dependencies and avoid cross‑module includes from `Private/`.

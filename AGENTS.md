# Repository Guidelines

## Project Structure & Modules
- Unreal_QuadSim.uproject — open this in Unreal Editor.
- Source/Unreal_QuadSim/ — game module (Unreal_QuadSim.cpp/.h, Unreal_QuadSim.Build.cs).
- Source/Unreal_QuadSim.Target.cs, Source/Unreal_QuadSimEditor.Target.cs — build targets.
- Plugins/QuadSimPlugin/ — project plugin; ThirdParty/MAVLink/ contains vendored headers (do not modify generated files).
- Content/ — maps and assets (uasset/umap tracked, many via Git LFS).

## Build, Test, and Development
- Open in editor: "<UE_Install>\\Engine\\Binaries\\Win64\\UnrealEditor.exe" .\\Unreal_QuadSim.uproject (UE5) or UE4Editor.exe (UE4).
- Generate IDE files: "<UE_Install>\\Engine\\Build\\BatchFiles\\GenerateProjectFiles.bat" -project="%CD%\\Unreal_QuadSim.uproject" -game -engine.
- Build from command line (example): msbuild Unreal_QuadSim.sln /p:Configuration=Development /m after generating project files.
- Package (optional): RunUAT BuildCookRun -project="<path>\\Unreal_QuadSim.uproject" -platform=Win64 -clientconfig=Development -build -cook -stage -pak.

## Coding Style & Naming
- Follow Unreal Engine C++ style.
  - Types: U (UObject), A (Actor), F (struct), I (interface), T (template).
  - Functions/Methods: PascalCase; variables: camelCase; booleans: IsArmed.
  - Indent with 4 spaces; wrap at ~120 cols; include headers in .Build.cs.
- Prefer UE containers (TArray, TMap) and math types (FVector, FRotator).

## Testing Guidelines
- Use Unreal Automation Tests. Name tests with a clear prefix, e.g., QuadSim.*.
- Run headless: UnrealEditor-Cmd.exe .\\Unreal_QuadSim.uproject -ExecCmds="Automation RunTests QuadSim.*; Quit" -nullrhi -nop4 -Unattended.
- Aim to keep fast unit/spec tests under 1 minute; isolate plugin logic where possible.

## Commit & Pull Requests
- Commit messages: short imperative subject, optional body (e.g., Fix: stabilize PID controller yaw).
- Prefer Conventional Commits (eat:, ix:, efactor:) for clarity.
- PRs must include: scope/summary, rationale, links to issues, test plan (commands used), and screenshots/video for editor-facing changes. Note any content changes (maps/assets) and LFS artifacts.

## Security & Configuration
- Do not commit local Saved/ or intermediate build artifacts.
- Keep secrets/config out of source; prefer Default*.ini patterns. Avoid editing generated MAVLink headers; extend via wrapper classes in the plugin.

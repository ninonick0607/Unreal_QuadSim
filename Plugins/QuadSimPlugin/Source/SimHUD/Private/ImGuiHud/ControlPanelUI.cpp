// Fill out your copyright notice in the Description page of Project Settings.


#include "ImGuiHud/ControlPanelUI.h"

#include "imgui.h"
#include "QuadSimCore/Public/Core/DroneManager.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "Controllers/PX4Component.h"
#include "Core/DroneJSONConfig.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "GameFramework/Actor.h"
#include "Utility/NavigationComponent.h"
#include <cfloat>

void UControlPanelUI::TickAndDraw(UWorld* World)
{
    if (!World) return;

    // Build a single unified window when at least one drone exists and panel is open
    ADroneManager* DM = ADroneManager::Get(World);
    if (!DM) return;
    const TArray<AQuadPawn*> Drones = DM->GetDroneList();
    if (Drones.Num() == 0) return;

    // Only draw when toggled open from the taskbar
    if (!bOpen) return;

    // Active/selected drone (via manager selected index)
    int32 ActiveIndex = FMath::Clamp(DM->SelectedDroneIndex, 0, Drones.Num()-1);
    AQuadPawn* ActivePawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
    if (!ActivePawn) return;
    UQuadDroneController* Controller = ActivePawn->QuadController;

    // Ensure we have a state record for this drone
    FDronePanelState& State = DroneStates.FindOrAdd(ActivePawn);

    // Start near right; allow free movement (no pivot lock). Only set on first use.
    ImGuiIO& IO = ImGui::GetIO();
    const float PanelW = 380.f;
    const float RightPad = 10.f;

    // Use the actual viewport size (not IO.DisplaySize) so the panel tracks with editor layout changes
    float VpW = IO.DisplaySize.x;
    float VpH = IO.DisplaySize.y;
    if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
    {
        const FIntPoint vp = GEngine->GameViewport->Viewport->GetSizeXY();
        VpW = vp.X; VpH = vp.Y;
    }

    // Initial placement near right edge only on first show
    if (PanelX < 0.f)
    {
        PanelX = FMath::Max(0.f, VpW - PanelW - RightPad);
    }
    ImGui::SetNextWindowPos(ImVec2(PanelX, FMath::Clamp(PanelY, 0.f, FMath::Max(0.f, VpH - 60.f))), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(PanelW, 600.f), ImGuiCond_FirstUseEver);
    // Lock width to PanelW and let height auto-fit to content; prevent user resize
    ImGui::SetNextWindowSizeConstraints(ImVec2(PanelW, 0.0f), ImVec2(PanelW, FLT_MAX));

    FString Title = FString::Printf(TEXT("Control Panel - %s##ControlPanel"), *ActivePawn->GetName());
    ImGuiWindowFlags WFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize;
    if (ImGui::Begin(TCHAR_TO_UTF8(*Title), &bOpen, WFlags))
    {
        // Clamp window into viewport if the editor layout changed
        ImVec2 winPos  = ImGui::GetWindowPos();
        ImVec2 size    = ImGui::GetWindowSize();
        float maxX = FMath::Max(0.f, VpW - size.x);
        float maxY = FMath::Max(0.f, VpH - size.y);
        float clampedX = FMath::Clamp(winPos.x, 0.f, maxX);
        float clampedY = FMath::Clamp(winPos.y, 0.f, maxY);
        if (clampedX != winPos.x || clampedY != winPos.y)
        {
            ImGui::SetWindowPos(ImVec2(clampedX, clampedY));
            winPos.x = clampedX; winPos.y = clampedY;
        }
        // Remember current pos for next frame to respect user-adjusted location
        PanelX = winPos.x;
        PanelY = winPos.y;
        // Drone selector at the top
        if (ImGui::BeginCombo("Active Drone", TCHAR_TO_UTF8(*ActivePawn->GetName())))
        {
            for (int32 i = 0; i < Drones.Num(); ++i)
            {
                AQuadPawn* P = Drones[i];
                bool bSelected = (i == ActiveIndex);
                if (ImGui::Selectable(TCHAR_TO_UTF8(*P->GetName()), bSelected))
                {
                    // Also possess the selected drone and switch camera
                    DM->SelectDroneByIndex(i, /*bAlsoPossess=*/true);
                }
                if (bSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        // Checkboxes above the buttons
        ImGui::Checkbox("Gamepad", &State.bGamepad);
        ImGui::SameLine();
        bool bPX4Now = State.bPX4;
        if (ImGui::Checkbox("PX4", &bPX4Now))
        {
            State.bPX4 = bPX4Now;
            if (Controller)
            {
                Controller->SetUseExternalController(State.bPX4);
            }
            if (ActivePawn)
            {
                if (UPX4Component* PX4 = ActivePawn->FindComponentByClass<UPX4Component>())
                {
                    PX4->SetPX4Active(State.bPX4);
                }
            }
        }
        // (Debug toggle moved next to mode selection below)
        // Note: Gamepad visualization/modes are controlled by selecting the Gamepad Angle/Acro buttons below.

        ImGui::Separator();
        // Top horizontal buttons for flight modes
        if (!State.bGamepad)
        {
            if (ImGui::Button("Position"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::AutoWaypoint); }
            }
            ImGui::SameLine();
            if (ImGui::Button("Velocity"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::VelocityControl); }
            }
            ImGui::SameLine();
            if (ImGui::Button("Angle"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::AngleControl); }
            }
            ImGui::SameLine();
            if (ImGui::Button("Acro"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::RateControl); }
            }

            // Debug toggle next to mode selection
            ImGui::SameLine();
            bool bDebugNow = State.bDebug;
            if (ImGui::Checkbox("Debug", &bDebugNow))
            {
                State.bDebug = bDebugNow;
                if (Controller) { Controller->SetDebugVisualsEnabled(State.bDebug); }
            }
        }
        else
        {
            // Gamepad-only modes; hide Position/Velocity completely
            if (ImGui::Button("Gamepad Angle"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::JoyStickAngleControl); }
            }
            ImGui::SameLine();
            if (ImGui::Button("Gamepad Acro"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::JoyStickAcroControl); }
            }

            // Debug toggle next to gamepad mode selection
            ImGui::SameLine();
            bool bDebugNow = State.bDebug;
            if (ImGui::Checkbox("Debug", &bDebugNow))
            {
                State.bDebug = bDebugNow;
                if (Controller) { Controller->SetDebugVisualsEnabled(State.bDebug); }
            }
        }

        ImGui::Separator();
        // Control Settings (above control sliders)
        if (ImGui::CollapsingHeader("Control Settings", ImGuiTreeNodeFlags_DefaultOpen))
        {
            auto& Cfg = UDroneJSONConfig::Get().Config;

            // Bounds from settings (read-only)
            const float maxVelBound   = FMath::Max(0.0f, Cfg.FlightParams.MaxVelocityBound);
            const float maxAngleBound = FMath::Max(0.0f, Cfg.FlightParams.MaxAngleBound);

            // Session values: initialize from settings once, then keep in panel state
            float UiMaxVel   = State.bHasSessionMaxVel   ? State.SessionMaxVel   : Cfg.FlightParams.MaxVelocity;
            float UiMaxAngle = State.bHasSessionMaxAngle ? State.SessionMaxAngle : Cfg.FlightParams.MaxAngle;
            float UiMaxRate  = State.bHasSessionMaxRate  ? State.SessionMaxRate  : Cfg.FlightParams.MaxAngleRate;

            if (ImGui::SliderFloat("Max Velocity (m/s)", &UiMaxVel, 0.0f, maxVelBound, "%.2f"))
            {
                State.SessionMaxVel = UiMaxVel;
                State.bHasSessionMaxVel = true;
                if (Controller) Controller->SetMaxVelocity(UiMaxVel); // session-only
            }
            if (ImGui::SliderFloat("Max Angle (deg)", &UiMaxAngle, 0.0f, maxAngleBound, "%.1f"))
            {
                State.SessionMaxAngle = UiMaxAngle;
                State.bHasSessionMaxAngle = true;
                if (Controller) Controller->SetMaxAngle(UiMaxAngle); // session-only
            }
            if (ImGui::SliderFloat("Max Angle Rate (deg/s)", &UiMaxRate, 0.0f, 180.0f, "%.1f"))
            {
                State.SessionMaxRate = UiMaxRate;
                State.bHasSessionMaxRate = true;
                if (Controller) Controller->SetMaxAngleRate(UiMaxRate); // session-only
            }
        }

        ImGui::Separator();
        // Position Control Settings (manual queue builder)
        if (Controller && Controller->GetFlightMode() == EFlightMode::AutoWaypoint && State.bManualPositionOverride && ImGui::CollapsingHeader("Position Control Settings", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // Input fields for a waypoint in meters
            // Change from pure text inputs to slider + editable value box per axis
            float ix = State.ManualInput.X;
            float iy = State.ManualInput.Y;
            float iz = State.ManualInput.Z;

            // X control: slider with editable box
            ImGui::PushID("ManualX");
            ImGui::SetNextItemWidth(200.f);
            ImGui::SliderFloat("X (m)", &ix, -50.f, 50.f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.f);
            ImGui::InputFloat("##XEdit", &ix, 0.1f, 1.0f, "%.2f");
            ImGui::PopID();

            // Y control: slider with editable box
            ImGui::PushID("ManualY");
            ImGui::SetNextItemWidth(200.f);
            ImGui::SliderFloat("Y (m)", &iy, -50.f, 50.f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.f);
            ImGui::InputFloat("##YEdit", &iy, 0.1f, 1.0f, "%.2f");
            ImGui::PopID();

            // Z control: slider with editable box
            ImGui::PushID("ManualZ");
            ImGui::SetNextItemWidth(200.f);
            ImGui::SliderFloat("Z (m)", &iz, -50.f, 50.f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.f);
            ImGui::InputFloat("##ZEdit", &iz, 0.1f, 1.0f, "%.2f");
            ImGui::PopID();
            State.ManualInput = FVector(ix, iy, iz);
            ImGui::SameLine();
            if (ImGui::Button("+ Add"))
            {
                State.ManualQueue.Add(State.ManualInput);
                if (ActivePawn && ActivePawn->NavigationComponent)
                {
                    ActivePawn->NavigationComponent->SetNavigationPlan(State.ManualQueue);
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear"))
            {
                State.ManualQueue.Reset();
                if (ActivePawn && ActivePawn->NavigationComponent)
                {
                    ActivePawn->NavigationComponent->SetNavigationPlan(State.ManualQueue);
                }
            }

            // List current queue
            ImGui::Separator();
            ImGui::TextUnformatted("Queue:");
            for (int i = 0; i < State.ManualQueue.Num(); ++i)
            {
                const FVector& P = State.ManualQueue[i];
                ImGui::BulletText("%d: (%.2f, %.2f, %.2f)", i, P.X, P.Y, P.Z);
            }
        }

        // Flight controls (collapsible)
        if (Controller && ImGui::CollapsingHeader("Flight Controls", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const auto& Cfg = UDroneJSONConfig::Get().Config;
            // Use session overrides if present so slider ranges update live
            const float MaxVel   = State.bHasSessionMaxVel   ? State.SessionMaxVel   : Cfg.FlightParams.MaxVelocity;     // m/s
            const float MaxAngle = State.bHasSessionMaxAngle ? State.SessionMaxAngle : Cfg.FlightParams.MaxAngle;        // deg
            const float MaxRate  = State.bHasSessionMaxRate  ? State.SessionMaxRate  : Cfg.FlightParams.MaxAngleRate;    // deg/s
            const float MaxYawRate = Cfg.ControllerParams.YawRate;       // deg/s

            EFlightMode Mode = Controller->GetFlightMode();

            // Hover Mode quick controls (verbatim style)
            // Config stores MinAltitudeLocal in centimeters; UI works in meters
            const float minAlt = FMath::Max(0.0f, Cfg.FlightParams.MinAltitudeLocal * 0.01f);
            if (State.HoverAlt <= 0.f) { State.HoverAlt =2; }
            // Toggleable hover button with green highlight when active
            if (State.bHoverActive)
            {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.20f, 0.70f, 0.30f, 1.f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.80f, 0.35f, 1.f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.18f, 0.60f, 0.28f, 1.f));
            }
            if (ImGui::Button("ACTIVATE HOVER MODE"))
            {
                State.bHoverActive = !State.bHoverActive;
                Controller->SetHoverMode(State.bHoverActive, State.HoverAlt);
            }
            if (State.bHoverActive) ImGui::PopStyleColor(3);
            ImGui::SliderFloat("Hover Altitude (m)", &State.HoverAlt, minAlt, 500.0f, "%.1f");

            // Mode label above sliders
            const char* sectionLabel = "";
            if (!State.bGamepad && Mode == EFlightMode::AutoWaypoint)
                sectionLabel = "Desired Position (X, Y, Z)";
            else if (!State.bGamepad && Mode == EFlightMode::VelocityControl)
                sectionLabel = "Desired Velocities (X, Y, Z) and Yaw Rate";
            else if (Mode == EFlightMode::AngleControl || Mode == EFlightMode::JoyStickAngleControl)
                sectionLabel = "Desired Angles (Roll, Pitch), Yaw Rate, and Z Velocity";
            else if (Mode == EFlightMode::RateControl || Mode == EFlightMode::JoyStickAcroControl)
                sectionLabel = "Desired Rates (Roll, Pitch, Yaw) and Z Velocity";
            if (sectionLabel[0] != '\0')
            {
                ImGui::Separator();
                ImGui::TextUnformatted(sectionLabel);
            }

            // Position mode: show current setpoint; hide original sliders when Manual Destination is active
            if (!State.bGamepad && Mode == EFlightMode::AutoWaypoint)
            {
                // Main toggle: Manual Destination shows queue UI and hides old sliders
                bool bManualDest = State.bManualPositionOverride;
                if (ImGui::Checkbox("Manual Destination", &bManualDest))
                {
                    State.bManualPositionOverride = bManualDest;
                    State.bManualPathMode = bManualDest; // keep controller debug spheres consistent
                    if (Controller) Controller->SetManualPathMode(State.bManualPathMode);
                    if (!State.bManualPositionOverride)
                    {
                        // Leaving manual: restore generic spiral/auto plan
                        if (ActivePawn && ActivePawn->NavigationComponent)
                        {
                            const FVector currPosM = ActivePawn->GetActorLocation() / 100.0f;
                            TArray<FVector> plan = ActivePawn->GenerateFigureEightWaypoints(currPosM);
                            ActivePawn->NavigationComponent->SetNavigationPlan(plan);
                        }
                    }
                    else if (ActivePawn && ActivePawn->NavigationComponent)
                    {
                        // Entering manual: apply current queue (may be empty)
                        ActivePawn->NavigationComponent->SetNavigationPlan(State.ManualQueue);
                    }
                }
                if (State.bManualPositionOverride)
                {
                    // Manual queue is driving nav; show read-only current setpoint for reference
                    const FVector pos = Controller->GetCurrentSetPoint();
                    ImGui::Text("Current Setpoint: X=%.2f Y=%.2f Z=%.2f m", pos.X, pos.Y, pos.Z);
                }
                else
                {
                    // Original single-target sliders: visible only when not using manual destination/queue
                    FVector pos = Controller->GetCurrentSetPoint();
                    float px = pos.X, py = pos.Y, pz = pos.Z;
                    ImGui::SliderFloat("Position X (m)", &px, -500.f, 500.f, "%.2f");
                    ImGui::SliderFloat("Position Y (m)", &py, -500.f, 500.f, "%.2f");
                    ImGui::SliderFloat("Position Z (m)", &pz, -500.f, 500.f, "%.2f");
                    const FVector dest(px, py, pz);
                    Controller->SetDestination(dest);
                    if (ActivePawn && ActivePawn->NavigationComponent)
                    {
                        ActivePawn->NavigationComponent->SetCurrentDestination(dest);
                    }
                }
            }

            // Velocity mode: XYZ velocity and yaw rate (hidden in Gamepad UI)
            if (!State.bGamepad && Mode == EFlightMode::VelocityControl)
            {
                FVector vel = Controller->GetDesiredVelocity();
                float vx = vel.X, vy = vel.Y, vz = vel.Z;
                ImGui::SliderFloat("Velocity X (m/s)", &vx, -MaxVel, MaxVel, "%.2f");
                ImGui::SliderFloat("Velocity Y (m/s)", &vy, -MaxVel, MaxVel, "%.2f");
                ImGui::SliderFloat("Velocity Z (m/s)", &vz, -MaxVel, MaxVel, "%.2f");
                Controller->SetDesiredVelocity(FVector(vx, vy, vz));

                float yawRate = Controller->GetDesiredYawRate();
                ImGui::SliderFloat("Yaw Rate (deg/s)", &yawRate, -MaxYawRate, MaxYawRate, "%.1f");
                Controller->SetDesiredYawRate(yawRate);
            }

            // Angle mode: roll, pitch, yaw rate, and Z velocity
            if (Mode == EFlightMode::AngleControl || Mode == EFlightMode::JoyStickAngleControl)
            {
                float roll  = Controller->GetDesiredRoll();
                float pitch = Controller->GetDesiredPitch();
                float yawR  = Controller->GetDesiredYawRate();
                ImGui::SliderFloat("Roll (deg)",  &roll,  -MaxAngle, MaxAngle, "%.1f");
                ImGui::SliderFloat("Pitch (deg)", &pitch, -MaxAngle, MaxAngle, "%.1f");
                ImGui::SliderFloat("Yaw Rate (deg/s)", &yawR, -MaxYawRate, MaxYawRate, "%.1f");
                Controller->SetDesiredRollAngle(roll);
                Controller->SetDesiredPitchAngle(pitch);
                Controller->SetDesiredYawRate(yawR);

                FVector vel = Controller->GetDesiredVelocity();
                float zVel = vel.Z;
                ImGui::SliderFloat("Z Velocity (m/s)", &zVel, -MaxVel, MaxVel, "%.2f");
                vel.Z = zVel; vel.X = 0.f; vel.Y = 0.f; // keep horizontal zero in angle mode
                Controller->SetDesiredVelocity(vel);
            }

            // Acro/Rate mode: roll rate, pitch rate, yaw rate, and Z velocity
            if (Mode == EFlightMode::RateControl || Mode == EFlightMode::JoyStickAcroControl)
            {
                // Auto-reset toggle for PID tuning (step response testing)
                ImGui::Checkbox("Auto-Reset Rates", &State.bAcroAutoReset);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Automatically reset roll/pitch rates to 0 after %.1fs\nUseful for PID tuning step response", State.AcroResetDelay);
                }

                if (State.bAcroAutoReset)
                {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(80.f);
                    ImGui::SliderFloat("##ResetDelay", &State.AcroResetDelay, 0.3f, 3.0f, "%.1fs");

                    // Show countdown if reset is pending
                    if (State.bAcroResetPending && State.AcroResetTimer > 0.0f)
                    {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Resetting in %.1fs", State.AcroResetTimer);
                    }
                }

                double rollRate = Controller->GetDesiredRollRate();
                double pitchRate = Controller->GetDesiredPitchRate();
                float yawR  = Controller->GetDesiredYawRate();
                float rr = (float)rollRate, pr = (float)pitchRate;

                bool rollChanged = ImGui::SliderFloat("Roll Rate (deg/s)",  &rr, -MaxRate, MaxRate, "%.1f");
                bool pitchChanged = ImGui::SliderFloat("Pitch Rate (deg/s)", &pr, -MaxRate, MaxRate, "%.1f");
                ImGui::SliderFloat("Yaw Rate (deg/s)",   &yawR, -MaxYawRate, MaxYawRate, "%.1f");

                Controller->SetDesiredRollRate(rr);
                Controller->SetDesiredPitchRate(pr);
                Controller->SetDesiredYawRate(yawR);

                // Trigger auto-reset timer when roll or pitch rate changes
                if (State.bAcroAutoReset && (rollChanged || pitchChanged))
                {
                    // Only restart timer if we're setting a non-zero rate
                    if (FMath::Abs(rr) > 0.1f || FMath::Abs(pr) > 0.1f)
                    {
                        State.bAcroResetPending = true;
                        State.AcroResetTimer = State.AcroResetDelay;
                    }
                }

                // Update timer and auto-reset
                if (State.bAcroResetPending && State.AcroResetTimer > 0.0f)
                {
                    State.AcroResetTimer -= World->GetDeltaSeconds();

                    if (State.AcroResetTimer <= 0.0f)
                    {
                        // Reset roll and pitch rates to zero
                        Controller->SetDesiredRollRate(0.0f);
                        Controller->SetDesiredPitchRate(0.0f);
                        State.bAcroResetPending = false;
                        State.AcroResetTimer = 0.0f;
                    }
                }

                FVector vel = Controller->GetDesiredVelocity();
                float zVel = vel.Z;
                ImGui::SliderFloat("Z Velocity (m/s)", &zVel, -MaxVel, MaxVel, "%.2f");
                vel.Z = zVel; vel.X = 0.f; vel.Y = 0.f;
                Controller->SetDesiredVelocity(vel);
            }

            // --- Reset-to-0 helpers under sliders ---
            ImGui::Separator();
            ImGui::TextUnformatted("Reset to 0:");
            ImGui::SameLine();
            if (ImGui::SmallButton("X"))
            {
                if (Mode == EFlightMode::AutoWaypoint)
                {
                    FVector sp = Controller->GetCurrentSetPoint(); sp.X = 0.f; Controller->SetDestination(sp);
                }
                else if (Mode == EFlightMode::VelocityControl)
                {
                    FVector v = Controller->GetDesiredVelocity(); v.X = 0.f; Controller->SetDesiredVelocity(v);
                }
                else if (Mode == EFlightMode::AngleControl || Mode == EFlightMode::JoyStickAngleControl)
                {
                    Controller->SetDesiredRollAngle(0.f);
                }
                else if (Mode == EFlightMode::RateControl || Mode == EFlightMode::JoyStickAcroControl)
                {
                    Controller->SetDesiredRollRate(0.f);
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Y"))
            {
                if (Mode == EFlightMode::AutoWaypoint)
                {
                    FVector sp = Controller->GetCurrentSetPoint(); sp.Y = 0.f; Controller->SetDestination(sp);
                }
                else if (Mode == EFlightMode::VelocityControl)
                {
                    FVector v = Controller->GetDesiredVelocity(); v.Y = 0.f; Controller->SetDesiredVelocity(v);
                }
                else if (Mode == EFlightMode::AngleControl || Mode == EFlightMode::JoyStickAngleControl)
                {
                    Controller->SetDesiredPitchAngle(0.f);
                }
                else if (Mode == EFlightMode::RateControl || Mode == EFlightMode::JoyStickAcroControl)
                {
                    Controller->SetDesiredPitchRate(0.f);
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Z"))
            {
                if (Mode == EFlightMode::AutoWaypoint)
                {
                    FVector sp = Controller->GetCurrentSetPoint(); sp.Z = 0.f; Controller->SetDestination(sp);
                }
                else // Velocity / Angle / Rate variants share Z velocity
                {
                    FVector v = Controller->GetDesiredVelocity(); v.Z = 0.f; Controller->SetDesiredVelocity(v);
                }
            }
            // Yaw is not present in AutoWaypoint block; show only when applicable
            if (Mode != EFlightMode::AutoWaypoint)
            {
                ImGui::SameLine();
                if (ImGui::SmallButton("Yaw"))
                {
                    Controller->SetDesiredYawRate(0.f);
                }
            }
        }

        // Persistent bottom control buttons (always visible)
        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(16.f, 9.f));
        if (ImGui::Button("Switch Camera"))
        {
            if (ActivePawn) ActivePawn->SwitchCamera();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset High"))
        {
            // Do not force hover mode when resetting
            if (Controller) Controller->SetHoverMode(false, 0.0f);
            if (ActivePawn) ActivePawn->ResetRotation();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Low"))
        {
            // Do not force hover mode when resetting
            if (Controller) Controller->SetHoverMode(false, 0.0f);
            if (Controller) Controller->ResetControllerState();
            if (ActivePawn) ActivePawn->ResetPosition();
        }
        ImGui::PopStyleVar();
        
        // Per-frame debug visuals when enabled
        if (State.bDebug)
        {
            if (Controller) { Controller->SetDebugVisualsEnabled(true); Controller->DrawMagneticDebugVisuals(); }
        }
    }
    ImGui::End();
}

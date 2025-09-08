// Fill out your copyright notice in the Description page of Project Settings.


#include "ImGuiHud/ControlPanelUI.h"

#include "imgui.h"
#include "QuadSimCore/Public/Core/DroneManager.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "Core/DroneJSONConfig.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "GameFramework/Actor.h"

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
    ImGui::SetNextWindowPos(ImVec2(IO.DisplaySize.x - PanelW - RightPad, PanelY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(PanelW, 600.f), ImGuiCond_FirstUseEver);

    FString Title = FString::Printf(TEXT("Control Panel - %s##ControlPanel"), *ActivePawn->GetName());
    if (ImGui::Begin(TCHAR_TO_UTF8(*Title), &bOpen))
    {
        // Remember current Y for next frame to respect user-adjusted vertical position
        PanelY = ImGui::GetWindowPos().y;
        // Drone selector at the top
        if (ImGui::BeginCombo("Active Drone", TCHAR_TO_UTF8(*ActivePawn->GetName())))
        {
            for (int32 i = 0; i < Drones.Num(); ++i)
            {
                AQuadPawn* P = Drones[i];
                bool bSelected = (i == ActiveIndex);
                if (ImGui::Selectable(TCHAR_TO_UTF8(*P->GetName()), bSelected))
                {
                    DM->SelectedDroneIndex = i;
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
        }
        // Note: Gamepad visualization/modes are controlled by selecting the Gamepad Angle/Acro buttons below.

        ImGui::Separator();
        // Top horizontal 4 buttons for flight modes (slightly tighter default helps sharpness)
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
        }
        else
        {
            ImGui::BeginDisabled(true);
            ImGui::Button("Position"); ImGui::SameLine(); ImGui::Button("Velocity");
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Gamepad Angle"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::JoyStickAngleControl); }
            }
            ImGui::SameLine();
            if (ImGui::Button("Gamepad Acro"))
            {
                if (Controller) { Controller->SetUseExternalController(false); Controller->SetFlightMode(EFlightMode::JoyStickAcroControl); }
            }
        }

        ImGui::Separator();
        // Control Settings (above control sliders)
        if (ImGui::CollapsingHeader("Control Settings", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const auto& Cfg = UDroneJSONConfig::Get().Config;
            static float UiMaxVel   = Cfg.FlightParams.MaxVelocity;
            static float UiMaxAngle = Cfg.FlightParams.MaxAngle;
            static float UiMaxRate  = Cfg.FlightParams.MaxAngleRate;

            const float maxVelBound = FMath::Max(0.1f, Cfg.FlightParams.MaxVelocityBound);
            if (ImGui::SliderFloat("Max Velocity (m/s)", &UiMaxVel, 0.0f, maxVelBound, "%.2f"))
            {
                if (Controller) Controller->SetMaxVelocity(UiMaxVel);
            }

            if (ImGui::SliderFloat("Max Angle (deg)", &UiMaxAngle, 0.0f, Cfg.FlightParams.MaxAngle, "%.1f"))
            {
                if (Controller) Controller->SetMaxAngle(UiMaxAngle);
            }

            if (ImGui::SliderFloat("Max Angle Rate (deg/s)", &UiMaxRate, 0.0f, Cfg.FlightParams.MaxAngleRate, "%.1f"))
            {
                if (Controller) Controller->SetMaxAngleRate(UiMaxRate);
            }
        }

        ImGui::Separator();
        // Flight controls (collapsible)
        if (Controller && ImGui::CollapsingHeader("Flight Controls", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const auto& Cfg = UDroneJSONConfig::Get().Config;
            const float MaxVel   = Cfg.FlightParams.MaxVelocity;        // m/s
            const float MaxAngle = Cfg.FlightParams.MaxAngle;            // deg
            const float MaxRate  = Cfg.FlightParams.MaxAngleRate;        // deg/s
            const float MaxYawRate = Cfg.ControllerParams.YawRate;       // deg/s

            EFlightMode Mode = Controller->GetFlightMode();

            // Hover Mode quick controls (verbatim style)
            const float minAlt = FMath::Max(0.0f, Cfg.FlightParams.MinAltitudeLocal);
            if (State.HoverAlt <= 0.f) { State.HoverAlt = FMath::Max(250.f, minAlt); }
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
            ImGui::SliderFloat("Hover Altitude (m)", &State.HoverAlt, minAlt, FMath::Max(minAlt + 500.f, State.HoverAlt + 100.f), "%.1f");

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

            // Position mode: XYZ position target (hidden in Gamepad UI)
            if (!State.bGamepad && Mode == EFlightMode::AutoWaypoint)
            {
                FVector pos = Controller->GetCurrentSetPoint();
                float px = pos.X, py = pos.Y, pz = pos.Z;
                ImGui::SliderFloat("Position X (m)", &px, -10000.f, 10000.f, "%.2f");
                ImGui::SliderFloat("Position Y (m)", &py, -10000.f, 10000.f, "%.2f");
                ImGui::SliderFloat("Position Z (m)", &pz, -10000.f, 10000.f, "%.2f");
                Controller->SetDestination(FVector(px, py, pz));
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
                double rollRate = Controller->GetDesiredRollRate();
                double pitchRate = Controller->GetDesiredPitchRate();
                float yawR  = Controller->GetDesiredYawRate();
                float rr = (float)rollRate, pr = (float)pitchRate;
                ImGui::SliderFloat("Roll Rate (deg/s)",  &rr, -MaxRate, MaxRate, "%.1f");
                ImGui::SliderFloat("Pitch Rate (deg/s)", &pr, -MaxRate, MaxRate, "%.1f");
                ImGui::SliderFloat("Yaw Rate (deg/s)",   &yawR, -MaxYawRate, MaxYawRate, "%.1f");
                Controller->SetDesiredRollRate(rr);
                Controller->SetDesiredPitchRate(pr);
                Controller->SetDesiredYawRate(yawR);

                FVector vel = Controller->GetDesiredVelocity();
                float zVel = vel.Z;
                ImGui::SliderFloat("Z Velocity (m/s)", &zVel, -MaxVel, MaxVel, "%.2f");
                vel.Z = zVel; vel.X = 0.f; vel.Y = 0.f;
                Controller->SetDesiredVelocity(vel);
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
            if (Controller) Controller->SetHoverMode(true, 250.0f);
            if (ActivePawn) ActivePawn->ResetRotation();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Low"))
        {
            const float lowAlt = FMath::Max(0.0f, UDroneJSONConfig::Get().Config.FlightParams.MinAltitudeLocal);
            if (Controller) Controller->SetHoverMode(true, lowAlt);
            if (ActivePawn) ActivePawn->ResetPosition();
        }
        ImGui::PopStyleVar();

        // (removed duplicate Control Settings block)
    }
    ImGui::End();
}

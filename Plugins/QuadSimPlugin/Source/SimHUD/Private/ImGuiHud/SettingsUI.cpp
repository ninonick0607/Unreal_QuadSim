#include "ImGuiHud/SettingsUI.h"

#include "imgui.h"
#include "ImGuiHud/ImGuiBootstrapSubsystem.h"
#include "ImGuiHud/ControlPanelUI.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "QuadSimCore/Public/Core/DroneManager.h"
#include "QuadSimCore/Public/QuadSimPlayerController.h"
#include "Pawns/QuadPawn.h"
#include "Kismet/GameplayStatics.h"
#include "Core/DroneJSONConfig.h"
#include "Misc/ConfigCacheIni.h"
// No soft object loading here; classes are set on BP_DroneManager

void USimSettingsUI::TickAndDraw(UWorld* World, USimHUDTaskbarSubsystem* TaskbarSubsystem)
{
    if (!bOpen || !World || !TaskbarSubsystem) return;

    ImGui::SetNextWindowSize(ImVec2(620.f, 520.f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings##QuadSim", &bOpen))
    {
        if (ImGui::BeginTabBar("##SettingsTabs"))
        {
            if (ImGui::BeginTabItem("Config"))
            {
                DrawConfigTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Toggables"))
            {
                DrawTogglablesTab(World, TaskbarSubsystem);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

void USimSettingsUI::ApplyStartupPreferences(UWorld* World, USimHUDTaskbarSubsystem* TaskbarSubsystem)
{
    if (!World || !TaskbarSubsystem) return;
    // Load persisted toggles before applying
    LoadPersistent();
    // Class selection is done in BP_DroneManager details; nothing to load here

    // Apply persistent panel states at startup
    if (bPersistentControlPanel)
    {
        if (UControlPanelUI* CP = TaskbarSubsystem->GetControlPanels())
        {
            if (!CP->IsOpen()) CP->ToggleOpen();
        }
    }
    if (bPersistentStateHUD)
    {
        TaskbarSubsystem->SetStateHUDVisible(true);
    }

    if (bAutoSpawnPossessOnStart)
    {
        if (ADroneManager* DM = ADroneManager::Get(World))
        {
            if (AQuadPawn* NewDrone = DM->SpawnDrone(FVector::ZeroVector, FRotator::ZeroRotator))
            {
                if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
                {
                    if (PC && NewDrone && PC->GetPawn() != NewDrone)
                    {
                        PC->Possess(NewDrone);
                        PC->SetViewTarget(NewDrone);
                        if (AQuadSimPlayerController* QPC = Cast<AQuadSimPlayerController>(PC))
                        {
                            QPC->ApplyGameOnly();
                        }
                    }
                }
            }
        }
    }

    // Apply obstacle spawning preference at startup
    if (ADroneManager* DM = ADroneManager::Get(World))
    {
        DM->SetSpawnObstacles(bSpawnObstacles, /*bApplyImmediately*/ true);
    }
}

void USimSettingsUI::DrawTogglablesTab(UWorld* World, USimHUDTaskbarSubsystem* TaskbarSubsystem)
{
    // Auto-spawn and possess on start
    ImGui::Checkbox("Auto-Spawn & Possess Drone On Start", &bAutoSpawnPossessOnStart);
    if (ImGui::IsItemDeactivatedAfterEdit()) SavePersistent();
    ImGui::SameLine();
    if (ImGui::Button("Spawn & Possess Now"))
    {
        if (ADroneManager* DM = ADroneManager::Get(World))
        {
            if (AQuadPawn* NewDrone = DM->SpawnDrone(FVector::ZeroVector, FRotator::ZeroRotator))
            {
                if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
                {
                    if (PC && NewDrone && PC->GetPawn() != NewDrone)
                    {
                        PC->Possess(NewDrone);
                        PC->SetViewTarget(NewDrone);
                        if (AQuadSimPlayerController* QPC = Cast<AQuadSimPlayerController>(PC))
                        {
                            QPC->ApplyGameOnly();
                        }
                    }
                }
            }
        }
    }

    ImGui::Separator();

    // Persistent toggles
    bool prevCP = bPersistentControlPanel;
    bool prevState = bPersistentStateHUD;
    ImGui::Checkbox("Persistent Control Panel", &bPersistentControlPanel);
    if (ImGui::IsItemDeactivatedAfterEdit()) SavePersistent();
    ImGui::Checkbox("Persistent State Info", &bPersistentStateHUD);
    if (ImGui::IsItemDeactivatedAfterEdit()) SavePersistent();

    // Apply immediate effect when toggled
    if (prevCP != bPersistentControlPanel)
    {
        if (UControlPanelUI* CP = TaskbarSubsystem->GetControlPanels())
        {
            bool isOpen = CP->IsOpen();
            if (bPersistentControlPanel && !isOpen) CP->ToggleOpen();
            if (!bPersistentControlPanel && isOpen) CP->ToggleOpen();
        }
    }
    if (prevState != bPersistentStateHUD)
    {
        TaskbarSubsystem->SetStateHUDVisible(bPersistentStateHUD);
    }

    ImGui::Separator();
    ImGui::Text("Spawning");
    // Spawn Obstacles toggle (applies immediately)
    bool prevSpawnOb = bSpawnObstacles;
    ImGui::Checkbox("Spawn Obstacles", &bSpawnObstacles);
    if (ImGui::IsItemDeactivatedAfterEdit())
    {
        SavePersistent();
        if (ADroneManager* DM = ADroneManager::Get(World))
        {
            DM->SetSpawnObstacles(bSpawnObstacles, /*bApplyImmediately*/ true);
        }
    }
    // Classes are selected in BP_DroneManager details; no class path UI here.
}

void USimSettingsUI::DrawConfigTab()
{
    auto& Cfg = UDroneJSONConfig::Get().Config;
    ImGui::Text("Flight Parameters"); ImGui::Separator();
    ImGui::InputFloat("Max Velocity Bound", &Cfg.FlightParams.MaxVelocityBound);
    if (ImGui::IsItemDeactivatedAfterEdit())
    {
        Cfg.FlightParams.MaxVelocity = FMath::Clamp(Cfg.FlightParams.MaxVelocity, 0.0f, Cfg.FlightParams.MaxVelocityBound);
    }
    ImGui::InputFloat("Max Velocity", &Cfg.FlightParams.MaxVelocity);
    ImGui::InputFloat("Max Angle Bound", &Cfg.FlightParams.MaxAngleBound);
    if (ImGui::IsItemDeactivatedAfterEdit())
    {
        Cfg.FlightParams.MaxAngle = FMath::Clamp(Cfg.FlightParams.MaxAngle, 0.0f, Cfg.FlightParams.MaxAngleBound);
    }
    ImGui::InputFloat("Max Angle", &Cfg.FlightParams.MaxAngle);
    ImGui::InputFloat("Max Angle Rate", &Cfg.FlightParams.MaxAngleRate);
    ImGui::InputFloat("Max PID Output", &Cfg.FlightParams.MaxPIDOutput);
    ImGui::InputFloat("Max Thrust", &Cfg.FlightParams.MaxThrust);
    ImGui::InputFloat("Altitude Threshold", &Cfg.FlightParams.AltitudeThreshold);
    ImGui::InputFloat("Min Altitude Local", &Cfg.FlightParams.MinAltitudeLocal);
    ImGui::InputFloat("Acceptable Distance", &Cfg.FlightParams.AcceptableDistance);
    ImGui::Separator();
    ImGui::Text("Controller Parameters"); ImGui::Separator();
    ImGui::InputFloat("Altitude Rate", &Cfg.ControllerParams.AltitudeRate);
    ImGui::InputFloat("Yaw Rate", &Cfg.ControllerParams.YawRate);
    ImGui::InputFloat("Min Vel For Yaw", &Cfg.ControllerParams.MinVelocityForYaw);
    ImGui::Separator();
    ImGui::Text("Obstacle Parameters"); ImGui::Separator();
    ImGui::InputFloat("Inner Boundary", &Cfg.ObstacleParams.InnerBoundarySize);
    ImGui::InputFloat("Outer Boundary", &Cfg.ObstacleParams.OuterBoundarySize);
    ImGui::InputFloat("Spawn Height", &Cfg.ObstacleParams.SpawnHeight);
    ImGui::Separator();
    if (ImGui::Button("Save")) { UDroneJSONConfig::Get().SaveConfig(); }
    ImGui::SameLine();
    if (ImGui::Button("Reload")) { UDroneJSONConfig::Get().ReloadConfig(); }
}

void USimSettingsUI::LoadPersistent()
{
    const TCHAR* Section = TEXT("SimHUDSettings");
    bool v = false;
    if (GConfig->GetBool(Section, TEXT("PersistentControlPanel"), v, GGameUserSettingsIni))
    {
        bPersistentControlPanel = v;
    }
    if (GConfig->GetBool(Section, TEXT("PersistentStateHUD"), v, GGameUserSettingsIni))
    {
        bPersistentStateHUD = v;
    }
    if (GConfig->GetBool(Section, TEXT("AutoSpawnPossessOnStart"), v, GGameUserSettingsIni))
    {
        bAutoSpawnPossessOnStart = v;
    }
    if (GConfig->GetBool(Section, TEXT("SpawnObstacles"), v, GGameUserSettingsIni))
    {
        bSpawnObstacles = v;
    }
}

void USimSettingsUI::SavePersistent()
{
    const TCHAR* Section = TEXT("SimHUDSettings");
    GConfig->SetBool(Section, TEXT("PersistentControlPanel"), bPersistentControlPanel, GGameUserSettingsIni);
    GConfig->SetBool(Section, TEXT("PersistentStateHUD"), bPersistentStateHUD, GGameUserSettingsIni);
    GConfig->SetBool(Section, TEXT("AutoSpawnPossessOnStart"), bAutoSpawnPossessOnStart, GGameUserSettingsIni);
    GConfig->SetBool(Section, TEXT("SpawnObstacles"), bSpawnObstacles, GGameUserSettingsIni);
    GConfig->Flush(false, GGameUserSettingsIni);
}

// No class pref I/O; left intentionally empty

#include "ImGuiHud/ImGuiBootstrapSubsystem.h"

#include "ImGuiDelegates.h"
#include "imgui.h"
#include "implot.h"
#include "Containers/Ticker.h"
#include "Modules/ModuleManager.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "ImGuiHud/Style/SimImGuiStyle.h"
#include "ImGuiHud/ControlPanelUI.h"
#include "ImGuiHud/SettingsUI.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "QuadSimCore/Public/Core/DroneManager.h"
#include "Pawns/QuadPawn.h"
#include "Sensors/SensorManagerComponent.h"
#include "Sensors/IMUSensor.h"
#include "Sensors/BaroSensor.h"
#include "Sensors/GPSSensor.h"
#include "Controllers/QuadDroneController.h"
#include "Core/DroneJSONConfig.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "QuadSimCore/Public/QuadSimPlayerController.h"
#include "Kismet/GameplayStatics.h"
// For PID settings and saving
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include <string>
#include <limits>

bool USimHUDTaskbarSubsystem::bPaused = false;
int32 USimHUDTaskbarSubsystem::SpeedMode = 0;
float USimHUDTaskbarSubsystem::SpeedScale = 1.0f;
bool USimHUDTaskbarSubsystem::bSpeedInit = false;

void USimHUDTaskbarSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    TickerHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &USimHUDTaskbarSubsystem::TryBindImGui),
        0.0f
    );
}

void USimHUDTaskbarSubsystem::Deinitialize()
{
    if (TickerHandle.IsValid())
    {
        FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
        TickerHandle.Reset();
    }
    if (DrawHandle.IsValid() && BoundWorld.IsValid())
    {
        FImGuiDelegates::OnWorldDebug(BoundWorld.Get()).Remove(DrawHandle);
        DrawHandle.Reset();
        BoundWorld = nullptr;
    }
    Super::Deinitialize();
}

bool USimHUDTaskbarSubsystem::TryBindImGui(float /*DeltaSeconds*/)
{
    if (!FModuleManager::Get().IsModuleLoaded("ImGui"))
        return true;
    if (!GEngine || !GEngine->GameViewport)
        return true;

    if (!DrawHandle.IsValid())
    {
        BoundWorld = GetWorld();
        DrawHandle = FImGuiDelegates::OnWorldDebug(BoundWorld.Get()).AddUObject(
            this, &USimHUDTaskbarSubsystem::HandleImGuiDraw);
        UE_LOG(LogTemp, Log, TEXT("[ImGuiBootstrapSubsystem] Bound to OnWorldDebug for taskbar rendering (world: %s)."), *BoundWorld->GetName());
    }

    FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
    TickerHandle.Reset();
    return false;
}

void USimHUDTaskbarSubsystem::HandleImGuiDraw()
{
    UWorld* World = GetWorld();
    if (!World) return;
    if (BoundWorld.Get() != World) return;
    if (!(World->IsGameWorld() || World->WorldType == EWorldType::PIE)) return;

    AQuadPawn* ResolvedPawn = nullptr;
    UQuadDroneController* ResolvedCtrl = nullptr;
    FSensorData ResolvedSensorData{};
    FRotator CurrentAtt = FRotator::ZeroRotator;

    if (ADroneManager* DM = ADroneManager::Get(World))
    {
        const TArray<AQuadPawn*> Drones = DM->GetDroneList();
        if (Drones.Num() > 0)
        {
            const int32 ActiveIndex = FMath::Clamp(DM->SelectedDroneIndex, 0, Drones.Num() - 1);
            ResolvedPawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
            ResolvedCtrl = ResolvedPawn ? ResolvedPawn->QuadController : nullptr;

            if (ResolvedPawn && ResolvedPawn->SensorManager)
            {
                ResolvedSensorData = ResolvedPawn->SensorManager->GetCurrentSensorData();
            }

            CurrentAtt = (ResolvedPawn && ResolvedPawn->SensorManager)
                ? ResolvedSensorData.IMUAttitude
                : (ResolvedPawn ? ResolvedPawn->GetActorRotation() : FRotator::ZeroRotator);


            const float DesiredXVel = ResolvedCtrl ? ResolvedCtrl
            const float DesiredRollDeg  = ResolvedCtrl ? ResolvedCtrl->GetDesiredRoll()  : 0.f;
            const float DesiredPitchDeg = ResolvedCtrl ? ResolvedCtrl->GetDesiredPitch() : 0.f;

            // Get angular rates for acro plots - with null checks
            FVector CurrentRateDeg = FVector::ZeroVector;
            if (ResolvedPawn && ResolvedPawn->SensorManager)
            {
                CurrentRateDeg = ResolvedSensorData.IMUAngVelDEGS;
            }

            const float DesiredRollRateDeg  = (ResolvedCtrl && IsValid(ResolvedCtrl)) ? static_cast<float>(ResolvedCtrl->GetDesiredRollRate())  : 0.f;
            const float DesiredPitchRateDeg = (ResolvedCtrl && IsValid(ResolvedCtrl)) ? static_cast<float>(ResolvedCtrl->GetDesiredPitchRate()) : 0.f;
            const float DesiredYawRateDeg   = (ResolvedCtrl && IsValid(ResolvedCtrl)) ? ResolvedCtrl->GetDesiredYawRate() : 0.f;

            UpdateControlPlotData(World,
                                  ResolvedPawn,
                                  ResolvedCtrl,
                                  World->GetDeltaSeconds(),
                                  CurrentAtt,
                                  DesiredRollDeg,
                                  DesiredPitchDeg,
                                  CurrentRateDeg,
                                  DesiredRollRateDeg,
                                  DesiredPitchRateDeg,
                                  DesiredYawRateDeg);
        }
    }

    if (!ControlPanels) ControlPanels = NewObject<UControlPanelUI>(this);
    if (!SettingsUI)    SettingsUI    = NewObject<USimSettingsUI>(this);

    if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
    {
        const FIntPoint Vp = GEngine->GameViewport->Viewport->GetSizeXY();
        ViewSize = FVector2D(Vp.X, Vp.Y);
    }

    static FSimImGuiStyle Theme;
    Theme.Apply();

    HandleTaskbar(World, Theme);
    ControlButtons(World, Theme);
    HandleStateData(World);
    JoyStickHandles(World);

    // PID plots (your old RenderImPlot content)
    if (bShowPIDPlots)
    {
        const float MaxAngleDeg = 45.0f; // pull from config if you have one
        DrawControlPlotsWindow(MaxAngleDeg);
    }

    // Dedicated PID Config window (wrapper)
    if (bShowPIDSettings)
    {
        DrawPIDConfigWindow(World);
    }
}



void USimHUDTaskbarSubsystem::HandleTaskbar(UWorld* World, const FSimImGuiStyle& Theme)
{
    const ImGuiStyle& Style = ImGui::GetStyle();
    const float padX = Style.FramePadding.x;
    const float padY = Style.FramePadding.y;
    const float itemX = Style.ItemSpacing.x;
    constexpr float LabelBoxW = 180.f;
    constexpr float WMLabelW  = 170.f;
    const float LabelH = BarHeight - 12.f;
    constexpr float SpeedBoxW = 56.f;
    ImGuiWindowFlags Flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
    ImGui::SetNextWindowSize(ImVec2(ViewSize.X, BarHeight));
    // Add a subtle dark outline around the entire taskbar window and inner padding
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.5f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.f, 6.f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.06f, 0.07f, 0.09f, 1.0f));
    ImGui::PushID(World); // ensure unique IDs per-world if ever called twice
    if (ImGui::Begin("QuadSim Taskbar##Main", nullptr, Flags))
    {
        if (!bSpeedInit)
        {
            float td = 1.0f;
            if (World && World->GetWorldSettings())
            {
                td = World->GetWorldSettings()->TimeDilation;
            }
            if (td <= 0.0f) td = 1.0f;
            SpeedScale = td;
            bSpeedInit = true;
        }
        static bool  bSimMgrActive  = false;
        static bool  bWorldMgrActive= false;

        // Use a two-column table: controls (stretch) and status (fixed width)
        const float StatusW = 200.f;
        if (ImGui::BeginTable("##TaskbarTable", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoPadInnerX | ImGuiTableFlags_BordersInnerV))
        {
            ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Status",   ImGuiTableColumnFlags_WidthFixed, StatusW);
            ImGui::TableNextRow();

            // Controls column
            ImGui::TableSetColumnIndex(0);
            // Pre-compute sizing to vertically center content

            const char* PauseTxt  = bPaused ? "Play" : "Pause";
            float btnH   = ImGui::GetTextLineHeight() + padY * 2.f;
            float labelTopY = FMath::Max(0.f, (BarHeight - LabelH) * 0.5f);
            float btnTopY   = labelTopY + FMath::Max(0.f, (LabelH - btnH) * 0.5f);

            // Left-anchored Settings button (opens panel), vertically centered
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Settings")) { if (SettingsUI) SettingsUI->ToggleOpen(); }
            ImGui::SameLine();
            // Pre-compute total width to center the main control group (excluding Settings)
            auto BtnW = [&](const char* txt){ return ImGui::CalcTextSize(txt).x + padX*2.f; };
            float totalW = 0.f;
            totalW += LabelBoxW;                                   // Sim label
            totalW += itemX;
            totalW += BtnW("Slower");
            totalW += itemX;
            totalW += BtnW(PauseTxt);
            totalW += itemX;
            totalW += BtnW("Faster>>");
            totalW += itemX;
            totalW += SpeedBoxW;                                   // speed box
            totalW += itemX;
            totalW += BtnW("Step");
            totalW += itemX;
            totalW += BtnW("Reset");
            totalW += itemX + 40.f;                                // extra spacing before WM
            totalW += WMLabelW;                                    // World label
            totalW += itemX;
            totalW += BtnW("Spawn Drone");

            // Center main group excluding Settings which is anchored left
            float colStartX = ImGui::GetCursorPosX();
            float avail = ImGui::GetContentRegionAvail().x;
            float remainingW = avail - (ImGui::CalcTextSize("Settings").x + padX*2.f + itemX);
            float startXWithin = FMath::Max(0.f, (remainingW - totalW) * 0.5f);
            ImGui::SetCursorPosY(labelTopY);
            ImGui::SetCursorPosX(colStartX + (ImGui::CalcTextSize("Settings").x + padX*2.f + itemX) + startXWithin);
            // Centered "Simulation Manager" clickable label with rounded background
            {
                //@TODO SimManager
                ImGui::InvisibleButton("##SimMgrLabelBtn", ImVec2(LabelBoxW, LabelH));
                ImVec2 min = ImGui::GetItemRectMin();
                ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                const float r = 8.f;
                ImU32 bg = ImGui::GetColorU32(Theme.ChildBg);
                ImU32 br = ImGui::GetColorU32(Theme.Border);
                dl->AddRectFilled(min, max, bg, r);
                dl->AddRect(min, max, br, r, 0, 1.0f);
                ImVec2 tsize = ImGui::CalcTextSize("Simulation Manager");
                ImVec2 tpos(min.x + (LabelBoxW - tsize.x) * 0.5f, min.y + (LabelH - tsize.y) * 0.5f);
                ImU32 tc = bSimMgrActive ? ImGui::GetColorU32(ImVec4(0.2f,0.9f,0.3f,1.0f))
                                         : ImGui::GetColorU32(Style.Colors[ImGuiCol_Text]);
                dl->AddText(tpos, tc, "Simulation Manager");
                if (ImGui::IsItemClicked()) bSimMgrActive = !bSimMgrActive;
                ImGui::SameLine();
            }

            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Slower"))
            {
                // Reduce current speed scale by 0.10x and apply via SimulationManager
                SpeedMode = -1;
                SpeedScale = FMath::Clamp(SpeedScale - 0.10f, 0.05f, 100.0f);
                if (ASimulationManager* SM = ASimulationManager::Get(World))
                {
                    SM->SetSimulationMode(ESimulationMode::FastForward);
                    SM->SetTimeScale(SpeedScale);
                }
            }
            ImGui::SameLine();
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button(bPaused ? "Play" : "Pause"))
            {
                if (ASimulationManager* SM = ASimulationManager::Get(World))
                {
                    if (bPaused)
                    {
                        SM->ResumePhysics();
                        SM->SetSimulationMode(ESimulationMode::Realtime);
                    }
                    else
                    {
                        SM->PausePhysics();
                        SM->SetSimulationMode(ESimulationMode::Paused);
                    }
                }
                bPaused = !bPaused;
            }
            ImGui::SameLine();
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Faster>>"))
            {
                SpeedMode = 1;
                SpeedScale = FMath::Max(1.0f, SpeedScale + 0.25f);
                if (ASimulationManager* SM = ASimulationManager::Get(World))
                {
                    SM->SetSimulationMode(ESimulationMode::FastForward);
                    SM->SetTimeScale(SpeedScale);
                }
            }
            ImGui::SameLine();
            // Speed display box with same rounded background (vertically centered with buttons)
            {
                const float BoxH = 28.f;
                ImGui::SetCursorPosY(btnTopY + (btnH - BoxH) * 0.5f);
                ImGui::InvisibleButton("##SpeedBox", ImVec2(SpeedBoxW, BoxH));
                ImVec2 min = ImGui::GetItemRectMin();
                ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                const float r = 8.f;
                ImU32 bg = ImGui::GetColorU32(Theme.ChildBg);
                ImU32 br = ImGui::GetColorU32(Theme.Border);
                dl->AddRectFilled(min, max, bg, r);
                dl->AddRect(min, max, br, r, 0, 1.0f);
                char buf[32]; snprintf(buf, sizeof(buf), "x%.2f", SpeedScale);
                ImVec2 tsize = ImGui::CalcTextSize(buf);
                ImVec2 tpos(min.x + (SpeedBoxW - tsize.x) * 0.5f, min.y + (BoxH - tsize.y) * 0.5f);
                dl->AddText(tpos, ImGui::GetColorU32(Style.Colors[ImGuiCol_Text]), buf);
            }
            ImGui::SameLine();
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Step"))
            {
                if (ASimulationManager* SM = ASimulationManager::Get(World))
                {
                    SM->RequestSimulationStep();
                }
            }
            ImGui::SameLine();
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Reset"))
            {
                SpeedMode = 0; bPaused = false; SpeedScale = 1.0f;
                if (ASimulationManager* SM = ASimulationManager::Get(World))
                {
                    SM->SetSimulationMode(ESimulationMode::Realtime);
                    SM->SetTimeScale(1.0f);
                    SM->ResetSimulation();
                }
            }
            
            //@TODO Drone Manager
            // Extra spacing before World Manager
            ImGui::SameLine(); ImGui::Dummy(ImVec2(40.f, 0.f)); ImGui::SameLine();
            // Centered clickable World Manager label with rounded background
            {
                ImGui::SetCursorPosY(labelTopY);
                ImGui::InvisibleButton("##WorldMgrLabelBtn", ImVec2(WMLabelW, LabelH));
                ImVec2 min = ImGui::GetItemRectMin();
                ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                const float r = 8.f;
                ImU32 bg = ImGui::GetColorU32(Theme.ChildBg);
                ImU32 br = ImGui::GetColorU32(Theme.Border);
                dl->AddRectFilled(min, max, bg, r);
                dl->AddRect(min, max, br, r, 0, 1.0f);
                ImVec2 tsize = ImGui::CalcTextSize("World Manager");
                ImVec2 tpos(min.x + (WMLabelW - tsize.x) * 0.5f, min.y + (LabelH - tsize.y) * 0.5f);
                ImU32 tc = bWorldMgrActive ? ImGui::GetColorU32(ImVec4(0.2f,0.9f,0.3f,1.0f))
                                           : ImGui::GetColorU32(Style.Colors[ImGuiCol_Text]);
                dl->AddText(tpos, tc, "World Manager");
                if (ImGui::IsItemClicked()) bWorldMgrActive = !bWorldMgrActive;
                ImGui::SameLine();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Spawn Drone"))
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
                                // Return to game-only input for control
                                if (AQuadSimPlayerController* QPC = Cast<AQuadSimPlayerController>(PC))
                                {
                                    QPC->ApplyGameOnly();
                                }
                                else
                                {
                                    FInputModeGameOnly Mode; Mode.SetConsumeCaptureMouseDown(true);
                                    PC->SetInputMode(Mode);
                                    PC->bShowMouseCursor = false;
                                }
                            }
                        }
                    }
                }
            }

            // Status column (FPS) with rounded background like labels
            ImGui::TableSetColumnIndex(1);
            {
                const float BoxH = BarHeight - 12.f;
                ImGui::InvisibleButton("##FPSBox", ImVec2(StatusW, BoxH));
                ImVec2 min = ImGui::GetItemRectMin();
                ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                const float r = 8.f;
                ImU32 bg = ImGui::GetColorU32(Theme.ChildBg);
                ImU32 br = ImGui::GetColorU32(Theme.Border);
                dl->AddRectFilled(min, max, bg, r);
                dl->AddRect(min, max, br, r, 0, 1.0f);
                char fps[32];
                const float dt = World->GetDeltaSeconds();
                if (dt > 0.f) snprintf(fps, sizeof(fps), "FPS: %.1f", 1.f / dt); else snprintf(fps, sizeof(fps), "FPS: --");
                ImVec2 tsize = ImGui::CalcTextSize(fps);
                ImVec2 tpos(min.x + (StatusW - tsize.x) * 0.5f, min.y + (BoxH - tsize.y) * 0.5f);
                dl->AddText(tpos, ImGui::GetColorU32(Style.Colors[ImGuiCol_Text]), fps);
            }
            ImGui::EndTable();
        }

        // Draw a thin outline around the taskbar window (explicit overlay)
        {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 wp = ImGui::GetWindowPos();
            ImVec2 ws = ImGui::GetWindowSize();
            ImU32  col = ImGui::GetColorU32(ImVec4(0.06f, 0.07f, 0.09f, 1.0f));
            dl->AddRect(wp, ImVec2(wp.x + ws.x, wp.y + ws.y), col, 0.0f, 0, 1.0f);
        }
    }
    ImGui::End();
    ImGui::PopID();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}
void USimHUDTaskbarSubsystem::ControlButtons(UWorld* World, const FSimImGuiStyle& Theme)
{
       // Bottom-right "Control Panel" floating button, visible only after a drone exists
    if (ADroneManager* DM = ADroneManager::Get(World))
    {
        if (DM->GetDroneList().Num() > 0)
        {
            const float Pad = 12.f;
            const char* cpLabel = "Control Panel";
            const char* sdLabel = "State Data";
            ImGui::SetNextWindowPos(ImVec2(ViewSize.X - 240.f - Pad, ViewSize.Y - 120.f));
            ImGui::SetNextWindowSize(ImVec2(240.f, 104.f));
            ImGui::SetNextWindowBgAlpha(0.0f); // transparent background
            ImGuiWindowFlags wflags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoFocusOnAppearing |
                                      ImGuiWindowFlags_NoBackground;
            if (ImGui::Begin("##QuadSimControlPanelBtn", nullptr, wflags))
            {
                // Make buttons ~30% larger via frame padding
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(24.f, 14.f));
                float totalH = ImGui::GetTextLineHeightWithSpacing() * 2 + ImGui::GetStyle().FramePadding.y * 4 + 8.f;
                float cy = (ImGui::GetContentRegionAvail().y - totalH) * 0.5f;
                ImGui::SetCursorPosY(FMath::Max(0.f, cy));
                float cx = (ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(cpLabel).x - ImGui::GetStyle().FramePadding.x*2) * 0.5f;

                // Brighten the buttons locally
                auto Lighten = [](ImVec4 c, float k){ return ImVec4(FMath::Clamp(c.x + k, 0.f, 1.f), FMath::Clamp(c.y + k, 0.f, 1.f), FMath::Clamp(c.z + k, 0.f, 1.f), c.w); };
                // Keep darker blue; only a subtle brighten
                ImGui::PushStyleColor(ImGuiCol_Button,        Lighten(Theme.Button,       0.08f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(Theme.ButtonHover,  0.08f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  Lighten(Theme.ButtonActive, 0.08f));

                ImGui::SetCursorPosX(FMath::Max(0.f, cx));
                if (ImGui::Button(cpLabel))
                {
                    if (ControlPanels) ControlPanels->ToggleOpen();
                }
                ImGui::SetCursorPosX(FMath::Max(0.f, cx));
                if (ImGui::Button(sdLabel))
                {
                    bShowStateHUD = !bShowStateHUD;
                }
                ImGui::PopStyleColor(3);
                ImGui::PopStyleVar();
            }
            ImGui::End();
        }
    }

    // Draw per-drone control panels (only appear when drones exist)
    if (ControlPanels)
    {
        ControlPanels->TickAndDraw(World);
    }

    // Apply startup settings once, then draw settings panel
    if (SettingsUI)
    {
        if (!bAppliedStartupSettings)
        {
            SettingsUI->ApplyStartupPreferences(World, this);
            bAppliedStartupSettings = true;
        }
        SettingsUI->TickAndDraw(World, this);
    } 
}
void USimHUDTaskbarSubsystem::HandleStateData(UWorld* World)
{
    if (bShowStateHUD)
    {
        // Resolve active drone similar to control panel
        if (ADroneManager* DM2 = ADroneManager::Get(World))
        {
            const TArray<AQuadPawn*> Drones = DM2->GetDroneList();
            if (Drones.Num() > 0)
            {
                int32 ActiveIndex = FMath::Clamp(DM2->SelectedDroneIndex, 0, Drones.Num()-1);
                AQuadPawn* Pawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
                UQuadDroneController* Ctrl = Pawn ? Pawn->QuadController : nullptr;
                if (Pawn && Ctrl)
                {
                    ImGui::SetNextWindowPos(ImVec2(10.f, 100.f), ImGuiCond_Always);
                    ImGui::SetNextWindowSize(ImVec2(340.f, 460.f), ImGuiCond_FirstUseEver);
                    if (ImGui::Begin("State Data##HUD", &bShowStateHUD, ImGuiWindowFlags_NoCollapse))
                    {
                        //@TODO ThrusterPower
                        FSensorData SensorData;
                        if (Pawn->SensorManager)
                        {
                            SensorData = Pawn->SensorManager->GetCurrentSensorData();
                        }

                        // Thruster Power (vertical bars at top) - normalized by MaxThrust
                        ImGui::Text("Thruster Power");
                        const float MaxThrust = UDroneJSONConfig::Get().Config.FlightParams.MaxThrust;
                        static const char* MotorLabels[] = { "FL", "FR", "BL", "BR" };
                        if (ImGui::BeginTable("##ThrustBars", Ctrl->Thrusts.Num(), ImGuiTableFlags_SizingFixedFit))
                        {
                            for (int32 i = 0; i < Ctrl->Thrusts.Num(); ++i)
                            {
                                ImGui::TableNextColumn();
                                ImGui::PushID(i);
                                float thrust = Ctrl->Thrusts[i];
                                float percent = (MaxThrust > 0.f) ? FMath::Clamp(thrust / MaxThrust, 0.0f, 1.0f) : 0.f;
                                // custom bar
                                ImVec2 p = ImGui::GetCursorScreenPos();
                                ImVec2 barSize(22.f, 100.f);
                                ImVec2 q(p.x + barSize.x, p.y + barSize.y);
                                ImGui::InvisibleButton("##bar", barSize);
                                ImDrawList* dl = ImGui::GetWindowDrawList();
                                dl->AddRect(p, q, IM_COL32(255,255,255,255));
                                float fillH = barSize.y * percent;
                                ImVec2 fillMin(p.x, q.y - fillH);
                                ImVec2 fillMax(q.x, q.y);
                                // Deep yellow fill strictly inside the bar
                                dl->AddRectFilled(fillMin, fillMax, IM_COL32(255, 200, 0, 255));
                                // Newton value centered inside bar
                                char nbuf[24]; snprintf(nbuf, sizeof(nbuf), "%.0fN", thrust);
                                ImVec2 tsize = ImGui::CalcTextSize(nbuf);
                                ImVec2 tpos(p.x + (barSize.x - tsize.x) * 0.5f, fillMin.y + (fillH - tsize.y) * 0.5f);
                                dl->AddText(tpos, IM_COL32(255,255,255,255), nbuf);
                                ImGui::Text("%s", (i < 4 ? MotorLabels[i] : "M"));
                                ImGui::PopID();
                            }
                            ImGui::EndTable();
                        }

                        ImGui::Separator();
                        ImGui::TextColored(ImVec4(0.6f,0.9f,1.0f,1.0f), "State Info");
                        ImGui::Separator();
                        // Drone Mass
                        ImGui::Text("Drone Mass: %.2f kg", Pawn->GetMass());

                        // IMU
                        ImGui::TextColored(ImVec4(0.6f,0.9f,1.0f,1.0f), "IMU");
                        FRotator curAtt = Pawn->SensorManager ? SensorData.IMUAttitude : Pawn->GetActorRotation();
                        FVector currRates = SensorData.IMUAngVelRADS;
                        FVector currRatesDEG = SensorData.IMUAngVelDEGS;
                        FVector currVel = SensorData.IMUVelMS;
                        FVector currAcc = SensorData.IMULinearAccelMS2;
                        FVector desVel = Ctrl->GetDesiredVelocity();
                        ImGui::Text("Current Roll/Pitch: %.2f / %.2f / %.2f deg", curAtt.Roll, curAtt.Pitch,curAtt.Yaw);
                        ImGui::Text("Current Angular Rates X,Y,Z  %.2f / %.2f / %.2f  rad/s", currRates.X, currRates.Y,currRates.Z);
                        ImGui::Text("Current Angular Rates X,Y,Z  %.2f / %.2f / %.2f  deg/s", currRatesDEG.X, currRatesDEG.Y,currRatesDEG.Z);
                        ImGui::Text("Current Velocity  X,Y,Z  %.2f / %.2f / %.2f  m/s", currVel.X, currVel.Y,currVel.Z);
                        ImGui::Text("Current Linear Acceleration X,Y,Z  %.2f / %.2f / %.2f  m/s", currAcc.X, currAcc.Y,currAcc.Z);
                        ImGui::Spacing();
                        ImGui::Text("Desired Roll/Pitch: %.2f / %.2f deg", Ctrl->GetDesiredRoll(), Ctrl->GetDesiredPitch());
                        ImGui::Text("Desired Rate Roll/Pitch: %.2f / %.2f deg/s", (float)Ctrl->GetDesiredRollRate(), (float)Ctrl->GetDesiredPitchRate());
                        ImGui::Text("Desired Velocity: X %.2f Y %.2f Z %.2f m/s", desVel.X, desVel.Y, desVel.Z);
                        ImGui::Separator();

                        // GPS
                        FVector geo = SensorData.GPSLatLong;
                        FVector curPos = SensorData.GPSPosMeters;
                        ImGui::TextColored(ImVec4(1.0f,0.9f,0.4f,1.0f), "GPS");
                        ImGui::Text("Current Position: X %.1f Y %.1f Z %.1f", curPos.X, curPos.Y, curPos.Z);
                        ImGui::Text("Geo Position: Lat %.3f Lon %.3f Alt %.1f", geo.X, geo.Y, geo.Z);
                        ImGui::Separator();

                        //Magnetometer
                        FVector magField = SensorData.MagFieldGauss;
                        float magHeading = SensorData.MagHeadingDeg;
                        float magDeclination = SensorData.MagDeclinationDeg;
                        ImGui::TextColored(ImVec4(1.0f,0.9f,0.4f,1.0f), "Magnetometer");
                        ImGui::Text("Current Mag Field: X %.1f Y %.1f Z %.1f Gauss", magField.X, magField.Y, magField.Z);
                        ImGui::Text("Current Heading: %.1f Deg", magHeading);
                        ImGui::Text("Current Declination: %.1f Deg", magDeclination);

                        ImGui::Separator();
                        //Baro
                        ImGui::TextColored(ImVec4(0.8f,0.8f,1.0f,1.0f), "Barometer Sensor");
                        if (Pawn->SensorManager && Pawn->SensorManager->Barometer)
                        {
                            UBaroSensor* Baro = Pawn->SensorManager->Barometer;
                            ImGui::Text("Pressure: %.2f hPa", SensorData.BaroLastPressureHPa);
                            ImGui::Text("Temperature: %.1f C", SensorData.BaroTemp);
                            ImGui::Text("Altitude: %.1f m", SensorData.BaroAltitudeM);
                        }

                        //@TODO PID Settings
                        // --- PID Settings toggle and UI (copied from ImGuiUtil) ---
                        ImGui::SameLine();
                        ImGui::Dummy(ImVec2(20.f, 0.f));

                        // PID controls: Button opens config window, checkbox toggles plots (errors/old RenderControlPlots)
                        ImGui::Checkbox("PID Gain Config", &bShowPIDSettings);
                        ImGui::Checkbox("PID Plots", &bShowPIDPlots);
                        
                        if (!bShowPIDSettings)
                        {
                            bShowPIDHistoryWindow = false;
                        }
                    }
                    ImGui::End();
                }
            }
        }
    }
}
void USimHUDTaskbarSubsystem::DrawPIDSettingsPanel(UQuadDroneController* InController)
{
    if (!InController)
    {
        ImGui::Text("No controller.");
        return;
    }

    FFullPIDSet* PIDSet = InController->GetPIDSet(InController->GetFlightMode());
    if (!PIDSet)
    {
        ImGui::Text("No PID Set found for this mode.");
        return;
    }

    static bool synchronizeXYGains = false;
    static bool synchronizeAngleGains = false;
    static bool synchronizeRateGains = false;

    // Editable gain ranges
    static float minVelocityGain = 0.01f;
    static float maxVelocityGain = 3.0f;
    static float minAngleGain = 0.01f;
    static float maxAngleGain = 3.0f;
    static float minRateGain = 0.01f;
    static float maxRateGain = 3.0f;
    static float minYawRateGain = 0.01f;
    static float maxYawRateGain = 2.0f;

    auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
    {
        float totalWidth = ImGui::GetContentRegionAvail().x;
        float inputWidth = 80.0f;
        float sliderWidth = totalWidth > (inputWidth + 20.0f) ? totalWidth - inputWidth - 20.0f : 100.0f;
        ImGui::PushItemWidth(sliderWidth);
        bool changed = ImGui::SliderFloat(label, value, minValue, maxValue);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(inputWidth);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
        std::string inputLabel = std::string("##Input_") + label;
        changed |= ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.4f");
        ImGui::PopStyleColor(2);
        ImGui::PopItemWidth();
        return changed;
    };

    // Gain range configuration collapsing header
    if (ImGui::CollapsingHeader("Gain Range Settings", ImGuiTreeNodeFlags_None))
    {
        ImGui::Text("Velocity Gain Ranges");
        ImGui::Indent();
        ImGui::DragFloat("Min Velocity Gain", &minVelocityGain, 0.001f, 0.0f, maxVelocityGain);
        ImGui::DragFloat("Max Velocity Gain", &maxVelocityGain, 0.1f, minVelocityGain, 100.0f);
        ImGui::Unindent();

        ImGui::Separator();
        ImGui::Text("Angle Gain Ranges");
        ImGui::Indent();
        ImGui::DragFloat("Min Angle Gain", &minAngleGain, 0.001f, 0.0f, maxAngleGain);
        ImGui::DragFloat("Max Angle Gain", &maxAngleGain, 0.1f, minAngleGain, 100.0f);
        ImGui::Unindent();

        ImGui::Separator();
        ImGui::Text("Rate Gain Ranges");
        ImGui::Indent();
        ImGui::DragFloat("Min Rate Gain", &minRateGain, 0.0001f, 0.0f, maxRateGain);
        ImGui::DragFloat("Max Rate Gain", &maxRateGain, 0.01f, minRateGain, 10.0f);
        ImGui::DragFloat("Min Yaw Rate Gain", &minYawRateGain, 0.0001f, 0.0f, maxYawRateGain);
        ImGui::DragFloat("Max Yaw Rate Gain", &maxYawRateGain, 0.01f, minYawRateGain, 10.0f);
        ImGui::Unindent();
        ImGui::Separator();
    }

    // Create tab bar for PID settings
    if (ImGui::BeginTabBar("PIDSettingsTabBar", ImGuiTabBarFlags_None))
    {
        // --- Velocity Tab ---
        if (ImGui::BeginTabItem("Velocity"))
        {
            ImGui::Spacing();
            ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);
            ImGui::Separator();

            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "X Axis");
            ImGui::Indent();
            if (PIDSet->XPID)
            {
                float xP = PIDSet->XPID->ProportionalGain;
                float xI = PIDSet->XPID->IntegralGain;
                float xD = PIDSet->XPID->DerivativeGain;
                bool xChanged = false;
                if (DrawPIDGainControl("X P", &xP, minVelocityGain, maxVelocityGain)) xChanged = true;
                if (DrawPIDGainControl("X I", &xI, minVelocityGain, maxVelocityGain)) xChanged = true;
                if (DrawPIDGainControl("X D", &xD, minVelocityGain, maxVelocityGain)) xChanged = true;
                if (xChanged)
                {
                    PIDSet->XPID->ProportionalGain = xP;
                    PIDSet->XPID->IntegralGain = xI;
                    PIDSet->XPID->DerivativeGain = xD;
                    if (synchronizeXYGains && PIDSet->YPID)
                    {
                        PIDSet->YPID->ProportionalGain = xP;
                        PIDSet->YPID->IntegralGain = xI;
                        PIDSet->YPID->DerivativeGain = xD;
                    }
                }
            }
            else
            {
                ImGui::TextDisabled("X PID Controller Unavailable");
            }
            ImGui::Unindent();

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Y Axis");
            ImGui::Indent();
            if (PIDSet->YPID)
            {
                float yP = PIDSet->YPID->ProportionalGain;
                float yI = PIDSet->YPID->IntegralGain;
                float yD = PIDSet->YPID->DerivativeGain;
                bool yChanged = false;
                if (DrawPIDGainControl("Y P", &yP, minVelocityGain, maxVelocityGain)) yChanged = true;
                if (DrawPIDGainControl("Y I", &yI, minVelocityGain, maxVelocityGain)) yChanged = true;
                if (DrawPIDGainControl("Y D", &yD, minVelocityGain, maxVelocityGain)) yChanged = true;
                if (yChanged)
                {
                    PIDSet->YPID->ProportionalGain = yP;
                    PIDSet->YPID->IntegralGain = yI;
                    PIDSet->YPID->DerivativeGain = yD;
                    if (synchronizeXYGains && PIDSet->XPID)
                    {
                        PIDSet->XPID->ProportionalGain = yP;
                        PIDSet->XPID->IntegralGain = yI;
                        PIDSet->XPID->DerivativeGain = yD;
                    }
                }
            }
            else
            {
                ImGui::TextDisabled("Y PID Controller Unavailable");
            }
            ImGui::Unindent();

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 0.3f, 1.0f, 1.0f), "Z Axis");
            ImGui::Indent();
            if (PIDSet->ZPID)
            {
                DrawPIDGainControl("Z P", &PIDSet->ZPID->ProportionalGain, minVelocityGain, maxVelocityGain);
                DrawPIDGainControl("Z I", &PIDSet->ZPID->IntegralGain, minVelocityGain, maxVelocityGain);
                DrawPIDGainControl("Z D", &PIDSet->ZPID->DerivativeGain, minVelocityGain, maxVelocityGain);
            }
            else
            {
                ImGui::TextDisabled("Z PID Controller Unavailable");
            }
            ImGui::Unindent();

            ImGui::EndTabItem();
        }

        // --- Angle Tab ---
        if (ImGui::BeginTabItem("Angle"))
        {
            ImGui::Spacing();
            ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeAngleGains);
            ImGui::Separator();

            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Roll");
            ImGui::Indent();
            if (PIDSet->RollPID)
            {
                if (synchronizeAngleGains && PIDSet->PitchPID)
                {
                    if (DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->PitchPID->ProportionalGain = PIDSet->RollPID->ProportionalGain;
                    }
                    if (DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->PitchPID->IntegralGain = PIDSet->RollPID->IntegralGain;
                    }
                    if (DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->PitchPID->DerivativeGain = PIDSet->RollPID->DerivativeGain;
                    }
                }
                else
                {
                    DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, minAngleGain, maxAngleGain);
                    DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, minAngleGain, maxAngleGain);
                    DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, minAngleGain, maxAngleGain);
                }
            }
            else
            {
                ImGui::TextDisabled("Roll PID Unavailable");
            }
            ImGui::Unindent();

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Pitch");
            ImGui::Indent();
            if (PIDSet->PitchPID)
            {
                if (synchronizeAngleGains && PIDSet->RollPID)
                {
                    if (DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->RollPID->ProportionalGain = PIDSet->PitchPID->ProportionalGain;
                    }
                    if (DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->RollPID->IntegralGain = PIDSet->PitchPID->IntegralGain;
                    }
                    if (DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, minAngleGain, maxAngleGain))
                    {
                        PIDSet->RollPID->DerivativeGain = PIDSet->PitchPID->DerivativeGain;
                    }
                }
                else
                {
                    DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, minAngleGain, maxAngleGain);
                    DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, minAngleGain, maxAngleGain);
                    DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, minAngleGain, maxAngleGain);
                }
            }
            else
            {
                ImGui::TextDisabled("Pitch PID Unavailable");
            }
            ImGui::Unindent();

            ImGui::EndTabItem();
        }

        // --- Acro (Rate) Tab ---
        if (ImGui::BeginTabItem("Acro"))
        {
            ImGui::Spacing();
            ImGui::Checkbox("Synchronize Roll and Pitch Rate Gains", &synchronizeRateGains);
            ImGui::Separator();

            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Roll Rate");
            ImGui::Indent();
            if (PIDSet->RollRatePID)
            {
                bool changed = DrawPIDGainControl("Roll Rate P", &PIDSet->RollRatePID->ProportionalGain, minRateGain, maxRateGain);
                if (synchronizeRateGains && changed && PIDSet->PitchRatePID)
                {
                    PIDSet->PitchRatePID->ProportionalGain = PIDSet->RollRatePID->ProportionalGain;
                }
                changed = DrawPIDGainControl("Roll Rate I", &PIDSet->RollRatePID->IntegralGain, minRateGain, maxRateGain);
                if (synchronizeRateGains && changed && PIDSet->PitchRatePID)
                {
                    PIDSet->PitchRatePID->IntegralGain = PIDSet->RollRatePID->IntegralGain;
                }
                changed = DrawPIDGainControl("Roll Rate D", &PIDSet->RollRatePID->DerivativeGain, minRateGain, maxRateGain);
                if (synchronizeRateGains && changed && PIDSet->PitchRatePID)
                {
                    PIDSet->PitchRatePID->DerivativeGain = PIDSet->RollRatePID->DerivativeGain;
                }
            }
            else
            {
                ImGui::TextDisabled("Roll Rate PID Unavailable");
            }
            ImGui::Unindent();

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Pitch Rate");
            ImGui::Indent();
            if (PIDSet->PitchRatePID)
            {
                DrawPIDGainControl("Pitch Rate P", &PIDSet->PitchRatePID->ProportionalGain, minRateGain, maxRateGain);
                DrawPIDGainControl("Pitch Rate I", &PIDSet->PitchRatePID->IntegralGain, minRateGain, maxRateGain);
                DrawPIDGainControl("Pitch Rate D", &PIDSet->PitchRatePID->DerivativeGain, minRateGain, maxRateGain);
            }
            else
            {
                ImGui::TextDisabled("Pitch Rate PID Unavailable");
            }
            ImGui::Unindent();

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 0.3f, 1.0f, 1.0f), "Yaw Rate");
            ImGui::Indent();
            if (PIDSet->YawRatePID)
            {
                DrawPIDGainControl("Yaw Rate P", &PIDSet->YawRatePID->ProportionalGain, minYawRateGain, maxYawRateGain);
                DrawPIDGainControl("Yaw Rate I", &PIDSet->YawRatePID->IntegralGain, minYawRateGain, maxYawRateGain);
                DrawPIDGainControl("Yaw Rate D", &PIDSet->YawRatePID->DerivativeGain, minYawRateGain, maxYawRateGain);
            }
            else
            {
                ImGui::TextDisabled("Yaw Rate PID Unavailable");
            }
            ImGui::Unindent();

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::Separator();

    if (ImGui::Button("Save PID Gains", ImVec2(200.f, 50.f)))
    {
        TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("QuadSimPlugin"));
        FString PluginDir = Plugin.IsValid() ? Plugin->GetBaseDir() : FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("QuadSimPlugin"));
        FString FilePath = FPaths::Combine(PluginDir, TEXT("PIDGains.csv"));
        IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
        bool bFileExists = PlatformFile.FileExists(*FilePath);
        FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,rollRateP,rollRateI,rollRateD,pitchRateP,pitchRateI,pitchRateD,yawRateP,yawRateI,yawRateD\n");
        if (!bFileExists)
        {
            FFileHelper::SaveStringToFile(Header, *FilePath);
        }
        FString GainData = FDateTime::Now().ToString() + TEXT(",");
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->XPID ? PIDSet->XPID->ProportionalGain : 0.f,
            PIDSet->XPID ? PIDSet->XPID->IntegralGain     : 0.f,
            PIDSet->XPID ? PIDSet->XPID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->YPID ? PIDSet->YPID->ProportionalGain : 0.f,
            PIDSet->YPID ? PIDSet->YPID->IntegralGain     : 0.f,
            PIDSet->YPID ? PIDSet->YPID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->ZPID ? PIDSet->ZPID->ProportionalGain : 0.f,
            PIDSet->ZPID ? PIDSet->ZPID->IntegralGain     : 0.f,
            PIDSet->ZPID ? PIDSet->ZPID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->RollPID ? PIDSet->RollPID->ProportionalGain : 0.f,
            PIDSet->RollPID ? PIDSet->RollPID->IntegralGain     : 0.f,
            PIDSet->RollPID ? PIDSet->RollPID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->PitchPID ? PIDSet->PitchPID->ProportionalGain : 0.f,
            PIDSet->PitchPID ? PIDSet->PitchPID->IntegralGain     : 0.f,
            PIDSet->PitchPID ? PIDSet->PitchPID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->RollRatePID ? PIDSet->RollRatePID->ProportionalGain : 0.f,
            PIDSet->RollRatePID ? PIDSet->RollRatePID->IntegralGain     : 0.f,
            PIDSet->RollRatePID ? PIDSet->RollRatePID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f,"),
            PIDSet->PitchRatePID ? PIDSet->PitchRatePID->ProportionalGain : 0.f,
            PIDSet->PitchRatePID ? PIDSet->PitchRatePID->IntegralGain     : 0.f,
            PIDSet->PitchRatePID ? PIDSet->PitchRatePID->DerivativeGain   : 0.f);
        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f\n"),
            PIDSet->YawRatePID ? PIDSet->YawRatePID->ProportionalGain : 0.f,
            PIDSet->YawRatePID ? PIDSet->YawRatePID->IntegralGain     : 0.f,
            PIDSet->YawRatePID ? PIDSet->YawRatePID->DerivativeGain   : 0.f);
        FFileHelper::SaveStringToFile(GainData, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);
    }

    ImGui::SameLine();
    ImGui::Checkbox("PID Configuration History", &bShowPIDHistoryWindow);
}
void USimHUDTaskbarSubsystem::DrawPIDConfigWindow(UWorld* World)
{
    ImGui::SetNextWindowPos(ImVec2(360.f, 120.f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.f, 600.f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("PID Configuration", &bShowPIDSettings, ImGuiWindowFlags_NoCollapse))
    {
        // Resolve controller each frame (in case selection changes)
        UQuadDroneController* Ctrl = nullptr;
        if (ADroneManager* DMc = ADroneManager::Get(World))
        {
            const TArray<AQuadPawn*> Drones = DMc->GetDroneList();
            if (Drones.Num() > 0)
            {
                const int32 ActiveIndex = FMath::Clamp(DMc->SelectedDroneIndex, 0, Drones.Num()-1);
                AQuadPawn* Pawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
                Ctrl = Pawn ? Pawn->QuadController : nullptr;
            }
        }

        DrawPIDSettingsPanel(Ctrl);
    }
    ImGui::End();

    if (bShowPIDHistoryWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(420.f, 520.f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(800.f, 400.f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("PID Configurations History", &bShowPIDHistoryWindow))
        {
            // Resolve controller for loading gains
            UQuadDroneController* Ctrl = nullptr;
            if (ADroneManager* DMh = ADroneManager::Get(World))
            {
                const TArray<AQuadPawn*> Drones = DMh->GetDroneList();
                if (Drones.Num() > 0)
                {
                    const int32 ActiveIndex = FMath::Clamp(DMh->SelectedDroneIndex, 0, Drones.Num()-1);
                    AQuadPawn* Pawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
                    Ctrl = Pawn ? Pawn->QuadController : nullptr;
                }
            }

            TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("QuadSimPlugin"));
            FString PluginDir = Plugin.IsValid() ? Plugin->GetBaseDir() : FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("QuadSimPlugin"));
            FString FilePath = FPaths::Combine(PluginDir, TEXT("PIDGains.csv"));

            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
            if (!PlatformFile.FileExists(*FilePath))
            {
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "PID history file not found: %s", TCHAR_TO_UTF8(*FilePath));
            }
            else
            {
                FString FileContent;
                if (!FFileHelper::LoadFileToString(FileContent, *FilePath))
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Failed to read PID history file");
                }
                else
                {
                    TArray<FString> Lines;
                    FileContent.ParseIntoArrayLines(Lines, false);
                    if (Lines.Num() < 2)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "PID history file is empty or invalid");
                    }
                    else
                    {
                        static ImGuiTableFlags TableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
                        if (ImGui::BeginTable("PIDHistoryTable", 25, TableFlags, ImVec2(0, 0), 0.0f))
                        {
                            TArray<FString> Headers;
                            Lines[0].ParseIntoArray(Headers, TEXT(","), true);
                            ImGui::TableSetupScrollFreeze(1, 1);
                            for (int32 ColIdx = 0; ColIdx < Headers.Num(); ColIdx++)
                            {
                                ImGui::TableSetupColumn(TCHAR_TO_UTF8(*Headers[ColIdx]), ImGuiTableColumnFlags_WidthFixed);
                            }
                            ImGui::TableHeadersRow();

                            for (int32 RowIdx = 1; RowIdx < Lines.Num(); RowIdx++)
                            {
                                if (Lines[RowIdx].IsEmpty()) continue;
                                ImGui::TableNextRow();
                                TArray<FString> Values;
                                Lines[RowIdx].ParseIntoArray(Values, TEXT(","), true);
                                for (int32 ColIdx = 0; ColIdx < Values.Num() && ColIdx < Headers.Num(); ColIdx++)
                                {
                                    ImGui::TableSetColumnIndex(ColIdx);
                                    if (ColIdx == 0)
                                    {
                                        ImGui::TextUnformatted(TCHAR_TO_UTF8(*Values[ColIdx]));
                                        ImGui::SameLine();
                                        FString ButtonLabel = "Load##" + FString::FromInt(RowIdx);
                                        if (ImGui::SmallButton(TCHAR_TO_UTF8(*ButtonLabel)))
                                        {
                                            // Load the selected configuration
                                            if (Ctrl)
                                            {
                                                FFullPIDSet* PIDSet = Ctrl->GetPIDSet(Ctrl->GetFlightMode());
                                                if (PIDSet && Values.Num() >= 25)
                                                {
                                                    // Expected CSV format: Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,rollRateP,rollRateI,rollRateD,pitchRateP,pitchRateI,pitchRateD,yawRateP,yawRateI,yawRateD
                                                    // Index 0 is timestamp, gain values start at index 1
                                                    if (PIDSet->XPID)
                                                    {
                                                        PIDSet->XPID->ProportionalGain = FCString::Atof(*Values[1]);
                                                        PIDSet->XPID->IntegralGain = FCString::Atof(*Values[2]);
                                                        PIDSet->XPID->DerivativeGain = FCString::Atof(*Values[3]);
                                                    }
                                                    if (PIDSet->YPID)
                                                    {
                                                        PIDSet->YPID->ProportionalGain = FCString::Atof(*Values[4]);
                                                        PIDSet->YPID->IntegralGain = FCString::Atof(*Values[5]);
                                                        PIDSet->YPID->DerivativeGain = FCString::Atof(*Values[6]);
                                                    }
                                                    if (PIDSet->ZPID)
                                                    {
                                                        PIDSet->ZPID->ProportionalGain = FCString::Atof(*Values[7]);
                                                        PIDSet->ZPID->IntegralGain = FCString::Atof(*Values[8]);
                                                        PIDSet->ZPID->DerivativeGain = FCString::Atof(*Values[9]);
                                                    }
                                                    if (PIDSet->RollPID)
                                                    {
                                                        PIDSet->RollPID->ProportionalGain = FCString::Atof(*Values[10]);
                                                        PIDSet->RollPID->IntegralGain = FCString::Atof(*Values[11]);
                                                        PIDSet->RollPID->DerivativeGain = FCString::Atof(*Values[12]);
                                                    }
                                                    if (PIDSet->PitchPID)
                                                    {
                                                        PIDSet->PitchPID->ProportionalGain = FCString::Atof(*Values[13]);
                                                        PIDSet->PitchPID->IntegralGain = FCString::Atof(*Values[14]);
                                                        PIDSet->PitchPID->DerivativeGain = FCString::Atof(*Values[15]);
                                                    }
                                                    if (PIDSet->RollRatePID)
                                                    {
                                                        PIDSet->RollRatePID->ProportionalGain = FCString::Atof(*Values[16]);
                                                        PIDSet->RollRatePID->IntegralGain = FCString::Atof(*Values[17]);
                                                        PIDSet->RollRatePID->DerivativeGain = FCString::Atof(*Values[18]);
                                                    }
                                                    if (PIDSet->PitchRatePID)
                                                    {
                                                        PIDSet->PitchRatePID->ProportionalGain = FCString::Atof(*Values[19]);
                                                        PIDSet->PitchRatePID->IntegralGain = FCString::Atof(*Values[20]);
                                                        PIDSet->PitchRatePID->DerivativeGain = FCString::Atof(*Values[21]);
                                                    }
                                                    if (PIDSet->YawRatePID)
                                                    {
                                                        PIDSet->YawRatePID->ProportionalGain = FCString::Atof(*Values[22]);
                                                        PIDSet->YawRatePID->IntegralGain = FCString::Atof(*Values[23]);
                                                        PIDSet->YawRatePID->DerivativeGain = FCString::Atof(*Values[24]);
                                                    }

                                                    UE_LOG(LogTemp, Log, TEXT("[PID Config] Loaded gains from row %d"), RowIdx);
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        float Value = FCString::Atof(*Values[ColIdx]);
                                        ImGui::Text("%6.3f", Value);
                                    }
                                }
                            }
                            ImGui::EndTable();
                        }
                    }
                }
            }
        }
        ImGui::End();
    }
}

void USimHUDTaskbarSubsystem::DrawControlPlotsWindow(float MaxAngleDeg)
{
    ImGui::SetNextWindowSize(ImVec2(700, 750), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(950, 10),   ImGuiCond_FirstUseEver);

    // Title + visibility are tied to PID Plots now
    if (!ImGui::Begin("PID Plots", &bShowPIDPlots))
    {
        ImGui::End();
        return;
    }

    const int dataCount = Control_Time.Num();
    if (dataCount <= 0)
    {
        ImGui::TextUnformatted("Collecting control telemetry...");
        ImGui::End();
        return;
    }

    const double tEnd  = Control_CumulativeTime;
    const double tBeg  = tEnd - static_cast<double>(Control_MaxPlotTime);

    // Create tab bar
    if (ImGui::BeginTabBar("PIDPlotsTabBar", ImGuiTabBarFlags_None))
    {
        // --- Velocity Tab ---
        if (ImGui::BeginTabItem("Velocity"))
        {
            ImVec2 avail = ImGui::GetContentRegionAvail();
            float plotHeight = (avail.y / 3.0f) - (ImGui::GetStyle().ItemSpacing.y * 2);
            ImVec2 plotSize(avail.x, plotHeight);

            if (ImPlot::BeginPlot("Velocity X (Local Frame)", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(1.f, 0.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Vel X", Control_Time.GetData(), Control_CurrVelX.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.6f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Vel X", Control_Time.GetData(), Control_DesVelX.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::Spacing();

            if (ImPlot::BeginPlot("Velocity Y (Local Frame)", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0.f, 1.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Vel Y", Control_Time.GetData(), Control_CurrVelY.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.0f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Vel Y", Control_Time.GetData(), Control_DesVelY.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::Spacing();

            if (ImPlot::BeginPlot("Velocity Z (Local Frame)", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0.f, 0.f, 1.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Vel Z", Control_Time.GetData(), Control_CurrVelZ.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.6f, 1.0f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Vel Z", Control_Time.GetData(), Control_DesVelZ.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::EndTabItem();
        }

        // --- Angle Tab ---
        if (ImGui::BeginTabItem("Angle"))
        {
            ImVec2 avail = ImGui::GetContentRegionAvail();
            float plotHeight = (avail.y / 2.0f) - ImGui::GetStyle().ItemSpacing.y;
            ImVec2 plotSize(avail.x, plotHeight);

            if (ImPlot::BeginPlot("Roll Angle", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Angle (deg)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -MaxAngleDeg - 10.f, MaxAngleDeg + 10.f, ImGuiCond_Once);

                ImPlot::SetNextLineStyle(ImVec4(1.f, 0.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Roll", Control_Time.GetData(), Control_CurrRollDeg.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(1.f, 0.6f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Roll", Control_Time.GetData(), Control_DesRollDeg.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::Spacing();

            if (ImPlot::BeginPlot("Pitch Angle", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Angle (deg)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -MaxAngleDeg - 10.f, MaxAngleDeg + 10.f, ImGuiCond_Once);

                ImPlot::SetNextLineStyle(ImVec4(0.f, 1.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Pitch", Control_Time.GetData(), Control_CurrPitchDeg.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Pitch", Control_Time.GetData(), Control_DesPitchDeg.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::EndTabItem();
        }

        // --- Acro (Rate) Tab ---
        if (ImGui::BeginTabItem("Acro"))
        {
            ImVec2 avail = ImGui::GetContentRegionAvail();
            float plotHeight = (avail.y / 3.0f) - (ImGui::GetStyle().ItemSpacing.y * 2);
            ImVec2 plotSize(avail.x, plotHeight);

            if (ImPlot::BeginPlot("Roll Rate", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Rate (deg/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(1.f, 0.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Roll Rate", Control_Time.GetData(), Control_CurrRollRateDeg.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(1.f, 0.6f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Roll Rate", Control_Time.GetData(), Control_DesRollRateDeg.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::Spacing();

            if (ImPlot::BeginPlot("Pitch Rate", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Rate (deg/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0.f, 1.f, 0.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Pitch Rate", Control_Time.GetData(), Control_CurrPitchRateDeg.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.f, 0.6f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Pitch Rate", Control_Time.GetData(), Control_DesPitchRateDeg.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::Spacing();

            if (ImPlot::BeginPlot("Yaw Rate", plotSize, ImPlotFlags_None))
            {
                ImPlot::SetupAxes("Time (s)", "Rate (deg/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                ImPlot::SetupAxisLimits(ImAxis_X1, tBeg, tEnd, ImGuiCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0.f, 0.f, 1.f, 1.f), 1.5f);
                ImPlot::PlotLine("Current Yaw Rate", Control_Time.GetData(), Control_CurrYawRateDeg.GetData(), dataCount);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.6f, 1.f, 1.f), 1.0f);
                ImPlot::PlotLine("Desired Yaw Rate", Control_Time.GetData(), Control_DesYawRateDeg.GetData(), dataCount);

                ImPlot::EndPlot();
            }

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}


void USimHUDTaskbarSubsystem::JoyStickHandles(UWorld* World)
{
    if (ADroneManager* DMJ = ADroneManager::Get(World))
    {
        const TArray<AQuadPawn*> Drones = DMJ->GetDroneList();
        if (Drones.Num() > 0)
        {
            int32 ActiveIndex = FMath::Clamp(DMJ->SelectedDroneIndex, 0, Drones.Num()-1);
            AQuadPawn* Pawn = Drones.IsValidIndex(ActiveIndex) ? Drones[ActiveIndex] : nullptr;
            UQuadDroneController* Ctrl = Pawn ? Pawn->QuadController : nullptr;
            if (Pawn && Ctrl)
            {
                EFlightMode Mode = Ctrl->GetFlightMode();
                if (Mode == EFlightMode::AngleControl || Mode == EFlightMode::JoyStickAngleControl ||
                    Mode == EFlightMode::RateControl  || Mode == EFlightMode::JoyStickAcroControl)
                {
                    const float W = 260.f; const float H = 160.f;
                    ImGui::SetNextWindowPos(ImVec2((ViewSize.X - W)*0.5f, ViewSize.Y - H - 10.f), ImGuiCond_Always);
                    ImGui::SetNextWindowSize(ImVec2(W, H), ImGuiCond_Always);
                    ImGuiWindowFlags wf = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground;
                    if (ImGui::Begin("Joystick##BottomViz", nullptr, wf))
                    {
                        ImVec2 boxSize(100, 100);
                        ImDrawList* drawList = ImGui::GetWindowDrawList();

                        ImGui::BeginGroup();
                        ImGui::Text("Left Stick");
                        ImGui::InvisibleButton("##leftStick", boxSize);
                        ImVec2 rp = ImGui::GetItemRectMin();
                        ImVec2 rq = ImGui::GetItemRectMax();
                        drawList->AddRect(rp, rq, IM_COL32(255,255,255,255));
                        float rx = Pawn->GamepadInputs.Yaw;
                        float ry = Pawn->GamepadInputs.Throttle;
                        ImVec2 rCenter((rp.x + rq.x) * 0.5f, (rp.y + rq.y) * 0.5f);
                        ImVec2 rPos(rCenter.x + rx * (boxSize.x * 0.5f), rCenter.y - ry * (boxSize.y * 0.5f));
                        drawList->AddCircleFilled(rPos, 5.0f, IM_COL32(255,255,0,255));
                        ImGui::EndGroup();

                        ImGui::SameLine(0, 30);

                        ImGui::BeginGroup();
                        ImGui::Text("Right Stick");
                        ImGui::InvisibleButton("##rightStick", boxSize);
                        ImVec2 lp = ImGui::GetItemRectMin();
                        ImVec2 lq = ImGui::GetItemRectMax();
                        drawList->AddRect(lp, lq, IM_COL32(255,255,255,255));
                        float lx = Pawn->GamepadInputs.Roll;
                        float ly = Pawn->GamepadInputs.Pitch;
                        ImVec2 lCenter((lp.x + lq.x) * 0.5f, (lp.y + lq.y) * 0.5f);
                        ImVec2 lPos(lCenter.x + lx * (boxSize.x * 0.5f), lCenter.y + ly * (boxSize.y * 0.5f));
                        drawList->AddCircleFilled(lPos, 5.0f, IM_COL32(255,255,0,255));
                        ImGui::EndGroup();
                    }
                    ImGui::End();
                }
            }
        }
    }

}

void USimHUDTaskbarSubsystem::UpdateControlPlotData(UWorld* World,
                                                    AQuadPawn* Pawn,
                                                    UQuadDroneController* Ctrl,
                                                    float DeltaSeconds,
                                                    const FRotator& CurrentAttitude,
                                                    float DesiredRollDeg,
                                                    float DesiredPitchDeg,
                                                    const FVector& CurrentRateDeg,
                                                    float DesiredRollRateDeg,
                                                    float DesiredPitchRateDeg,
                                                    float DesiredYawRateDeg)
{
    if (!World || !Pawn || !Ctrl) return;

    // Advance timeline
    Control_CumulativeTime += FMath::Max(0.f, DeltaSeconds);
    Control_Time.Add(Control_CumulativeTime);

    // --- Velocity (Local Frame) ---
    // Try to obtain local-frame current velocity. If your controller already exposes
    // GetCurrentLocalVelocity(), you can swap in that call here.
    FVector worldVel = Pawn->GetVelocity();
    const FRotationMatrix rot(Pawn->GetActorRotation());
    const FVector localVel = rot.GetTransposed().TransformVector(worldVel); // world->local

    // Desired velocity already in controller (assumed local or control frame)
    const FVector desVel = Ctrl->GetDesiredVelocity();

    Control_CurrVelX.Add(static_cast<float>(localVel.X));
    Control_CurrVelY.Add(static_cast<float>(localVel.Y));
    Control_CurrVelZ.Add(static_cast<float>(localVel.Z));
    Control_DesVelX.Add(static_cast<float>(desVel.X));
    Control_DesVelY.Add(static_cast<float>(desVel.Y));
    Control_DesVelZ.Add(static_cast<float>(desVel.Z));

    // --- Angles (deg) ---
    Control_CurrRollDeg.Add(CurrentAttitude.Roll);
    Control_DesRollDeg.Add(DesiredRollDeg);
    Control_CurrPitchDeg.Add(CurrentAttitude.Pitch);
    Control_DesPitchDeg.Add(DesiredPitchDeg);

    // --- Angle Errors (deg) ---
    Control_RollErrorDeg.Add(DesiredRollDeg - CurrentAttitude.Roll);
    Control_PitchErrorDeg.Add(DesiredPitchDeg - CurrentAttitude.Pitch);

    // --- Angular Rates (deg/s) ---
    Control_CurrRollRateDeg.Add(CurrentRateDeg.X);
    Control_DesRollRateDeg.Add(DesiredRollRateDeg);
    Control_CurrPitchRateDeg.Add(CurrentRateDeg.Y);
    Control_DesPitchRateDeg.Add(DesiredPitchRateDeg);
    Control_CurrYawRateDeg.Add(CurrentRateDeg.Z);
    Control_DesYawRateDeg.Add(DesiredYawRateDeg);

    // --- Rate Errors (deg/s) ---
    Control_RollRateErrorDeg.Add(DesiredRollRateDeg - CurrentRateDeg.X);
    Control_PitchRateErrorDeg.Add(DesiredPitchRateDeg - CurrentRateDeg.Y);
    Control_YawRateErrorDeg.Add(DesiredYawRateDeg - CurrentRateDeg.Z);

    // --- Prune by time window ---
    while (Control_Time.Num() > 0 && (Control_CumulativeTime - Control_Time[0] > Control_MaxPlotTime))
    {
        Control_Time.RemoveAt(0);
        if (Control_CurrVelX.Num()     > 0) Control_CurrVelX.RemoveAt(0);
        if (Control_CurrVelY.Num()     > 0) Control_CurrVelY.RemoveAt(0);
        if (Control_CurrVelZ.Num()     > 0) Control_CurrVelZ.RemoveAt(0);
        if (Control_DesVelX.Num()      > 0) Control_DesVelX.RemoveAt(0);
        if (Control_DesVelY.Num()      > 0) Control_DesVelY.RemoveAt(0);
        if (Control_DesVelZ.Num()      > 0) Control_DesVelZ.RemoveAt(0);
        if (Control_CurrRollDeg.Num()  > 0) Control_CurrRollDeg.RemoveAt(0);
        if (Control_DesRollDeg.Num()   > 0) Control_DesRollDeg.RemoveAt(0);
        if (Control_CurrPitchDeg.Num() > 0) Control_CurrPitchDeg.RemoveAt(0);
        if (Control_DesPitchDeg.Num()  > 0) Control_DesPitchDeg.RemoveAt(0);
        if (Control_RollErrorDeg.Num() > 0) Control_RollErrorDeg.RemoveAt(0);
        if (Control_PitchErrorDeg.Num() > 0) Control_PitchErrorDeg.RemoveAt(0);
        if (Control_CurrRollRateDeg.Num()  > 0) Control_CurrRollRateDeg.RemoveAt(0);
        if (Control_DesRollRateDeg.Num()   > 0) Control_DesRollRateDeg.RemoveAt(0);
        if (Control_CurrPitchRateDeg.Num() > 0) Control_CurrPitchRateDeg.RemoveAt(0);
        if (Control_DesPitchRateDeg.Num()  > 0) Control_DesPitchRateDeg.RemoveAt(0);
        if (Control_CurrYawRateDeg.Num()   > 0) Control_CurrYawRateDeg.RemoveAt(0);
        if (Control_DesYawRateDeg.Num()    > 0) Control_DesYawRateDeg.RemoveAt(0);
        if (Control_RollRateErrorDeg.Num() > 0) Control_RollRateErrorDeg.RemoveAt(0);
        if (Control_PitchRateErrorDeg.Num() > 0) Control_PitchRateErrorDeg.RemoveAt(0);
        if (Control_YawRateErrorDeg.Num()  > 0) Control_YawRateErrorDeg.RemoveAt(0);
    }

    // --- Prune by max points (stable array sizes) ---
    while (Control_Time.Num() > Control_MaxDataPoints)
    {
        Control_Time.RemoveAt(0);
        if (Control_CurrVelX.Num()     > 0) Control_CurrVelX.RemoveAt(0);
        if (Control_CurrVelY.Num()     > 0) Control_CurrVelY.RemoveAt(0);
        if (Control_CurrVelZ.Num()     > 0) Control_CurrVelZ.RemoveAt(0);
        if (Control_DesVelX.Num()      > 0) Control_DesVelX.RemoveAt(0);
        if (Control_DesVelY.Num()      > 0) Control_DesVelY.RemoveAt(0);
        if (Control_DesVelZ.Num()      > 0) Control_DesVelZ.RemoveAt(0);
        if (Control_CurrRollDeg.Num()  > 0) Control_CurrRollDeg.RemoveAt(0);
        if (Control_DesRollDeg.Num()   > 0) Control_DesRollDeg.RemoveAt(0);
        if (Control_CurrPitchDeg.Num() > 0) Control_CurrPitchDeg.RemoveAt(0);
        if (Control_DesPitchDeg.Num()  > 0) Control_DesPitchDeg.RemoveAt(0);
        if (Control_RollErrorDeg.Num() > 0) Control_RollErrorDeg.RemoveAt(0);
        if (Control_PitchErrorDeg.Num() > 0) Control_PitchErrorDeg.RemoveAt(0);
        if (Control_CurrRollRateDeg.Num()  > 0) Control_CurrRollRateDeg.RemoveAt(0);
        if (Control_DesRollRateDeg.Num()   > 0) Control_DesRollRateDeg.RemoveAt(0);
        if (Control_CurrPitchRateDeg.Num() > 0) Control_CurrPitchRateDeg.RemoveAt(0);
        if (Control_DesPitchRateDeg.Num()  > 0) Control_DesPitchRateDeg.RemoveAt(0);
        if (Control_CurrYawRateDeg.Num()   > 0) Control_CurrYawRateDeg.RemoveAt(0);
        if (Control_DesYawRateDeg.Num()    > 0) Control_DesYawRateDeg.RemoveAt(0);
        if (Control_RollRateErrorDeg.Num() > 0) Control_RollRateErrorDeg.RemoveAt(0);
        if (Control_PitchRateErrorDeg.Num() > 0) Control_PitchRateErrorDeg.RemoveAt(0);
        if (Control_YawRateErrorDeg.Num()  > 0) Control_YawRateErrorDeg.RemoveAt(0);
    }
}



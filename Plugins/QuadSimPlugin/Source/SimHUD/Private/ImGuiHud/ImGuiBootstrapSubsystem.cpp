#include "ImGuiHud/ImGuiBootstrapSubsystem.h"

#include "ImGuiDelegates.h"
#include "imgui.h"
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
    if (!World)
        return;
    // Draw only for our bound world to avoid duplicates.
    if (BoundWorld.Get() != World)
        return;
    if (!(World->IsGameWorld() || World->WorldType == EWorldType::PIE))
        return;

    // Lazy-create control panels manager
    if (!ControlPanels)
    {
        ControlPanels = NewObject<UControlPanelUI>(this);
    }
    // Lazy-create settings panel
    if (!SettingsUI)
    {
        SettingsUI = NewObject<USimSettingsUI>(this);
    }

    if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
    {
        const FIntPoint Vp = GEngine->GameViewport->Viewport->GetSizeXY();
        ViewSize = FVector2D(Vp.X, Vp.Y);
    }

    static FSimImGuiStyle Theme;
    Theme.Apply();
    
    //Taskbar
    HandleTaskbar(World,Theme);
    //Control Panel and State Data button
    ControlButtons(World,Theme);
    //State Data
    HandleStateData(World);
    //Joystick Visualization
    JoyStickHandles(World);
    
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
                        //@TODO SensorData
                        FSensorData SensorData;
                        if (Pawn->SensorManager)
                        {
                            SensorData = Pawn->SensorManager->GetCurrentSensorData();
                        }
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
                        ImGui::Separator();
                        ImGui::Checkbox("Enable PID Settings", &bShowPIDSettings);
                        if (bShowPIDSettings)
                        {
                            // Helper lambda to render the PID settings for the active controller and mode
                            auto DrawPIDSettings = [this](UQuadDroneController* InController, EFlightMode Mode)
                            {
                                if (!InController) { ImGui::Text("No controller."); return; }
                                FFullPIDSet* PIDSet = InController->GetPIDSet(Mode);
                                if (!PIDSet)
                                {
                                    ImGui::Text("No PID Set found for this mode.");
                                    return;
                                }

                                static bool synchronizeXYGains = false;
                                static bool synchronizeGains   = false;

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

                                if (ImGui::CollapsingHeader("PID Settings", ImGuiTreeNodeFlags_DefaultOpen))
                                {
                                    ImGui::Text("Position PID Gains");
                                    ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);
                                    ImGui::Indent();

                                    float totalWidth = ImGui::GetContentRegionAvail().x;
                                    float inputWidth = 80.0f;
                                    float sliderWidth = totalWidth > (inputWidth + 20.0f) ? totalWidth - inputWidth - 20.0f : 100.0f;

                                    const float minXYGain = 0.0001f;
                                    const float maxXYGain = 30.f;
                                    const float minRollPitchGain = 0.0001f;
                                    const float maxRollPitchGain = 30.f;

                                    // X Axis
                                    ImGui::Text("X Axis");
                                    ImGui::Indent();
                                    if (PIDSet->XPID)
                                    {
                                        float xP = PIDSet->XPID->ProportionalGain;
                                        float xI = PIDSet->XPID->IntegralGain;
                                        float xD = PIDSet->XPID->DerivativeGain;
                                        bool x_changed = false;
                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("X P", &xP, minXYGain, maxXYGain, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##XP_Input", &xP, 0.0f, 0.0f, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth();
                                        if (x_changed) { PIDSet->XPID->ProportionalGain = xP; if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->ProportionalGain = xP; } x_changed = false; }

                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("X I", &xI, minXYGain, maxXYGain, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##XI_Input", &xI, 0.0f, 0.0f, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth();
                                        if (x_changed) { PIDSet->XPID->IntegralGain = xI; if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->IntegralGain = xI; } x_changed = false; }

                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("X D", &xD, minXYGain, maxXYGain, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##XD_Input", &xD, 0.0f, 0.0f, "%.4f")) x_changed = true;
                                        ImGui::PopItemWidth();
                                        if (x_changed) { PIDSet->XPID->DerivativeGain = xD; if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->DerivativeGain = xD; } }
                                    }
                                    else { ImGui::TextDisabled("X PID Controller Unavailable"); }
                                    ImGui::Unindent();

                                    // Y Axis
                                    ImGui::Text("Y Axis");
                                    ImGui::Indent();
                                    if (PIDSet->YPID)
                                    {
                                        float yP = PIDSet->YPID->ProportionalGain;
                                        float yI = PIDSet->YPID->IntegralGain;
                                        float yD = PIDSet->YPID->DerivativeGain;
                                        bool y_changed = false;
                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("Y P", &yP, minXYGain, maxXYGain, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##YP_Input", &yP, 0.0f, 0.0f, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth();
                                        if (y_changed) { PIDSet->YPID->ProportionalGain = yP; if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->ProportionalGain = yP; } y_changed = false; }

                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("Y I", &yI, minXYGain, maxXYGain, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##YI_Input", &yI, 0.0f, 0.0f, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth();
                                        if (y_changed) { PIDSet->YPID->IntegralGain = yI; if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->IntegralGain = yI; } y_changed = false; }

                                        ImGui::PushItemWidth(sliderWidth);
                                        if (ImGui::SliderFloat("Y D", &yD, minXYGain, maxXYGain, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
                                        if (ImGui::InputFloat("##YD_Input", &yD, 0.0f, 0.0f, "%.4f")) y_changed = true;
                                        ImGui::PopItemWidth();
                                        if (y_changed) { PIDSet->YPID->DerivativeGain = yD; if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->DerivativeGain = yD; } }
                                    }
                                    else { ImGui::TextDisabled("Y PID Controller Unavailable"); }
                                    ImGui::Unindent();

                                    // Z Axis
                                    if (PIDSet->ZPID)
                                    {
                                        DrawPIDGainControl("Z P", &PIDSet->ZPID->ProportionalGain, 0.0001f, 10.0f);
                                        DrawPIDGainControl("Z I", &PIDSet->ZPID->IntegralGain, 0.0001f, 10.0f);
                                        DrawPIDGainControl("Z D", &PIDSet->ZPID->DerivativeGain, 0.0001f, 10.0f);
                                    }
                                    else { ImGui::TextDisabled("Z PID Controller Unavailable"); }
                                    ImGui::Unindent();

                                    // Attitude PID
                                    ImGui::Separator();
                                    ImGui::Text("Attitude PID Gains");
                                    ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);
                                    ImGui::Indent();

                                    // Roll
                                    ImGui::Text("Roll");
                                    ImGui::Indent();
                                    if (PIDSet->RollPID)
                                    {
                                        float rollP = PIDSet->RollPID->ProportionalGain;
                                        float rollI = PIDSet->RollPID->IntegralGain;
                                        float rollD = PIDSet->RollPID->DerivativeGain;
                                        bool changed = false;
                                        if (synchronizeGains && PIDSet->PitchPID)
                                        {
                                            if (DrawPIDGainControl("Roll P", &rollP, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->PitchPID->ProportionalGain = rollP; }
                                            if (DrawPIDGainControl("Roll I", &rollI, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->PitchPID->IntegralGain     = rollI; }
                                            if (DrawPIDGainControl("Roll D", &rollD, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->PitchPID->DerivativeGain  = rollD; }
                                        }
                                        else
                                        {
                                            changed |= DrawPIDGainControl("Roll P", &rollP, minRollPitchGain, maxRollPitchGain);
                                            changed |= DrawPIDGainControl("Roll I", &rollI, minRollPitchGain, maxRollPitchGain);
                                            changed |= DrawPIDGainControl("Roll D", &rollD, minRollPitchGain, maxRollPitchGain);
                                        }
                                        if (changed && PIDSet->RollPID)
                                        {
                                            PIDSet->RollPID->ProportionalGain = rollP;
                                            PIDSet->RollPID->IntegralGain     = rollI;
                                            PIDSet->RollPID->DerivativeGain   = rollD;
                                        }
                                    }
                                    else { ImGui::TextDisabled("Roll PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Pitch
                                    ImGui::Text("Pitch");
                                    ImGui::Indent();
                                    if (PIDSet->PitchPID)
                                    {
                                        float pitchP = PIDSet->PitchPID->ProportionalGain;
                                        float pitchI = PIDSet->PitchPID->IntegralGain;
                                        float pitchD = PIDSet->PitchPID->DerivativeGain;
                                        bool changed = false;
                                        if (synchronizeGains && PIDSet->RollPID)
                                        {
                                            if (DrawPIDGainControl("Pitch P", &pitchP, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->RollPID->ProportionalGain = pitchP; }
                                            if (DrawPIDGainControl("Pitch I", &pitchI, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->RollPID->IntegralGain     = pitchI; }
                                            if (DrawPIDGainControl("Pitch D", &pitchD, minRollPitchGain, maxRollPitchGain)) { changed = true; PIDSet->RollPID->DerivativeGain  = pitchD; }
                                        }
                                        else
                                        {
                                            changed |= DrawPIDGainControl("Pitch P", &pitchP, minRollPitchGain, maxRollPitchGain);
                                            changed |= DrawPIDGainControl("Pitch I", &pitchI, minRollPitchGain, maxRollPitchGain);
                                            changed |= DrawPIDGainControl("Pitch D", &pitchD, minRollPitchGain, maxRollPitchGain);
                                        }
                                        if (changed && PIDSet->PitchPID)
                                        {
                                            PIDSet->PitchPID->ProportionalGain = pitchP;
                                            PIDSet->PitchPID->IntegralGain     = pitchI;
                                            PIDSet->PitchPID->DerivativeGain   = pitchD;
                                        }
                                    }
                                    else { ImGui::TextDisabled("Pitch PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Angular Rate PIDs
                                    ImGui::Separator();
                                    ImGui::Text("Angular Rate PID Gains");
                                    static bool syncRateGains = false;
                                    ImGui::Checkbox("Synchronize Roll and Pitch Rate Gains", &syncRateGains);
                                    ImGui::Indent();
                                    // Roll Rate
                                    ImGui::Text("Roll Rate");
                                    ImGui::Indent();
                                    if (PIDSet->RollRatePID)
                                    {
                                        float rP = PIDSet->RollRatePID->ProportionalGain;
                                        float rI = PIDSet->RollRatePID->IntegralGain;
                                        float rD = PIDSet->RollRatePID->DerivativeGain;
                                        bool changed = false;
                                        if (DrawPIDGainControl("Roll Rate P", &rP, 0.0001f, .01f))
                                        {
                                            changed = true;
                                            if (syncRateGains && PIDSet->PitchRatePID) PIDSet->PitchRatePID->ProportionalGain = rP;
                                        }
                                        if (DrawPIDGainControl("Roll Rate I", &rI, 0.0001f, .01f))
                                        {
                                            changed = true;
                                            if (syncRateGains && PIDSet->PitchRatePID) PIDSet->PitchRatePID->IntegralGain = rI;
                                        }
                                        if (DrawPIDGainControl("Roll Rate D", &rD, 0.0001f, .01f))
                                        {
                                            changed = true;
                                            if (syncRateGains && PIDSet->PitchRatePID) PIDSet->PitchRatePID->DerivativeGain = rD;
                                        }
                                        if (changed && PIDSet->RollRatePID)
                                        {
                                            PIDSet->RollRatePID->ProportionalGain = rP;
                                            PIDSet->RollRatePID->IntegralGain     = rI;
                                            PIDSet->RollRatePID->DerivativeGain   = rD;
                                        }
                                    }
                                    else { ImGui::TextDisabled("Roll Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Pitch Rate
                                    ImGui::Text("Pitch Rate");
                                    ImGui::Indent();
                                    if (PIDSet->PitchRatePID)
                                    {
                                        float pP = PIDSet->PitchRatePID->ProportionalGain;
                                        float pI = PIDSet->PitchRatePID->IntegralGain;
                                        float pD = PIDSet->PitchRatePID->DerivativeGain;
                                        bool changed = false;
                                        changed |= DrawPIDGainControl("Pitch Rate P", &pP, 0.0001f, .01f);
                                        changed |= DrawPIDGainControl("Pitch Rate I", &pI, 0.0001f, .01f);
                                        changed |= DrawPIDGainControl("Pitch Rate D", &pD, 0.0001f, .01f);
                                        if (changed && PIDSet->PitchRatePID)
                                        {
                                            PIDSet->PitchRatePID->ProportionalGain = pP;
                                            PIDSet->PitchRatePID->IntegralGain     = pI;
                                            PIDSet->PitchRatePID->DerivativeGain   = pD;
                                        }
                                    }
                                    else { ImGui::TextDisabled("Pitch Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Yaw Rate
                                    ImGui::Text("Yaw Rate");
                                    ImGui::Indent();
                                    if (PIDSet->YawRatePID)
                                    {
                                        float yP = PIDSet->YawRatePID->ProportionalGain;
                                        float yI = PIDSet->YawRatePID->IntegralGain;
                                        float yD = PIDSet->YawRatePID->DerivativeGain;
                                        bool changed = false;
                                        changed |= DrawPIDGainControl("Yaw Rate P", &yP, 0.0001f, 2.0f);
                                        changed |= DrawPIDGainControl("Yaw Rate I", &yI, 0.0001f, 2.0f);
                                        changed |= DrawPIDGainControl("Yaw Rate D", &yD, 0.0001f, 2.0f);
                                        if (changed && PIDSet->YawRatePID)
                                        {
                                            PIDSet->YawRatePID->ProportionalGain = yP;
                                            PIDSet->YawRatePID->IntegralGain     = yI;
                                            PIDSet->YawRatePID->DerivativeGain   = yD;
                                        }
                                    }
                                    else { ImGui::TextDisabled("Yaw Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    ImGui::Unindent(); // Attitude PID section
                                    ImGui::Separator();
//@TODO Save PIDS
                                    // Save to CSV button (same as ImGuiUtil)
                                    if (ImGui::Button("Save PID Gains", ImVec2(200, 50)))
                                    {
                                        TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("QuadSimPlugin"));
                                        FString PluginDir = Plugin.IsValid() ? Plugin->GetBaseDir() : FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("QuadSimPlugin"));
                                        FString FilePath = FPaths::Combine(PluginDir, TEXT("PIDGains.csv"));
                                        IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
                                        bool bFileExists = PlatformFile.FileExists(*FilePath);
                                        FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");
                                        if (!bFileExists) { FFileHelper::SaveStringToFile(Header, *FilePath); }
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
                                        GainData += FString::Printf(TEXT("%.6f,%.6f,%.6f\n"),
                                            PIDSet->YawRatePID ? PIDSet->YawRatePID->ProportionalGain : 0.f,
                                            PIDSet->YawRatePID ? PIDSet->YawRatePID->IntegralGain     : 0.f,
                                            PIDSet->YawRatePID ? PIDSet->YawRatePID->DerivativeGain   : 0.f);
                                        FFileHelper::SaveStringToFile(GainData, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);
                                    }

                                    ImGui::SameLine();
                                    ImGui::Checkbox("PID Configuration History", &bShowPIDHistoryWindow);
                                }
                            };

                            // Render PID settings for the active controller and mode
                            DrawPIDSettings(Ctrl, Ctrl->GetFlightMode());
//@TODO PID COnfig Window
                            // Render PID Configuration History window outside the lambda to avoid any Begin/End nesting issues
                            if (bShowPIDHistoryWindow)
                            {
                                ImGui::SetNextWindowPos(ImVec2(420, 520), ImGuiCond_FirstUseEver);
                                ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_FirstUseEver);
                                if (ImGui::Begin("PID Configurations History", &bShowPIDHistoryWindow))
                                {
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
                                                if (ImGui::BeginTable("PIDHistoryTable", 19, TableFlags, ImVec2(0, 0), 0.0f))
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
                                                                    // TODO: apply selected row gains
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
                    }
                    ImGui::End();
                }
            }
        }
    }
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


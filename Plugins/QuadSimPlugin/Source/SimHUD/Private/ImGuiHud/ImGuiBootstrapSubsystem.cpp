#include "ImGuiHud/ImGuiBootstrapSubsystem.h"

#include "ImGuiDelegates.h"
#include "imgui.h"
#include "Containers/Ticker.h"
#include "Modules/ModuleManager.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "ImGuiHud/Style/SimImGuiStyle.h"
#include "ImGuiHud/ControlPanelUI.h"
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
// For PID settings and saving
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/FileHelper.h"
#include <string>

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

    // Taskbar at top
    FVector2D ViewSize(1280, 720);
    if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
    {
        const FIntPoint Vp = GEngine->GameViewport->Viewport->GetSizeXY();
        ViewSize = FVector2D(Vp.X, Vp.Y);
    }

    static FSimImGuiStyle Theme;
    Theme.Apply();

    const float BarHeight = 58.f; // a bit taller for extra bottom padding
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
        static bool  bPaused        = false;
        static int   SpeedMode      = 0;    // -1 = reverse, 0 = normal, 1 = fast
        static float SpeedScale     = 1.0f; // displayed and applied in fast-forward
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
            // Left-anchored Settings button and popup (JSON config)
            if (ImGui::Button("Settings")) { ImGui::OpenPopup("SettingsMenu"); }
            if (ImGui::BeginPopup("SettingsMenu"))
            {
                auto& Cfg = UDroneJSONConfig::Get().Config;
                ImGui::Text("Flight Parameters"); ImGui::Separator();
                ImGui::InputFloat("Max Velocity Bound", &Cfg.FlightParams.MaxVelocityBound);
                ImGui::InputFloat("Max Velocity", &Cfg.FlightParams.MaxVelocity);
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
                ImGui::EndPopup();
            }
            ImGui::SameLine();
            // Pre-compute total width to center the main control group (excluding Settings)
            ImGuiStyle& st = ImGui::GetStyle();
            const float padX = st.FramePadding.x;
            const float padY = st.FramePadding.y;
            const float itemX = st.ItemSpacing.x;
            auto BtnW = [&](const char* txt){ return ImGui::CalcTextSize(txt).x + padX*2.f; };
            const float LabelBoxW = 180.f;
            const float WMLabelW  = 170.f;
            const float LabelH    = BarHeight - 12.f;
            const float SpeedBoxW = 56.f;
            const char* PauseTxt  = bPaused ? "Play" : "Pause";
            float totalW = 0.f;
            totalW += LabelBoxW;                                   // Sim label
            totalW += itemX;
            totalW += BtnW("Reverse");
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
            // Vertical centering: align BUTTONS to the vertical middle of label boxes
            float btnH   = ImGui::GetTextLineHeight() + padY * 2.f;
            float labelTopY = FMath::Max(0.f, (BarHeight - LabelH) * 0.5f);
            float btnTopY   = labelTopY + FMath::Max(0.f, (LabelH - btnH) * 0.5f);
            ImGui::SetCursorPosY(labelTopY);
            ImGui::SetCursorPosX(colStartX + (ImGui::CalcTextSize("Settings").x + padX*2.f + itemX) + startXWithin);

            // Centered "Simulation Manager" clickable label with rounded background
            {
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
                                         : ImGui::GetColorU32(st.Colors[ImGuiCol_Text]);
                dl->AddText(tpos, tc, "Simulation Manager");
                if (ImGui::IsItemClicked()) bSimMgrActive = !bSimMgrActive;
                ImGui::SameLine();
            }

            ImGui::SetCursorPosY(btnTopY);
            if (ImGui::Button("Reverse")) { SpeedMode = -1; }
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
                dl->AddText(tpos, ImGui::GetColorU32(st.Colors[ImGuiCol_Text]), buf);
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
                                           : ImGui::GetColorU32(st.Colors[ImGuiCol_Text]);
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
                    const FVector Loc = DM->GetActorLocation();
                    DM->SpawnDrone(Loc, FRotator::ZeroRotator);
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
                dl->AddText(tpos, ImGui::GetColorU32(st.Colors[ImGuiCol_Text]), fps);
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
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
    ImGui::PopID();

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
                // Bigger buttons via frame padding
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(16.f, 10.f));
                float totalH = ImGui::GetTextLineHeightWithSpacing() * 2 + ImGui::GetStyle().FramePadding.y * 4 + 8.f;
                float cy = (ImGui::GetContentRegionAvail().y - totalH) * 0.5f;
                ImGui::SetCursorPosY(FMath::Max(0.f, cy));
                float cx = (ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(cpLabel).x - ImGui::GetStyle().FramePadding.x*2) * 0.5f;
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

    // Draw left-side State Data HUD when toggled
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
                                dl->AddRectFilled(fillMin, fillMax, IM_COL32(80,160,255,255));
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
                        ImGui::TextColored(ImVec4(0.6f,0.9f,1.0f,1.0f), "Attitude");
                        ImGui::Separator();

                        // Drone Mass
                        ImGui::Text("Drone Mass: %.2f kg", Pawn->GetMass());

                        // Attitude (current & desired)
                        FRotator curAtt = Pawn->SensorManager ? Pawn->SensorManager->IMU->GetLastAttitude() : Pawn->GetActorRotation();
                        ImGui::Text("Current Roll/Pitch: %.2f / %.2f deg", curAtt.Roll, curAtt.Pitch);
                        ImGui::Text("Desired Roll/Pitch: %.2f / %.2f deg", Ctrl->GetDesiredRoll(), Ctrl->GetDesiredPitch());

                        ImGui::TextColored(ImVec4(1.0f,0.6f,1.0f,1.0f), "Angular Rate");
                        FVector curRates = Pawn->SensorManager ? Pawn->SensorManager->IMU->GetLastGyroscopeDegrees() : (Pawn->DroneBody ? Pawn->DroneBody->GetPhysicsAngularVelocityInDegrees() : FVector::ZeroVector);
                        ImGui::Text("Current Rate Pitch/Roll: %.2f / %.2f deg/s", curRates.Y, curRates.X);
                        ImGui::Text("Desired Rate Roll/Pitch: %.2f / %.2f deg/s", (float)Ctrl->GetDesiredRollRate(), (float)Ctrl->GetDesiredPitchRate());

                        ImGui::TextColored(ImVec4(1.0f,0.9f,0.4f,1.0f), "Acceleration");
                        FVector curAcc = Pawn->SensorManager ? Pawn->SensorManager->IMU->GetLastAccelerometer() : FVector::ZeroVector;
                        ImGui::Text("Current Acceleration: X %.2f Y %.2f Z %.2f m/s^2", curAcc.X, curAcc.Y, curAcc.Z);

                        ImGui::TextColored(ImVec4(0.6f,1.0f,0.6f,1.0f), "Position");
                        FVector curPos = Pawn->SensorManager ? Pawn->SensorManager->GPS->GetLastGPS() : Pawn->GetActorLocation();
                        ImGui::Text("Current Position: X %.1f Y %.1f Z %.1f", curPos.X, curPos.Y, curPos.Z);
                        ImGui::TextColored(ImVec4(0.6f,1.0f,0.8f,1.0f), "Geo Position");
                        FVector geo = Pawn->SensorManager ? Pawn->SensorManager->GPS->GetGeographicCoordinates() : FVector::ZeroVector;
                        ImGui::Text("Geo Position: Lat %.3f Lon %.3f Alt %.1f", geo.X, geo.Y, geo.Z);

                        ImGui::TextColored(ImVec4(0.5f,0.7f,1.0f,1.0f), "Velocity");
                        FVector curVel = Pawn->SensorManager ? Pawn->SensorManager->IMU->GetLastVelocity() : Pawn->GetVelocity();
                        ImGui::Text("Current Velocity: X %.2f Y %.2f Z %.2f m/s", curVel.X, curVel.Y, curVel.Z);
                        FVector desVel = Ctrl->GetDesiredVelocity();
                        ImGui::Text("Desired Velocity: X %.2f Y %.2f Z %.2f m/s", desVel.X, desVel.Y, desVel.Z);

                        ImGui::Separator();
                        ImGui::TextColored(ImVec4(0.8f,0.8f,1.0f,1.0f), "Barometer Sensor");
                        if (Pawn->SensorManager && Pawn->SensorManager->Barometer)
                        {
                            UBaroSensor* Baro = Pawn->SensorManager->Barometer;
                            ImGui::Text("Pressure: %.2f hPa", Baro->GetLastPressureHPa());
                            ImGui::Text("Temperature: %.1f C", Baro->GetLastTemperature());
                            ImGui::Text("Altitude: %.1f m", Baro->GetEstimatedAltitude());
                        }

                        // (Removed legacy bottom thruster progress bars)

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
                                        if (synchronizeGains && PIDSet->PitchPID)
                                        {
                                            if (DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, minRollPitchGain, maxRollPitchGain)) PIDSet->PitchPID->ProportionalGain = PIDSet->RollPID->ProportionalGain;
                                            if (DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, minRollPitchGain, maxRollPitchGain)) PIDSet->PitchPID->IntegralGain = PIDSet->RollPID->IntegralGain;
                                            if (DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, minRollPitchGain, maxRollPitchGain)) PIDSet->PitchPID->DerivativeGain = PIDSet->RollPID->DerivativeGain;
                                        }
                                        else
                                        {
                                            DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, minRollPitchGain, maxRollPitchGain);
                                            DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, minRollPitchGain, maxRollPitchGain);
                                            DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, minRollPitchGain, maxRollPitchGain);
                                        }
                                    }
                                    else { ImGui::TextDisabled("Roll PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Pitch
                                    ImGui::Text("Pitch");
                                    ImGui::Indent();
                                    if (PIDSet->PitchPID)
                                    {
                                        if (synchronizeGains && PIDSet->RollPID)
                                        {
                                            if (DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, minRollPitchGain, maxRollPitchGain)) PIDSet->RollPID->ProportionalGain = PIDSet->PitchPID->ProportionalGain;
                                            if (DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, minRollPitchGain, maxRollPitchGain)) PIDSet->RollPID->IntegralGain = PIDSet->PitchPID->IntegralGain;
                                            if (DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, minRollPitchGain, maxRollPitchGain)) PIDSet->RollPID->DerivativeGain = PIDSet->PitchPID->DerivativeGain;
                                        }
                                        else
                                        {
                                            DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, minRollPitchGain, maxRollPitchGain);
                                            DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, minRollPitchGain, maxRollPitchGain);
                                            DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, minRollPitchGain, maxRollPitchGain);
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
                                        bool changed = DrawPIDGainControl("Roll Rate P", &PIDSet->RollRatePID->ProportionalGain, 0.0001f, 1.0f);
                                        if (syncRateGains && changed && PIDSet->PitchRatePID) PIDSet->PitchRatePID->ProportionalGain = PIDSet->RollRatePID->ProportionalGain;
                                        changed = DrawPIDGainControl("Roll Rate I", &PIDSet->RollRatePID->IntegralGain, 0.0001f, 1.0f);
                                        if (syncRateGains && changed && PIDSet->PitchRatePID) PIDSet->PitchRatePID->IntegralGain = PIDSet->RollRatePID->IntegralGain;
                                        changed = DrawPIDGainControl("Roll Rate D", &PIDSet->RollRatePID->DerivativeGain, 0.0001f, 1.0f);
                                        if (syncRateGains && changed && PIDSet->PitchRatePID) PIDSet->PitchRatePID->DerivativeGain = PIDSet->RollRatePID->DerivativeGain;
                                    }
                                    else { ImGui::TextDisabled("Roll Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Pitch Rate
                                    ImGui::Text("Pitch Rate");
                                    ImGui::Indent();
                                    if (PIDSet->PitchRatePID)
                                    {
                                        DrawPIDGainControl("Pitch Rate P", &PIDSet->PitchRatePID->ProportionalGain, 0.0001f, 1.0f);
                                        DrawPIDGainControl("Pitch Rate I", &PIDSet->PitchRatePID->IntegralGain, 0.0001f, 1.0f);
                                        DrawPIDGainControl("Pitch Rate D", &PIDSet->PitchRatePID->DerivativeGain, 0.0001f, 1.0f);
                                    }
                                    else { ImGui::TextDisabled("Pitch Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    // Yaw Rate
                                    ImGui::Text("Yaw Rate");
                                    ImGui::Indent();
                                    if (PIDSet->YawRatePID)
                                    {
                                        DrawPIDGainControl("Yaw Rate P", &PIDSet->YawRatePID->ProportionalGain, 0.0001f, 2.0f);
                                        DrawPIDGainControl("Yaw Rate I", &PIDSet->YawRatePID->IntegralGain, 0.0001f, 2.0f);
                                        DrawPIDGainControl("Yaw Rate D", &PIDSet->YawRatePID->DerivativeGain, 0.0001f, 2.0f);
                                    }
                                    else { ImGui::TextDisabled("Yaw Rate PID Unavailable"); }
                                    ImGui::Unindent();

                                    ImGui::Unindent(); // Attitude PID section
                                    ImGui::Separator();

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

    // Bottom-center Joystick Visualization for Angle/Acro modes
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

#if 0
    // Legacy floating window (disabled)
    if (bShowMain && ImGui::Begin("QuadSim Session HUD"))
    {
        static bool bDemo = false;
        ImGui::Checkbox("Show ImGui Demo", &bDemo);
        if (bDemo) ImGui::ShowDemoWindow(&bDemo);
    }
    if (bShowMain) ImGui::End();
#endif
}


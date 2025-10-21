#pragma once
#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Containers/Ticker.h"
#include "ImGuiHud/Style/SimImGuiStyle.h"
#include "ImGuiBootstrapSubsystem.generated.h"

class UControlPanelUI;
class USimSettingsUI;
class AQuadPawn;
class UQuadDroneController;
struct FSensorData;

UCLASS()
class SIMHUD_API USimHUDTaskbarSubsystem : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    void SetStateHUDVisible(bool bVisible) { bShowStateHUD = bVisible; }
    bool IsStateHUDVisible() const { return bShowStateHUD; }
    UControlPanelUI* GetControlPanels() const { return ControlPanels; }

private:
    bool TryBindImGui(float DeltaSeconds);
    void HandleImGuiDraw();
    void HandleTaskbar(UWorld* World, const FSimImGuiStyle& Theme);
    void ControlButtons(UWorld* World, const FSimImGuiStyle& Theme);
    void HandleStateData(UWorld* World);
    void JoyStickHandles(UWorld* World);

    // PID config window (wrapper) -> calls DrawPIDSettingsPanel(...)
    void DrawPIDConfigWindow(UWorld* World);
    // Actual PID config controls (unchanged signature)
    void DrawPIDSettingsPanel(UQuadDroneController* InController);

    // "Old RenderImPlot" plots (velocity/angles)
    void UpdateControlPlotData(UWorld* World, AQuadPawn* Pawn, UQuadDroneController* Ctrl, float DeltaSeconds, const FRotator& CurrentAttitude, const FVector& CurrentVel, float DesiredRollDeg, float DesiredPitchDeg, const FVector& CurrentRateDeg, float DesiredRollRateDeg, float DesiredPitchRateDeg, float DesiredYawRateDeg);
    void DrawControlPlotsWindow(float MaxAngleDeg);

private:
    FDelegateHandle                DrawHandle;
    FTSTicker::FDelegateHandle     TickerHandle;
    bool                           bShowMain = true;
    TWeakObjectPtr<UWorld>         BoundWorld;

    UPROPERTY() UControlPanelUI*   ControlPanels = nullptr;
    UPROPERTY() USimSettingsUI*    SettingsUI    = nullptr;

    bool   bShowStateHUD            = false;
    bool   bAppliedStartupSettings  = false;

    static constexpr float BarHeight = 58.f;
    FVector2D               ViewSize = FVector2D(1280.f, 720.f);

    static bool  bPaused;
    static int32 SpeedMode;
    static float SpeedScale;
    static bool  bSpeedInit;

    // ---------------- PID plots (RenderImPlot style) ----------------
    bool        bShowPIDPlots = false;   // single checkbox to open plots
    bool        bShowPIDSettings = false;// single button to open PID config
    bool        bShowPIDHistoryWindow = false;

    // Timebase (float: MUST match Y type for ImPlot::PlotLine(xs,ys,...))
    float       Control_CumulativeTime = 0.0f;
    TArray<float> Control_Time;

    // Velocity (local frame)
    TArray<float> Control_CurrVelX;
    TArray<float> Control_CurrVelY;
    TArray<float> Control_CurrVelZ;
    TArray<float> Control_DesVelX;
    TArray<float> Control_DesVelY;
    TArray<float> Control_DesVelZ;

    // Angles
    TArray<float> Control_CurrRollDeg;
    TArray<float> Control_DesRollDeg;
    TArray<float> Control_CurrPitchDeg;
    TArray<float> Control_DesPitchDeg;

    // Angle Errors (for Angle tab)
    TArray<float> Control_RollErrorDeg;
    TArray<float> Control_PitchErrorDeg;

    // Angular Rates (for Acro tab)
    TArray<float> Control_CurrRollRateDeg;
    TArray<float> Control_DesRollRateDeg;
    TArray<float> Control_CurrPitchRateDeg;
    TArray<float> Control_DesPitchRateDeg;
    TArray<float> Control_CurrYawRateDeg;
    TArray<float> Control_DesYawRateDeg;

    // Rate Errors (for Acro tab)
    TArray<float> Control_RollRateErrorDeg;
    TArray<float> Control_PitchRateErrorDeg;
    TArray<float> Control_YawRateErrorDeg;

    // Limits / pruning
    float   Control_MaxPlotTime   = 10.0f;  // seconds of history to show
    int32   Control_MaxDataPoints = 1000;   // cap arrays to avoid unbounded growth
};

#pragma once
#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "ImGuiBootstrapSubsystem.generated.h"

UCLASS()
class SIMHUD_API USimHUDTaskbarSubsystem : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

private:
    // Try to bind once ImGui + viewport are ready.
    bool TryBindImGui(float DeltaSeconds);
    void HandleImGuiDraw(); // void()
    void HandleTaskbar(UWorld* World, FSimImGuiStyle Theme);
    void ControlButtons(UWorld* World, FSimImGuiStyle Theme);
    void HandleStateData(UWorld* World);
    void JoyStickHandles(UWorld* World);


    void TaskbarSimMan(static int SpeedMode, static float SpeedScale, static bool bSimMgrActive, UWorld* World, FSimImGuiStyle Theme);
    FDelegateHandle DrawHandle;                // ImGui delegate handle
    FTSTicker::FDelegateHandle TickerHandle;   // late-bind ticker
    bool bShowMain = true;
    TWeakObjectPtr<UWorld> BoundWorld;         // world we bound the draw delegate for

    UPROPERTY()
    class UControlPanelUI* ControlPanels = nullptr; // per-drone panels

    UPROPERTY()
    class USimSettingsUI* SettingsUI = nullptr; // central settings panel

    bool bShowStateHUD = false; // toggle for left-side state data HUD
    bool bShowPIDSettings = false; // toggle to show PID settings inside State HUD
    bool bShowPIDHistoryWindow = false; // toggle to show PID configuration history window

    bool bAppliedStartupSettings = false; // run-once gate

    const float BarHeight = 58.f;
    FVector2D ViewSize(1280, 720);
    
    static bool bPaused = false;
    static int SpeedMode = 0;
    static float SpeedScale = 1.0f;
    static bool bSpeedInit = false;
    ImGuiStyle& st = ImGui::GetStyle();
    const float padX = st.FramePadding.x;
    const float padY = st.FramePadding.y;
    const float itemX = st.ItemSpacing.x;
    const float LabelBoxW = 180.f;
    const float WMLabelW  = 170.f;
    const float LabelH    = BarHeight - 12.f;
    const float SpeedBoxW = 56.f;
    
public:
    // Accessors for settings UI to control HUD state
    void SetStateHUDVisible(bool bVisible) { bShowStateHUD = bVisible; }
    bool IsStateHUDVisible() const { return bShowStateHUD; }
    class UControlPanelUI* GetControlPanels() const { return ControlPanels; }
};

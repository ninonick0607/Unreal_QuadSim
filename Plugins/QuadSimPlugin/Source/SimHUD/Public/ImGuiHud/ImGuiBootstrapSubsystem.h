#pragma once
#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Containers/Ticker.h"
#include "ImGuiHud/Style/SimImGuiStyle.h"
#include "ImGuiBootstrapSubsystem.generated.h"

class UControlPanelUI;
class USimSettingsUI;

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

    FDelegateHandle DrawHandle;
    FTSTicker::FDelegateHandle TickerHandle;
    bool bShowMain = true;
    TWeakObjectPtr<UWorld> BoundWorld;

    UPROPERTY()
    UControlPanelUI* ControlPanels = nullptr;

    UPROPERTY()
    USimSettingsUI* SettingsUI = nullptr;

    bool bShowStateHUD = false;
    bool bShowPIDSettings = false;
    bool bShowPIDHistoryWindow = false;

    bool bAppliedStartupSettings = false;

    static constexpr float BarHeight = 58.f;
    FVector2D ViewSize = FVector2D(1280.f, 720.f);

    static bool bPaused;
    static int32 SpeedMode;
    static float SpeedScale;
    static bool bSpeedInit;
};

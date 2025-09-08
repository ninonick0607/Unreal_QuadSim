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

    FDelegateHandle DrawHandle;                // ImGui delegate handle
    FTSTicker::FDelegateHandle TickerHandle;   // late-bind ticker
    bool bShowMain = true;
    TWeakObjectPtr<UWorld> BoundWorld;         // world we bound the draw delegate for

    UPROPERTY()
    class UControlPanelUI* ControlPanels = nullptr; // per-drone panels

    bool bShowStateHUD = false; // toggle for left-side state data HUD
    bool bShowPIDSettings = false; // toggle to show PID settings inside State HUD
    bool bShowPIDHistoryWindow = false; // toggle to show PID configuration history window
};

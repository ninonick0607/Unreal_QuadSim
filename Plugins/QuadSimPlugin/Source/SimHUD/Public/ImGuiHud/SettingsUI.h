#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "SettingsUI.generated.h"

class USimHUDTaskbarSubsystem;

UCLASS()
class SIMHUD_API USimSettingsUI : public UObject
{
    GENERATED_BODY()
public:
    void TickAndDraw(UWorld* World, class USimHUDTaskbarSubsystem* TaskbarSubsystem);
    void ToggleOpen() { bOpen = !bOpen; }
    bool IsOpen() const { return bOpen; }

    // Apply preferences once on startup (when first created)
    void ApplyStartupPreferences(UWorld* World, class USimHUDTaskbarSubsystem* TaskbarSubsystem);

private:
    bool bOpen = false;

    // Togglables
    bool bPersistentControlPanel = false;
    bool bPersistentStateHUD = false;
    bool bAutoSpawnPossessOnStart = false;
    bool bSpawnObstacles = true; // new toggle

    // Internal helper
    void DrawTogglablesTab(UWorld* World, class USimHUDTaskbarSubsystem* TaskbarSubsystem);
    void DrawConfigTab();

    // Persistence helpers
    void LoadPersistent();
    void SavePersistent();

    // Internal helpers for class I/O
    // no class prefs; classes are set on BP_DroneManager via details panel
};

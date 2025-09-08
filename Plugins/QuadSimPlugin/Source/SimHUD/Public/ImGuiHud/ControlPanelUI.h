// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "ControlPanelUI.generated.h"

/**
 * 
 */
USTRUCT()
struct FDronePanelState
{
    GENERATED_BODY()
    bool bOpen = false;        // panel open state
    bool bGamepad = false;     // toggle for gamepad mode UI
    bool bPX4 = false;         // toggle for PX4 external controller
    float HoverAlt = 250.f;    // desired hover altitude (meters)
    bool bHoverActive = false; // toggle for hover mode
    bool bAppliedGamepad = false; // tracks if ToggleGamepadMode applied
};

UCLASS()
class SIMHUD_API UControlPanelUI : public UObject
{
    GENERATED_BODY()
public:
    // Draw single unified control panel window; maintains per-drone state internally.
    void TickAndDraw(UWorld* World);
    void ToggleOpen() { bOpen = !bOpen; }
    bool IsOpen() const { return bOpen; }

private:
    // Per-drone UI state (checkboxes), but a single window is drawn.
    TMap<TWeakObjectPtr<class AQuadPawn>, FDronePanelState> DroneStates;
    bool bOpen = false;
    float PanelY = 80.f; // remember Y so right edge stays locked but vertical can move/resize
};

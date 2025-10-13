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
    bool bDebug = false;       // toggle for debug drawings
    float HoverAlt = 250.f;    // desired hover altitude (meters)
    bool bHoverActive = false; // toggle for hover mode
    bool bAppliedGamepad = false; // tracks if ToggleGamepadMode applied

    // Session-only overrides (do NOT persist to config)
    bool  bHasSessionMaxVel   = false;
    bool  bHasSessionMaxAngle = false;
    bool  bHasSessionMaxRate  = false;
    float SessionMaxVel       = 0.f;   // m/s
    float SessionMaxAngle     = 0.f;   // deg
    float SessionMaxRate      = 0.f;   // deg/s

    // Position mode: require explicit opt-in to override nav with UI sliders
    bool  bManualPositionOverride = false;

    // Manual path (queue) state for Position Control Settings
    bool  bManualPathMode = false;     // when true, nav follows ManualQueue
    FVector ManualInput = FVector::ZeroVector; // meters
    TArray<FVector> ManualQueue;       // meters, in order

    // Auto-reset for Acro mode (step response testing for PID tuning)
    bool  bAcroAutoReset = true;       // enable auto-reset after setting rate
    float AcroResetDelay = 1.0f;       // seconds to wait before resetting to zero
    float AcroResetTimer = 0.0f;       // countdown timer
    bool  bAcroResetPending = false;   // true when waiting to reset
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
    float PanelX = -1.f; // remember last X (set on first show)
    float PanelY = 80.f; // remember last Y
};

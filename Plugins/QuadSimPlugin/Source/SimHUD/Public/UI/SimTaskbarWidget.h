#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/TextBlock.h" 
#include "SimTaskbarWidget.generated.h"

class UButton;
class UUserWidget;
class ASimulationManager;
class ADroneManager;

UCLASS()
class SIMHUD_API USimTaskbarWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    virtual void NativeConstruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override; // optional live refresh

protected:
    // === Manager references: set these in the SimTaskbar instance in your MainHUD (preferred).
    // If left null, we fall back to world lookups.
    UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category="Managers")
    ASimulationManager* SimulationManagerRef = nullptr;

    UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category="Managers")
    ADroneManager* DroneManagerRef = nullptr;

    // === Direct child bindings from the Designer ===
    // If the child is a Button, bind here. If it’s a UserWidget, bind that here and we’ll grab its inner "Button".
    UPROPERTY(meta=(BindWidgetOptional)) UUserWidget* BtnPausePlay = nullptr; // WBP_ToggleIconButton
    UPROPERTY(meta=(BindWidgetOptional)) UUserWidget* BtnFaster    = nullptr; // WBP_IconButton
    UPROPERTY(meta=(BindWidgetOptional)) UUserWidget* BtnSlower    = nullptr; // WBP_IconButton
    UPROPERTY(meta=(BindWidgetOptional)) UUserWidget* BtnStep      = nullptr; // WBP_IconButton
    UPROPERTY(meta=(BindWidgetOptional)) UButton*     BtnSpawn     = nullptr; // direct UButton

    // Name for the inner button inside WBP_* widgets (rename if your inner button differs)
    UPROPERTY(EditAnywhere, Category="Taskbar")
    FName InnerButtonName = TEXT("Button");

	UPROPERTY(meta=(BindWidgetOptional)) UTextBlock* TxtSpeed = nullptr;  // ← NEW

private:
    // Cached weak refs (we read from the edit-instance ptrs first)
    TWeakObjectPtr<ASimulationManager> SimMgr;
    TWeakObjectPtr<ADroneManager>      DroneMgr;

    // Wiring
    void InitializeManagers();
    static UButton* ResolveInnerButton(UUserWidget* Child, const FName& InnerName);

    // Handlers
    UFUNCTION() void OnPausePlayClicked();
    UFUNCTION() void OnFasterClicked();
    UFUNCTION() void OnSlowerClicked();
    UFUNCTION() void OnStepClicked();
    UFUNCTION() void OnSpawnClicked();

    // Helpers
    ASimulationManager* GetSim() const { return SimMgr.Get(); }
    ADroneManager*      GetDM()  const { return DroneMgr.Get(); }

    float ReadTimeScale() const;           // from WorldSettings->TimeDilation
    void  SetTimeScaleClamped(float s);    // via SimulationManager->SetTimeScale (clamp 0.1..10)

	void UpdateSpeedText();                       // ← NEW
	mutable float LastShownScale = -999.f; 
};

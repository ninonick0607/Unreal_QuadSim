#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styles/SimControlLayout.h"
#include "ModeSelector.generated.h"

class UCheckBox;
class UIconButtonGeneral; // your wrapper (BP parented to this C++), optional
class UButton;            // fallback if wrapper doesn't expose OnPressed
class UWidget;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnModeChanged, EControlMode, NewMode);

UCLASS()
class SIMHUD_API UModeSelector : public UUserWidget
{
    GENERATED_BODY()

public:
    virtual void NativeConstruct() override;

    UFUNCTION(BlueprintCallable) EControlMode GetCurrentMode() const { return CurrentMode; }
    UFUNCTION(BlueprintCallable) void SetMode(EControlMode NewMode, bool bBroadcast = true);

    UFUNCTION(BlueprintCallable) void SetGamepadOnly(bool bOn, bool bBroadcast = true);
    UFUNCTION(BlueprintPure)     bool IsGamepadOnly() const { return bGamepadOnly; }

    UPROPERTY(BlueprintAssignable) FOnModeChanged OnModeChanged;

protected:
    // Use Optional so you can iterate fast in the editor without binding errors
    UPROPERTY(meta=(BindWidgetOptional)) UIconButtonGeneral* BtnPosition = nullptr;
    UPROPERTY(meta=(BindWidgetOptional)) UIconButtonGeneral* BtnVelocity = nullptr;
    UPROPERTY(meta=(BindWidgetOptional)) UIconButtonGeneral* BtnAngle    = nullptr;
    UPROPERTY(meta=(BindWidgetOptional)) UIconButtonGeneral* BtnAcro     = nullptr;

    UPROPERTY(meta=(BindWidgetOptional)) UCheckBox* ChkGamepad = nullptr;

private:
    EControlMode CurrentMode = EControlMode::Position;
    bool bGamepadOnly = false;

    // Helpers
    void RefreshVisibilityForGamepad();
	void RefreshSelectedVisuals();  

    // Handlers
    UFUNCTION() void OnPositionPressed();
    UFUNCTION() void OnVelocityPressed();
    UFUNCTION() void OnAnglePressed();
    UFUNCTION() void OnAcroPressed();
    UFUNCTION() void OnGamepadToggled(bool bChecked);

    // Fallback search for a raw UButton inside any wrapper
    UButton* FindFirstButtonDeep(UWidget* Root) const;

    // Bind either to wrapper->OnPressed or fallback to inner UButton->OnClicked
    void BindWrapperOrFallback(UIconButtonGeneral* Wrapper, EControlMode Mode);
};

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styles/SimControlLayout.h"   // FAxisSpec, EAxisChannel
#include "SliderRow.generated.h"

class UTextBlock;
class USpinBox;
class USlider;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnAxisChanged, EAxisChannel, Channel, float, Value);

UCLASS()
class SIMHUD_API USliderRow : public UUserWidget
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category="SliderRow")
    void Init(const FAxisSpec& InSpec);  // uses InSpec.UnitText only

    UFUNCTION(BlueprintCallable, Category="SliderRow")
    void InitCustom(FText InLabelIgnored, float InMin, float InMax, float InDefault, float InStep, FText InUnits);

    UFUNCTION(BlueprintCallable, Category="SliderRow")
    void SetValue(float NewValue, bool bSilent);

    UFUNCTION(BlueprintPure, Category="SliderRow")
    EAxisChannel GetChannel() const { return Channel; }
	UFUNCTION(BlueprintPure, Category="SliderRow")
	float GetValue() const { return Value; }

    UPROPERTY(BlueprintAssignable, Category="SliderRow")
    FOnAxisChanged OnAxisChanged;

protected:
    virtual void NativeConstruct() override;

    // Bind these in WBP_SliderRow (Label optional/not used)
    UPROPERTY(meta=(BindWidgetOptional)) UTextBlock* TxtLabel = nullptr; // ignored
    UPROPERTY(meta=(BindWidget))         UTextBlock* TxtUnits = nullptr; // REQUIRED
    UPROPERTY(meta=(BindWidgetOptional)) USpinBox*   SpinBox  = nullptr;
    UPROPERTY(meta=(BindWidgetOptional)) USlider*    Slider   = nullptr;

private:
    // model
    EAxisChannel Channel = EAxisChannel::X;
    FText Units;
    float Min   = 0.f;
    float Max   = 1.f;
    float Step  = 0.1f;
    float Value = 0.f;

    bool bWiringDone      = false;
    bool bUpdatingWidgets = false;

    // handlers
    UFUNCTION() void HandleSpinChanged(float NewVal);
    UFUNCTION() void HandleSpinCommitted(float NewVal, ETextCommit::Type CommitType);
    UFUNCTION() void HandleSliderChanged(float Raw0to1);
    UFUNCTION() void HandleSliderCommit();

    // helpers
    void ApplyModelToUI();
    void ApplyValueToWidgets();
    float Quantize(float In) const;
    float SliderToValue(float Raw) const;
    float ValueToSlider(float V) const;

    // defensive fallback if the BindWidget name doesn't match
    UTextBlock* FindUnitsTextFallback() const;
};

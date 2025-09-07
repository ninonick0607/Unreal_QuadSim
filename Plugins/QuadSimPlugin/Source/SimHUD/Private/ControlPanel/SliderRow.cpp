#include "ControlPanel/SliderRow.h"
#include "Components/TextBlock.h"
#include "Components/SpinBox.h"
#include "Components/Slider.h"
#include "Blueprint/WidgetTree.h"

#define ROWLOG(Verbosity, Fmt, ...) UE_LOG(LogTemp, Verbosity, TEXT("[SliderRow] " Fmt), ##__VA_ARGS__)

void USliderRow::NativeConstruct()
{
    Super::NativeConstruct();

    if (!bWiringDone)
    {
        bWiringDone = true;

        if (SpinBox)
        {
            SpinBox->OnValueChanged.AddDynamic(this, &USliderRow::HandleSpinChanged);
            SpinBox->OnValueCommitted.AddDynamic(this, &USliderRow::HandleSpinCommitted);
        }
        if (Slider)
        {
            Slider->OnValueChanged.AddDynamic(this, &USliderRow::HandleSliderChanged);
            Slider->OnMouseCaptureEnd.AddDynamic(this, &USliderRow::HandleSliderCommit);
            Slider->OnControllerCaptureEnd.AddDynamic(this, &USliderRow::HandleSliderCommit);
        }
    }

    ApplyModelToUI();
}

void USliderRow::Init(const FAxisSpec& InSpec)
{
	Channel = InSpec.Channel;
	Min     = InSpec.Min;
	Max     = InSpec.Max;
	Step    = FMath::Max(InSpec.Step, KINDA_SMALL_NUMBER);
	Value   = FMath::Clamp(InSpec.Default, Min, Max);

	Units   = InSpec.Units;

	if (TxtUnits)
	{
		TxtUnits->SetText(Units);
		UE_LOG(LogTemp, Log, TEXT("[SliderRow] %s units set to %s"),
			*UEnum::GetValueAsString(Channel),
			*Units.ToString());
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("[SliderRow] TxtUnits not bound!"));
	}

	ApplyValueToWidgets();
}


void USliderRow::InitCustom(FText /*InLabelIgnored*/, float InMin, float InMax, float InDefault, float InStep, FText InUnits)
{
    Channel = EAxisChannel::X; // settings rows don't use it
    Units   = InUnits;
    Min     = InMin;
    Max     = InMax;
    Step    = FMath::Max(InStep, KINDA_SMALL_NUMBER);
    Value   = FMath::Clamp(InDefault, Min, Max);

    ApplyModelToUI();

    ROWLOG(Log, "InitCustom: Units='%s' [%g..%g] step=%g def=%g",
        *Units.ToString(), Min, Max, Step, Value);
}

void USliderRow::SetValue(float NewValue, bool bSilent)
{
    Value = FMath::Clamp(Quantize(NewValue), Min, Max);
    ApplyValueToWidgets();
    if (!bSilent)
    {
        OnAxisChanged.Broadcast(Channel, Value);
    }
}

void USliderRow::HandleSpinChanged(float NewVal)
{
    if (bUpdatingWidgets) return;
    Value = FMath::Clamp(Quantize(NewVal), Min, Max);
    ApplyValueToWidgets();
    OnAxisChanged.Broadcast(Channel, Value);
}

void USliderRow::HandleSpinCommitted(float NewVal, ETextCommit::Type)
{
    if (bUpdatingWidgets) return;
    Value = FMath::Clamp(Quantize(NewVal), Min, Max);
    ApplyValueToWidgets();
    OnAxisChanged.Broadcast(Channel, Value);
}

void USliderRow::HandleSliderChanged(float Raw0to1)
{
    if (bUpdatingWidgets) return;
    Value = FMath::Clamp(Quantize(SliderToValue(Raw0to1)), Min, Max);
    ApplyValueToWidgets();
    OnAxisChanged.Broadcast(Channel, Value);
}

void USliderRow::HandleSliderCommit()
{
    // no-op
}

void USliderRow::ApplyModelToUI()
{
    // Write units ONLY
    UTextBlock* UnitsText = TxtUnits ? TxtUnits : FindUnitsTextFallback();
    if (UnitsText)
    {
        UnitsText->SetText(Units);
        UnitsText->SetVisibility(Units.IsEmpty() ? ESlateVisibility::Hidden : ESlateVisibility::Visible);

        // Make sure opacity isn't 0 (common accidental style issue)
        FSlateColor C = UnitsText->GetColorAndOpacity();
        if (C.GetSpecifiedColor().A <= 0.f)
        {
            UnitsText->SetColorAndOpacity(FLinearColor(1,1,1,1));
            ROWLOG(Warning, "TxtUnits had zero alpha; forcing to opaque white.");
        }
    }
    else
    {
        ROWLOG(Error, "No Units TextBlock found. Ensure a TextBlock named 'TxtUnits' exists (IsVariable=TRUE).");
    }

    if (SpinBox)
    {
        SpinBox->SetMinValue(Min);
        SpinBox->SetMaxValue(Max);
        SpinBox->SetMinSliderValue(Min);
        SpinBox->SetMaxSliderValue(Max);
        SpinBox->SetDelta(Step);
        SpinBox->SetValue(Value);
    }

    if (Slider)
    {
        Slider->SetMinValue(0.f);
        Slider->SetMaxValue(1.f);
        Slider->SetStepSize((Max > Min) ? (Step / (Max - Min)) : 1.f);
        Slider->SetValue(ValueToSlider(Value));
    }

    ApplyValueToWidgets();
}

void USliderRow::ApplyValueToWidgets()
{
    TGuardValue<bool> Guard(bUpdatingWidgets, true);

    if (SpinBox)
        SpinBox->SetValue(Value);

    if (Slider)
        Slider->SetValue(ValueToSlider(Value));
}

float USliderRow::Quantize(float In) const
{
    if (Step <= KINDA_SMALL_NUMBER) return In;
    const float Q = FMath::RoundToFloat((In - Min) / Step) * Step + Min;
    return FMath::Clamp(Q, Min, Max);
}

float USliderRow::SliderToValue(float Raw) const
{
    return FMath::Lerp(Min, Max, Raw);
}

float USliderRow::ValueToSlider(float V) const
{
    return (Max > Min) ? (V - Min) / (Max - Min) : 0.f;
}

UTextBlock* USliderRow::FindUnitsTextFallback() const
{
    // Try to find a child named "TxtUnits" if BindWidget didn't hook up
    if (WidgetTree)
    {
        if (UWidget* W = WidgetTree->FindWidget(TEXT("TxtUnits")))
        {
            if (UTextBlock* TB = Cast<UTextBlock>(W))
                return TB;
        }
    }
    return nullptr;
}

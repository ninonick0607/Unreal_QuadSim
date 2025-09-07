#include "ControlPanel/ModeSelector.h"
#include "IconButtonGeneral.h"            // wrapper
#include "Components/CheckBox.h"
#include "Components/Button.h"
#include "Components/PanelWidget.h"
#include "Components/Border.h"
#include "Blueprint/UserWidget.h"
#include "Engine/Engine.h"

#define LOGF(Verbosity, Fmt, ...) UE_LOG(LogTemp, Verbosity, TEXT("[ModeSelector] " Fmt), ##__VA_ARGS__)

void UModeSelector::NativeConstruct()
{
    Super::NativeConstruct();

    // Bind four wrappers (or fall back to inner UButton)
    BindWrapperOrFallback(BtnPosition, EControlMode::Position);
    BindWrapperOrFallback(BtnVelocity, EControlMode::Velocity);
    BindWrapperOrFallback(BtnAngle,    EControlMode::Angle);
    BindWrapperOrFallback(BtnAcro,     EControlMode::Acro);

    // Gamepad toggle
    if (ChkGamepad)
    {
        ChkGamepad->OnCheckStateChanged.Clear();
        ChkGamepad->OnCheckStateChanged.AddDynamic(this, &UModeSelector::OnGamepadToggled);
        bGamepadOnly = ChkGamepad->IsChecked();
    }

    RefreshVisibilityForGamepad();
	RefreshSelectedVisuals();

}

void UModeSelector::SetMode(EControlMode NewMode, bool bBroadcast /*=true*/)
{
    if (bGamepadOnly && (NewMode == EControlMode::Position || NewMode == EControlMode::Velocity))
    {
        NewMode = EControlMode::Angle;
    }
    if (CurrentMode == NewMode) return;

    CurrentMode = NewMode;
	RefreshSelectedVisuals();
    if (bBroadcast)
    {
        LOGF(Log, "Broadcast OnModeChanged: %s", *UEnum::GetValueAsString(CurrentMode));
        OnModeChanged.Broadcast(CurrentMode);
    }
}

void UModeSelector::SetGamepadOnly(bool bOn, bool /*bBroadcast*/)
{
    if (bGamepadOnly == bOn) return;
    bGamepadOnly = bOn;

    if (ChkGamepad && ChkGamepad->IsChecked() != bGamepadOnly)
        ChkGamepad->SetIsChecked(bGamepadOnly);

    if (bGamepadOnly && (CurrentMode == EControlMode::Position || CurrentMode == EControlMode::Velocity))
    {
        CurrentMode = EControlMode::Angle;
        OnModeChanged.Broadcast(CurrentMode);
    }

    RefreshVisibilityForGamepad();
}

void UModeSelector::OnGamepadToggled(bool bChecked)
{
    SetGamepadOnly(bChecked, /*bBroadcast=*/true);
}

// Button handlers
void UModeSelector::OnPositionPressed() { SetMode(EControlMode::Position, true); }
void UModeSelector::OnVelocityPressed() { SetMode(EControlMode::Velocity, true); }
void UModeSelector::OnAnglePressed()    { SetMode(EControlMode::Angle,    true); }
void UModeSelector::OnAcroPressed()     { SetMode(EControlMode::Acro,     true); }

// Visibility policy
void UModeSelector::RefreshVisibilityForGamepad()
{
    auto SetVis = [](UWidget* W, bool bShow)
    {
        if (!W) return;
        W->SetVisibility(bShow ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
    };

    SetVis(BtnPosition, !bGamepadOnly);
    SetVis(BtnVelocity, !bGamepadOnly);
    SetVis(BtnAngle,    true);
    SetVis(BtnAcro,     true);
}

// ---- Binding helpers ----
void UModeSelector::BindWrapperOrFallback(UIconButtonGeneral* Wrapper, EControlMode Mode)
{
	const FString ModeStr = UEnum::GetValueAsString(Mode);

	if (!Wrapper)
	{
		LOGF(Warning, "Wrapper for %s is NULL (BindWidget name mismatch or not present)", *ModeStr);
		return;
	}

	// Bind to the wrapper's multicast delegate
	Wrapper->OnPressed.Clear();
	switch (Mode)
	{
	case EControlMode::Position: Wrapper->OnPressed.AddDynamic(this, &UModeSelector::OnPositionPressed); break;
	case EControlMode::Velocity: Wrapper->OnPressed.AddDynamic(this, &UModeSelector::OnVelocityPressed); break;
	case EControlMode::Angle:    Wrapper->OnPressed.AddDynamic(this, &UModeSelector::OnAnglePressed);    break;
	case EControlMode::Acro:     Wrapper->OnPressed.AddDynamic(this, &UModeSelector::OnAcroPressed);     break;
	}
	LOGF(Log, "Bound Wrapper.OnPressed for %s on '%s'", *ModeStr, *Wrapper->GetName());

	// Optional: also try to hook a raw button as a fallback (harmless if none)
	if (UButton* Raw = FindFirstButtonDeep(Wrapper))
	{
		Raw->OnClicked.Clear();
		switch (Mode)
		{
		case EControlMode::Position: Raw->OnClicked.AddDynamic(this, &UModeSelector::OnPositionPressed); break;
		case EControlMode::Velocity: Raw->OnClicked.AddDynamic(this, &UModeSelector::OnVelocityPressed); break;
		case EControlMode::Angle:    Raw->OnClicked.AddDynamic(this, &UModeSelector::OnAnglePressed);    break;
		case EControlMode::Acro:     Raw->OnClicked.AddDynamic(this, &UModeSelector::OnAcroPressed);     break;
		}
		LOGF(Log, "Additionally bound inner UButton.OnClicked for %s under '%s'", *ModeStr, *Wrapper->GetName());
	}
}


UButton* UModeSelector::FindFirstButtonDeep(UWidget* Root) const
{
    if (!Root) return nullptr;

    if (UButton* AsButton = Cast<UButton>(Root))
        return AsButton;

    if (UUserWidget* AsUW = Cast<UUserWidget>(Root))
    {
        if (UWidget* Inner = AsUW->GetRootWidget())
            if (UButton* B = FindFirstButtonDeep(Inner)) return B;
    }

    if (UPanelWidget* Panel = Cast<UPanelWidget>(Root))
    {
        const int32 Num = Panel->GetChildrenCount();
        for (int32 i = 0; i < Num; ++i)
            if (UWidget* Child = Panel->GetChildAt(i))
                if (UButton* B = FindFirstButtonDeep(Child)) return B;
    }

    if (UBorder* Border = Cast<UBorder>(Root))
    {
        if (UWidget* Cont = Border->GetContent())
            if (UButton* B = FindFirstButtonDeep(Cont)) return B;
    }

    return nullptr;
}

void UModeSelector::RefreshSelectedVisuals()
{
	auto TrySet = [](UIconButtonGeneral* W, bool bSel)
	{
		if (W) W->SetSelected(bSel);
	};

	TrySet(BtnPosition, CurrentMode == EControlMode::Position);
	TrySet(BtnVelocity, CurrentMode == EControlMode::Velocity);
	TrySet(BtnAngle,    CurrentMode == EControlMode::Angle);
	TrySet(BtnAcro,     CurrentMode == EControlMode::Acro);
}
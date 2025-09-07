#include "ControlPanel/ControlPanelWidget.h"
#include "ControlPanel/ModeSelector.h"
#include "ControlPanel/SliderRow.h"
#include "Styles/SimControlLayout.h"
#include "Components/CheckBox.h"
#include "Components/ScrollBox.h"
#include "Components/VerticalBox.h"
#include "Components/SizeBox.h"
#include "Components/PanelWidget.h"
#include "Components/Button.h"
#include "Blueprint/UserWidget.h"
#include "Blueprint/WidgetTree.h"
#include "IconButtonGeneral.h" 
#include "Components/ContentWidget.h"
#include "Components/ExpandableArea.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "Controllers/QuadDroneController.h"
#include "Core/DroneManager.h"
#include "Pawns/QuadPawn.h"

void UControlPanelWidget::NativeConstruct()
{
    Super::NativeConstruct();

	if (!ModeSelector && ModeSelectorOverride)
	{
		ModeSelector = ModeSelectorOverride;
	}

    // 3) If still null, do a simple global lookup next tick (handles creation order, nesting, ExpandableArea, etc.)
    if (!ModeSelector)
    {
        GetWorld()->GetTimerManager().SetTimerForNextTick([this]()
        {
            if (!ModeSelector)
            {
                TArray<UUserWidget*> Found;
                UWidgetBlueprintLibrary::GetAllWidgetsOfClass(GetWorld(), Found, UModeSelector::StaticClass(), /*TopLevelOnly=*/false);
                if (Found.Num() > 0)
                {
                    ModeSelector = Cast<UModeSelector>(Found[0]);
                    UE_LOG(LogTemp, Warning, TEXT("[CP] Late-bound ModeSelector: %s"),
                        ModeSelector ? *ModeSelector->GetName() : TEXT("NULL"));
                    BindToModeSelector();
                }
                else
                {
                    UE_LOG(LogTemp, Error, TEXT("[CP] Could not find any ModeSelector. Rows won’t update on mode change."));
                }
            }
        });
    }

    // If we already have it (child bind or override), bind now
    BindToModeSelector();

    UE_LOG(LogTemp, Log, TEXT("[CP] NativeConstruct: ModeSelector=%s"),
        ModeSelector ? *ModeSelector->GetName() : TEXT("NULL"));

    // ---- Your existing init below ----
	if (ChkGamepad)
	{
		ChkGamepad->OnCheckStateChanged.AddDynamic(this, &UControlPanelWidget::OnGamepadChanged);
	}

	if (SizeBox_BodyLimiter)
	{
		SizeBox_BodyLimiter->SetMaxDesiredHeight(380.f);
	}

	if (Scroll_Sliders)
	{
		Scroll_Sliders->SetScrollbarThickness(FVector2D(4.f, 4.f));
	}

	if (Layout)
	{
		if (Row_MaxVelocity)
		{
			Row_MaxVelocity->InitCustom(
				FText::GetEmpty(),
				0.f, Layout->MaxVelocityBound, Layout->MaxVelocity,
				0.1f, FText::FromString("m/s"));
			Row_MaxVelocity->OnAxisChanged.AddDynamic(this, &UControlPanelWidget::OnMaxVelocityChanged);
		}
		if (Row_MaxAngle)
		{
			Row_MaxAngle->InitCustom(
				FText::GetEmpty(),
				0.f, 180.f, Layout->MaxAngle,
				0.5f, FText::FromString("deg"));
			Row_MaxAngle->OnAxisChanged.AddDynamic(this, &UControlPanelWidget::OnMaxAngleChanged);
		}
		if (Row_MaxAngleRate)
		{
			Row_MaxAngleRate->InitCustom(
				FText::GetEmpty(),
				0.f, 1000.f, Layout->MaxAngleRate,
				1.f, FText::FromString("deg/s"));
			Row_MaxAngleRate->OnAxisChanged.AddDynamic(this, &UControlPanelWidget::OnMaxAngleRateChanged);
		}

		// Listen for changes coming from SimTaskbar / JSON reload / editor tweaks
		Layout->OnLayoutChanged.RemoveAll(this);
		Layout->OnLayoutChanged.AddDynamic(this, &UControlPanelWidget::HandleLayoutChangedExternal);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("[CP] No Layout asset set. Sliders will not reflect JSON or taskbar changes."));
	}

    if (Btn_ActivateHoverMode)
        Btn_ActivateHoverMode->OnPressed.AddDynamic(this, &UControlPanelWidget::OnActivateHoverModeClicked);
    if (Row_HoverAltitude)
        Row_HoverAltitude->InitCustom(FText::GetEmpty(), 0.f, 50.f, 0.f, 0.1f, FText::FromString("m"));

    ApplyGamepadFilterToButtons();
    RebuildSliderList();
    UpdateModeSpecificVisibility();
	ResolveControllers();
	PushFlightModeToControllers(); 
	// Ensure our widget reacts to its own slider changes
	OnControlParamChanged.RemoveDynamic(this, &UControlPanelWidget::ApplyParamToController);
	OnControlParamChanged.AddDynamic(this, &UControlPanelWidget::ApplyParamToController);

}


// ---------------- Public API ----------------

void UControlPanelWidget::SetMode(EControlMode NewMode)
{
	if (bGamepadOnly && (NewMode == EControlMode::Position || NewMode == EControlMode::Velocity))
	{
		NewMode = EControlMode::Angle;
	}
	if (CurrentMode == NewMode) return;

	CurrentMode = NewMode;

	if (bSyncModeSelectorUI && ModeSelector)
	{
		ModeSelector->SetMode(NewMode, /*bBroadcast=*/false);
	}

	RebuildSliderList();
	UpdateModeSpecificVisibility();

	// Only apply to controllers after the first explicit user click
	if (bUserPickedMode)
	{
		PushFlightModeToControllers();
	}
}


void UControlPanelWidget::SetGamepadOnly(bool bOn)
{
	if (bGamepadOnly == bOn) return;
	bGamepadOnly = bOn;

	ApplyGamepadFilterToButtons();
	RebuildSliderList();
	UpdateModeSpecificVisibility();

	if (bUserPickedMode)
	{
		PushFlightModeToControllers();
	}
}


void UControlPanelWidget::SetChannelValue(EAxisChannel Channel, float Value, bool bSilent)
{
    auto& MapRef = ValuesFor(CurrentMode);
    MapRef.Add(Channel, Value);

    if (VBox_Sliders)
    {
        const int32 N = VBox_Sliders->GetChildrenCount();
        for (int32 i=0; i<N; ++i)
        {
            if (USliderRow* Row = Cast<USliderRow>(VBox_Sliders->GetChildAt(i)))
            {
                if (Row->GetChannel() == Channel)
                {
                    Row->SetValue(Value, bSilent);
                    break;
                }
            }
        }
    }

    if (!bSilent)
    {
        OnControlParamChanged.Broadcast(CurrentMode, Channel, Value);
    }
}

// ---------------- Helpers ----------------

TMap<EAxisChannel,float>& UControlPanelWidget::ValuesFor(EControlMode Mode)
{
    switch (Mode)
    {
    case EControlMode::Position: return Values_Position;
    case EControlMode::Velocity: return Values_Velocity;
    case EControlMode::Angle:    return Values_Angle;
    default:                     return Values_Acro;
    }
}

const TMap<EAxisChannel,float>& UControlPanelWidget::ValuesFor(EControlMode Mode) const
{
    switch (Mode)
    {
    case EControlMode::Position: return Values_Position;
    case EControlMode::Velocity: return Values_Velocity;
    case EControlMode::Angle:    return Values_Angle;
    default:                     return Values_Acro;
    }
}

void UControlPanelWidget::ApplyGamepadFilterToButtons()
{
    if (bGamepadOnly && (CurrentMode == EControlMode::Position || CurrentMode == EControlMode::Velocity))
    {
        CurrentMode = EControlMode::Angle;
        if (bSyncModeSelectorUI && ModeSelector)
        {
            ModeSelector->SetMode(CurrentMode, /*bBroadcast=*/false);
        }
    }
}

void UControlPanelWidget::RebuildSliderList()
{
	UE_LOG(LogTemp, Log, TEXT("[CP] RebuildSliderList: Mode=%s, Layout=%s"),
	   *UEnum::GetValueAsString(CurrentMode),
	   Layout ? *Layout->GetName() : TEXT("NULL"));

    if (!VBox_Sliders || !SliderRowClass) return;

    VBox_Sliders->ClearChildren();

    const FModeLayout* ModeLayoutPtr = nullptr;
    static const FModeLayout Empty;

    if (Layout)
    {
        ModeLayoutPtr = &Layout->GetLayout(CurrentMode, bGamepadOnly);
    }
    const FModeLayout& ModeLayout = ModeLayoutPtr ? *ModeLayoutPtr : Empty;

    auto& MapRef = ValuesFor(CurrentMode);

    for (const FAxisSpec& Spec : ModeLayout.Axes)
    {
        USliderRow* Row = CreateWidget<USliderRow>(this, SliderRowClass);
        if (!Row) continue;

        Row->Init(Spec);

        if (float* Saved = MapRef.Find(Spec.Channel))
        {
            Row->SetValue(*Saved, /*bSilent=*/true);
        }
        else
        {
            MapRef.Add(Spec.Channel, Spec.Default);
            Row->SetValue(Spec.Default, /*bSilent=*/true);
        }

        Row->OnAxisChanged.AddDynamic(this, &UControlPanelWidget::HandleRowChanged);
        VBox_Sliders->AddChild(Row);
    }
}

void UControlPanelWidget::UpdateModeSpecificVisibility()
{
	if (Btn_ActivateHoverMode)
		Btn_ActivateHoverMode->SetVisibility(ESlateVisibility::Visible);

	if (Row_HoverAltitude)
		Row_HoverAltitude->SetVisibility(ESlateVisibility::Visible);

	// If you still want resets to vary by mode, keep that logic
	const bool bResets = (CurrentMode != EControlMode::Position);
	if (HBox_Resets)
		HBox_Resets->SetVisibility(bResets ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
}


// ---------------- UI events ----------------

void UControlPanelWidget::OnGamepadChanged(bool bChecked)
{
    SetGamepadOnly(bChecked);
}

void UControlPanelWidget::HandleRowChanged(EAxisChannel Channel, float Value)
{
    auto& MapRef = ValuesFor(CurrentMode);
    MapRef.Add(Channel, Value);
    OnControlParamChanged.Broadcast(CurrentMode, Channel, Value);
}

void UControlPanelWidget::HandleModeChanged(EControlMode NewMode)
{
	UE_LOG(LogTemp, Log, TEXT("[CP] HandleModeChanged: %s"), *UEnum::GetValueAsString(NewMode));

	bUserPickedMode = true;          // <- first real user action
	SetMode(NewMode);                // rebuild rows / UI
	PushFlightModeToControllers();   // now actually set EFlightMode
}


// Settings
void UControlPanelWidget::OnMaxVelocityChanged(EAxisChannel /*Channel*/, float Value)
{
	if (Layout) Layout->SetMaxVelocity(Value);
	OnControlSettingChanged.Broadcast(EControlSetting::MaxVelocity, Value);
}
void UControlPanelWidget::OnMaxAngleChanged(EAxisChannel /*Channel*/, float Value)
{
	if (Layout) Layout->SetMaxAngle(Value);
	OnControlSettingChanged.Broadcast(EControlSetting::MaxAngle, Value);
}
void UControlPanelWidget::OnMaxAngleRateChanged(EAxisChannel /*Channel*/, float Value)
{
	if (Layout) Layout->SetMaxAngleRate(Value);
	OnControlSettingChanged.Broadcast(EControlSetting::MaxAngleRate, Value);
}

void UControlPanelWidget::OnActivateHoverModeClicked()
{
	float Alt = 0.f;
	if (Row_HoverAltitude)
	{
		Alt = Row_HoverAltitude->GetValue();
	}

	ForEachTargetController([&](UQuadDroneController* C)
	{
		C->SetHoverMode(true, Alt);
		// Optional: zero Z-velocity right away
		FVector Cur = C->GetDesiredVelocity();
		C->SetDesiredVelocity(FVector(Cur.X, Cur.Y, 0.f));
	});
}
// ---------------- Find ModeSelector recursively ----------------

static UModeSelector* FindModeSelectorDeep(UWidget* Root)
{
    if (!Root) return nullptr;

    if (UModeSelector* AsMS = Cast<UModeSelector>(Root))
        return AsMS;

    if (UUserWidget* AsUW = Cast<UUserWidget>(Root))
    {
        if (UWidget* SubRoot = AsUW->GetRootWidget())
            if (auto* Found = FindModeSelectorDeep(SubRoot)) return Found;
    }

    if (UPanelWidget* Panel = Cast<UPanelWidget>(Root))
    {
        const int32 Num = Panel->GetChildrenCount();
        for (int32 i=0; i<Num; ++i)
            if (auto* Found = FindModeSelectorDeep(Panel->GetChildAt(i))) return Found;
    }
    return nullptr;
}

UModeSelector* UControlPanelWidget::FindFirstModeSelector() const
{
    return FindModeSelectorDeep(GetRootWidget());
}

void UControlPanelWidget::BindToModeSelector()
{
	if (!ModeSelector) return;

	ModeSelector->OnModeChanged.RemoveDynamic(this, &UControlPanelWidget::HandleModeChanged);
	ModeSelector->OnModeChanged.AddDynamic(this, &UControlPanelWidget::HandleModeChanged);

	CurrentMode = ModeSelector->GetCurrentMode();

	UE_LOG(LogTemp, Log, TEXT("[CP] Bound to ModeSelector.OnModeChanged (start mode=%s)"),
		*UEnum::GetValueAsString(CurrentMode));
}

void UControlPanelWidget::HandleLayoutChangedExternal()
{
	// Update the settings rows’ current values (silent)
	if (Layout)
	{
		if (Row_MaxVelocity)   Row_MaxVelocity->SetValue(Layout->MaxVelocity,   /*bSilent=*/true);
		if (Row_MaxAngle)      Row_MaxAngle->SetValue(Layout->MaxAngle,         /*bSilent=*/true);
		if (Row_MaxAngleRate)  Row_MaxAngleRate->SetValue(Layout->MaxAngleRate, /*bSilent=*/true);
	}

	// Rebuild the mode slider rows to pick up new Min/Max ranges
	RebuildSliderList();
}

void UControlPanelWidget::ResolveControllers()
{
	ResolvedController = nullptr;

	if (ControllerOverride)
	{
		ResolvedController = ControllerOverride;
		return;
	}

	// Try Player 0’s pawn
	if (UWorld* W = GetWorld())
	{
		if (APawn* P = W->GetFirstPlayerController() ? W->GetFirstPlayerController()->GetPawn() : nullptr)
		{
			if (AQuadPawn* QP = Cast<AQuadPawn>(P))
			{
				ResolvedController = QP->QuadController; // assuming public member like in ImGui
				if (ResolvedController) return;
			}
		}
	}

	// Fallback: first drone from DroneManager (single target)
	if (ADroneManager* Mgr = ADroneManager::Get(GetWorld()))
	{
		const TArray<AQuadPawn*>& List = Mgr->GetDroneList();
		for (AQuadPawn* QP : List)
		{
			if (QP && QP->QuadController)
			{
				ResolvedController = QP->QuadController;
				break;
			}
		}
	}
}

void UControlPanelWidget::ForEachTargetController(TFunctionRef<void(UQuadDroneController*)> Fn)
{
	ADroneManager* Mgr = ADroneManager::Get(GetWorld());

	const bool bDoSwarm =
		bApplyToSwarm &&
		Mgr && Mgr->IsSwarmMode();

	if (bDoSwarm)
	{
		const TArray<AQuadPawn*>& List = Mgr->GetDroneList();
		for (AQuadPawn* QP : List)
		{
			if (QP && QP->QuadController) Fn(QP->QuadController);
		}
		return;
	}

	if (!ResolvedController) ResolveControllers();
	if (ResolvedController) Fn(ResolvedController);
}

float UControlPanelWidget::GetValueOrDefault(EAxisChannel Ch, float Default) const
{
	const TMap<EAxisChannel,float>& MapRef = ValuesFor(CurrentMode);
	if (const float* V = MapRef.Find(Ch)) return *V;
	return Default;
}

void UControlPanelWidget::ApplyParamToController(EControlMode Mode, EAxisChannel Channel, float /*Value*/)
{
	if (Mode == EControlMode::Velocity)
	{
		switch (Channel)
		{
		case EAxisChannel::X:
		case EAxisChannel::Y:
		case EAxisChannel::Z:
			PushVelocityVector();
			break;
		case EAxisChannel::Yaw:
		case EAxisChannel::YawRate:
			PushYawRate();
			break;
		default: break;
		}
		return;
	}

	if (Mode == EControlMode::Angle)
	{
		switch (Channel)
		{
			// Any of these changing requires pushing the whole Angle target set
		case EAxisChannel::Roll:
		case EAxisChannel::Pitch:
		case EAxisChannel::Yaw:
		case EAxisChannel::YawRate:
		case EAxisChannel::Z:   // vertical speed in Angle mode
			PushAngleTargets();
			break;
		default: break;
		}
		return;
	}

	// (You can add EControlMode::Rate and EControlMode::Position here later.)
}


void UControlPanelWidget::PushVelocityVector()
{
	// Read the current desired velocity from the per-mode map
	float Vx = GetValueOrDefault(EAxisChannel::X, 0.f);
	float Vy = GetValueOrDefault(EAxisChannel::Y, 0.f);
	float Vz = GetValueOrDefault(EAxisChannel::Z, 0.f);

	// If hover mode is active, force Z velocity = 0
	bool bHoverActive = false;

	ForEachTargetController([&](UQuadDroneController* C)
	{
		// Read controller’s hover state if available
		// (If you don’t have a getter, just rely on your UI’s toggle.)
		if (!bHoverActive)
		{
			// optional: if you have IsHoverModeActive()
			// bHoverActive = C->IsHoverModeActive();
		}

		const float ZOut = bHoverActive ? 0.f : Vz;
		const FVector Desired(Vx, Vy, ZOut);
		C->SetDesiredVelocity(Desired);
	});
}

void UControlPanelWidget::PushYawRate()
{
	// Support either YawRate or Yaw channel naming
	float Yr = 0.f;
	if (const TMap<EAxisChannel,float>& MapRef = ValuesFor(CurrentMode); true)
	{
		if (const float* P = MapRef.Find(EAxisChannel::YawRate))      Yr = *P;
		else if (const float* Q = MapRef.Find(EAxisChannel::Yaw))     Yr = *Q;
	}

	ForEachTargetController([&](UQuadDroneController* C)
	{
		C->SetDesiredYawRate(Yr);
	});
}

void UControlPanelWidget::PushAngleTargets()
{
	// Read desireds from the Angle map
	const TMap<EAxisChannel,float>& Map = ValuesFor(EControlMode::Angle);

	const float DesiredRoll  = Map.FindRef(EAxisChannel::Roll);    // deg
	const float DesiredPitch = Map.FindRef(EAxisChannel::Pitch);   // deg
	float       YawRate      = 0.f;                                // deg/s
	if (const float* p = Map.Find(EAxisChannel::YawRate)) YawRate = *p;
	else if (const float* q = Map.Find(EAxisChannel::Yaw)) YawRate = *q; // accept either

	const float DesiredZVel  = Map.FindRef(EAxisChannel::Z);       // m/s (vertical speed slider)

	ForEachTargetController([&](UQuadDroneController* C)
	{
		if (!C) return;

		// 1) Roll / Pitch angles
		C->SetDesiredRollAngle(DesiredRoll);
		C->SetDesiredPitchAngle(DesiredPitch);

		// 2) Yaw as a rate in Angle mode (matches ImGui)
		C->SetDesiredYawRate(YawRate);

		// 3) Vertical control: only write Z velocity if not in hover
		if (!C->IsHoverModeActive())
		{
			const FVector cur = C->GetDesiredVelocity();
			C->SetDesiredVelocity(FVector(cur.X, cur.Y, DesiredZVel));
		}
	});
}
EFlightMode UControlPanelWidget::MapControlModeToFlightMode(EControlMode Mode) const
{
	// ImGui logic:
	// - Non-gamepad: Position→AutoWaypoint, Velocity→VelocityControl, Angle→AngleControl, (Rate)→RateControl
	// - Gamepad: Angle→JoyStickAngleControl, Acro→JoyStickAcroControl
	if (!bGamepadOnly)
	{
		switch (Mode)
		{
		case EControlMode::Position: return EFlightMode::AutoWaypoint;
		case EControlMode::Velocity: return EFlightMode::VelocityControl;
		case EControlMode::Angle:    return EFlightMode::AngleControl;
		case EControlMode::Acro:     return EFlightMode::RateControl; // “Acro” button = Rate (non-gamepad)
		default:                     return EFlightMode::None;
		}
	}
	else
	{
		// Gamepad path: UI only exposes Angle & Acro after your filter
		switch (Mode)
		{
		case EControlMode::Angle: return EFlightMode::JoyStickAngleControl;
		case EControlMode::Acro:  return EFlightMode::JoyStickAcroControl;
			// If user somehow selects others, fold into Angle for safety
		default:                  return EFlightMode::JoyStickAngleControl;
		}
	}
}

void UControlPanelWidget::PushFlightModeToControllers()
{
	const EFlightMode FM = MapControlModeToFlightMode(CurrentMode);

	ForEachTargetController([&](UQuadDroneController* C)
	{
		if (!C) return;

		// Mirror ImGui: SetFlightMode() also resets altitude PID for certain modes.
		C->SetFlightMode(FM);
	});

	UE_LOG(LogTemp, Log, TEXT("[CP] FlightMode set via UI: %s -> %s"),
		*UEnum::GetValueAsString(CurrentMode),
		*UEnum::GetValueAsString(FM));
}


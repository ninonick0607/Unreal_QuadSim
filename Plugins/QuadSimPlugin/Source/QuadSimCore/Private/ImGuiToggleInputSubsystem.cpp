#include "ImGuiToggleInputSubsystem.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/Application/IInputProcessor.h" 
#include "GameFramework/InputSettings.h"
#include "HAL/IConsoleManager.h"
#include "GenericPlatform/ICursor.h"   
#include "Engine/Engine.h"

static const FName kToggleActionName(TEXT("ToggleImGui"));

class FImGuiTogglePreprocessor : public IInputProcessor
{
public:
	explicit FImGuiTogglePreprocessor(TWeakObjectPtr<UImGuiToggleInputSubsystem> InOwner)
		: Owner(InOwner) {}

	// REQUIRED by UE 5.6: implement the pure virtual
	virtual void Tick(const float /*DeltaTime*/,
					  FSlateApplication& /*SlateApp*/,
					  TSharedRef<ICursor> /*Cursor*/) override
	{
		// no-op
	}

	virtual bool HandleKeyDownEvent(FSlateApplication&, const FKeyEvent& E) override
	{
		if (auto* S = Owner.Get(); S && S->ToggleKeys.Contains(E.GetKey()))
		{
			S->ToggleImGuiInput();
			return true;
		}
		return false;
	}

	virtual bool HandleMouseButtonDownEvent(FSlateApplication&, const FPointerEvent& E) override
	{
		if (auto* S = Owner.Get(); S && S->ToggleKeys.Contains(E.GetEffectingButton()))
		{
			S->ToggleImGuiInput();
			return true;
		}
		return false;
	}

private:
	TWeakObjectPtr<UImGuiToggleInputSubsystem> Owner;
};

static APlayerController* GetPrimaryPC(UWorld* World)
{
	if (!World) return nullptr;
	ULocalPlayer* LP = World->GetFirstLocalPlayerFromController();
	return LP ? LP->PlayerController.Get() : World->GetFirstPlayerController();
}

void UImGuiToggleInputSubsystem::Initialize(FSubsystemCollectionBase&)
{
	UE_LOG(LogTemp, Warning, TEXT("[ImGuiToggle] Subsystem Initialize"));

	RefreshToggleKeys();

	ExecImGuiCmd(TEXT("ImGui.Enable on"));
	// Start with ImGui INPUT enabled (true) so UI is interactive on play
	bImGuiInputActive = true;
	SetImGuiInput(true);

	TSharedPtr<FImGuiTogglePreprocessor> Impl = MakeShared<FImGuiTogglePreprocessor>(TWeakObjectPtr<UImGuiToggleInputSubsystem>(this));
	Preprocessor = StaticCastSharedPtr<IInputProcessor>(Impl);
	FSlateApplication::Get().RegisterInputPreProcessor(Preprocessor);

	UE_LOG(LogTemp, Warning, TEXT("[ImGuiToggle] Preprocessor registered; Keys=%d"), ToggleKeys.Num());
}

void UImGuiToggleInputSubsystem::Deinitialize()
{
	if (Preprocessor.IsValid())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(Preprocessor);
		Preprocessor.Reset();
	}
	ToggleKeys.Empty();
}

void UImGuiToggleInputSubsystem::RefreshToggleKeys()
{
	ToggleKeys.Empty();

	if (const UInputSettings* Settings = UInputSettings::GetInputSettings())
	{
		TArray<FInputActionKeyMapping> Mappings;
		Settings->GetActionMappingByName(kToggleActionName, Mappings);
		for (const auto& M : Mappings)
			if (M.Key.IsValid()) ToggleKeys.Add(M.Key);
	}

	if (ToggleKeys.Num() == 0)
	{
		ToggleKeys.Add(EKeys::MiddleMouseButton);
	}
}

void UImGuiToggleInputSubsystem::ToggleImGuiInput()
{
	const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	if (Now - LastToggleTime < DebounceSeconds)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[ImGuiToggle] Debounced (%.3fs since last)"), Now - LastToggleTime);
		return;
	}
	LastToggleTime = Now;

	const bool bNew = !bImGuiInputActive;
	UE_LOG(LogTemp, Warning, TEXT("[ImGuiToggle] Toggle key pressed -> switching to %s"), bNew ? TEXT("ON") : TEXT("OFF"));

	if (GEngine)
	{
		const FString Msg = FString::Printf(TEXT("[ImGuiToggle] Toggle key -> %s"),
											bNew ? TEXT("ON") : TEXT("OFF"));
		GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan, Msg);
	}

	SetImGuiInput(bNew);
}

void UImGuiToggleInputSubsystem::SetImGuiInput(bool bEnable)
{
	// Prevent re-entrant calls or redundant toggles
	if (bImGuiInputActive == bEnable)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[ImGuiToggle] Already in desired state %s, skipping"), bEnable ? TEXT("ON") : TEXT("OFF"));
		return;
	}

	bool bAppliedViaCVar = false;

	// 0) Keep ImGui visible always (harmless if unsupported)
	ExecImGuiCmd(TEXT("ImGui.Enable on"));

	// 1) CVAR first (if present in your plugin)
	if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(TEXT("ImGui.InputEnabled")))
	{
		V->Set(bEnable ? 1 : 0, ECVF_SetByCode);
		bAppliedViaCVar = true;
	}

	// 2) Known command spellings across common Unreal ImGui forks
	if (bEnable)
	{
		ExecImGuiCmd(TEXT("ImGui.SetInputEnabled 1"));   // variant A
		ExecImGuiCmd(TEXT("ImGui.ToggleInput on"));      // variant B
	}
	else
	{
		ExecImGuiCmd(TEXT("ImGui.SetInputEnabled 0"));   // variant A
		ExecImGuiCmd(TEXT("ImGui.ToggleInput off"));     // variant B
	}

	// 3) Update state BEFORE changing input mode to avoid race conditions
	bImGuiInputActive = bEnable;

	// 4) Make the change visible: cursor + input mode flip
	UWorld* W = GetWorld();
	APlayerController* PC = W ? GetPrimaryPC(W) : nullptr;

	if (PC)
	{
		if (bImGuiInputActive)
		{
			// UI interactive - ImGui enabled
			PC->bShowMouseCursor = true;
			PC->SetIgnoreLookInput(true);
			PC->SetIgnoreMoveInput(true);

			FInputModeGameAndUI Mode;
			Mode.SetHideCursorDuringCapture(false);
			Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
			PC->SetInputMode(Mode);
		}
		else
		{
			// Gameplay - ImGui disabled, camera control enabled
			PC->bShowMouseCursor = false;
			PC->SetIgnoreLookInput(false);
			PC->SetIgnoreMoveInput(false);

			FInputModeGameOnly Mode;
			Mode.SetConsumeCaptureMouseDown(true);
			PC->SetInputMode(Mode);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("[ImGuiToggle] SetImGuiInput %s (viaCVar=%d, PC=%s)"),
		bEnable ? TEXT("ON") : TEXT("OFF"),
		bAppliedViaCVar ? 1 : 0,
		PC ? TEXT("Valid") : TEXT("NULL"));

	if (GEngine)
	{
		const FString Msg = FString::Printf(TEXT("[ImGuiToggle] INPUT %s"),
			bImGuiInputActive ? TEXT("ON (UI)") : TEXT("OFF (Game)"));
		GEngine->AddOnScreenDebugMessage(-1, 3.0f, bImGuiInputActive ? FColor::Green : FColor::Yellow, Msg);
	}
}


void UImGuiToggleInputSubsystem::ExecImGuiCmd(const TCHAR* Cmd)
{
	if (UWorld* W = GetWorld())
	{
		if (APlayerController* PC = GetPrimaryPC(W))
		{
			PC->ConsoleCommand(Cmd, /*bWriteToLog*/ true);
			return;
		}
		if (GEngine)
		{
			GEngine->Exec(W, Cmd);
			return;
		}
	}
}

bool UImGuiToggleInputSubsystem::HasAnyImGuiCVar() const
{
	return IConsoleManager::Get().FindConsoleVariable(TEXT("ImGui.InputEnabled"))
	    || IConsoleManager::Get().FindConsoleVariable(TEXT("ImGui.Enabled"));
}

bool UImGuiToggleInputSubsystem::ReadImGuiCVar() const
{
	if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(TEXT("ImGui.InputEnabled")))
		return V->GetInt() != 0;
	if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(TEXT("ImGui.Enabled")))
		return V->GetInt() != 0;
	return bImGuiInputActive;
}

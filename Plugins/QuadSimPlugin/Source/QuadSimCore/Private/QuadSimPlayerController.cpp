#include "QuadSimPlayerController.h"
#include "Blueprint/UserWidget.h"

void AQuadSimPlayerController::BeginPlay()
{
	Super::BeginPlay();

	// Don’t create UI here (HUD does that in Option B).
	// Just set friendly defaults so when HUD calls into us, things behave.
	SetDefaultMouseFlags(/*bEnable=*/true);
	bShowMouseCursor = false; // HUD will turn this on when it creates the widget
}

void AQuadSimPlayerController::SetupInputComponent()
{
	Super::SetupInputComponent();

	if (InputComponent)
	{
		// You already had this binding
		InputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadSimPlayerController::ToggleImguiInput);
	}
}

void AQuadSimPlayerController::ToggleImguiInput()
{
    ConsoleCommand(TEXT("ImGui.ToggleInput"));
    bImGuiInputActive = !bImGuiInputActive;
    if (bImGuiInputActive)
    {
        // Free the mouse and allow UI clicks when ImGui input is active
        ApplyGameAndUIFocus(nullptr, /*bShowCursor=*/true);
    }
    else
    {
        // Return to game-only controls
        ApplyGameOnly();
    }
}

void AQuadSimPlayerController::ApplyGameAndUIFocus(UUserWidget* WidgetToFocus, bool bShowCursorIn)
{
	SetDefaultMouseFlags(/*bEnable=*/true);
	bShowMouseCursor = bShowCursorIn;

	FInputModeGameAndUI Mode;
	if (WidgetToFocus)
	{
		Mode.SetWidgetToFocus(WidgetToFocus->TakeWidget());
	}
	Mode.SetHideCursorDuringCapture(false);
	Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);

	SetInputMode(Mode);
}

void AQuadSimPlayerController::ApplyGameOnly()
{
	SetDefaultMouseFlags(/*bEnable=*/false);
	bShowMouseCursor = false;

	FInputModeGameOnly Mode;
	Mode.SetConsumeCaptureMouseDown(true);
	SetInputMode(Mode);
}

void AQuadSimPlayerController::SetDefaultMouseFlags(bool bEnable)
{
	bEnableClickEvents = bEnable;
	bEnableMouseOverEvents = bEnable;
}

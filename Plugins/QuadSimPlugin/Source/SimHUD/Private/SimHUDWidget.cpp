#include "SimHUDWidget.h"
#include "GameFramework/PlayerController.h"

void ASimHUDWidget::BeginPlay()
{
	Super::BeginPlay();

	if (MainWidgetClass)
	{
		if (APlayerController* PC = GetOwningPlayerController())
		{
			MainWidget = CreateWidget<UUserWidget>(PC, MainWidgetClass);
			if (MainWidget)
			{
				MainWidget->AddToViewport(100);

				// Give UI focus & mouse
				PC->bShowMouseCursor = true;
				PC->bEnableClickEvents = true;
				PC->bEnableMouseOverEvents = true;

				FInputModeGameAndUI Mode;
				Mode.SetWidgetToFocus(MainWidget->TakeWidget());
				Mode.SetHideCursorDuringCapture(false);
				Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
				PC->SetInputMode(Mode);
			}
		}
	}
}

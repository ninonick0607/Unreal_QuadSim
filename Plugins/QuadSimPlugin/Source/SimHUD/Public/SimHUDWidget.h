#pragma once
#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "Blueprint/UserWidget.h"
#include "SimHUDWidget.generated.h"

UCLASS()
class ASimHUDWidget : public AHUD
{
	GENERATED_BODY()
protected:
	virtual void BeginPlay() override;

	UPROPERTY(EditDefaultsOnly, Category="UI")
	TSubclassOf<UUserWidget> MainWidgetClass;

	UPROPERTY()
	UUserWidget* MainWidget = nullptr;
};

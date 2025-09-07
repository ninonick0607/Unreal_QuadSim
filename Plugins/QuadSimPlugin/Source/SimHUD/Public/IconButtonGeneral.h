#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "IconButtonGeneral.generated.h"

class UButton;

/** Fires when the button is clicked */
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnIconButtonPressed);

UCLASS()
class SIMHUD_API UIconButtonGeneral : public UUserWidget
{
	GENERATED_BODY()

public:
	/** Broadcast on click */
	UPROPERTY(BlueprintAssignable, Category="IconButton")
	FOnIconButtonPressed OnPressed;

	/** Set/get selected (selected appears as “hovered” at rest) */
	UFUNCTION(BlueprintCallable, Category="IconButton")
	void SetSelected(bool bInSelected);

	UFUNCTION(BlueprintPure, Category="IconButton")
	bool IsSelected() const { return bSelected; }

protected:
	virtual void NativeConstruct() override;

	/** Bind this to your actual UMG Button in the WBP */
	UPROPERTY(meta=(BindWidget))
	UButton* Button = nullptr;

private:
	// Cached styles
	UPROPERTY() FButtonStyle DefaultStyle;   // whatever the Button has in BP
	UPROPERTY() FButtonStyle SelectedStyle;  // Default with Normal=Hovered

	// State
	UPROPERTY() bool bSelected = false;

	// Internals
	void ApplyCurrentStyle();

	// Button handlers
	UFUNCTION() void HandleClicked();
	UFUNCTION() void HandleHovered();
	UFUNCTION() void HandleUnhovered();
};

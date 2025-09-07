#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "MainWidget.generated.h"

class UUserWidget;
class UWidget;
class UButton;

/**
 * C++ parent for WBP_MainWidget.
 * - Finds & binds the Control Panel button whether it's a UButton or a wrapper UserWidget.
 * - Spawns/toggles the Control Panel.
 */
UCLASS()
class SIMHUD_API UMainWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;

	/** Toggle from code or Blueprint */
	UFUNCTION(BlueprintCallable, Category="UI")
	void ToggleControlPanel();

	/** Force show/hide (creates on demand if bShow=true and not created yet) */
	UFUNCTION(BlueprintCallable, Category="UI")
	void ShowControlPanel(bool bShow);

protected:
	/** Name of the clickable widget in WBP_MainWidget (must exist). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="UI")
	FName ControlPanelButtonName = TEXT("Button_ControlPanel");

	/** Set this to WBP_ControlPanel in WBP_MainWidget defaults */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="UI")
	TSubclassOf<UUserWidget> ControlPanelClass;

	/** Z-order for the control panel widget */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="UI")
	int32 ControlPanelZOrder = 5;

private:
	/** Runtime instance we spawn/toggle */
	UPROPERTY()
	UUserWidget* ControlPanelInstance = nullptr;

	/** Cached resolved button (the real inner UButton) */
	UPROPERTY(Transient)
	UButton* ControlPanelButton = nullptr;

private:
	UFUNCTION()
	void HandleControlPanelClicked();

	/** Locate widget by name, then drill down to find the first real UButton. */
	UButton* ResolveControlPanelButton();

	/** Recursive search for a UButton under any widget. */
	UButton* FindFirstButtonDeep(UWidget* Root) const;
};

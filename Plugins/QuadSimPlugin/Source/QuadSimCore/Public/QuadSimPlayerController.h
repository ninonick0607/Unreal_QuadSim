#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "QuadSimPlayerController.generated.h"

class UUserWidget;

/**
 * PlayerController used with the Sim HUD (HUD owns the main widget).
 * - No widget creation here.
 * - Provides handy helpers to switch input modes.
 * - Binds ToggleImGui input.
 */
UCLASS()
class QUADSIMCORE_API AQuadSimPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	virtual void BeginPlay() override;
	virtual void SetupInputComponent() override;

	/** Focus UI and keep game input: shows cursor, sets Game+UI, and focuses the widget. */
	UFUNCTION(BlueprintCallable, Category="Input")
	void ApplyGameAndUIFocus(UUserWidget* WidgetToFocus, bool bShowCursor = true);

	/** Return to game-only input (hides cursor). */
	UFUNCTION(BlueprintCallable, Category="Input")
	void ApplyGameOnly();

protected:
    /** Console toggle for ImGui input (expects an input action named "ToggleImGui"). */
    UFUNCTION()
    void ToggleImguiInput();

    /** Common mouse flags we like for UI work. */
    void SetDefaultMouseFlags(bool bEnable);

private:
    // Track whether we've toggled ImGui input on, to switch input modes appropriately
    bool bImGuiInputActive = false;
};

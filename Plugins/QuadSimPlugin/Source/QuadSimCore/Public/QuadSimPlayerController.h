#pragma once
#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "QuadSimPlayerController.generated.h"

UCLASS()
class QUADSIMCORE_API AQuadSimPlayerController : public APlayerController
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void SetupInputComponent() override;

    // Arrow-key camera axes
    void OnYawAxis(float Value);
    void OnPitchAxis(float Value);
    void OnZoomAxis(float Value);

    // Shims other modules might still call
    UFUNCTION(BlueprintCallable, Category="QuadSim|InputModes")
    void ApplyGameAndUIFocus(bool bShowCursorIn);
    UFUNCTION(BlueprintCallable, Category="QuadSim|InputModes")
    void ApplyGameOnly();
    UFUNCTION(BlueprintCallable, Category="QuadSim|InputModes")
    void SetDefaultMouseFlags(bool bEnable);

private:
    float YawRate   = 1.5f;
    float PitchRate = 1.5f;
    float ZoomStep  = 1.0f;

    void ForwardLookToViewTarget(const FVector2D& Delta);
    void ForwardZoomToViewTarget(float Delta);
};

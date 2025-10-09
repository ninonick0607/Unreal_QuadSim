#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "TopDownCameraPawn.generated.h"

UCLASS()
class SIMULATIONCORE_API ATopDownCameraPawn : public APawn
{
	GENERATED_BODY()
public:
	ATopDownCameraPawn();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
	class UCameraComponent* Camera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
	class USpectatorPawnMovement* Movement;

	void SetCameraTransform(const FVector& Location, const FRotator& Rotation);
};

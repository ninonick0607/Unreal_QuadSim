#include "Camera/TopDownCameraPawn.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpectatorPawnMovement.h"
#include "Components/SceneComponent.h"

ATopDownCameraPawn::ATopDownCameraPawn()
{
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(RootComponent);

	Movement = CreateDefaultSubobject<USpectatorPawnMovement>(TEXT("Movement"));
	Movement->MaxSpeed = 1200.f;
	Movement->bIgnoreTimeDilation = true;

	AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

void ATopDownCameraPawn::SetCameraTransform(const FVector& Location, const FRotator& Rotation)
{
	SetActorLocation(Location);
	SetActorRotation(FRotator(Rotation));
	Camera->SetWorldLocation(Location);
	Camera->SetWorldRotation(Rotation);
}

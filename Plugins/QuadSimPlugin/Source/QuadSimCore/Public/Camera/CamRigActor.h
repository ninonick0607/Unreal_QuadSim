// Camera/CamRigActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CamRigActor.generated.h"

class UCameraRigComponent;

UCLASS()
class QUADSIMCORE_API ACamRigActor : public AActor
{
	GENERATED_BODY()
public:
	ACamRigActor();

	UPROPERTY(VisibleAnywhere) USceneComponent*     Root;
	UPROPERTY(VisibleAnywhere) UCameraRigComponent* Rig;

	virtual void BeginPlay() override;
};

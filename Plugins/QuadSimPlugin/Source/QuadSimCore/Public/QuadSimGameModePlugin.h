#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "QuadSimGameModePlugin.generated.h"

class ADroneManager;

UCLASS()
class QUADSIMCORE_API AQuadSimGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AQuadSimGameMode();
    virtual void BeginPlay() override;

private:
    UPROPERTY()
    ADroneManager* DroneManagerRef = nullptr;  // keep if you like for camera handoff, etc.
};

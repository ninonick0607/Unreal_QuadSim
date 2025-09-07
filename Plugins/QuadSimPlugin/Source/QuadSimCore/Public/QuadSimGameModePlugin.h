// QuadSimGameModePlugin.h
#pragma once

#include "GameFramework/GameModeBase.h"
#include "QuadSimGameModePlugin.generated.h"

UCLASS()
class QUADSIMCORE_API AQuadSimGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    AQuadSimGameMode();
    virtual void BeginPlay() override;
};
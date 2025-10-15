// SimWorldSubsystem.cpp
#include "Runtime/SimWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "SimulationCore/Public/Core/SimulationManager.h"

void USimWorldSubsystem::EnsureSimulationManager()
{
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<ASimulationManager> It(World); It; ++It) return;
        World->SpawnActor<ASimulationManager>(ASimulationManager::StaticClass(), FTransform::Identity);
        UE_LOG(LogTemp, Log, TEXT("Spawned SimulationManager"));
    }
}

void USimWorldSubsystem::InitializeWorld()
{
    // GeoReferencing is now placed directly in the map
    EnsureSimulationManager();
}

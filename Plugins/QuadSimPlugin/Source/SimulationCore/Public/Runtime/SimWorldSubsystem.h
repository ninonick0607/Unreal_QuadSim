// SimWorldSubsystem.h
#pragma once
#include "Subsystems/WorldSubsystem.h"
#include "SimWorldSubsystem.generated.h"

UCLASS()
class SIMULATIONCORE_API USimWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override {}

	// Call this once from GameMode::BeginPlay
	UFUNCTION(BlueprintCallable, Category="Simulation|World")
	void InitializeWorld();

private:
	void EnsureSimulationManager();
};

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

	// --- GeoRef defaults you requested ---
	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	bool bRoundPlanet = true;

	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	double OriginLonDeg = -86.5732;

	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	double OriginLatDeg = 30.4752;

	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	double OriginHeightM = 0.0;

	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	FString GeographicCRS = TEXT("EPSG:4326");

	UPROPERTY(EditAnywhere, Category="Simulation|GeoRef")
	FString ProjectedCRS  = TEXT("EPSG:32631");

private:
	void EnsureGeoReferencing();
	void EnsureSimulationManager();
};

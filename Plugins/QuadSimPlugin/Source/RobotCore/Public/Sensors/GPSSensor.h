// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "GPSSensor.generated.h"

// Forward declarations
class AGeoReferencingSystem;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class ROBOTCORE_API UGPSSensor : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UGPSSensor();
	float SensorNoise();
	void UpdateSensor(float DeltaTime, bool bNoise );
	FVector SampleRawGPS() const;
	FVector GetLastGPS() const { return LastGPS; }

	void Initialize();
	
	UFUNCTION(BlueprintCallable, Category = "GPS")
	FVector GetGeographicCoordinates() const; 
    
private:
	float UpdateRate = 10.0f;          
	float AccumulatedTime = 0.0f;      

	float LatLonNoiseStdDev = 0.02f;   // meters (2cm)
	float AltNoiseStdDev = 0.05f;      // meters (5cm)

	FVector LastGPS;
	
	FVector LastGeographicCoords;
    
	UPROPERTY()
	AGeoReferencingSystem* GeoRefSystem = nullptr;

};

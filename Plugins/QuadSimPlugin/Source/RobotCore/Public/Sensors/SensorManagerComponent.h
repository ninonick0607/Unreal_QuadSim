// SensorManagerComponent.h
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "SensorManagerComponent.generated.h"

// Forward declarations
class UGPSSensor;
class UIMUSensor;
class UMagSensor;
class UBaroSensor;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ROBOTCORE_API USensorManagerComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	USensorManagerComponent();
	void BeginPlay() override;
	UFUNCTION(BlueprintCallable, Category = "Sensors")
	void InitializeSensors();
    
	UFUNCTION(BlueprintCallable, Category = "Sensors")
	void UpdateAllSensors(float DeltaTime, bool bAddNoise = true);
    
	UFUNCTION(BlueprintPure, Category = "Sensors")
	bool AreSensorsInitialized() const { return bSensorsInitialized; }
    
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UGPSSensor* GPS;
    
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UIMUSensor* IMU;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UMagSensor* Magnetometer; 

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UBaroSensor* Barometer;
    
	// UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	// URangefinderSensor* Rangefinder;

private:
	bool bSensorsInitialized;
    
//	void SetupSensorAttachments();
};
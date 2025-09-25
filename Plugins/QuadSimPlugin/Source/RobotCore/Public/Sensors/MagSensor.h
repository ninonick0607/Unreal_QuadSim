// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MagSensor.generated.h"
class AGeoReferencingSystem;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ROBOTCORE_API UMagSensor : public USceneComponent
{
	GENERATED_BODY()
    
public:
	UMagSensor();
    
	float SensorNoise();
	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	bool IsInitialized() const { return bInitialized; }
    
	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	float GetLastUpdateTime() const { return LastUpdateTime; }

	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	float GetHeading() const { return MagneticHeading; }
    
	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	bool IsCalibrating() const { return bIsCalibrating; }
    
	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	float GetDeclination() const { return MagneticDeclination; }
	
	void UpdateSensor(float DeltaTime, bool bNoise);
	FVector GetLastMagField() const { return LastMagField; }
	FVector GetLastMagFieldMilliGauss() const { return LastMagField * 1000.0f; } // Convert to milligauss
    void CalculateMagneticHeading();
	void Initialize();
    
private:
	float UpdateRate = 100.0f;  
	float AccumulatedTime = 0.0f;
	bool bInitialized = false;
	bool bIsCalibrating = false;
	float LastUpdateTime = 0.0f;
	float CalibrationTime = 0.0f;
	static constexpr float CALIBRATION_DURATION = 2.0f; // 3 seconds calibration

	float MagneticHeading = 0.0f;
	float MagneticDeclination = 0.0f;
    FVector LastMagField;       
    FVector EarthMagField;      
    bool bEarthMagFieldValid = false;
    bool bDeclinationLogged = false;
    
	// Noise parameters
	float MagNoiseStdDev = 0.00001f;  
	
	UPROPERTY(EditAnywhere, Category = "Magnetometer")
	float MagOffsetX = 0.0f;
    
	UPROPERTY(EditAnywhere, Category = "Magnetometer")
	float MagOffsetY = 0.0f;
    
	UPROPERTY(EditAnywhere, Category = "Magnetometer")
	float MagOffsetZ = 0.0f;
    
	void UpdateEarthMagField();
	FVector TransformToBodyFrame(const FVector& EarthField);
	FVector NoiseGauss3f(float StdX, float StdY, float StdZ);

	UPROPERTY()
	AGeoReferencingSystem* GeoRefSystem = nullptr;
};

// In a new file: SensorData.h
#pragma once
#include "CoreMinimal.h"
#include "SensorData.generated.h"

USTRUCT(BlueprintType)
struct FSensorData
{
	GENERATED_BODY()

	// IMU Data
	UPROPERTY()
	FVector IMUAngVelRADS = FVector::ZeroVector;
    
	UPROPERTY()
	FVector IMULinearAccelMS2 = FVector::ZeroVector;
    
	UPROPERTY()
	FRotator IMUAttitude = FRotator::ZeroRotator;
	
	UPROPERTY()
	FVector IMUVelMS = FVector::ZeroVector;
	
	// GPS Data
	UPROPERTY()
	FVector GPSPosMeters = FVector::ZeroVector;
	UPROPERTY()
	FVector GPSLatLong = FVector::ZeroVector;
    
	// Barometer Data
	UPROPERTY()
	float BaroAltitudeM = 0.0f;
    float BaroTemp = 0.0f;
	float BaroLastPressureHPa = 0.0f;
	
	// Magnetometer Data
	UPROPERTY()
	FVector MagFieldGauss = FVector::ZeroVector;
    
	UPROPERTY()
	float MagHeadingDeg = 0.0f;
	
	UPROPERTY()
	float MagDeclinationDeg = 0.0f;
	// Timestamps
	UPROPERTY()
	float IMUTimestamp = 0.0f;
    
	UPROPERTY()
	float GPSTimestamp = 0.0f;
    
	// Data validity flags
	UPROPERTY()
	bool bIMUValid = false;
    
	UPROPERTY()
	bool bGPSValid = false;
    
	UPROPERTY()
	bool bBaroValid = false;
    
	UPROPERTY()
	bool bMagValid = false;
};
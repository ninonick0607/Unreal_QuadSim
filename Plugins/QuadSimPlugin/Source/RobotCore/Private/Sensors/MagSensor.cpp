// Fill out your copyright notice in the Description page of Project Settings.

#include "Sensors/MagSensor.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/SensorManagerComponent.h"
#include "WorldMagneticModel/GeoMagDeclination.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "Math/UnrealMathUtility.h"
#include "EngineUtils.h" 
#include "GeoReferencingSystem.h"
#include "GeographicCoordinates.h"
UMagSensor::UMagSensor()
{
    PrimaryComponentTick.bCanEverTick = false;
    LastMagField = FVector::ZeroVector;
    EarthMagField = FVector::ZeroVector;
}

void UMagSensor::Initialize()
{
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It)
		{
			GeoRefSystem = *It;
			break;
		}
        
		if (!GeoRefSystem)
		{
			UE_LOG(LogTemp, Warning, TEXT("MagSensor: No GeoReferencingSystem found in level! Please add one."));
		}
	}
    // Any initialization if needed
    bEarthMagFieldValid = false;
    UE_LOG(LogTemp, Display, TEXT("MagSensor: Initialized"));
}

void UMagSensor::UpdateSensor(float DeltaTime, bool bNoise)
{
    AccumulatedTime += DeltaTime;
    const float Period = 1.0f / UpdateRate;
    
    if (AccumulatedTime < Period)
    {
        return;
    }
    AccumulatedTime -= Period;
    
    // First, update the Earth's magnetic field based on GPS position
    UpdateEarthMagField();
    
    if (!bEarthMagFieldValid)
    {
        // No valid magnetic field data yet
        return;
    }
    
    // Transform Earth field to body frame
    FVector BodyMagField = TransformToBodyFrame(EarthMagField);
    // Add noise if enabled
    if (bNoise)
    {
        BodyMagField += NoiseGauss3f (MagNoiseStdDev, MagNoiseStdDev, MagNoiseStdDev);
    }
    
    // Add sensor bias/offset
    BodyMagField.X += MagOffsetX;
    BodyMagField.Y += MagOffsetY;
    BodyMagField.Z += MagOffsetZ;
    
    // Store the result
    LastMagField = BodyMagField;
}

void UMagSensor::UpdateEarthMagField()
{
	// Check if we have georeferencing system
	if (!GeoRefSystem)
	{
		return;
	}
    
	// Get the drone's world position
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
    
	// Get current position in Unreal coordinates
	FVector WorldPosition = Owner->GetActorLocation();
    
	// Convert to geographic coordinates using the Georeferencing plugin
	FGeographicCoordinates GeoCoords;
	GeoRefSystem->EngineToGeographic(WorldPosition, GeoCoords);
    
	// Get the actual latitude and longitude
	float Latitude = GeoCoords.Latitude;
	float Longitude = GeoCoords.Longitude;
	float Altitude = GeoCoords.Altitude;
    
	// Get magnetic field vector from World Magnetic Model
	FVector MagFieldNED = FGeoMagDeclination::GetMagFieldVector(Latitude, Longitude);
    
	// The magnetic field is already in NED (North-East-Down) frame
	// We need to convert it to Unreal Engine coordinates
	// Get the ENU vectors at this location
	FVector EastVector, NorthVector, UpVector;
	GeoRefSystem->GetENUVectorsAtGeographicLocation(GeoCoords, EastVector, NorthVector, UpVector);
    
	// Transform magnetic field from NED to Unreal coordinates
	// NED: North, East, Down
	// Transform to world coordinates using the local tangent frame
	EarthMagField = MagFieldNED.X * NorthVector + 
				   MagFieldNED.Y * EastVector + 
				   MagFieldNED.Z * (-UpVector); // Down = -Up
    
	bEarthMagFieldValid = true;

}

FVector UMagSensor::TransformToBodyFrame(const FVector& EarthField)
{
    // Get the vehicle's rotation
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        return EarthField;
    }
    
    // Get the inverse rotation to transform from world to body frame
    FQuat VehicleRotation = Owner->GetActorQuat();
    FQuat InverseRotation = VehicleRotation.Inverse();
    
    // Transform the magnetic field vector from world frame to body frame
    return InverseRotation.RotateVector(EarthField);
}

FVector UMagSensor::NoiseGauss3f(float StdX, float StdY, float StdZ)
{
    return FVector(
        SensorNoise() * StdX,
        SensorNoise() * StdY,
        SensorNoise() * StdZ
    );
}

float UMagSensor::SensorNoise()
{
    // Box-Muller transform for Gaussian noise (same as other sensors)
    static bool bPhase = true;
    static float V1, V2, S;
    float X;
    
    if (bPhase)
    {
        do {
            float U1 = FMath::FRand();
            float U2 = FMath::FRand();
            V1 = 2.0f * U1 - 1.0f;
            V2 = 2.0f * U2 - 1.0f;
            S = V1 * V1 + V2 * V2;
        } while (S >= 1.0f || S < KINDA_SMALL_NUMBER);
        
        X = V1 * FMath::Sqrt(-2.0f * FMath::Loge(S) / S);
    }
    else
    {
        X = V2 * FMath::Sqrt(-2.0f * FMath::Loge(S) / S);
    }
    
    bPhase = !bPhase;
    return X;
}
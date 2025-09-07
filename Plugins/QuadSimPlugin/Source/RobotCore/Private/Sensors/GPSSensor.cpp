#include "Sensors/GPSSensor.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"
#include "GeoReferencingSystem.h"
#include "GeographicCoordinates.h"

// EVERYTHING IN M

UGPSSensor::UGPSSensor()
{

	PrimaryComponentTick.bCanEverTick = false;
	LastGPS = FVector::ZeroVector;
	LastGeographicCoords = FVector::ZeroVector;

}

void UGPSSensor::Initialize()
{
	// Find the GeoReferencingSystem in the world
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It)
		{
			GeoRefSystem = *It;
			break;
		}
        
		if (!GeoRefSystem)
		{
			UE_LOG(LogTemp, Warning, TEXT("GPSSensor: No GeoReferencingSystem found in level! Please add one."));
		}
		else
		{
			UE_LOG(LogTemp, Display, TEXT("GPSSensor: Found GeoReferencingSystem"));
		}
	}
}

FVector UGPSSensor::GetGeographicCoordinates() const
{
	if (!GeoRefSystem || !GetOwner())
	{
		UE_LOG(LogTemp, Warning, TEXT("GPSSensor: Cannot get geographic coordinates - no GeoRefSystem or Owner"));
		return LastGeographicCoords;
	}
    
	// Get current position in Unreal coordinates
	FVector WorldPosition = GetOwner()->GetActorLocation();
    
	// Convert to geographic coordinates using the Georeferencing plugin
	FGeographicCoordinates GeoCoords;
	GeoRefSystem->EngineToGeographic(WorldPosition, GeoCoords);
	// Return as FVector (Latitude, Longitude, Altitude)
	return FVector(GeoCoords.Latitude, GeoCoords.Longitude, GeoCoords.Altitude);
}

FVector UGPSSensor::SampleRawGPS() const
{

	if (AActor* Owner = GetOwner())
	{
		return Owner->GetActorLocation()/100;
	}
	
	// Fallback to component location if no owner
	return GetComponentLocation()/100;
}

void UGPSSensor::UpdateSensor(float DeltaTime, bool bNoise)
{
	AccumulatedTime += DeltaTime;
	const float Period = 1.f / UpdateRate;

	if (AccumulatedTime < Period)
		return;
	AccumulatedTime -= Period;

	FVector Pos = SampleRawGPS();

	if (bNoise)
	{
		Pos.X += SensorNoise() * LatLonNoiseStdDev;
		Pos.Y += SensorNoise() * LatLonNoiseStdDev;
		Pos.Z += SensorNoise() * AltNoiseStdDev;
	}

	LastGPS = Pos;
	LastGeographicCoords = GetGeographicCoordinates();

}

float UGPSSensor::SensorNoise()
{
	// Box–Muller
	static bool bPhase = true;
	static float V1, V2, S;
	float X;

	if (bPhase)
	{
		do {
			float U1 = FMath::FRand();
			float U2 = FMath::FRand();
			V1 = 2.f * U1 - 1.f;
			V2 = 2.f * U2 - 1.f;
			S  = V1 * V1 + V2 * V2;
		} while (S >= 1.f || S < KINDA_SMALL_NUMBER);

		X = V1 * FMath::Sqrt(-2.f * FMath::Loge(S) / S);
	}
	else
	{
		X = V2 * FMath::Sqrt(-2.f * FMath::Loge(S) / S);
	}

	bPhase = !bPhase;
	return X;
}
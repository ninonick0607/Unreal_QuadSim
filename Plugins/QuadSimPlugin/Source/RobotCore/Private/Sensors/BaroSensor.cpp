// BaroSensor.cpp
#include "Sensors/BaroSensor.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"
#include "GeoReferencingSystem.h"
#include "GeographicCoordinates.h"
#include "Math/UnrealMathUtility.h"

UBaroSensor::UBaroSensor()
{
	PrimaryComponentTick.bCanEverTick = false;
	
	// Initialize with sea level values
	LastPressure = ISA_PRESSURE_MSL;
	LastTemperature = ISA_TEMPERATURE_MSL - 273.15f;  // Convert to Celsius
	EstimatedAltitude = 0.0f;
}

void UBaroSensor::Initialize()
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
			UE_LOG(LogTemp, Warning, TEXT("BaroSensor: No GeoReferencingSystem found in level! Using default altitude."));
		}
	}
	
	// Reset drift
	BaroDriftPa = 0.0f;
	
	// Initialize random seed for noise generation
	FMath::RandInit(FDateTime::Now().GetTicks());
	
	UE_LOG(LogTemp, Display, TEXT("BaroSensor: Initialized"));
}

void UBaroSensor::UpdateSensor(float DeltaTime, bool bNoise)
{
	AccumulatedTime += DeltaTime;
	const float Period = 1.0f / UpdateRate;
	
	if (AccumulatedTime < Period)
	{
		return;
	}
	
	// Calculate actual time step
	float ActualDeltaTime = AccumulatedTime;
	AccumulatedTime = 0.0f;
	
	// Get altitude from owner's position
	float AltitudeMSL = 0.0f;
	
	AActor* Owner = GetOwner();
	if (Owner && GeoRefSystem)
	{
		FVector WorldPosition = Owner->GetActorLocation();
		
		// Convert to geographic coordinates to get altitude
		FGeographicCoordinates GeoCoords;
		GeoRefSystem->EngineToGeographic(WorldPosition, GeoCoords);
		
		// GeoCoords.Altitude is in meters above the ellipsoid
		// For barometric purposes, we want altitude above mean sea level
		AltitudeMSL = GeoCoords.Altitude;
	}
	else if (Owner)
	{
		// Fallback: use Z coordinate as altitude (convert from cm to m)
		AltitudeMSL = Owner->GetActorLocation().Z / 100.0f;
	}
	
	// Store the estimated altitude
	EstimatedAltitude = AltitudeMSL;
	
	// Calculate pressure and temperature based on altitude
	CalculatePressureFromAltitude(AltitudeMSL);
	
	// Apply drift
	if (bNoise)
	{
		BaroDriftPa += BaroDriftPaPerSec * ActualDeltaTime;
		LastPressure += BaroDriftPa;
		
		// Add Gaussian noise
		float PressureNoise = PressureNoiseStdDev * GenerateGaussianNoise();
		LastPressure += PressureNoise;
	}
	
	// Apply user-defined offsets
	LastPressure += PressureOffset;
	LastTemperature += TemperatureOffset;
	
	// Log periodically for debugging
	static float LogTimer = 0.0f;
	LogTimer += Period;
	if (LogTimer >= 1.0f)
	{
		UE_LOG(LogTemp, Verbose, TEXT("BaroSensor: Alt=%.2fm, Pressure=%.2fhPa, Temp=%.2f°C"), 
			   AltitudeMSL, LastPressure / 100.0f, LastTemperature);
		LogTimer = 0.0f;
	}
}

void UBaroSensor::CalculatePressureFromAltitude(float AltitudeMSL)
{

	float TemperatureLocal = ISA_TEMPERATURE_MSL - ISA_LAPSE_RATE * AltitudeMSL;
	float TemperatureRatio = ISA_TEMPERATURE_MSL / TemperatureLocal;
	float PressureRatio = FMath::Pow(TemperatureRatio, 5.256f);
	
	LastPressure = ISA_PRESSURE_MSL / PressureRatio;
	LastTemperature = TemperatureLocal - 273.15f;
}


float UBaroSensor::GenerateGaussianNoise()
{
	// Box-Muller transform implementation matching PX4
	if (!bUseLastGaussian)
	{
		float x1, x2, w;
		
		// Generate two uniform random numbers in the range (-1, 1)
		// and ensure they are within the unit circle
		do
		{
			x1 = 2.0f * FMath::FRand() - 1.0f;
			x2 = 2.0f * FMath::FRand() - 1.0f;
			w = x1 * x1 + x2 * x2;
		} while (w >= 1.0f || w == 0.0f);
		
		// Box-Muller transform
		w = FMath::Sqrt((-2.0f * FMath::Loge(w)) / w);
		
		// Calculate two independent Gaussian values
		float y1 = x1 * w;
		LastGaussianValue = x2 * w;
		
		bUseLastGaussian = true;
		return y1;
	}
	else
	{
		// Use the second value from the previous calculation
		bUseLastGaussian = false;
		return LastGaussianValue;
	}
}
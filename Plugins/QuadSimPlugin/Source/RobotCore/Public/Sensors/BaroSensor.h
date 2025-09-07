// BaroSensor.h
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "BaroSensor.generated.h"

// Forward declarations
class AGeoReferencingSystem;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ROBOTCORE_API UBaroSensor : public USceneComponent
{
	GENERATED_BODY()
    
public:
	UBaroSensor();
    
	void Initialize();
	void UpdateSensor(float DeltaTime, bool bNoise);
	
	// Get sensor readings
	UFUNCTION(BlueprintPure, Category = "Barometer")
	float GetLastPressure() const { return LastPressure; }  // Pascal
	
	UFUNCTION(BlueprintPure, Category = "Barometer")
	float GetLastPressureHPa() const { return LastPressure / 100.0f; }  // Hectopascal
	
	UFUNCTION(BlueprintPure, Category = "Barometer")
	float GetLastTemperature() const { return LastTemperature; }  // Celsius
	
	UFUNCTION(BlueprintPure, Category = "Barometer")
	float GetEstimatedAltitude() const { return EstimatedAltitude; }  // Meters
    
private:
	// Sensor update rate (20Hz from PX4)
	float UpdateRate = 20.0f;
	float AccumulatedTime = 0.0f;
    
	// Last sensor readings
	float LastPressure;      // Pascal
	float LastTemperature;   // Celsius
	float EstimatedAltitude; // Meters
	
	// Drift simulation
	float BaroDriftPa = 0.0f;
	
	float PressureNoiseStdDev = 0.5f;        // Pa (~0.04m altitude)
	float BaroDriftPaPerSec = 0.005f;        // Pa/s drift rate  
	float TemperatureNoiseStdDev = 0.1f;     // °C
	
	// Box-Muller transform state for noise generation
	bool bUseLastGaussian = false;
	float LastGaussianValue = 0.0f;
	
	// Sensor offsets (configurable)
	UPROPERTY(EditAnywhere, Category = "Barometer", meta = (DisplayName = "Pressure Offset (Pa)"))
	float PressureOffset = 0.0f;
	
	UPROPERTY(EditAnywhere, Category = "Barometer", meta = (DisplayName = "Temperature Offset (C)"))
	float TemperatureOffset = 0.0f;
	
	// ISA model constants
	static constexpr float ISA_LAPSE_RATE = 0.0065f;        // K/m
	static constexpr float ISA_TEMPERATURE_MSL = 288.15f;   // K (15°C)
	static constexpr float ISA_PRESSURE_MSL = 101325.0f;    // Pa
	static constexpr float ISA_GAS_CONSTANT = 287.05f;      // J/(kg·K)
	static constexpr float ISA_GRAVITY = 9.80665f;          // m/s²
	
	// Helper functions
	float GenerateGaussianNoise();
	void CalculatePressureFromAltitude(float AltitudeMSL);
	
	// Reference to georeferencing system for altitude
	UPROPERTY()
	AGeoReferencingSystem* GeoRefSystem = nullptr;
};
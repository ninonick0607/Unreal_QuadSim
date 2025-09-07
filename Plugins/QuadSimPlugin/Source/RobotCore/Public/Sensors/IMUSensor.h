// IMUSensor.h
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "IMUSensor.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class ROBOTCORE_API UIMUSensor : public USceneComponent
{
	GENERATED_BODY()

public: 
	UIMUSensor();
    
	float SensorNoise();
	void UpdateSensor(float DeltaTime, bool bNoise);
	FVector SampleRawAcceleration(float DeltaTime);
	FVector SampleRawAngularVelocity();
	FVector SampleRawVelocity();
	FRotator SampleRawAttitude();
	
	FVector GetLastAccelerometer() { return LastAccelerometer; }
	FVector GetLastGyroscope() { return LastGyroscope; }
	FVector GetLastGyroscopeDegrees() { return  FMath::RadiansToDegrees(LastGyroscope); }
	FVector GetLastVelocity(){return LastVelocity;}
	FRotator GetLastAttitude(){return LastAttitude;}
	void Initialize();
	void BeginFrame() { bNeedsFirstAccelSample = true; }

private:
	float UpdateRate = 250.0f;
	float AccumulatedTime = 0.0f;
	FVector LastAccelerometer;
	FVector LastGyroscope;
    FVector LastVelocity;
	FRotator LastAttitude;
	FVector PreviousVelocity;
	bool bInitialized = false;
    
	// Noise parameters
	float VelNoiseStdDev = 0.001f;       // m/s
	float AccelVelNoiseStdDev = 0.0001f; // m/s²
	float AttNoiseStdDev = 0.01f;        // degrees  
	float GyroAttNoiseStdDev = 0.0001f;  // rad/s
	
	bool bNeedsFirstAccelSample = true;

	UPROPERTY()
	UPrimitiveComponent* AttachedBody = nullptr;
};
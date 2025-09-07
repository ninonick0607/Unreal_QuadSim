// QuadPIDController.h
#pragma once

#include "CoreMinimal.h"

class QUADSIMCORE_API QuadPIDController
{
public:
	QuadPIDController();

	void SetGains(float pGain, float iGain, float dGain, float tau = 0.1f);
	void SetLimits(float min_output, float max_output);
	
	void Reset();
	void ResetIntegral();

	double Calculate(float desiredState, float measuredState, float dt);
	
	// Getters for buffer info
	int32 GetBufferSize() const { return integralBuffer.Num(); }
	float GetCurrentBufferSum() const { return currentBufferSum; }
	float lastOutput;
	
	// PID Gains
	float ProportionalGain;
	float IntegralGain;
	float DerivativeGain;

private:
	// Integral window duration in seconds
	static constexpr float INTEGRAL_WINDOW_DURATION = 2.0f;
	static constexpr int32 ESTIMATED_BUFFER_SIZE = 140;
    
	struct IntegralPoint
	{
		float timestamp;  // Time when this point was added
		float value;      // The error * dt value
	};
    
	// Remove expired points from the sliding window
	void RemoveExpiredPoints();
    
	TArray<IntegralPoint> integralBuffer;
	float absoluteTime;       // Track total elapsed time
	float currentBufferSum;   // Running sum of integral values

	// Output limits
	float minOutput;
	float maxOutput;
    
	// Previous error for derivative calculation
	float prevError;
	
	float lastState; 
	float filteredDerivative;
	float tau; 
};
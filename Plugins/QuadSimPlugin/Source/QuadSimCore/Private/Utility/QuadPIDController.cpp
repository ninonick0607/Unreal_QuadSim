// QuadPIDController.cpp
#include "Utility/QuadPIDController.h"
#include "Math/UnrealMathUtility.h"

QuadPIDController::QuadPIDController()
    : lastOutput(0.0f)
    , ProportionalGain(0.0f)
    , IntegralGain(0.0f)
    , DerivativeGain(0.0f)
    , absoluteTime(0.0f)
    , currentBufferSum(0.0f)
    , minOutput(0.0f)
    , maxOutput(1.0f)
    , prevError(0.0f)
    , lastState(0.0f)
    , filteredDerivative(0.0f)
    , tau(0.1f)
{
    // Pre-allocate buffer to avoid reallocations
    integralBuffer.Reserve(ESTIMATED_BUFFER_SIZE);
}

void QuadPIDController::SetGains(float pGain, float iGain, float dGain, float newTau)
{
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
    
    tau = FMath::Max(KINDA_SMALL_NUMBER, newTau);  
}

void QuadPIDController::SetLimits(float min_output, float max_output)
{
    minOutput = min_output;
    maxOutput = max_output;
}

void QuadPIDController::Reset()
{
    integralBuffer.Empty();
    currentBufferSum = 0.0f;
    prevError = 0.0f;
    absoluteTime = 0.0f;
    filteredDerivative = 0.0f;  // Reset the filtered derivative
}

void QuadPIDController::ResetIntegral()
{
    integralBuffer.Empty();
    currentBufferSum = 0.0f;
}

void QuadPIDController::RemoveExpiredPoints()
{
    const float windowStart = absoluteTime - INTEGRAL_WINDOW_DURATION;
    
    // Remove points older than the window duration
    while (!integralBuffer.IsEmpty() && integralBuffer[0].timestamp < windowStart)
    {
        currentBufferSum -= integralBuffer[0].value;
        integralBuffer.RemoveAt(0, 1, EAllowShrinking::No);
    }
}

double QuadPIDController::Calculate(float desiredState,float measuredState, float dt)
{
    if (dt <= KINDA_SMALL_NUMBER)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPIDController::Calculate called with dt <= KINDA_SMALL_NUMBER"));
        return 0.0f;
    }

    absoluteTime += dt;
    RemoveExpiredPoints();

    /* ---------- Error & proportional ---------- */
    float error = desiredState - measuredState;
    double p_term = ProportionalGain * error;

    /* ---------- Integral  ---------- */
    IntegralPoint newPoint{absoluteTime,error*dt};
    integralBuffer.Add(newPoint);
    currentBufferSum += newPoint.value;
    double i_term = IntegralGain * currentBufferSum;
    
    /* ---------- Derivative (bilinear filtered on measurement) ---------- */
    
    float dx = error - lastState;
    filteredDerivative = ((2 * tau - dt) / (2 * tau + dt)) * filteredDerivative + (2 / (2 * tau + dt)) * dx;
    lastState = error;
    double d_term = DerivativeGain * filteredDerivative;
    
    /* ---------- Combine & clamp ---------- */
    double unclamped_output = p_term + i_term + d_term;
    double output = FMath::Clamp(unclamped_output, minOutput, maxOutput);

    if (output != unclamped_output && FMath::Abs(i_term) > FMath::Abs(output - p_term - d_term) && IntegralGain > 0.0f)
    {
        currentBufferSum = (output - p_term - d_term) / IntegralGain;
    }
    
    UE_LOG(LogTemp, VeryVerbose, TEXT("Time: %.3f | BufferSize: %d | Sum: %.4f | dRaw: %.4f | dFiltered: %.4f"),
           absoluteTime, integralBuffer.Num(), currentBufferSum, dx / dt, filteredDerivative);
    
    prevError   = error;
    lastOutput = output;
    return output;
}


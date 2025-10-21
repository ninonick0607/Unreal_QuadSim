#pragma once

#include "CoreMinimal.h"

/**
 * Units are SI (N, m, kg, m/s).
 */
struct FRotorModel
{
    // --- Tunables (defaults match AirSim's UIUC GWS 9x5 example) ---
    float C_T = 0.109919f;        // thrust coefficient
    float C_P = 0.040164f;        // torque coefficient
    float AirDensity = 1.225f;    // kg/m^3
    float MaxRPM = 6396.667f;     // rpm at full command
    float PropDiameter = 0.2286f; // m
    float ControlSignalFilterTC = 0.005f; // (not used here; for ESC filtering if you add it)

    // --- Derived ---
    float RevPerSec = 0.f;
    float MaxOmega = 0.f;         // rad/s
    float MaxOmegaSq = 0.f;
    float MaxThrust = 4.179446268f; // N per motor @ control 1.0
    float MaxTorque = 0.055562f;    // N·m per motor @ control 1.0

    // Call after changing any tunables above.
    void Recalculate()
    {
        RevPerSec   = MaxRPM / 60.f;
        MaxOmega    = RevPerSec * 2.f * PI;
        MaxOmegaSq  = MaxOmega * MaxOmega;

        const float n2 = RevPerSec * RevPerSec;
        MaxThrust = C_T * AirDensity * n2 * FMath::Pow(PropDiameter, 4);
        
        MaxTorque = C_P * AirDensity * n2 * FMath::Pow(PropDiameter, 5) / (2.f * PI);

    }

    // Convenience: hover throttle 0..1 for a given vehicle mass (kg)
    float ComputeHoverThrottle01(float VehicleMassKg, int32 MotorCount = 4) const
    {
        if (MotorCount <= 0 || MaxThrust <= 0.f) return 0.5f;
        const float perMotorHoverN = (VehicleMassKg * 9.81f) / float(MotorCount);
        UE_LOG(LogTemp, Warning, TEXT("Per Hover: %f N"), perMotorHoverN);

        float hoverThrust = FMath::Clamp(perMotorHoverN / float(perMotorHoverN),0.f,1.f);
        UE_LOG(LogTemp, Warning, TEXT("Max Thrust: %f N"), hoverThrust);
        return hoverThrust;
    }

    // Map a normalized control signal (0..1) to thrust (N) and yaw torque (N·m).
    // turningDir: +1 = CCW, -1 = CW (choose per motor).
    FORCEINLINE float ControlToThrustN(float Control01, float AirDensityRatio = 1.f) const
    {
        return FMath::Clamp(Control01, 0.f, 1.f) * MaxThrust * AirDensityRatio;
    }
    FORCEINLINE float ControlToYawTorqueNm(float Control01, int32 TurningDir, float AirDensityRatio = 1.f) const
    {
        return FMath::Clamp(Control01, 0.f, 1.f) * MaxTorque * float(TurningDir) * AirDensityRatio;
    }
};

// Optional helper for your project’s “which way does this motor spin” convention.
enum class ERotorTurningDirection : int32
{
    CCW = +1,
    CW  = -1
};

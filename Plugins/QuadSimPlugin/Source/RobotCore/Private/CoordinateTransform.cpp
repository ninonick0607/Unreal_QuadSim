// CoordinateTransform.cpp
#include "CoordinateTransform.h"
#include "GeoReferencingSystem.h"
#include "GeographicCoordinates.h"
#include "EngineUtils.h"
#include "Engine/World.h"

// Unreal: X=Forward, Y=Right, Z=Up (FRU)
// NED:    X=North,  Y=East,  Z=Down
// ENU:    X=East,   Y=North, Z=Up
// NOTE: All transforms below are AXIS-ONLY (no unit scaling).

const FMatrix UCoordinateTransform::UnrealToNEDMat = FMatrix(
	FPlane(1.f,   0.f,   0.f, 0.f),  // X_NED =  +X_U
	FPlane(0.f,  -1.f,   0.f, 0.f),  // Y_NED =  -Y_U
	FPlane(0.f,   0.f,  -1.f, 0.f),  // Z_NED =  -Z_U
	FPlane(0.f,   0.f,   0.f, 1.f)
);

const FMatrix UCoordinateTransform::UnrealToENUMat = FMatrix(
	FPlane(0.f,  -1.f,   0.f, 0.f),  // X_ENU =  -Y_U
	FPlane(1.f,   0.f,   0.f, 0.f),  // Y_ENU =  +X_U
	FPlane(0.f,   0.f,   1.f, 0.f),  // Z_ENU =  +Z_U
	FPlane(0.f,   0.f,   0.f, 1.f)
);

const FMatrix UCoordinateTransform::NEDToUnrealMat = FMatrix(
	FPlane(1.f,   0.f,   0.f, 0.f),  // X_U =  +X_NED
	FPlane(0.f,  -1.f,   0.f, 0.f),  // Y_U =  -Y_NED
	FPlane(0.f,   0.f,  -1.f, 0.f),  // Z_U =  -Z_NED
	FPlane(0.f,   0.f,   0.f, 1.f)
);

const FMatrix UCoordinateTransform::ENUToUnrealMat = FMatrix(
	FPlane(0.f,   1.f,   0.f, 0.f),  // X_U =  +Y_ENU
	FPlane(-1.f,  0.f,   0.f, 0.f),  // Y_U =  -X_ENU
	FPlane(0.f,   0.f,   1.f, 0.f),  // Z_U =  +Z_ENU
	FPlane(0.f,   0.f,   0.f, 1.f)
);

UCoordinateTransform::UCoordinateTransform()
{
    // Nothing to initialize for static class
}

// ==================== POSITION / VECTOR (AXIS-ONLY) ====================

FVector UCoordinateTransform::UnrealToNED(const FVector& vU)
{
	return FVector(+vU.X, -vU.Y, -vU.Z);
}

FVector UCoordinateTransform::NEDToUnreal(const FVector& vNED)
{
	return FVector(+vNED.X, -vNED.Y, -vNED.Z);
}

FVector UCoordinateTransform::UnrealToENU(const FVector& vU)
{
	return FVector(-vU.Y, +vU.X, +vU.Z);
}

FVector UCoordinateTransform::ENUToUnreal(const FVector& vENU)
{
	return FVector(+vENU.Y, -vENU.X, +vENU.Z);
}

// ==================== VELOCITY (AXIS-ONLY) ====================

FVector UCoordinateTransform::UnrealVelocityToNED(const FVector& velU)
{
    return UnrealToNED(velU);
}

FVector UCoordinateTransform::UnrealVelocityToENU(const FVector& velU)
{
    return UnrealToENU(velU);
}

// ==================== ROTATIONS (AXIS-ONLY) ====================

FRotator UCoordinateTransform::UnrealRotationToNED(const FRotator& eulU_deg)
{
    // FRU -> NED Euler (deg): roll same, pitch & yaw flip signs
    return FRotator(-eulU_deg.Pitch, -eulU_deg.Yaw, eulU_deg.Roll);
}

FRotator UCoordinateTransform::UnrealRotationToENU(const FRotator& eulU_deg)
{
    // One consistent option: FRU -> ENU via a +90° yaw offset in U-space and axis swap.
    // For most use cases in this project, prefer UnrealRotationToNED. Keep this as-is if you use ENU.
    return FRotator(eulU_deg.Pitch, eulU_deg.Yaw + 90.f, eulU_deg.Roll);
}

FQuat UCoordinateTransform::UnrealQuaternionToNED(const FQuat& qU)
{
    // FRU -> NED mapping as a coefficient flip (equivalent to 180° about +X)
    FQuat q(qU.W, qU.X, -qU.Y, -qU.Z);
    q.Normalize();
    return q;
}

FQuat UCoordinateTransform::UnrealQuaternionToENU(const FQuat& qU)
{
    // Simple, robust way: convert to rotator, map with UnrealRotationToENU, back to quat
    return FQuat(UnrealRotationToENU(qU.Rotator()));
}

// ==================== ANGULAR VELOCITY (AXIS-ONLY; DEG->RAD ONLY IF YOU WANT) ====================

FVector UCoordinateTransform::UnrealAngularVelocityToNED(const FVector& wU_degps)
{
    // If you want axis-only with no unit change, remove conversion to radians.
    // Keeping your original behavior (deg/s -> rad/s) because it’s commonly needed:
    return FVector(
        FMath::DegreesToRadians(wU_degps.X),   // roll
        -FMath::DegreesToRadians(wU_degps.Y),  // pitch (flip)
        -FMath::DegreesToRadians(wU_degps.Z)   // yaw   (flip)
    );
}

FVector UCoordinateTransform::UnrealAngularVelocityToENU(const FVector& wU_degps)
{
    return FVector(
        FMath::DegreesToRadians(wU_degps.X),
        FMath::DegreesToRadians(wU_degps.Y),
        FMath::DegreesToRadians(wU_degps.Z)
    );
}

// ==================== FRAME TRANSFORMS ====================

FVector UCoordinateTransform::WorldToBodyFrame(const FVector& WorldVector, const FRotator& BodyRotation)
{
    // Transform from world frame to body frame
    return BodyRotation.UnrotateVector(WorldVector);
}

FVector UCoordinateTransform::WorldToBodyFrameQuat(const FVector& WorldVector, const FQuat& BodyQuat)
{
    // Transform from world frame to body frame using quaternion
    return BodyQuat.UnrotateVector(WorldVector);
}

FVector UCoordinateTransform::BodyToWorldFrame(const FVector& BodyVector, const FRotator& BodyRotation)
{
    // Transform from body frame to world frame
    return BodyRotation.RotateVector(BodyVector);
}

// ==================== BATCH OPERATIONS ====================

void UCoordinateTransform::BatchUnrealToNED(const TArray<FVector>& UnrealPositions, TArray<FVector>& OutNEDPositions)
{
    OutNEDPositions.Reset(UnrealPositions.Num());
    
    // Use matrix multiplication for efficiency
    for (const FVector& Pos : UnrealPositions)
    {
        OutNEDPositions.Add(UnrealToNED(Pos));
    }
}

void UCoordinateTransform::BatchUnrealToENU(const TArray<FVector>& UnrealPositions, TArray<FVector>& OutENUPositions)
{
    OutENUPositions.Reset(UnrealPositions.Num());
    
    for (const FVector& Pos : UnrealPositions)
    {
        OutENUPositions.Add(UnrealToENU(Pos));
    }
}

// ==================== GEOGRAPHIC CONVERSIONS ====================

FVector UCoordinateTransform::GeographicToUnreal(float Latitude, float Longitude, float Altitude, 
                                                  UObject* WorldContextObject, AGeoReferencingSystem* GeoRef)
{
    // Find GeoReferencingSystem if not provided
    if (!GeoRef && WorldContextObject)
    {
        UWorld* World = WorldContextObject->GetWorld();
        if (World)
        {
            for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It)
            {
                GeoRef = *It;
                break;
            }
        }
    }
    
    if (!GeoRef)
    {
        UE_LOG(LogTemp, Warning, TEXT("CoordinateTransform: No GeoReferencingSystem found!"));
        return FVector::ZeroVector;
    }
    
    // Create geographic coordinates
    FGeographicCoordinates GeoCoords;
    GeoCoords.Latitude = Latitude;
    GeoCoords.Longitude = Longitude;
    GeoCoords.Altitude = Altitude;
    
    // Convert to Unreal coordinates
    FVector UnrealPos;
    GeoRef->GeographicToEngine(GeoCoords, UnrealPos);
    
    return UnrealPos;
}

void UCoordinateTransform::UnrealToGeographic(const FVector& UnrealPos, float& OutLatitude, float& OutLongitude, 
                                               float& OutAltitude, UObject* WorldContextObject, AGeoReferencingSystem* GeoRef)
{
    // Find GeoReferencingSystem if not provided
    if (!GeoRef && WorldContextObject)
    {
        UWorld* World = WorldContextObject->GetWorld();
        if (World)
        {
            for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It)
            {
                GeoRef = *It;
                break;
            }
        }
    }
    
    if (!GeoRef)
    {
        UE_LOG(LogTemp, Warning, TEXT("CoordinateTransform: No GeoReferencingSystem found!"));
        OutLatitude = 0.0f;
        OutLongitude = 0.0f;
        OutAltitude = 0.0f;
        return;
    }
    
    // Convert to geographic coordinates
    FGeographicCoordinates GeoCoords;
    GeoRef->EngineToGeographic(UnrealPos, GeoCoords);
    
    OutLatitude = GeoCoords.Latitude;
    OutLongitude = GeoCoords.Longitude;
    OutAltitude = GeoCoords.Altitude;
}

// ==================== UTILITY FUNCTIONS ====================

FMatrix UCoordinateTransform::GetUnrealToNEDMatrix()
{
    return UnrealToNEDMat;
}

FMatrix UCoordinateTransform::GetUnrealToENUMatrix()
{
    return UnrealToENUMat;
}

FQuat UCoordinateTransform::NormalizeQuaternion(const FQuat& Quat)
{
    FQuat NormalizedQuat = Quat;
    NormalizedQuat.Normalize();
    return NormalizedQuat;
}

FRotator UCoordinateTransform::ValidateEulerAngles(const FRotator& Euler)
{
    // Clamp angles to valid ranges
    FRotator ValidatedEuler;
    ValidatedEuler.Pitch = FMath::ClampAngle(Euler.Pitch, -90.0f, 90.0f);
    ValidatedEuler.Yaw = FMath::ClampAngle(Euler.Yaw, -180.0f, 180.0f);
    ValidatedEuler.Roll = FMath::ClampAngle(Euler.Roll, -180.0f, 180.0f);
    return ValidatedEuler;
}
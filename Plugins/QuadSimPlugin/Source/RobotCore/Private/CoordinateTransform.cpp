// CoordinateTransform.cpp
#include "CoordinateTransform.h"
#include "GeoReferencingSystem.h"
#include "GeographicCoordinates.h"
#include "EngineUtils.h"
#include "Engine/World.h"

// Define conversion matrices
// Unreal: X=Forward, Y=Right, Z=Up (FRU in cm)
// NED: X=North, Y=East, Z=Down (in meters)
// ENU: X=East, Y=North, Z=Up (in meters)

const FMatrix UCoordinateTransform::UnrealToNEDMat = FMatrix(
    FPlane(0.01f,  0.0f,   0.0f, 0.0f),  // X_NED = X_Unreal / 100
    FPlane(0.0f,  -0.01f,  0.0f, 0.0f),  // Y_NED = -Y_Unreal / 100 (left-handed -> right-handed)
    FPlane(0.0f,   0.0f,  -0.01f, 0.0f), // Z_NED = -Z_Unreal / 100
    FPlane(0.0f,   0.0f,   0.0f, 1.0f)
);

const FMatrix UCoordinateTransform::UnrealToENUMat = FMatrix(
    FPlane(0.0f,  -0.01f,  0.0f, 0.0f),  // X_ENU = -Y_Unreal / 100
    FPlane(0.01f,  0.0f,   0.0f, 0.0f),  // Y_ENU = X_Unreal / 100
    FPlane(0.0f,   0.0f,   0.01f, 0.0f), // Z_ENU = Z_Unreal / 100
    FPlane(0.0f,   0.0f,   0.0f, 1.0f)
);

const FMatrix UCoordinateTransform::NEDToUnrealMat = FMatrix(
    FPlane(100.0f,  0.0f,    0.0f, 0.0f),   // X_Unreal = X_NED * 100
    FPlane(0.0f,   -100.0f,  0.0f, 0.0f),   // Y_Unreal = -Y_NED * 100
    FPlane(0.0f,    0.0f,   -100.0f, 0.0f), // Z_Unreal = -Z_NED * 100
    FPlane(0.0f,    0.0f,    0.0f, 1.0f)
);

const FMatrix UCoordinateTransform::ENUToUnrealMat = FMatrix(
    FPlane(0.0f,   100.0f,  0.0f, 0.0f),   // X_Unreal = Y_ENU * 100
    FPlane(-100.0f, 0.0f,   0.0f, 0.0f),   // Y_Unreal = -X_ENU * 100
    FPlane(0.0f,    0.0f,   100.0f, 0.0f), // Z_Unreal = Z_ENU * 100
    FPlane(0.0f,    0.0f,   0.0f, 1.0f)
);

UCoordinateTransform::UCoordinateTransform()
{
    // Nothing to initialize for static class
}

// ==================== POSITION TRANSFORMS ====================

FVector UCoordinateTransform::UnrealToNED(const FVector& UnrealPos)
{
    // Unreal (cm): X=Forward, Y=Right, Z=Up
    // NED (m): X=North, Y=East, Z=Down
    return FVector(
        UnrealPos.X * 0.01f,   // Forward -> North (cm to m)
        -UnrealPos.Y * 0.01f,  // Right -> East (flip and cm to m)
        -UnrealPos.Z * 0.01f   // Up -> Down (flip and cm to m)
    );
}

FVector UCoordinateTransform::NEDToUnreal(const FVector& NEDPos)
{
    // NED (m): X=North, Y=East, Z=Down
    // Unreal (cm): X=Forward, Y=Right, Z=Up
    return FVector(
        NEDPos.X * 100.0f,    // North -> Forward (m to cm)
        -NEDPos.Y * 100.0f,   // East -> Right (flip and m to cm)
        -NEDPos.Z * 100.0f    // Down -> Up (flip and m to cm)
    );
}

FVector UCoordinateTransform::UnrealMetersToNED(const FVector& UnrealPosMeters)
{
    // Unreal (m): X=Forward, Y=Right, Z=Up (FRU, left-handed)
    // NED (m):    X=North,  Y=East,  Z=Down (right-handed)
    // When inputs are already meters, do not scale, only flip Y and Z for handedness
    return FVector(
        UnrealPosMeters.X,   // Forward -> North (m)
        -UnrealPosMeters.Y,  // Right -> East (flip for handedness)
        -UnrealPosMeters.Z   // Up -> Down (flip)
    );
}

FVector UCoordinateTransform::UnrealToENU(const FVector& UnrealPos)
{
    // Unreal (cm): X=Forward, Y=Right, Z=Up
    // ENU (m): X=East, Y=North, Z=Up
    return FVector(
        -UnrealPos.Y * 0.01f,  // Right -> East (flip and cm to m)
        UnrealPos.X * 0.01f,   // Forward -> North (cm to m)
        UnrealPos.Z * 0.01f    // Up -> Up (cm to m)
    );
}

FVector UCoordinateTransform::ENUToUnreal(const FVector& ENUPos)
{
    // ENU (m): X=East, Y=North, Z=Up
    // Unreal (cm): X=Forward, Y=Right, Z=Up
    return FVector(
        ENUPos.Y * 100.0f,    // North -> Forward (m to cm)
        -ENUPos.X * 100.0f,   // East -> Right (flip and m to cm)
        ENUPos.Z * 100.0f     // Up -> Up (m to cm)
    );
}

// ==================== VELOCITY TRANSFORMS ====================

FVector UCoordinateTransform::UnrealVelocityToNED(const FVector& UnrealVel)
{
    // Same transformation as position but for velocity (cm/s to m/s)
    // Already handles unit conversion within UnrealToNED
    return UnrealToNED(UnrealVel);
}

FVector UCoordinateTransform::UnrealVelocityToENU(const FVector& UnrealVel)
{
    // Same transformation as position but for velocity (cm/s to m/s)
    return UnrealToENU(UnrealVel);
}

FVector UCoordinateTransform::UnrealMetersVelocityToNED(const FVector& UnrealVelMS)
{
    // Same mapping as position, but velocity already in m/s
    return FVector(
        UnrealVelMS.X,     // Forward -> North
        -UnrealVelMS.Y,    // Right -> East (flip)
        -UnrealVelMS.Z     // Up -> Down (flip)
    );
}

// ==================== ROTATION TRANSFORMS ====================

FRotator UCoordinateTransform::UnrealRotationToNED(const FRotator& UnrealRot)
{
    // Unreal coordinate system: X=Forward, Y=Right, Z=Up
    // NED coordinate system: X=North, Y=East, Z=Down
    //
    // For body frame rotations:
    // - Roll: rotation about X axis (forward) - same direction
    // - Pitch: rotation about Y axis (right) - flip for NED
    // - Yaw: rotation about Z axis - flip Z direction (up->down)

    // CRITICAL FIX: Ensure angles are normalized to prevent attitude failures
    FRotator NormalizedRot = UnrealRot;
    NormalizedRot.Normalize();

    return FRotator(
        -NormalizedRot.Pitch,  // Pitch: flip for coordinate system change
        -NormalizedRot.Yaw,    // Yaw: flip for Z-axis direction change (Up->Down)
        NormalizedRot.Roll     // Roll: same direction for X-axis
    );
}

FRotator UCoordinateTransform::UnrealRotationToENU(const FRotator& UnrealRot)
{
    // Convert to ENU frame
    // ENU typically uses different conventions
    
    return FRotator(
        UnrealRot.Pitch,           // Pitch
        UnrealRot.Yaw + 90.0f,     // Yaw offset by 90 degrees
        UnrealRot.Roll             // Roll
    );
}

FQuat UCoordinateTransform::UnrealQuaternionToNED(const FQuat& UnrealQuat)
{
    // For quaternions, we need to consider the coordinate system change
    // This matches your PX4Component conversion
    return FQuat(
        UnrealQuat.W,   // W stays same
        UnrealQuat.X,   // X stays same
        -UnrealQuat.Y,  // Y flips
        -UnrealQuat.Z   // Z flips
    );
}

FQuat UCoordinateTransform::UnrealQuaternionToENU(const FQuat& UnrealQuat)
{
    // Convert quaternion to ENU frame
    // First convert to rotation, transform, then back to quaternion
    FRotator UnrealRot = UnrealQuat.Rotator();
    FRotator ENURot = UnrealRotationToENU(UnrealRot);
    return FQuat(ENURot);
}

// ==================== ANGULAR VELOCITY TRANSFORMS ====================

FVector UCoordinateTransform::UnrealAngularVelocityToNED(const FVector& UnrealAngVel)
{
    // Convert from deg/s to rad/s and apply coordinate transform
    // For body frame angular velocity, we need to consider the coordinate system change:
    // Unreal body: X=Forward(Roll), Y=Right(Pitch), Z=Up(Yaw)
    // NED body: X=Forward(Roll), Y=Right(Pitch), Z=Down(Yaw)
    // The angular velocity is about the body axes, so transformation is:

    return FVector(
        FMath::DegreesToRadians(UnrealAngVel.X),      // Roll about X (forward) axis
        FMath::DegreesToRadians(-UnrealAngVel.Y),     // Pitch about Y (right) axis - flip for NED
        FMath::DegreesToRadians(-UnrealAngVel.Z)      // Yaw about Z axis - flip Z direction Unreal(up) to NED(down)
    );
}

FVector UCoordinateTransform::UnrealAngularVelocityToENU(const FVector& UnrealAngVel)
{
    // Convert from deg/s to rad/s and apply coordinate transform
    return FVector(
        FMath::DegreesToRadians(UnrealAngVel.X),      // Roll
        FMath::DegreesToRadians(UnrealAngVel.Y),      // Pitch
        FMath::DegreesToRadians(UnrealAngVel.Z)       // Yaw
    );
}

// ==================== FRAME TRANSFORMS ====================

FVector UCoordinateTransform::UnrealBodyAccelToFRD(const FVector& UnrealBodyAccel)
{
    // Transform from Unreal body frame (FLU) to PX4 body frame (FRD)
    // Unreal FRU: X=Forward, Y=Right, Z=Up (left-handed)
    // PX4 FRD:    X=Forward, Y=Right, Z=Down (right-handed)
    //
    // Key insight: When drone is level, gravity appears as:
    // - Unreal FLU: (0, 0, +9.81) - positive Z is up, gravity pulls down but IMU measures up
    // - PX4 FRD:    (0, 0, -9.81) - negative Z is down, gravity should be negative Z
    //
    // Transformation:
    // To convert from left-handed FRU to right-handed FRD, flip Y and Z
    // X_FRD =  X_FRU  (Forward stays forward)
    // Y_FRD = -Y_FRU  (flip)
    // Z_FRD = -Z_FRU  (flip)

    return FVector(
        UnrealBodyAccel.X,   // Forward stays forward
        -UnrealBodyAccel.Y,  // Left -> Right (flip)
        -UnrealBodyAccel.Z   // Up -> Down (flip) - This fixes the gravity sign!
    );
}

FVector UCoordinateTransform::UnrealBodyAngVelToFRD(const FVector& UnrealBodyAngVel)
{
    // Transform angular velocity from Unreal body frame (FLU) to PX4 body frame (FRD)
    // Unreal FRU: X=Forward(Roll), Y=Right(Pitch), Z=Up(Yaw)
    // PX4 FRD:    X=Forward(Roll), Y=Right(Pitch), Z=Down(Yaw)
    //
    // For angular velocities about body axes:
    // - Roll about X axis (forward): same direction
    // - Pitch about Y axis: flip because Left becomes Right
    // - Yaw about Z axis: flip because Up becomes Down
    //
    // Also convert from degrees/s to radians/s

    return FVector(
        FMath::DegreesToRadians(UnrealBodyAngVel.X),   // Roll about X (forward) - same
        FMath::DegreesToRadians(-UnrealBodyAngVel.Y),  // Pitch about Y - flip (handedness)
        FMath::DegreesToRadians(-UnrealBodyAngVel.Z)   // Yaw about Z - flip (Up->Down)
    );
}

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

FVector UCoordinateTransform::UnrealBodyToFRD(const FVector& UnrealBodyVec)
{
    // Generic body-frame mapping from Unreal FRU (left-handed) to FRD (right-handed)
    return FVector(
        UnrealBodyVec.X,
        -UnrealBodyVec.Y,
        -UnrealBodyVec.Z
    );
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

// CoordinateTransform.h
#pragma once

#include "CoreMinimal.h"
#include "CoordinateTransform.generated.h"

/**
 * Unified coordinate transformation class for QuadSim
 * Handles all coordinate system conversions, unit conversions, and frame transformations
 * 
 * Coordinate Systems:
 * - Unreal: Forward-Left-Up (FLU), centimeters
 * - NED: North-East-Down, meters (aerospace/PX4 standard)
 * - ENU: East-North-Up, meters (ROS standard)
 * - Body: Vehicle-fixed frame
 * - World: Global frame
 */
UCLASS(BlueprintType)
class ROBOTCORE_API UCoordinateTransform : public UObject
{
    GENERATED_BODY()

public:
    UCoordinateTransform();

    // ==================== POSITION TRANSFORMS ====================
    
    /**
     * Convert position from Unreal (FLU, cm) to NED (m)
     * @param UnrealPos Position in Unreal coordinates (cm)
     * @return Position in NED coordinates (m)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealToNED(const FVector& UnrealPos);
    
    /**
     * Convert position from NED (m) to Unreal (FLU, cm)
     * @param NEDPos Position in NED coordinates (m)
     * @return Position in Unreal coordinates (cm)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector NEDToUnreal(const FVector& NEDPos);
    
    /**
     * Convert position from Unreal (FLU, cm) to ENU (m)
     * @param UnrealPos Position in Unreal coordinates (cm)
     * @return Position in ENU coordinates (m)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealToENU(const FVector& UnrealPos);
    
    /**
     * Convert position from ENU (m) to Unreal (FLU, cm)
     * @param ENUPos Position in ENU coordinates (m)
     * @return Position in Unreal coordinates (cm)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector ENUToUnreal(const FVector& ENUPos);

    // ==================== VELOCITY TRANSFORMS ====================
    
    /**
     * Convert velocity from Unreal (FLU, cm/s) to NED (m/s)
     * @param UnrealVel Velocity in Unreal coordinates (cm/s)
     * @return Velocity in NED coordinates (m/s)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealVelocityToNED(const FVector& UnrealVel);
    
    /**
     * Convert velocity from Unreal (FLU, cm/s) to ENU (m/s)
     * @param UnrealVel Velocity in Unreal coordinates (cm/s)
     * @return Velocity in ENU coordinates (m/s)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealVelocityToENU(const FVector& UnrealVel);

    // ==================== ROTATION TRANSFORMS ====================
    
    /**
     * Convert rotation from Unreal to NED frame
     * @param UnrealRot Rotation in Unreal frame
     * @return Rotation in NED frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FRotator UnrealRotationToNED(const FRotator& UnrealRot);
    
    /**
     * Convert rotation from Unreal to ENU frame
     * @param UnrealRot Rotation in Unreal frame
     * @return Rotation in ENU frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FRotator UnrealRotationToENU(const FRotator& UnrealRot);
    
    /**
     * Convert quaternion from Unreal to NED frame
     * @param UnrealQuat Quaternion in Unreal frame
     * @return Quaternion in NED frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FQuat UnrealQuaternionToNED(const FQuat& UnrealQuat);
    
    /**
     * Convert quaternion from Unreal to ENU frame
     * @param UnrealQuat Quaternion in Unreal frame
     * @return Quaternion in ENU frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FQuat UnrealQuaternionToENU(const FQuat& UnrealQuat);

    // ==================== ANGULAR VELOCITY TRANSFORMS ====================
    
    /**
     * Convert angular velocity from Unreal (deg/s) to NED (rad/s)
     * @param UnrealAngVel Angular velocity in Unreal frame (deg/s)
     * @return Angular velocity in NED frame (rad/s)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealAngularVelocityToNED(const FVector& UnrealAngVel);
    
    /**
     * Convert angular velocity from Unreal (deg/s) to ENU (rad/s)
     * @param UnrealAngVel Angular velocity in Unreal frame (deg/s)
     * @return Angular velocity in ENU frame (rad/s)
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector UnrealAngularVelocityToENU(const FVector& UnrealAngVel);

    // ==================== FRAME TRANSFORMS ====================
    
    /**
     * Transform vector from world frame to body frame
     * @param WorldVector Vector in world frame
     * @param BodyRotation Current body rotation in world frame
     * @return Vector in body frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector WorldToBodyFrame(const FVector& WorldVector, const FRotator& BodyRotation);
    
    /**
     * Transform vector from world frame to body frame using quaternion
     * @param WorldVector Vector in world frame
     * @param BodyQuat Current body quaternion in world frame
     * @return Vector in body frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector WorldToBodyFrameQuat(const FVector& WorldVector, const FQuat& BodyQuat);
    
    /**
     * Transform vector from body frame to world frame
     * @param BodyVector Vector in body frame
     * @param BodyRotation Current body rotation in world frame
     * @return Vector in world frame
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FVector BodyToWorldFrame(const FVector& BodyVector, const FRotator& BodyRotation);

    // ==================== UNIT CONVERSIONS ====================
    
    /**
     * Convert centimeters to meters
     * @param Centimeters Value in centimeters
     * @return Value in meters
     */
    UFUNCTION(BlueprintPure, Category = "Unit Conversion")
    static FORCEINLINE float CentimetersToMeters(float Centimeters) { return Centimeters * 0.01f; }
    
    /**
     * Convert meters to centimeters
     * @param Meters Value in meters
     * @return Value in centimeters
     */
    UFUNCTION(BlueprintPure, Category = "Unit Conversion")
    static FORCEINLINE float MetersToCentimeters(float Meters) { return Meters * 100.0f; }
    
    /**
     * Convert degrees to radians
     * @param Degrees Angle in degrees
     * @return Angle in radians
     */
    UFUNCTION(BlueprintPure, Category = "Unit Conversion")
    static FORCEINLINE float DegreesToRadians(float Degrees) { return FMath::DegreesToRadians(Degrees); }
    
    /**
     * Convert radians to degrees
     * @param Radians Angle in radians
     * @return Angle in degrees
     */
    UFUNCTION(BlueprintPure, Category = "Unit Conversion")
    static FORCEINLINE float RadiansToDegrees(float Radians) { return FMath::RadiansToDegrees(Radians); }

    // ==================== BATCH OPERATIONS ====================
    
    /**
     * Convert array of positions from Unreal to NED
     * @param UnrealPositions Array of positions in Unreal coordinates
     * @param OutNEDPositions Output array of positions in NED coordinates
     */
    static void BatchUnrealToNED(const TArray<FVector>& UnrealPositions, TArray<FVector>& OutNEDPositions);
    
    /**
     * Convert array of positions from Unreal to ENU
     * @param UnrealPositions Array of positions in Unreal coordinates
     * @param OutENUPositions Output array of positions in ENU coordinates
     */
    static void BatchUnrealToENU(const TArray<FVector>& UnrealPositions, TArray<FVector>& OutENUPositions);

    // ==================== GEOGRAPHIC CONVERSIONS ====================
    
    /**
     * Convert geographic coordinates to local Unreal coordinates
     * @param Latitude Latitude in degrees
     * @param Longitude Longitude in degrees
     * @param Altitude Altitude in meters
     * @param GeoRef GeoReferencing system (optional, will find in world if null)
     * @return Position in Unreal coordinates (cm)
     */
    UFUNCTION(BlueprintPure, Category = "Geographic Transform", meta=(WorldContext="WorldContextObject"))
    static FVector GeographicToUnreal(float Latitude, float Longitude, float Altitude, 
                                      UObject* WorldContextObject, class AGeoReferencingSystem* GeoRef = nullptr);
    
    /**
     * Convert Unreal coordinates to geographic coordinates
     * @param UnrealPos Position in Unreal coordinates (cm)
     * @param OutLatitude Output latitude in degrees
     * @param OutLongitude Output longitude in degrees
     * @param OutAltitude Output altitude in meters
     * @param GeoRef GeoReferencing system (optional, will find in world if null)
     */
    UFUNCTION(BlueprintPure, Category = "Geographic Transform", meta=(WorldContext="WorldContextObject"))
    static void UnrealToGeographic(const FVector& UnrealPos, float& OutLatitude, float& OutLongitude, float& OutAltitude,
                                   UObject* WorldContextObject, class AGeoReferencingSystem* GeoRef = nullptr);

    // ==================== UTILITY FUNCTIONS ====================
    
    /**
     * Get a combined transformation matrix for Unreal to NED
     * Useful for batch transformations
     */
    static FMatrix GetUnrealToNEDMatrix();
    
    /**
     * Get a combined transformation matrix for Unreal to ENU
     * Useful for batch transformations
     */
    static FMatrix GetUnrealToENUMatrix();
    
    /**
     * Normalize a quaternion (ensures unit length)
     * @param Quat Quaternion to normalize
     * @return Normalized quaternion
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FQuat NormalizeQuaternion(const FQuat& Quat);
    
    /**
     * Validate Euler angles are within valid ranges
     * @param Euler Euler angles to validate
     * @return Clamped Euler angles
     */
    UFUNCTION(BlueprintPure, Category = "Coordinate Transform")
    static FRotator ValidateEulerAngles(const FRotator& Euler);

private:
    // Conversion matrices (cached for performance)
    static const FMatrix UnrealToNEDMat;
    static const FMatrix UnrealToENUMat;
    static const FMatrix NEDToUnrealMat;
    static const FMatrix ENUToUnrealMat;
};
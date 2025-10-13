#pragma once

#include "CoreMinimal.h"
#include "SensorData.h"
#include "Utility/QuadPIDController.h"


class AQuadPawn;
class AGeoReferencingSystem;
#include "QuadDroneController.generated.h"

UENUM(BlueprintType)
enum class EFlightMode : uint8
{
    None UMETA(DisplayName = "None"),
    AutoWaypoint UMETA(DisplayName = "AutoWaypoint"),
    VelocityControl UMETA(DisplayName = "VelocityControl"),
    AngleControl UMETA(DisplayName = "AngleControl"),
    RateControl UMETA(DisplayName = "RateControl"),
    JoyStickAngleControl UMETA(DisplayName = "JoyStickAngleControl"),
    JoyStickAcroControl UMETA(DisplayName = "JoyStickAcroControl")
};

struct FGamepadInputs;

USTRUCT()
struct FFullPIDSet
{
    GENERATED_BODY()

    QuadPIDController* XPID;
    QuadPIDController* YPID;
    QuadPIDController* ZPID;
    QuadPIDController* RollPID;
    QuadPIDController* PitchPID;
    QuadPIDController* PitchRatePID;
    QuadPIDController* RollRatePID;
    QuadPIDController* YawRatePID;
    FFullPIDSet()
        : XPID(nullptr)
        , YPID(nullptr)
        , ZPID(nullptr)
        , RollPID(nullptr)
        , PitchPID(nullptr)
        , PitchRatePID(nullptr)
        , RollRatePID(nullptr)
        , YawRatePID(nullptr)
    {
    }
};


UCLASS(Blueprintable, BlueprintType)
class QUADSIMCORE_API UQuadDroneController : public UObject
{
    GENERATED_BODY()

public:

    UPROPERTY()
    AQuadPawn* dronePawn;
    UPROPERTY()
    TArray<float> Thrusts;
    
    //Constructor
    UQuadDroneController(const FObjectInitializer& ObjectInitializer);
    
    //Init
    void Initialize(AQuadPawn* InPawn);
    void Update(const FSensorData& SensorData, float DeltaTime);
    
    //Update
    void GamepadController(const FSensorData& SensorData,double DeltaTime);
    void FlightController(const FSensorData& SensorData, double DeltaTime);
    void ThrustMixer(double currentRoll, double currentPitch, double zOutput, double rollOutput, double pitchOutput, double yawOutput);
    float YawRateControl(double DeltaTime);

    //Reset
    void ResetPID();
    void ResetDroneIntegral();
    // Reset full controller state for a fresh Play session
    void ResetControllerState();

    //Hover and Nav
    void SetHoverMode(bool bActive, float TargetAltitude = 250.0f);
    void SetDestination(FVector desiredSetPoints);

    //Helpers
    void DrawDebugVisuals(const FVector& currentPosition)const;
    void DrawDebugVisualsVel(const FVector& horizontalVelocity) const;
    void DrawMagneticDebugVisuals();
    void SetManualPathMode(bool bManual) { bManualPathMode = bManual; }
    float GetCurrentThrustOutput(int32 ThrusterIndex) const;
    UFUNCTION(BlueprintCallable, Category = "Flight")
    void SetFlightMode(EFlightMode NewMode);

    //PX4
    UFUNCTION(BlueprintCallable, Category = "External Control")
    void ApplyMotorCommands(const TArray<float>& MotorCommands);
    
    // ROS2
    UFUNCTION(BlueprintCallable, Category = "External Control")
    void SetUseExternalController(bool bUseExternal);

    // Setters and Getters
    FVector GetDesiredVelocity() const { return desiredNewVelocity; }
    bool GetDebugVisualsEnabled() const { return bDebugVisualsEnabled; }
    
    float GetDesiredRoll() const { return desiredRoll; }
    float GetDesiredPitch() const { return desiredPitch; }
    
    double GetDesiredRollRate() const {return desiredRollRate;}
    double GetDesiredPitchRate() const {return desiredPitchRate;}
    float GetDesiredYawRate() const { return desiredYawRate; }

    EFlightMode GetFlightMode() const { return currentFlightMode; }
    
    FVector GetCurrentSetPoint() const { return setPoint; }
    
    void SetDebugVisualsEnabled(bool bEnabled) { bDebugVisualsEnabled = bEnabled; }
    void SetDesiredAngle(float newAngle) { maxAngle = newAngle; }
    void SetMaxVelocity(float newMaxVelocity) { maxVelocity = newMaxVelocity;}
    void SetMaxAngle(float newMaxAngle) { maxAngle = newMaxAngle;}
    void SetMaxAngleRate(float newMaxAngleRate) { maxAngleRate = newMaxAngleRate; }
    bool IsHoverModeActive() const { return bHoverModeActive; }

    void SetDesiredVelocity(const FVector& NewVelocity){desiredNewVelocity = NewVelocity;};
    
    void SetDesiredPitchAngle(const float& NewPitch){desiredNewPitch=NewPitch;};
    void SetDesiredRollAngle(const float& NewRoll){desiredNewRoll=NewRoll;};

    void SetDesiredPitchRate(const float& NewPitchRate){desiredNewPitchRate = NewPitchRate;};
    void SetDesiredRollRate(const float& NewRollRate){desiredNewRollRate = NewRollRate;};
    void SetDesiredYawRate(float NewYawRate) { desiredYawRate = NewYawRate; }

    FFullPIDSet* GetPIDSet(EFlightMode Mode){return &PIDSet;}

    // External Control Support (PX4 integration)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "External Control")
    bool bUseExternalController = false;


private:

    UPROPERTY()
    FFullPIDSet PIDSet;
    QuadPIDController* AltitudePID;
    EFlightMode currentFlightMode;
    FSensorData LastSensorData;
    bool bHasValidSensorData = false;
    
    //Global Drone Variables
    FVector currentLocalVelocity;
    bool bDebugVisualsEnabled = false;

    // Position Control
    FVector setPoint;
    
    // Velocity Control
    FVector desiredNewVelocity;

    // Angle Controlv
    float desiredRoll;
    float desiredPitch;
    double desiredNewRoll;
    double desiredNewPitch;

    // Angle Rate Control
    double desiredRollRate;
    double desiredPitchRate;
    double desiredNewRollRate;
    double desiredNewPitchRate;
    float desiredYawRate;
    FVector localAngularRateDeg;
    
    // Hover Mode
    float hoverTargetAltitude;
    bool bHoverModeActive;
    bool bManualThrustMode = false;
    
	float maxThrust;
    float maxVelocity;
    float maxAngle;
    float maxAngleRate;
    float maxPIDOutput;
    float minAltitudeLocal;
    float acceptableDistance;

    // Cascaded yaw control parameters
    float maxYawRate;
    float minVelocityForYaw;

    bool bGamepadModeUI = false; 
    bool bManualPathMode = false;

    // Debug-draw guard to avoid duplicate draws per frame from multiple callers
    bool bDrewMagDebugThisFrame = false;
};

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Utility/NavigationComponent.h"
#include "QuadPawn.generated.h"

class USensorManagerComponent;
class UQuadDroneController;
class UThrusterComponent;
class UPX4Component;
class UCameraRigComponent;
class UCameraComponent;
class USpringArmComponent;

UENUM(BlueprintType)
enum class ECameraMode : uint8
{
    ThirdPerson UMETA(DisplayName = "Third Person"),
    FPV         UMETA(DisplayName = "First Person"),
    GroundTrack UMETA(DisplayName = "Ground Track"),
    FreeOrbit   UMETA(DisplayName = "Free Orbit")
};

UENUM()
enum class EWaypointMode : uint8
{
    WaitingForModeSelection,
    ManualWaypointInput,
    ReadyToStart
};

enum class EFlightMode : uint8;

USTRUCT(BlueprintType)
struct FGamepadInputs
{
    GENERATED_BODY()
    UPROPERTY(BlueprintReadWrite) float Throttle = 0.f;
    UPROPERTY(BlueprintReadWrite) float Yaw      = 0.f;
    UPROPERTY(BlueprintReadWrite) float Pitch    = 0.f;
    UPROPERTY(BlueprintReadWrite) float Roll     = 0.f;
};

UCLASS()
class QUADSIMCORE_API AQuadPawn : public APawn
{
    GENERATED_BODY()

public:
    AQuadPawn();

    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
    virtual void PossessedBy(AController* NewController) override;
    virtual void UnPossessed() override;

    void SetupPropellers();
    void SetupCameras(); // now only creates the rig

    // Drone body
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* DroneBody;

    // Sensors
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    USensorManagerComponent* SensorManager;

    // Camera Rig (single entry point for all cameras)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
    UCameraRigComponent* CameraRig;

    // Thrusters / visuals
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    TArray<UStaticMeshComponent*> Propellers;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    TArray<UThrusterComponent*> Thrusters;

    // Config / state
    UPROPERTY(EditDefaultsOnly, Category="Drone Configuration")
    TArray<bool> MotorClockwiseDirections = { false, true, true, false };

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Drone State")
    TArray<float> PropellerRPMs;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Controller")
    UQuadDroneController* QuadController;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Identification")
    FString DroneID;

    EWaypointMode WaypointMode;
    TArray<FVector> ManualWaypoints;
    FVector NewWaypoint;

    // Helper functions
    void SwitchCamera();
    void ReloadJSONConfig();
    void ResetRotation();
    void ResetPosition();
    void UpdatePropellerVisuals(float DeltaTime);

    UFUNCTION(BlueprintPure, Category="Drone State")
    float GetMass();

    bool getCollisionState(){ return bHasCollidedWithObstacle; }

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Collision")
    bool bHasCollidedWithObstacle;

    UFUNCTION(BlueprintPure, Category="Collision")
    bool HasCollided() const { return bHasCollidedWithObstacle; }

    UFUNCTION(BlueprintCallable, Category="Collision")
    void ResetCollisionStatus();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
    UNavigationComponent* NavigationComponent;

    UFUNCTION(BlueprintCallable, Category="Navigation")
    TArray<FVector> GenerateFigureEightWaypoints(FVector Altitude) const;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Input")
    FGamepadInputs GamepadInputs;

    UFUNCTION(BlueprintCallable, Category="ROS Control")
    void SetExternalAttitudeCommand(float InRoll, float InPitch);

    UFUNCTION(BlueprintCallable, Category="ROS Control")
    void SetExternalVelocityCommand(const FVector& LinearMps, const FVector& AngularRadps);

    UFUNCTION(BlueprintCallable, Category="ROS Control")
    void SetExternalHoverHeight(float HeightMeters);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Simulation")
    bool bIsSimulationControlled = false;

    void UpdateControl(float DeltaTime);

    // Camera API (thin wrappers to the rig)
    void SetCameraMode(ECameraMode NewMode);
    void SwitchCameraCycle();
    UFUNCTION(BlueprintCallable, Category="Camera")
    void ForceFPVCameraActive();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="PX4")
    UPX4Component* PX4Component;

    UFUNCTION()
    void OnDroneHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

    void OnThrottleAxis(float Value);
    void OnYawAxis(float Value);
    void OnPitchAxis(float Value);
    void OnRollAxis(float Value);

    void ToggleGamepadMode();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
    ECameraMode CurrentCameraMode = ECameraMode::ThirdPerson;

    // Ground track handled inside the rig now.
    void UpdateGroundCameraTracking() {} // no-op; kept for compatibility

private:
    float LastCollisionTime = 0.f;
    float CollisionTimeout = 0.2f;
    bool bWaypointModeSelected = false;
    bool bSensorsReady = false;
    bool bControllerReady = false;
    float NextCollisionCheckTime = 0.0f;
};








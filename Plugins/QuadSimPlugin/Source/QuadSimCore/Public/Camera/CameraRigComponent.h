#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "CameraRigComponent.generated.h"

class USpringArmComponent;
class UCameraComponent;

UENUM(BlueprintType)
enum class ERigCameraMode : uint8
{
    ThirdPerson UMETA(DisplayName="Third Person"),
    FPV         UMETA(DisplayName="FPV"),
    GroundTrack UMETA(DisplayName="Ground Track"),
    FreeOrbit   UMETA(DisplayName="Free Orbit") // attached orbit with input
};

UCLASS(ClassGroup=(Camera), meta=(BlueprintSpawnableComponent))
class QUADSIMCORE_API UCameraRigComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    UCameraRigComponent();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTick) override;

    // Setup and mode control
    void InitializeRig(USceneComponent* ParentComp, FName FPVSocketName);
    void SetMode(ERigCameraMode NewMode);
    ERigCameraMode GetMode() const { return CurrentMode; }

    // Input (forwarded from PlayerController)
    void AddLookInput(const FVector2D& LookDelta);  // yaw/pitch
    void AddZoomInput(float ZoomDelta);              // dolly arm length
    void AddPanInput(const FVector2D& PanDelta);     // ground-track pan

    // Active camera component (for diagnostics or custom blends)
    UCameraComponent* GetActiveCamera() const;

    // Ground mode helpers
    void ResetGroundTrack();
    void UpdateGroundTrack(float DeltaSeconds);

private:
    // Components we own/manage
    UPROPERTY() USpringArmComponent* ThirdPersonArm = nullptr;
    UPROPERTY() UCameraComponent*    ThirdPersonCam = nullptr;
    UPROPERTY() UCameraComponent*    FPVCam         = nullptr;
    UPROPERTY() UCameraComponent*    GroundTrackCam = nullptr;

    // state
    ERigCameraMode CurrentMode = ERigCameraMode::ThirdPerson;

    // orbit/zoom
    float TargetArmLength = 250.f;
    FRotator OrbitAngles  = FRotator(-30.f, 0.f, 0.f); // pitch,yaw (looking down at 30 degrees)
    float OrbitInterp     = 10.f;
    float ZoomInterp      = 10.f;

    // limits
    float MinArm = 150.f;
    float MaxArm = 1500.f;

    // Pivot offset - raises the spring arm pivot point above the drone
    FVector SpringArmPivotOffset = FVector(0.f, 0.f, 50.f); // 50cm above drone center

    // ground-track offset in XY (meters)
    FVector2D GroundOffset = FVector2D(200.f, 0.f);
    float GroundLookPanScale = 10.f;    

    bool bEnableFPVLook = true;
    FRotator FpvAngles  = FRotator::ZeroRotator; // local offset relative to socket
    float FpvLookSpeed  = 1.0f;                  // multiplier for mouse deltas
    float FpvMinPitch   = -90.f;                 // can look straight down
    float FpvMaxPitch   = 0.f;                   // cannot look up (0 = level horizon)
    float FpvMaxYaw     = 60.f;
    float FpvInterp     = 10.f;      
    
    // internals
    void EnsureComponents(USceneComponent* ParentComp, FName FPVSocketName);
    void ActivateOnly(UCameraComponent* CamToActivate);
};

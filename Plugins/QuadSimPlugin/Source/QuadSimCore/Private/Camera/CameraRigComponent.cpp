#include "Camera/CameraRigComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Kismet/KismetMathLibrary.h"

UCameraRigComponent::UCameraRigComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UCameraRigComponent::BeginPlay()
{
    Super::BeginPlay();
}

void UCameraRigComponent::InitializeRig(USceneComponent* ParentComp, FName FPVSocketName)
{
    EnsureComponents(ParentComp, FPVSocketName);
    SetMode(ERigCameraMode::FPV); // default start; tweak as desired
}

void UCameraRigComponent::EnsureComponents(USceneComponent* ParentComp, FName FPVSocketName)
{
    // Third-person arm/cam
    if (!ThirdPersonArm)
    {
        ThirdPersonArm = NewObject<USpringArmComponent>(GetOwner(), TEXT("Rig_ThirdPersonArm"));
        ThirdPersonArm->TargetArmLength = TargetArmLength;
        ThirdPersonArm->bDoCollisionTest = true;
        ThirdPersonArm->bUsePawnControlRotation = false;
        ThirdPersonArm->SetupAttachment(ParentComp);

        // CRITICAL FIX: Raise the spring arm pivot above the drone so camera doesn't sit on ground
        ThirdPersonArm->SetRelativeLocation(SpringArmPivotOffset);

        // Initialize rotation
        ThirdPersonArm->SetRelativeRotation(OrbitAngles);

        ThirdPersonArm->RegisterComponent();
    }
    if (!ThirdPersonCam)
    {
        ThirdPersonCam = NewObject<UCameraComponent>(GetOwner(), TEXT("Rig_ThirdPersonCam"));
        ThirdPersonCam->SetupAttachment(ThirdPersonArm, USpringArmComponent::SocketName);
        ThirdPersonCam->bAutoActivate = false;
        ThirdPersonCam->RegisterComponent();
    }

    // FPV camera mounted to a socket on the body
    if (!FPVCam)
    {
        FPVCam = NewObject<UCameraComponent>(GetOwner(), TEXT("Rig_FPVCam"));
        FPVCam->SetupAttachment(ParentComp, FPVSocketName);
        FPVCam->bAutoActivate = false;
        FPVCam->RegisterComponent();
    }

    // Ground-track camera (world-placed, not attached)
    if (!GroundTrackCam)
    {
        GroundTrackCam = NewObject<UCameraComponent>(GetOwner(), TEXT("Rig_GroundTrackCam"));
        GroundTrackCam->bAutoActivate = false;
        GroundTrackCam->SetupAttachment(nullptr);
        
        USceneComponent* SafeParent = ParentComp ? ParentComp : GetOwner()->GetRootComponent();
        GroundTrackCam->SetupAttachment(SafeParent);
        GroundTrackCam->SetUsingAbsoluteLocation(true);
        GroundTrackCam->SetUsingAbsoluteRotation(true);
        GroundTrackCam->RegisterComponent();
    }
}

void UCameraRigComponent::SetMode(ERigCameraMode NewMode)
{
    CurrentMode = NewMode;
    switch (NewMode)
    {
        case ERigCameraMode::ThirdPerson: ActivateOnly(ThirdPersonCam); break;
        case ERigCameraMode::FPV:         ActivateOnly(FPVCam);         break;
        case ERigCameraMode::GroundTrack: ActivateOnly(GroundTrackCam); ResetGroundTrack(); break;
        case ERigCameraMode::FreeOrbit:   ActivateOnly(ThirdPersonCam); break; // same arm as ThirdPerson
    }
}

void UCameraRigComponent::ActivateOnly(UCameraComponent* CamToActivate)
{
    if (ThirdPersonCam) ThirdPersonCam->SetActive(false);
    if (FPVCam)         FPVCam->SetActive(false);
    if (GroundTrackCam) GroundTrackCam->SetActive(false);
    if (CamToActivate)  CamToActivate->SetActive(true);
}

UCameraComponent* UCameraRigComponent::GetActiveCamera() const
{
    switch (CurrentMode)
    {
        case ERigCameraMode::ThirdPerson:
        case ERigCameraMode::FreeOrbit: return ThirdPersonCam;
        case ERigCameraMode::FPV:       return FPVCam;
        case ERigCameraMode::GroundTrack: return GroundTrackCam;
    }
    return nullptr;
}

void UCameraRigComponent::AddLookInput(const FVector2D& LookDelta)
{
    if (CurrentMode == ERigCameraMode::ThirdPerson || CurrentMode == ERigCameraMode::FreeOrbit)
    {
        OrbitAngles.Yaw   += LookDelta.X;
        OrbitAngles.Pitch = FMath::Clamp(OrbitAngles.Pitch + LookDelta.Y, -80.f, 80.f);
        return;
    }

    if (CurrentMode == ERigCameraMode::FPV && bEnableFPVLook)
    {
        // Yaw left/right, pitch down only (no looking up)
        FpvAngles.Yaw   = FMath::Clamp(FpvAngles.Yaw   + LookDelta.X * FpvLookSpeed, -FpvMaxYaw,   FpvMaxYaw);
        FpvAngles.Pitch = FMath::Clamp(FpvAngles.Pitch + LookDelta.Y * FpvLookSpeed, FpvMinPitch, FpvMaxPitch);
        return;
    }

    if (CurrentMode == ERigCameraMode::GroundTrack)
    {
        // RMB drag pans the ground camera on XY plane
        GroundOffset += LookDelta * GroundLookPanScale;

        // --- ALTERNATIVE: orbit around the drone instead of panning ---
        // const float OrbitSpeed = 0.2f;  // deg per pixel
        // GroundOrbitYaw += LookDelta.X * OrbitSpeed;
        // GroundOrbitPitch = FMath::Clamp(GroundOrbitPitch + LookDelta.Y * OrbitSpeed, -80.f, 80.f);
        // (would need to position camera on a sphere around drone instead of GroundOffset)
        return;
    }
}

void UCameraRigComponent::AddZoomInput(float ZoomDelta)
{
    TargetArmLength = FMath::Clamp(TargetArmLength + ZoomDelta * -50.f, MinArm, MaxArm);
}

void UCameraRigComponent::AddPanInput(const FVector2D& PanDelta)
{
    if (CurrentMode == ERigCameraMode::GroundTrack)
    {
        GroundOffset += PanDelta * 10.f;
    }
}

void UCameraRigComponent::ResetGroundTrack()
{
    GroundOffset = FVector2D(200.f, 0.f);
}

void UCameraRigComponent::UpdateGroundTrack(float DeltaSeconds)
{
    if (CurrentMode != ERigCameraMode::GroundTrack || !GroundTrackCam) return;
    const AActor* Owner = GetOwner();
    if (!Owner) return;

    const FVector DroneLoc = Owner->GetActorLocation();
    const FRotator YawOnly(0.f, Owner->GetActorRotation().Yaw, 0.f);
    const FVector Right = UKismetMathLibrary::GetRightVector(YawOnly);

    const FVector Ground(DroneLoc.X, DroneLoc.Y, 10.f);
    const FVector CamPos = Ground + Right * GroundOffset.X + UKismetMathLibrary::GetForwardVector(YawOnly) * GroundOffset.Y;

    GroundTrackCam->SetWorldLocation(CamPos);
    const FRotator LookAt = UKismetMathLibrary::FindLookAtRotation(CamPos, DroneLoc);
    GroundTrackCam->SetWorldRotation(FMath::RInterpTo(GroundTrackCam->GetComponentRotation(), LookAt, DeltaSeconds, 10.f));
}

void UCameraRigComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    if ((CurrentMode == ERigCameraMode::ThirdPerson || CurrentMode == ERigCameraMode::FreeOrbit) && ThirdPersonArm)
    {
        ThirdPersonArm->TargetArmLength =
            FMath::FInterpTo(ThirdPersonArm->TargetArmLength, TargetArmLength, DeltaTime, ZoomInterp);
        ThirdPersonArm->SetRelativeRotation(
            FMath::RInterpTo(ThirdPersonArm->GetRelativeRotation(), OrbitAngles, DeltaTime, OrbitInterp)
        );
    }
    else if (CurrentMode == ERigCameraMode::FPV && FPVCam)
    {
        // Smoothly apply the FPV gimbal angles as a local relative rotation
        const FRotator Current = FPVCam->GetRelativeRotation();
        const FRotator Target  = FRotator(FpvAngles.Pitch, FpvAngles.Yaw, 0.f);
        const FRotator NewRot  = FMath::RInterpTo(Current, Target, DeltaTime, FpvInterp);
        FPVCam->SetRelativeRotation(NewRot);
    }
    else if (CurrentMode == ERigCameraMode::GroundTrack)
    {
        UpdateGroundTrack(DeltaTime);
    }
}

#include "QuadSimPlayerController.h"

#include "ImGuiToggleInputSubsystem.h"
#include "Camera/CameraRigComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

void AQuadSimPlayerController::BeginPlay()
{
	Super::BeginPlay();

	// Let the subsystem handle initial input mode setup
	UImGuiToggleInputSubsystem* Subsys = nullptr;
	if (UGameInstance* GI = GetGameInstance())
	{
		Subsys = GI->GetSubsystem<UImGuiToggleInputSubsystem>();
		if (Subsys)
		{
			UE_LOG(LogTemp, Warning, TEXT("[PlayerController] ImGuiToggle subsystem found - it will manage input modes"));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("[PlayerController] ImGuiToggle subsystem MISSING - input may not work correctly"));
		}
	}

	// Enable mouse events for ImGui
	SetDefaultMouseFlags(true);

	// Initial state: ImGui enabled (UI mode)
	// The subsystem will have already set GameAndUI mode in its Initialize()
	bShowMouseCursor = true;
	SetIgnoreLookInput(true);   // Start with camera controls OFF
	SetIgnoreMoveInput(true);   // Start with camera controls OFF

	// Set initial input mode - subsystem will override as needed
	FInputModeGameAndUI Mode;
	Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	Mode.SetHideCursorDuringCapture(false);
	SetInputMode(Mode);
}

void AQuadSimPlayerController::SetupInputComponent()
{
	Super::SetupInputComponent();
	if (!InputComponent) return;

	// Only bind the camera axes. DO NOT bind "ToggleImGui" here.
	InputComponent->BindAxis("Cam_Yaw",   this, &AQuadSimPlayerController::OnYawAxis);
	InputComponent->BindAxis("Cam_Pitch", this, &AQuadSimPlayerController::OnPitchAxis);
	InputComponent->BindAxis("Cam_Zoom",  this, &AQuadSimPlayerController::OnZoomAxis);
}

void AQuadSimPlayerController::OnYawAxis(float Value)
{
	if (FMath::IsNearlyZero(Value)) return;
	ForwardLookToViewTarget(FVector2D(Value * YawRate, 0.f));
}

void AQuadSimPlayerController::OnPitchAxis(float Value)
{
	if (FMath::IsNearlyZero(Value)) return;
	ForwardLookToViewTarget(FVector2D(0.f, -Value * PitchRate));
}

void AQuadSimPlayerController::OnZoomAxis(float Value)
{
	if (FMath::IsNearlyZero(Value)) return;
	ForwardZoomToViewTarget(Value * ZoomStep);
}

void AQuadSimPlayerController::ForwardLookToViewTarget(const FVector2D& Delta)
{
	if (AActor* VT = GetViewTarget())
	{
		if (UCameraRigComponent* Rig = VT->FindComponentByClass<UCameraRigComponent>())
		{
			Rig->AddLookInput(Delta);
			return;
		}
	}
	FRotator R = GetControlRotation();
	R.Yaw   += Delta.X;
	R.Pitch  = FMath::Clamp(R.Pitch + Delta.Y, -89.0f, 89.0f);
	SetControlRotation(R);
}

void AQuadSimPlayerController::ForwardZoomToViewTarget(float Delta)
{
	if (AActor* VT = GetViewTarget())
	{
		if (UCameraRigComponent* Rig = VT->FindComponentByClass<UCameraRigComponent>())
		{
			Rig->AddZoomInput(Delta);
			return;
		}
	}
	// Optional: fallback FOV
}

void AQuadSimPlayerController::ApplyGameAndUIFocus(bool bShowCursorIn)
{
	// We stay in GameAndUI; just honor the cursor toggle
	bShowMouseCursor = bShowCursorIn;
}

void AQuadSimPlayerController::ApplyGameOnly() { /* no-op by design */ }

void AQuadSimPlayerController::SetDefaultMouseFlags(bool bEnable)
{
	bEnableClickEvents     = bEnable;
	bEnableMouseOverEvents = bEnable;
}

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Core/DroneJSONConfig.h"
#include "GameFramework/PlayerStart.h"
#include "Blueprint/UserWidget.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PrimitiveComponent.h" 
#include "GameFramework/Actor.h"        
#include "Core/ThrusterComponent.h"       
#include "Sensors/MagSensor.h"
#include "Components/ChildActorComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "SimulationCore/Public/Interfaces/ISimulatable.h" // Add this for interface check
#include "Controllers/PX4Component.h"
#include "Sensors/BaroSensor.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/SensorManagerComponent.h"

namespace DroneWaypointConfig
{
	// All values in meters to keep flight control domain in meters
	static constexpr float startHeight     = 5.0f;    // m above current Z
	static constexpr float totalHeight     = 20.0f;   // m rise across the spiral
	static constexpr float radius          = 10.0f;   // m spiral radius
	static constexpr int32 segmentsPerRev  = 72;      // higher = smoother curve
	static constexpr int32 numRevolutions  = 3;       // total revolutions
}
const FVector start = FVector(0, 0, 1000);
static TArray<FVector> spiralWaypoints(const FVector& startPos)
{
    TArray<FVector> waypoints;

    // Base position in meters
    const FVector basePos = startPos;

    // Parameters
    const float startHeight = DroneWaypointConfig::startHeight;    // m
    const float totalHeight = DroneWaypointConfig::totalHeight;    // m
    const float radius      = DroneWaypointConfig::radius;         // m
    const int32 segPerRev   = DroneWaypointConfig::segmentsPerRev; // segments per revolution
    const int32 revs        = DroneWaypointConfig::numRevolutions; // total revolutions
    const int32 totalSeg    = segPerRev * revs;
    const float dTheta      = 2.0f * PI / static_cast<float>(segPerRev);
    const float dzPerSeg    = totalHeight / static_cast<float>(totalSeg);

    // Add a vertical climb to the start height for clarity
    const float startZ = basePos.Z + startHeight;
    waypoints.Add(FVector(basePos.X, basePos.Y, basePos.Z));
    waypoints.Add(FVector(basePos.X, basePos.Y, startZ));

    // Build a helix that rises totalHeight over 'revs' revolutions
    float z = startZ;
    for (int32 i = 0; i <= totalSeg; ++i)
    {
        const float theta = i * dTheta;
        const float x = basePos.X + radius * FMath::Cos(theta);
        const float y = basePos.Y + radius * FMath::Sin(theta);
        waypoints.Add(FVector(x, y, z));
        z += dzPerSeg;
    }

    return waypoints;
}
const FName ObstacleCollisionTag = FName("Obstacle");
TArray<FVector> AQuadPawn::GenerateFigureEightWaypoints(FVector Altitude) const
{
    return spiralWaypoints(Altitude);
}

//=========================== Constructor =========================== //
AQuadPawn::AQuadPawn()
	: DroneBody(nullptr)
	, SpringArm(nullptr)
	, Camera(nullptr)
	, CameraFPV(nullptr)
	, CameraGroundTrack(nullptr)
	, QuadController(nullptr)	
	, WaypointMode(EWaypointMode::WaitingForModeSelection)
	, NewWaypoint(FVector::ZeroVector)
	, bHasCollidedWithObstacle(false)
	, CurrentCameraMode(ECameraMode::ThirdPerson)
	, bWaypointModeSelected(false)
{
	PrimaryActorTick.bCanEverTick = true;

    // Skeletal mesh for drone body (physics & visuals)
    DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    RootComponent = DroneBody;
    DroneBody->SetSimulatePhysics(true);
    DroneBody->SetNotifyRigidBodyCollision(true);
    DroneBody->SetGenerateOverlapEvents(true);
    DroneBody->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);

	SetupCameras();
	SetupPropellers();
	
	// SENSORS
	SensorManager = CreateDefaultSubobject<USensorManagerComponent>(TEXT("SensorManager"));
	SensorManager->SetupAttachment(DroneBody);

	// Controller
	QuadController = CreateDefaultSubobject<UQuadDroneController>(TEXT("QuadDroneController"));
	
	// Nav Component
	NavigationComponent = CreateDefaultSubobject<UNavigationComponent>(TEXT("NavigationComponent"));

	// Create PX4 component
	PX4Component = CreateDefaultSubobject<UPX4Component>(TEXT("PX4Component"));

	
    AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

void AQuadPawn::SetupCameras()
{
    // Camera setup code moved to separate function for clarity
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(DroneBody);
    SpringArm->TargetArmLength = 200.f;
    SpringArm->SetRelativeRotation(FRotator(-20.f, 0.f, 0.f));
    SpringArm->bDoCollisionTest = false;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;
    
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
    Camera->bAutoActivate = false;
    
    CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
    CameraFPV->SetupAttachment(DroneBody, TEXT("FPVCam"));
    CameraFPV->bAutoActivate = true;
    
    CameraGroundTrack = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraGroundTrack"));
    CameraGroundTrack->bAutoActivate = false;
}
void AQuadPawn::SetupPropellers()
{
    const FString propellerNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
    const FString socketNames[] = { TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR") };
    
    Propellers.SetNum(4);
    Thrusters.SetNum(4);
    PropellerRPMs.SetNum(4);
    
    for (int i = 0; i < 4; i++)
    {
        Propellers[i] = CreateDefaultSubobject<UStaticMeshComponent>(*propellerNames[i]);
        Propellers[i]->SetSimulatePhysics(false);
    	Propellers[i]->SetMassOverrideInKg(*propellerNames[i],0);
        Propellers[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Propellers[i]->SetupAttachment(DroneBody, *socketNames[i]);
        
        Thrusters[i] = CreateDefaultSubobject<UThrusterComponent>(
            *FString::Printf(TEXT("Thruster_%s"), *propellerNames[i])
        );
        Thrusters[i]->SetupAttachment(DroneBody, *socketNames[i]);
        Thrusters[i]->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
        
        PropellerRPMs[i] = 0.f;
    }
}

//=========================== BeginPlay & Tick =========================== //

void AQuadPawn::BeginPlay()
{
	Super::BeginPlay();
	ToggleImguiInput();
	
    DroneID = GetName();
	UE_LOG(LogTemp, Display, TEXT("QuadPawn BeginPlay: DroneID set to %s"), *DroneID);

    // Initialize sensors and mark readiness
    if (SensorManager)
    {
        SensorManager->InitializeSensors();
        bSensorsReady = SensorManager->AreSensorsInitialized();
        UE_LOG(LogTemp, Display, TEXT("QuadPawn: SensorManager init complete (ready=%d)"), bSensorsReady);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("QuadPawn: SensorManager is null!"));
    }
    
    // Ensure controller exists and is initialized once sensors are ready
    if (!QuadController)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPawn: QuadController missing, creating at BeginPlay"));
        QuadController = NewObject<UQuadDroneController>(this);
    }
    if (QuadController && bSensorsReady)
    {
        QuadController->Initialize(this);
        QuadController->ResetControllerState();
        bControllerReady = true;
        UE_LOG(LogTemp, Display, TEXT("QuadPawn: Controller initialized"));
    }
	
	if (DroneBody)
	{
		DroneBody->OnComponentHit.AddDynamic(this, &AQuadPawn::OnDroneHit);
	}
	ResetCollisionStatus();

	SetCameraMode(ECameraMode::FPV);
}
void AQuadPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	
	Super::EndPlay(EndPlayReason);
}
void AQuadPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (!bSensorsReady || !bControllerReady)
		return;

	if (!bIsSimulationControlled)
    {
        UpdateControl(DeltaTime);
    }
	
	UpdatePropellerVisuals(DeltaTime);

	if (CurrentCameraMode == ECameraMode::GroundTrack)
	{
		UpdateGroundCameraTracking();
	}
	
	if (bHasCollidedWithObstacle)
	{
		float CurrentTime = GetWorld()->GetTimeSeconds();
		if (CurrentTime >= NextCollisionCheckTime)
		{
			if (CurrentTime - LastCollisionTime > CollisionTimeout)
			{
				bHasCollidedWithObstacle = false;
				UE_LOG(LogTemp, Display, TEXT("%s collision cleared"), *GetName());
			}
			NextCollisionCheckTime = CurrentTime + 1.0f; // Check every second
		}
	}
}

//=========================== Update =========================== //

void AQuadPawn::UpdateControl(float DeltaTime)
{
    	if (!SensorManager)
    		return;
    
	// Update sensors
	SensorManager->UpdateAllSensors(DeltaTime, false);
    
    	// Get unified sensor data
	FSensorData SensorData = SensorManager->GetCurrentSensorData();
    
    	// Late-initialize controller if needed
    	if (!QuadController)
    	{
    		UE_LOG(LogTemp, Warning, TEXT("QuadPawn: QuadController missing in UpdateControl, creating now"));
    		QuadController = NewObject<UQuadDroneController>(this);
    	}
    	if (QuadController)
    	{
    		// Ensure controller has pawn reference
    		if (!bControllerReady)
    		{
    			QuadController->Initialize(this);
    			QuadController->ResetControllerState();
    			bControllerReady = true;
    		}
    		// Pass to controller
    		QuadController->Update(SensorData, DeltaTime);
    	}
    
	// Update navigation
	if (NavigationComponent && SensorData.bGPSValid)
	{
		FVector CurrentPos(
			SensorData.GPSPosMeters.X,
			SensorData.GPSPosMeters.Y,
			SensorData.bBaroValid ? SensorData.BaroAltitudeM : SensorData.GPSPosMeters.Z
		);
        
		NavigationComponent->UpdateNavigation(CurrentPos);
		FVector NextGoal = NavigationComponent->GetCurrentSetpoint();
		QuadController->SetDestination(NextGoal);
	}
}
void AQuadPawn::UpdatePropellerVisuals(float DeltaTime)
{
	
    for (int32 i = 0; i < Propellers.Num(); i++)
    {
        if (Propellers[i] && QuadController)
        {
            float ThrustVal = QuadController->GetCurrentThrustOutput(i);
            PropellerRPMs[i] = FMath::Abs(ThrustVal);
            
            float Direction = (MotorClockwiseDirections.IsValidIndex(i) && 
                             MotorClockwiseDirections[i]) ? -1.0f : 1.0f;
            float DeltaRotation = PropellerRPMs[i] * 6.0f * DeltaTime * Direction;
            
            Propellers[i]->AddLocalRotation(FRotator(0.f, DeltaRotation, 0.f));
        }
    }
}

//=========================== Cam Settings =========================== //

void AQuadPawn::SetCameraMode(ECameraMode NewMode)
{
    // Deactivate all cameras
    if (Camera) Camera->SetActive(false);
    if (CameraFPV) CameraFPV->SetActive(false);
    if (CameraGroundTrack) CameraGroundTrack->SetActive(false);
    
    // Activate the selected camera
    CurrentCameraMode = NewMode;
    switch (NewMode)
    {
        case ECameraMode::ThirdPerson:
            if (Camera) Camera->SetActive(true);
            break;
        case ECameraMode::FPV:
            if (CameraFPV) CameraFPV->SetActive(true);
            break;
        case ECameraMode::GroundTrack:
            if (CameraGroundTrack) 
            {
                CameraGroundTrack->SetActive(true);
                ResetGroundCameraPosition();
            }
            break;
    }
}
void AQuadPawn::SwitchCamera()
{
    // Cycle through camera modes
    ECameraMode NextMode = ECameraMode::ThirdPerson;
    switch (CurrentCameraMode)
    {
        case ECameraMode::ThirdPerson:
            NextMode = ECameraMode::FPV;
            break;
        case ECameraMode::FPV:
            NextMode = ECameraMode::GroundTrack;
            break;
        case ECameraMode::GroundTrack:
            NextMode = ECameraMode::ThirdPerson;
            break;
    }
    SetCameraMode(NextMode);
}
void AQuadPawn::ForceFPVCameraActive()
{
	if (!Camera || !CameraFPV || !CameraGroundTrack) return;
	Camera->SetActive(false);
	CameraGroundTrack->SetActive(false);
	CameraFPV->SetActive(true);
	CurrentCameraMode = ECameraMode::FPV;
}
void AQuadPawn::ResetGroundCameraPosition()
{
	if (!CameraGroundTrack || !DroneBody) return;

	const float GroundOffsetDistance = 200.0f; // 5 meters

	FVector DroneLocation = GetActorLocation();
	FRotator DroneYawRotation(0, GetActorRotation().Yaw, 0);

	FVector RightVector = UKismetMathLibrary::GetRightVector(DroneYawRotation);

	FVector GroundPos = FVector(DroneLocation.X, DroneLocation.Y, 10.0f);
	FVector CameraTargetPosition = GroundPos + RightVector * GroundOffsetDistance;

	CameraGroundTrack->SetWorldLocation(CameraTargetPosition);

	UpdateGroundCameraTracking();
}
void AQuadPawn::UpdateGroundCameraTracking()
{
	if (CurrentCameraMode == ECameraMode::GroundTrack && CameraGroundTrack && CameraGroundTrack->IsActive() && DroneBody)
	{
		FVector CameraLocation = CameraGroundTrack->GetComponentLocation();
		FVector DroneLocation = GetActorLocation(); 

		if (FVector::DistSquaredXY(CameraLocation, DroneLocation) < 1.0f) 
		{
			return;
		}

		FRotator LookAtRotation = UKismetMathLibrary::FindLookAtRotation(CameraLocation, DroneLocation);

		FRotator CurrentRotation = CameraGroundTrack->GetComponentRotation();
		FRotator TargetRotation = FMath::RInterpTo(CurrentRotation, LookAtRotation, GetWorld()->GetDeltaSeconds(), 10.0f); // Adjust interp speed
		CameraGroundTrack->SetWorldRotation(TargetRotation);

	}
}

//=========================== Control Input =========================== //

void AQuadPawn::ToggleImguiInput()
{
	UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("ImGui.ToggleInput");
}
void AQuadPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadPawn::ToggleImguiInput);
	PlayerInputComponent->BindAction("ReloadJSON", IE_Pressed, this, &AQuadPawn::ReloadJSONConfig);
	PlayerInputComponent->BindAction("GP_ToggleFlightMode", IE_Pressed, this, &AQuadPawn::ToggleGamepadMode);

	PlayerInputComponent->BindAxis("GP_Throttle", this, &AQuadPawn::OnThrottleAxis);
	PlayerInputComponent->BindAxis("GP_Yaw",      this, &AQuadPawn::OnYawAxis);
	PlayerInputComponent->BindAxis("GP_Pitch",    this, &AQuadPawn::OnPitchAxis);
	PlayerInputComponent->BindAxis("GP_Roll",     this, &AQuadPawn::OnRollAxis);
	
	PlayerInputComponent->BindAction("GP_SwitchCamera", IE_Pressed, this, &AQuadPawn::SwitchCamera);
	PlayerInputComponent->BindAction("GP_ResetRotation", IE_Pressed, this, &AQuadPawn::ResetRotation);
	PlayerInputComponent->BindAction("GP_ResetPosition", IE_Pressed, this, &AQuadPawn::ResetPosition);
}
void AQuadPawn::ToggleGamepadMode()
{
	const bool bGP = (QuadController->GetFlightMode() == EFlightMode::JoyStickAngleControl);
	QuadController->SetFlightMode(bGP ? EFlightMode::None: EFlightMode::JoyStickAngleControl);
}
void AQuadPawn::OnThrottleAxis(float Value) { GamepadInputs.Throttle = Value; }
void AQuadPawn::OnYawAxis(float Value)      { GamepadInputs.Yaw      = Value; }
void AQuadPawn::OnPitchAxis(float Value)    { GamepadInputs.Pitch    = Value; }
void AQuadPawn::OnRollAxis(float Value)     { GamepadInputs.Roll     = Value; }
void AQuadPawn::ReloadJSONConfig(){UDroneJSONConfig::Get().ReloadConfig();}

//=========================== Collision =========================== //

void AQuadPawn::OnDroneHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
    if (OtherActor && OtherActor != this && OtherActor->ActorHasTag(ObstacleCollisionTag))
    {
        LastCollisionTime = GetWorld()->GetTimeSeconds();
        if (!bHasCollidedWithObstacle)
        {
            bHasCollidedWithObstacle = true;
            UE_LOG(LogTemp, Display, TEXT("%s collided with obstacle: %s"), *GetName(), *OtherActor->GetName());
        }
    }
}
void AQuadPawn::ResetCollisionStatus()
{
	if (bHasCollidedWithObstacle) 
	{
		UE_LOG(LogTemp, Log, TEXT("%s collision status reset."), *GetName());
	}
	bHasCollidedWithObstacle = false;
}

float AQuadPawn::GetMass()
{
	return DroneBody ? DroneBody->GetMass() : 0.0f;
}

void AQuadPawn::PossessedBy(AController* NewController)
{
    Super::PossessedBy(NewController);
    ForceFPVCameraActive();
}
void AQuadPawn::UnPossessed()
{
    Super::UnPossessed();

}

void AQuadPawn::ResetRotation()
{
	if (DroneBody)
	{
		const FRotator CurrentRotation = GetActorRotation();
		const FRotator UprightRotation(0.0f, CurrentRotation.Yaw, 0.0f);

		SetActorRotation(UprightRotation);

		DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
		DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
		QuadController->ResetPID(); 

		UE_LOG(LogTemp, Log, TEXT("Drone rotation reset."));
	}
}
void AQuadPawn::ResetPosition()
{
    if (GetWorld())
    {
        AActor* PlayerStart = UGameplayStatics::GetActorOfClass(GetWorld(), APlayerStart::StaticClass());

        if (PlayerStart)
        {
            // Preserve current scale when resetting position/orientation
            const FVector CurrentScale = GetActorScale3D();
            SetActorLocationAndRotation(PlayerStart->GetActorLocation(),PlayerStart->GetActorRotation(),false,nullptr,ETeleportType::TeleportPhysics);
            SetActorScale3D(CurrentScale);

        	if (DroneBody)
        	{
        		DroneBody->SetSimulatePhysics(true);
        		DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
        		DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
        		DroneBody->WakeAllRigidBodies();
        	}
        	QuadController->ResetPID();
			QuadController->SetDesiredVelocity(FVector::ZeroVector);
        	
			UE_LOG(LogTemp, Log, TEXT("Drone position reset to PlayerStart."));
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Could not reset position: No PlayerStart actor found in the level."));
		}
	}
}

void AQuadPawn::SetExternalAttitudeCommand(float InRoll, float InPitch)
{
	if (QuadController)
	{
		// IMPORTANT: We must also tell the controller to enter a mode
		// that will actually use these angle commands.
		QuadController->SetFlightMode(EFlightMode::AngleControl);

		// Now, pass the desired angles to the controller.
		QuadController->SetDesiredRollAngle(InRoll);
		QuadController->SetDesiredPitchAngle(InPitch);
        
		UE_LOG(LogTemp, Log, TEXT("QuadPawn: Passed external attitude to controller (Roll: %.2f, Pitch: %.2f)"), InRoll, InPitch);
	}
}
void AQuadPawn::SetExternalVelocityCommand(const FVector& LinearMps, const FVector& AngularRadps)
{
    if (!QuadController)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPawn::SetExternalVelocityCommand: QuadController is null"));
        return;
    }

    // Ensure the controller uses velocity mode
    QuadController->SetFlightMode(EFlightMode::VelocityControl);

    // Desired linear velocity is expected in local FLU (m/s)
    QuadController->SetDesiredVelocity(LinearMps);

    // Controller yaw rate is in deg/s; convert from rad/s
    const float YawRateDeg = FMath::RadiansToDegrees(AngularRadps.Z);
    QuadController->SetDesiredYawRate(YawRateDeg);

    UE_LOG(LogTemp, Log, TEXT("QuadPawn: External velocity set v=(%.2f,%.2f,%.2f) m/s, yawRate=%.2f deg/s"),
           LinearMps.X, LinearMps.Y, LinearMps.Z, YawRateDeg);
}
void AQuadPawn::SetExternalHoverHeight(float HeightMeters)
{
    if (!QuadController)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPawn::SetExternalHoverHeight: QuadController is null"));
        return;
    }
    // Controller expects hover target in meters
    QuadController->SetHoverMode(true, HeightMeters);
    UE_LOG(LogTemp, Log, TEXT("QuadPawn: External hover height set to %.2f m"), HeightMeters);
}

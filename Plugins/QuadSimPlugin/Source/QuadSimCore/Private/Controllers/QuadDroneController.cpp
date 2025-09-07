// QuadDroneController.cpp

#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include <random>

#ifndef EXCLUDE_PX4_COMPONENT
#include "Controllers/PX4Component.h"
#endif
#include "GeographicCoordinates.h"
#include "UI/ImGuiUtil.h"
#include "Core/DroneJSONConfig.h"
#include "Core/DroneManager.h"
#include "Kismet/GameplayStatics.h"
#include "Math/UnrealMathUtility.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/MagSensor.h"
#include "Sensors/SensorManagerComponent.h"
#include "WorldMagneticModel/GeoMagDeclination.h"
#include "GeoReferencingSystem.h"         
#include "Sensors/BaroSensor.h"
#include "Sensors/IMUSensor.h"


// ---------------------- Constructor ------------------------


UQuadDroneController::UQuadDroneController(const FObjectInitializer& ObjectInitializer)
	: dronePawn(nullptr)
	, Thrusts({ 0, 0, 0, 0 })
	, AltitudePID(nullptr)
	, currentLocalVelocity(FVector::ZeroVector)
	, bDebugVisualsEnabled(false)
	, setPoint(FVector::ZeroVector)
	, desiredNewVelocity(FVector::ZeroVector)
	, desiredRoll(0.0f)
	, desiredPitch(0.0f)
	, desiredNewRoll(0.0f)
	, desiredNewPitch(0.0f)
	, desiredRollRate(0.0f)
	, desiredPitchRate(0.0f)
	, desiredNewRollRate(0.0f)
	, desiredNewPitchRate(0.0f)
	, desiredYawRate(0.0f)
	, localAngularRateDeg(FVector::ZeroVector)
	, hoverTargetAltitude(0.0f)
	, bHoverModeActive(false)
	, bManualThrustMode(false)
	
{
	const auto& Config = UDroneJSONConfig::Get().Config;
	maxVelocity = Config.FlightParams.MaxVelocity;
	maxThrust = Config.FlightParams.MaxThrust;
	maxAngle = Config.FlightParams.MaxAngle;
	maxAngleRate = Config.FlightParams.MaxAngleRate;
	maxPIDOutput = Config.FlightParams.MaxPIDOutput;
	minAltitudeLocal = Config.FlightParams.MinAltitudeLocal;
    acceptableDistance = Config.FlightParams.AcceptableDistance;
	
    // Yaw control parameters from config
    maxYawRate = Config.ControllerParams.YawRate;
    minVelocityForYaw = Config.ControllerParams.MinVelocityForYaw;

    // Start with flight mode None (motors off) until mode is selected via UI
    currentFlightMode = EFlightMode::None;
    Debug_DrawDroneCollisionSphere = true;
	Debug_DrawDroneWaypoint = true;
	
	FFullPIDSet ControllerSet;

	ControllerSet.XPID = new QuadPIDController();
	ControllerSet.XPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.XPID->SetGains(3.f, 0.f, 0.0f);

	ControllerSet.YPID = new QuadPIDController();
	ControllerSet.YPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.YPID->SetGains(3.f, 0.0f, 0.0f);

	ControllerSet.ZPID = new QuadPIDController();
	ControllerSet.ZPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.ZPID->SetGains(5.f, 0.0f, 0.0f);

	ControllerSet.RollPID = new QuadPIDController();
	ControllerSet.RollPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.RollPID->SetGains(0.2f, 0.18f, 0.3f);

	ControllerSet.PitchPID = new QuadPIDController();
	ControllerSet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.PitchPID->SetGains(0.2f, 0.18f, 0.3f);

	ControllerSet.RollRatePID = new QuadPIDController();
	ControllerSet.RollRatePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.RollRatePID->SetGains(0.35f, 0.16f, 0.25f);
	
	ControllerSet.PitchRatePID = new QuadPIDController();
	ControllerSet.PitchRatePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.PitchRatePID->SetGains(0.35f, 0.16f, 0.25f);
	
	ControllerSet.YawRatePID = new QuadPIDController();
	ControllerSet.YawRatePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.YawRatePID->SetGains(1.0f, 0.0f, 0.0f);
//--------------------------------------------------------------
	PIDSet = MoveTemp(ControllerSet);

	AltitudePID = new QuadPIDController();
	AltitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	AltitudePID->SetGains(5.f, 1.f, 0.1f);
}

// ---------------------- Initialization ------------------------

void UQuadDroneController::Initialize(AQuadPawn* InPawn)
{
	if (!InPawn)
	{
		UE_LOG(LogTemp, Error, TEXT("Initialize called with null pawn"));
		return;
	}

	if (dronePawn != InPawn)
	{
		UE_LOG(LogTemp, Display, TEXT("Initializing controller for pawn: %s"), *InPawn->GetName());
		dronePawn = InPawn;
		// Register this controller with the global DroneManager for swarm broadcasts
		if (UWorld* World = dronePawn->GetWorld())
		{
			if (ADroneManager* Manager = ADroneManager::Get(World))
			{
				Manager->RegisterDroneController(this);
			}
		}
	}
}

// ---------------------- Update ------------------------

void UQuadDroneController::Update(double a_deltaTime)
{
    ADroneManager* Manager = ADroneManager::Get(dronePawn ? dronePawn->GetWorld() : nullptr);
    bool bShowUI = true;
    if (Manager)
    {
       if (!Manager->IsSwarmMode())
       {
          APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
          if (PC && PC->GetPawn() != dronePawn)
          {
             bShowUI = false;
          }
       }
       else
       {
          int32 MyIndex = Manager->GetDroneIndex(dronePawn);
          if (MyIndex != 0)
          {
             bShowUI = false;
          }
       }
    }
	
    if (currentFlightMode != EFlightMode::None)
    {
       if (bGamepadModeUI)
          GamepadController(a_deltaTime);      
       else
          FlightController(a_deltaTime);
    }
}
void UQuadDroneController::GamepadController(double DeltaTime)
{
	if (!dronePawn) return;
	const FGamepadInputs& GP = dronePawn->GamepadInputs;

	/* ---------------- state ---------------- */
	const FVector  currPos = dronePawn->GetActorLocation();
	const FVector  currVel = dronePawn->GetVelocity();
	const FRotator currRot = dronePawn->GetActorRotation();
	const FRotator yawOnlyRot(0.f, currRot.Yaw, 0.f);
	FVector localVel = yawOnlyRot.UnrotateVector(currVel);

	const FVector worldAngDeg = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	localAngularRateDeg = yawOnlyRot.UnrotateVector(worldAngDeg);

	/* ---------------- altitude / throttle ---------------- */
	hoverTargetAltitude += GP.Throttle * 150.f * DeltaTime;          // ±1 m s-¹

	double altVelSetpoint = AltitudePID->Calculate(hoverTargetAltitude, currPos.Z, DeltaTime);
	double zEffort = PIDSet.ZPID->Calculate(altVelSetpoint, localVel.Z, DeltaTime);

	/* ---------------- desired attitudes from sticks ---------------- */
	desiredRoll       =  GP.Roll   *  maxAngle;     // right-stick X
	desiredPitch      = -GP.Pitch  *  maxAngle;     // right-stick Y (invert)
	desiredYawRate    =  GP.Yaw    *  maxYawRate;   // left-stick X

	/* ---------------- PID cascades (same gains as normal) ---------- */
	const double rollOut  = PIDSet.RollPID ->Calculate(desiredRoll,  currRot.Roll,      DeltaTime);
	const double pitchOut = PIDSet.PitchPID->Calculate(desiredPitch, -currRot.Pitch,    DeltaTime);
	const float  yawOut   = YawRateControl(DeltaTime);   // uses desiredYawRate

	/* no XY position hold in this simple ANGLE mode */
	ThrustMixer(desiredRoll, desiredPitch, zEffort,
				rollOut,   pitchOut,      yawOut);

	/* optional HUD / debug */
	DrawDebugVisualsVel(FVector::ZeroVector);
	DrawMagneticDebugVisuals();
	if (dronePawn && dronePawn->ImGuiUtil)
	{
		// Display HUD only for the selected drone (independent) or all (swarm)
		bool bShowUI = true;
		if (ADroneManager* Manager = ADroneManager::Get(dronePawn->GetWorld()))
		{
			if (!Manager->IsSwarmMode())
			{
				const int32 myIdx = Manager->GetDroneIndex(dronePawn);
				bShowUI = (myIdx == Manager->SelectedDroneIndex);
			}
		}

		if (bShowUI){dronePawn->ImGuiUtil->ImGuiHud(currentFlightMode,DeltaTime);}
	}
}
void UQuadDroneController::FlightController(double DeltaTime)
{
	FFullPIDSet* CurrentSet = &PIDSet;
	if (!CurrentSet || !dronePawn) return;
	
	/* ───── World-space state ───── */
	FVector GPSData = dronePawn->SensorManager->GPS->GetLastGPS();
	float Altitude = dronePawn->SensorManager->Barometer->GetEstimatedAltitude();
	const FVector  currPos = {GPSData.X, GPSData.Y, Altitude};     
	const FVector  currVel = dronePawn->SensorManager->IMU->GetLastVelocity();          
	const FRotator currRot = dronePawn->SensorManager->IMU->GetLastAttitude();
	const FRotator yawOnlyRot(0.f, currRot.Yaw, 0.f);
	
	// Get world angular velocity and transform it to local frame using yaw-only rotation
	const FVector worldAngularRateDeg = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	localAngularRateDeg = yawOnlyRot.UnrotateVector(worldAngularRateDeg);

	if (bUseExternalController)
	{
        
		DrawDebugVisualsVel(FVector::ZeroVector);
		// Show UI if needed
		if (dronePawn && dronePawn->ImGuiUtil)
		{
			bool bShowUI = true;
			if (ADroneManager* Manager = ADroneManager::Get(dronePawn->GetWorld()))
			{
				if (!Manager->IsSwarmMode())
				{
					const int32 myIdx = Manager->GetDroneIndex(dronePawn);
					bShowUI = (myIdx == Manager->SelectedDroneIndex);
				}
			}
			if (bShowUI) { dronePawn->ImGuiUtil->ImGuiHud(currentFlightMode, DeltaTime); }
		}
        
		return; 
	}
	
	/*-------- Position P Control -------- */

	FVector desiredLocalVelocity;
	switch (currentFlightMode)
	{
	case EFlightMode::AutoWaypoint:
		{
			// ───── Update / fetch next set-point ─────
			if (UNavigationComponent* Nav = dronePawn->FindComponentByClass<UNavigationComponent>())
			{
				Nav->UpdateNavigation(currPos);
				setPoint =(Nav->GetCurrentSetpoint())/100;
			}
			DrawDebugVisuals(currPos);
			const FVector posErr = setPoint - currPos;
			const FVector localPosErr = yawOnlyRot.UnrotateVector(posErr);
			desiredLocalVelocity = localPosErr.GetSafeNormal()*maxVelocity;
			break;
		}
	case EFlightMode::VelocityControl:
		{
			desiredLocalVelocity = desiredNewVelocity;
			DrawMagneticDebugVisuals();

			break;
		}
	case EFlightMode::AngleControl:
	case EFlightMode::RateControl:
		{
			desiredLocalVelocity = FVector(0.0f, 0.0f, desiredNewVelocity.Z);
			break;
		}
	default:
		desiredLocalVelocity = FVector::ZeroVector;
		break;
	}
	/*-------- Hover Move -------- */
	if (bHoverModeActive)
	{
		desiredLocalVelocity.Z = AltitudePID->Calculate(hoverTargetAltitude,currPos.Z, DeltaTime);
		desiredLocalVelocity.Z = FMath::Clamp(desiredLocalVelocity.Z, -1.f, 1.f);
	}
	
	/*-------- Velocity PID Control (FLU) -------- */ 
	currentLocalVelocity = currVel;
	
	const double xOut = CurrentSet->XPID -> Calculate(desiredLocalVelocity.X,currentLocalVelocity.X, DeltaTime);
	const double yOut = CurrentSet->YPID -> Calculate(desiredLocalVelocity.Y,currentLocalVelocity.Y, DeltaTime);
	const double zOut = CurrentSet->ZPID -> Calculate(desiredLocalVelocity.Z,currentLocalVelocity.Z, DeltaTime)*100;
	
	/*-------- Angle P Control -------- */ 
	desiredPitch = (currentFlightMode == EFlightMode::AngleControl) ? desiredNewPitch: FMath::Clamp( xOut, -maxAngle,  maxAngle);
	desiredRoll  = (currentFlightMode == EFlightMode::AngleControl) ? desiredNewRoll: FMath::Clamp( yOut, -maxAngle,  maxAngle);
	
	const double rollOut  = CurrentSet ->RollPID->Calculate(desiredRoll,currRot.Roll , DeltaTime);
	const double pitchOut = CurrentSet ->PitchPID->Calculate(desiredPitch,-currRot.Pitch, DeltaTime);
	// UE_LOG(LogTemp, Warning, TEXT("Roll Output: %f deg/s"), rollOut);
	// UE_LOG(LogTemp, Warning, TEXT("Pitch Output: %f deg/s"), pitchOut);

	/*-------- Angle Rate PID Control -------- */ 

	// desiredPitchRate = (currentFlightMode == EFlightMode::RateControl) ? desiredNewPitchRate: FMath::Clamp(pitchOut, -maxAngleRate, maxAngleRate); // Implement switch here for angle control using DesiredNewPitchRate
	// desiredRollRate = (currentFlightMode == EFlightMode::RateControl) ? desiredNewRollRate: FMath::Clamp(rollOut, -maxAngleRate,  maxAngleRate); // Implement switch here for angle control 
	//
	// UE_LOG(LogTemp, Warning, TEXT("Desired Pitch Rate: %f deg/s"), desiredPitchRate);
	// UE_LOG(LogTemp, Warning, TEXT("Desired Roll Rate: %f deg/s"), desiredRollRate);
	// UE_LOG(LogTemp, Warning, TEXT("Current Pitch Angular Rate: %f deg/s"), localAngularRateDeg.Y);
	// UE_LOG(LogTemp, Warning, TEXT("Current Roll Angular Rate: %f deg/s"), localAngularRateDeg.X);
	//
	// const double rollRateOut  = CurrentSet->RollRatePID->Calculate(desiredRollRate,localAngularRateDeg.X,  DeltaTime);
	// const double pitchRateOut = CurrentSet->PitchRatePID->Calculate(desiredPitchRate,localAngularRateDeg.Y, DeltaTime);

	const float yawOutput = YawRateControl(DeltaTime);

	//  Mix & apply motor thrusts / torques
	ThrustMixer(xOut, yOut, zOut, rollOut, pitchOut, yawOutput);
	
	//  Debug drawing and on‑screen HUD (optional)
	DrawDebugVisualsVel(FVector(desiredLocalVelocity.X, desiredLocalVelocity.Y, 0.f));

	if (dronePawn && dronePawn->ImGuiUtil)
	{
		bool bShowUI = true;
		if (ADroneManager* Manager = ADroneManager::Get(dronePawn->GetWorld()))
		{
			if (!Manager->IsSwarmMode())
			{
				const int32 myIdx = Manager->GetDroneIndex(dronePawn);
				bShowUI = (myIdx == Manager->SelectedDroneIndex);
			}
		}

		if (bShowUI){dronePawn->ImGuiUtil->ImGuiHud(currentFlightMode,DeltaTime);}
	}
	
}

// ---------------------- Thrust Functions ------------------------

void UQuadDroneController::ThrustMixer(double xOut, double yOut, double zOut, double rollOutput, double pitchOutput, double yawOutput)
{
	float droneMass = dronePawn->DroneBody->GetMass();
	const float gravity = 980.f	;
	const float hoverThrust = (droneMass * gravity) / 4.0f; 
 
	float baseThrust = hoverThrust + zOut / 4.0f;
	baseThrust /= FMath::Cos(FMath::DegreesToRadians(FMath::Sqrt(FMath::Pow(xOut, 2) + FMath::Pow(yOut, 2))));
 
	Thrusts[0] = baseThrust - pitchOutput + rollOutput - yawOutput; //FL
	Thrusts[1] = baseThrust - pitchOutput - rollOutput + yawOutput; //FR
	Thrusts[2] = baseThrust + pitchOutput + rollOutput + yawOutput; //BL
	Thrusts[3] = baseThrust + pitchOutput - rollOutput - yawOutput; //BR
	
	// Paper is FR, BL, FL, BR
	for (int i = 0; i < Thrusts.Num(); i++)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, maxThrust);
		
		if (!dronePawn || !dronePawn->Thrusters.IsValidIndex(i))
			continue;
		double force = Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(force);
	}
	double YawTorqueForce = 2.0f;
	FVector upVector = dronePawn->GetActorUpVector();
	FVector virtualTorque = upVector * yawOutput * YawTorqueForce;
	for (UThrusterComponent* Thruster : dronePawn->Thrusters)
	{
		if (Thruster)
		{
			Thruster->ApplyTorque(virtualTorque, true);
		}
	}

}

float  UQuadDroneController::YawRateControl(double DeltaTime)
{
	if (!dronePawn || !dronePawn->DroneBody) return 0.0f;
	
	FFullPIDSet* CurrentSet = &PIDSet;
	if (!CurrentSet) return 0.0f;
	
	FVector currentAngularVelocity = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	float currentYawRate = currentAngularVelocity.Z;

	float currentdesiredYawRate = desiredYawRate;
	float yawTorqueFeedback = CurrentSet->YawRatePID->Calculate(currentdesiredYawRate,currentYawRate, DeltaTime);
	return yawTorqueFeedback;
}
// ------ Reset Functions ------------------------
void UQuadDroneController::ResetPID()
{
	PIDSet.XPID      ->Reset();
	PIDSet.YPID      ->Reset();
	PIDSet.ZPID      ->Reset();
	PIDSet.RollPID   ->Reset();
	PIDSet.PitchPID  ->Reset();
	PIDSet.RollRatePID ->Reset();
	PIDSet.PitchRatePID->Reset();
	PIDSet.YawRatePID  ->Reset();
}
void UQuadDroneController::ResetDroneIntegral()
{
	FFullPIDSet* CurrentSet = &PIDSet;
	if (!CurrentSet)
	{
		UE_LOG(LogTemp, Warning, TEXT("ResetDroneIntegral: No PID set found for current flight mode %d"), (int32)currentFlightMode);
		return;
	}

	CurrentSet->XPID->ResetIntegral();
	CurrentSet->YPID->ResetIntegral();
	CurrentSet->ZPID->ResetIntegral();
	CurrentSet->RollPID->ResetIntegral();
	CurrentSet->PitchPID->ResetIntegral();
	CurrentSet->RollRatePID->ResetIntegral();
	CurrentSet->PitchRatePID->ResetIntegral();
	CurrentSet->YawRatePID->ResetIntegral();
}
void UQuadDroneController::ResetDroneRotation()
{
	if (dronePawn)
	{
		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(false);
		}

		dronePawn->SetActorRotation(FRotator::ZeroRotator);

		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(true);
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			dronePawn->DroneBody->WakeAllRigidBodies();
		}

		// Reset controller states
		ResetPID();
		desiredNewVelocity = FVector::ZeroVector;
	}
}
void UQuadDroneController::ResetDroneOrigin()
{
	if (dronePawn)
	{
		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(false);
		}

		dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 0.0f), false, nullptr, ETeleportType::TeleportPhysics);
		dronePawn->SetActorRotation(FRotator::ZeroRotator);

		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(true);
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			dronePawn->DroneBody->WakeAllRigidBodies();
		}

		ResetPID();
		desiredNewVelocity = FVector::ZeroVector;
	}
}
// ------------ Setter and Getter -------------------
void UQuadDroneController::SetHoverMode(bool bActive, float TargetAltitude)
{
	if (bActive && bHoverModeActive && dronePawn && TargetAltitude != hoverTargetAltitude)
	{
		hoverTargetAltitude = TargetAltitude;
		UE_LOG(LogTemp, Display, TEXT("Hover mode target altitude updated to: %.2f"), hoverTargetAltitude);
	}
	else if (bActive && !bHoverModeActive && dronePawn)
	{
		bHoverModeActive = true;
		hoverTargetAltitude = dronePawn->GetActorLocation().Z;
		AltitudePID->Reset();
		UE_LOG(LogTemp, Display, TEXT("Hover mode activated - Target altitude: %.2f"), hoverTargetAltitude);
	}
	else if (!bActive && bHoverModeActive)
	{
		bHoverModeActive = false;
		UE_LOG(LogTemp, Display, TEXT("Hover mode deactivated"));
	}
}
void UQuadDroneController::SetDestination(FVector desiredSetPoints) {
	setPoint = desiredSetPoints;
}
// ---------------------- Helper Functions -----------------------
void UQuadDroneController::DrawDebugVisuals(const FVector& currentPosition) const
{
   // Draw only a line connecting the current position to the setpoint (no spheres).
   DrawDebugLine(
       dronePawn->GetWorld(),
       currentPosition,
       setPoint,
       FColor::Green,
       /*bPersistent=*/false,
       /*LifeTime=*/0.0f
   );
}
 void UQuadDroneController::DrawDebugVisualsVel(const FVector& horizontalVelocity) const
 {
 	if (!bDebugVisualsEnabled || !dronePawn || !dronePawn->DroneBody) return;

	FVector GPSData = dronePawn->SensorManager->GPS->GetLastGPS();
	float Altitude = dronePawn->SensorManager->Barometer->GetEstimatedAltitude()*100;
	const FVector  dronePos = {GPSData.X, GPSData.Y, Altitude};
	const float scaleXYZ = 0.5f;

 	// Velocity debug lines
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(desiredNewVelocity.X, 0, 0) * scaleXYZ, FColor::Red, false, -1.0f, 0, 2.0f);
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, desiredNewVelocity.Y, 0) * scaleXYZ, FColor::Green, false, -1.0f, 0, 2.0f);
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, 0, desiredNewVelocity.Z) * scaleXYZ, FColor::Blue, false, -1.0f, 0, 2.0f);
	
 	// Motor labels
 	for (int i = 0; i < dronePawn->Thrusters.Num(); i++) {
 		FVector MotorPos = dronePawn->Thrusters[i]->GetComponentLocation();
 		FString DirText = dronePawn->MotorClockwiseDirections[i] ? TEXT("CW") : TEXT("CCW");
 		DrawDebugString(GetWorld(), MotorPos + FVector(0, 0, 15),
 			FString::Printf(TEXT("M%d\n%s"), i, *DirText),
 			nullptr, FColor::White, 0.0f, true, 1.2f);
 	}
 }
float UQuadDroneController::GetCurrentThrustOutput(int32 ThrusterIndex) const
{
	if (Thrusts.IsValidIndex(ThrusterIndex))
	{
		return Thrusts[ThrusterIndex];
	}
	return 0.0f;
}
void UQuadDroneController::SetFlightMode(EFlightMode NewMode)
{
	switch (NewMode)
	{
	case EFlightMode::AngleControl:
	case EFlightMode::JoyStickAngleControl:
	case EFlightMode::JoyStickAcroControl:
		hoverTargetAltitude = dronePawn ? dronePawn->GetActorLocation().Z : 0.f;
		AltitudePID->Reset();                       // forget old integral wind-up
		break;

	default:
		break;
	}

	FVector GPSData = dronePawn->SensorManager->GPS->GetLastGPS();
	float Altitude = dronePawn->SensorManager->Barometer->GetEstimatedAltitude()*100;
	const FVector  currPos = {GPSData.X, GPSData.Y, Altitude};     
    currentFlightMode = NewMode;
    // On selecting AutoWaypoint, generate and load the figure-8 navigation plan
    if (NewMode == EFlightMode::AutoWaypoint && dronePawn)
    {
        // Generate waypoints from pawn's position
        TArray<FVector> plan = dronePawn->GenerateFigureEightWaypoints(currPos);
        // Set navigation plan
        if (UNavigationComponent* Nav = dronePawn->FindComponentByClass<UNavigationComponent>())
        {
            Nav->SetNavigationPlan(plan);
        }
        // Debug draw the path: spheres and connecting lines
        UWorld* World = dronePawn->GetWorld();
        if (World)
        {
            // Persistent debug: sample path sparsely for performance
            const float SphereSize = 50.0f;
            const int32 debugStep = 5;
            const bool bPersistent = true;
            const float LifeTime = 0.0f;
            for (int32 i = 0; i < plan.Num(); i += debugStep)
            {
                // Draw only connecting lines for auto-waypoint path (no spheres)
                int32 nextIdx = i + debugStep;
                if (nextIdx < plan.Num())
                {
                    DrawDebugLine(World,
                        plan[i],
                        plan[nextIdx],
                        FColor::Green,
                        /*bPersistent=*/true,
                        /*LifeTime=*/0.0f,
                        /*DepthPriority=*/0,
                        /*Thickness=*/5.0f);
                }
            }
        }
    }
}
//=========================== PX4 Implementation =========================== //

void UQuadDroneController::ApplyMotorCommands(const TArray<float>& MotorCommands)
{
    float DroneMass = dronePawn->GetMass();
    const float Gravity = 980.0f; // cm/s^2 in Unreal units
	
    float TotalHoverThrust = DroneMass * Gravity; // Total thrust needed to hover in centiNewtons
    float HoverThrustPerMotor = TotalHoverThrust / 4.0f;
    
    // Maximum thrust per motor (hover thrust * some factor, e.g., 2x for good control authority)
    const float MaxThrustPerMotor = HoverThrustPerMotor * 2.5f;
    
    // Debug logging
    static int32 LogCounter = 0;
    bool bShouldLog = (LogCounter++ % 50 == 0); // Log every 50 calls
    
    if (bShouldLog)
    {
        UE_LOG(LogTemp, Warning, TEXT("ApplyMotorCommands: DroneMass=%.2f kg, HoverThrustPerMotor=%.2f cN, Commands=[%.3f, %.3f, %.3f, %.3f]"),
               DroneMass, HoverThrustPerMotor, 
               MotorCommands.IsValidIndex(0) ? MotorCommands[0] : 0.0f,
               MotorCommands.IsValidIndex(1) ? MotorCommands[1] : 0.0f,
               MotorCommands.IsValidIndex(2) ? MotorCommands[2] : 0.0f,
               MotorCommands.IsValidIndex(3) ? MotorCommands[3] : 0.0f);
    }

	
    for (int32 i = 0; i < FMath::Min(MotorCommands.Num(), 4); i++)
    {
        if (dronePawn->Thrusters.IsValidIndex(i) && dronePawn->Thrusters[i])
        {
            float Command = FMath::Clamp(MotorCommands[i], 0.0f, 1.0f);
            // Linear mapping instead of quadratic for better response
        	float ThrustForce = MaxThrustPerMotor * Command;
            dronePawn->Thrusters[i]->ApplyForce(ThrustForce*100);
            
            if (bShouldLog)
            {
                UE_LOG(LogTemp, Warning, TEXT("Motor %d: Command=%.3f, ThrustForce=%.2f cN"), 
                       i, Command, ThrustForce);
            }
        }
    }
}

void UQuadDroneController::SetUseExternalController(bool bUseExternal)
{
    if (bUseExternalController != bUseExternal)
    {
        bUseExternalController = bUseExternal;
        
        if (bUseExternal)
        {
            UE_LOG(LogTemp, Warning, TEXT("Switched to external controller mode (PX4)"));
            // Reset PIDs to avoid windup when switching back
            ResetPID();
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Switched to internal controller mode"));
            // Reset PIDs when switching back to internal control
            ResetPID();
        }
    }
}



void UQuadDroneController::DrawMagneticDebugVisuals()
{
    if (!dronePawn || !bDebugVisualsEnabled) return;
    
    FVector DronePos = dronePawn->GetActorLocation();
    
    // Get georeferencing system
    AGeoReferencingSystem* GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(GetWorld());
    if (!GeoRef) return;
    
    // Get geographic coordinates for declination
    FGeographicCoordinates GeoCoords;
    GeoRef->EngineToGeographic(DronePos, GeoCoords);
    
    // Get magnetic declination at this location
    float Declination = FGeoMagDeclination::GetMagDeclinationDegrees(GeoCoords.Latitude, GeoCoords.Longitude);
    
    // Get ENU vectors (East, North, Up) in Unreal coordinates
    FVector EastVector, NorthVector, UpVector;
    GeoRef->GetENUVectorsAtGeographicLocation(GeoCoords, EastVector, NorthVector, UpVector);
    
    // Draw True North (Green arrow)
    FVector TrueNorthEnd = DronePos + NorthVector * 500.0f; // 5 meter arrow
    DrawDebugDirectionalArrow(GetWorld(), DronePos, TrueNorthEnd, 100.0f, FColor::Green, false, -1.0f, 0, 5.0f);
    DrawDebugString(GetWorld(), TrueNorthEnd + UpVector * 50.0f, TEXT("TRUE NORTH"), nullptr, FColor::Green, 0.0f, true, 1.5f);
    
    // Draw Magnetic North (Red arrow) - rotated by declination
    FQuat DeclinationRot = FQuat(UpVector, FMath::DegreesToRadians(Declination));
    FVector MagNorthVector = DeclinationRot.RotateVector(NorthVector);
    FVector MagNorthEnd = DronePos + MagNorthVector * 500.0f;
    DrawDebugDirectionalArrow(GetWorld(), DronePos, MagNorthEnd, 100.0f, FColor::Red, false, -1.0f, 0, 5.0f);
    DrawDebugString(GetWorld(), MagNorthEnd + UpVector * 50.0f, 
                    FString::Printf(TEXT("MAG NORTH (%.1f°)"), Declination), 
                    nullptr, FColor::Red, 0.0f, true, 1.5f);
    
    // Draw drone heading (Blue arrow)
    FVector DroneForward = dronePawn->GetActorForwardVector();
    FVector DroneHeadingEnd = DronePos + DroneForward * 400.0f;
    DrawDebugDirectionalArrow(GetWorld(), DronePos, DroneHeadingEnd, 80.0f, FColor::Blue, false, -1.0f, 0, 4.0f);
    
    // Calculate and display heading relative to true north
    float HeadingToTrueNorth = FMath::RadiansToDegrees(FMath::Atan2(
        FVector::DotProduct(DroneForward, EastVector),
        FVector::DotProduct(DroneForward, NorthVector)
    ));
    
    DrawDebugString(GetWorld(), DroneHeadingEnd + UpVector * 50.0f, 
                    FString::Printf(TEXT("DRONE (%.1f°)"), HeadingToTrueNorth), 
                    nullptr, FColor::Blue, 0.0f, true, 1.5f);
    
    // Draw magnetic field vector (Purple)
    FVector MagField = dronePawn->SensorManager->Magnetometer->GetLastMagField();
    FVector MagFieldWorld = dronePawn->GetActorRotation().RotateVector(MagField);
    FVector MagFieldScaled = MagFieldWorld * 1000.0f; // Scale for visibility
    DrawDebugLine(GetWorld(), DronePos, DronePos + MagFieldScaled, FColor::Purple, false, -1.0f, 0, 3.0f);
    DrawDebugString(GetWorld(), DronePos + MagFieldScaled + UpVector * 30.0f, 
                    FString::Printf(TEXT("MAG FIELD (%.2f G)"), MagField.Size()), 
                    nullptr, FColor::Purple, 0.0f, true, 1.2f);
	float MagneticHeading = FMath::RadiansToDegrees(FMath::Atan2(MagField.Y, MagField.X));
    // Draw compass rose on ground
    float CompassRadius = 300.0f;
    DrawDebugCircle(GetWorld(), DronePos, CompassRadius, 64, FColor::White, false, -1.0f, 0, 2.0f, 
                    FVector(0,0,1), FVector(1,0,0), false);
	FVector TextPos = DronePos + UpVector * 200.0f;
	DrawDebugString(GetWorld(), TextPos, 
		FString::Printf(TEXT("True Heading: %.1f°"), HeadingToTrueNorth), 
		nullptr, FColor::Green, 0.0f, true, 1.5f);
    
	DrawDebugString(GetWorld(), TextPos - UpVector * 30.0f, 
		FString::Printf(TEXT("Magnetic Heading: %.1f°"), HeadingToTrueNorth + Declination), 
		nullptr, FColor::Red, 0.0f, true, 1.5f);
    
	DrawDebugString(GetWorld(), TextPos - UpVector * 60.0f, 
		FString::Printf(TEXT("Body Mag X: %.3f Y: %.3f Z: %.3f"), 
		MagField.X, MagField.Y, MagField.Z), 
		nullptr, FColor::Purple, 0.0f, true, 1.5f);
    // Draw cardinal directions
    DrawDebugString(GetWorld(), DronePos + NorthVector * (CompassRadius + 50.0f), TEXT("N"), 
                    nullptr, FColor::White, 0.0f, true, 2.0f);
    DrawDebugString(GetWorld(), DronePos + EastVector * (CompassRadius + 50.0f), TEXT("E"), 
                    nullptr, FColor::White, 0.0f, true, 2.0f);
    DrawDebugString(GetWorld(), DronePos - NorthVector * (CompassRadius + 50.0f), TEXT("S"), 
                    nullptr, FColor::White, 0.0f, true, 2.0f);
    DrawDebugString(GetWorld(), DronePos - EastVector * (CompassRadius + 50.0f), TEXT("W"), 
                    nullptr, FColor::White, 0.0f, true, 2.0f);
}

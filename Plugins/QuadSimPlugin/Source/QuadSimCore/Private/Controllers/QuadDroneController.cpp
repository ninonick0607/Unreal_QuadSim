// QuadDroneController.cpp

#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "DrawDebugHelpers.h"
#include <random>

#ifndef EXCLUDE_PX4_COMPONENT
#include "Controllers/PX4Component.h"
#endif
#include "GeographicCoordinates.h"
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


//=========================== Constructor =========================== //


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
	ControllerSet.RollPID->SetGains(0.9f, 0.2f, 0.34f);

	ControllerSet.PitchPID = new QuadPIDController();
	ControllerSet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.PitchPID->SetGains(0.9f, 0.2f, 0.34f);

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

//=========================== Initialization =========================== //

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

//=========================== Update =========================== //

void UQuadDroneController::Update(const FSensorData& SensorData, float DeltaTime)
{
	// Store sensor data for use throughout the controller
	LastSensorData = SensorData;
	bHasValidSensorData = true;
    
	// Early exit if critical sensors are invalid
	if (!SensorData.bIMUValid || !SensorData.bGPSValid)
	{
		UE_LOG(LogTemp, Warning, TEXT("Controller: Missing critical sensor data (IMU:%d, GPS:%d)"),
			   SensorData.bIMUValid, SensorData.bGPSValid);
		return;
	}

	ADroneManager* Manager = ADroneManager::Get(dronePawn ? dronePawn->GetWorld() : nullptr);
	bool bShowUI = true;
	if (Manager)
	{
		if (!Manager->IsSwarmMode())
		{
            APlayerController* PC = UGameplayStatics::GetPlayerController(dronePawn ? dronePawn->GetWorld() : nullptr, 0);
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
			GamepadController(SensorData,DeltaTime);      
		else
			FlightController(SensorData,DeltaTime);
	}
	
}
void UQuadDroneController::GamepadController(const FSensorData& SensorData,double DeltaTime)
{
	if (!dronePawn) return;
	const FGamepadInputs& GP = dronePawn->GamepadInputs;

	/* ---------------- state ---------------- */
	FVector GPSData = SensorData.GPSPosMeters;
	float Altitude = SensorData.BaroAltitudeM;
	const FVector  currPos = {GPSData.X, GPSData.Y, Altitude};     
	const FVector  currVel = SensorData.IMUVelMS;          
	const FRotator currRot = SensorData.IMUAttitude;
	const FRotator yawOnlyRot(0.f, currRot.Yaw, 0.f);
	FVector localVel = yawOnlyRot.UnrotateVector(currVel);

	const FVector worldAngDeg = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	localAngularRateDeg = yawOnlyRot.UnrotateVector(worldAngDeg);

	/* ---------------- altitude / throttle ---------------- */
	hoverTargetAltitude += GP.Throttle * 150.f * DeltaTime;          // Â±1 m s-Â¹

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
}
void UQuadDroneController::FlightController(const FSensorData& SensorData,double DeltaTime)
{
	FFullPIDSet* CurrentSet = &PIDSet;
	if (!CurrentSet || !dronePawn) return;
	
	/* â”€â”€â”€â”€â”€ World-space state â”€â”€â”€â”€â”€ */
	FVector GPSData = SensorData.GPSPosMeters;
	float Altitude = SensorData.BaroAltitudeM;
	const FVector  currPos = {GPSData.X, GPSData.Y, Altitude};     
	const FVector  currVel = SensorData.IMUVelMS;          
	const FRotator currRot = SensorData.IMUAttitude;
	const FRotator yawOnlyRot(0.f, currRot.Yaw, 0.f);
	
	// Get world angular velocity and transform it to local frame using yaw-only rotation
	const FVector worldAngularRateDeg = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	localAngularRateDeg = yawOnlyRot.UnrotateVector(worldAngularRateDeg);

	if (bUseExternalController)
	{
        
		DrawDebugVisualsVel(FVector::ZeroVector);
        
		return; 
	}
	
	/*-------- Position P Control -------- */

	FVector desiredLocalVelocity;
    switch (currentFlightMode)
    {
    case EFlightMode::AutoWaypoint:
        {
            // â”€â”€â”€â”€â”€ Update / fetch next set-point â”€â”€â”€â”€â”€
            if (UNavigationComponent* Nav = dronePawn->FindComponentByClass<UNavigationComponent>())
            {
                // Navigation operates in meters; pass meters and receive meters
                Nav->UpdateNavigation(currPos);
                setPoint = Nav->GetCurrentSetpoint();
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

    //  Mix & apply motor thrusts / torques (use commanded tilt for compensation)
    ThrustMixer(xOut, yOut, zOut, rollOut, pitchOut, yawOutput);
	//  Debug drawing and on-screen HUD (optional)
	DrawDebugVisualsVel(FVector(desiredLocalVelocity.X, desiredLocalVelocity.Y, 0.f));
	
	// Nav path debug: draw polyline always; spheres only in manual path mode
	if (dronePawn)
	{
		if (UNavigationComponent* Nav = dronePawn->FindComponentByClass<UNavigationComponent>())
		{
			const TArray<FVector> Wps = Nav->GetWaypoints(); // meters
			UWorld* World = dronePawn->GetWorld();
			if (World && Wps.Num() > 0)
			{
				for (int32 i = 0; i + 1 < Wps.Num(); ++i)
				{
					DrawDebugLine(World, Wps[i]*100.0f, Wps[i+1]*100.0f, FColor::Green, false, -1.0f, 0, 3.0f);
				}
				if (bManualPathMode)
				{
					for (const FVector& P : Wps)
					{
						DrawDebugSphere(World, P*100.0f, 20.0f, 16, FColor::Green, false, -1.0f, 0, 2.0f);
					}
				}
			}
		}
	}
	//  Debug drawing and onâ€‘screen HUD (optional)
	DrawDebugVisualsVel(FVector(desiredLocalVelocity.X, desiredLocalVelocity.Y, 0.f));
}

//=========================== Thrusts =========================== //

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

//=========================== Reset =========================== //

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

void UQuadDroneController::ResetControllerState()
{
    // Clear desired commands
    desiredNewVelocity = FVector::ZeroVector;
    desiredYawRate = 0.0f;
    desiredRollRate = 0.0;
    desiredPitchRate = 0.0;
    desiredNewRollRate = 0.0;
    desiredNewPitchRate = 0.0;
    desiredRoll = 0.0f;
    desiredPitch = 0.0f;
    desiredNewRoll = 0.0;
    desiredNewPitch = 0.0;

    // Clear outputs and integrators
    ResetPID();
    ResetDroneIntegral();

    // Mode and flags
    bHoverModeActive = false;
    hoverTargetAltitude = 0.0f;
    bUseExternalController = false;
    bGamepadModeUI = false;
    currentFlightMode = EFlightMode::None;

    // Clear thrusts cache for HUD
    if (Thrusts.Num() == 4)
    {
        Thrusts[0] = Thrusts[1] = Thrusts[2] = Thrusts[3] = 0.0f;
    }
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

//=========================== Hover and Nav =========================== //
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
		// Initialize hover target to current altitude in meters
		if (dronePawn->SensorManager && dronePawn->SensorManager->AreSensorsInitialized())
		{
			hoverTargetAltitude = dronePawn->GetActorLocation().Z/100;
		}
		else
		{
			hoverTargetAltitude = dronePawn->GetActorLocation().Z / 100.0f; // cm -> m
		}
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

//=========================== Helper Functions =========================== //
void UQuadDroneController::DrawDebugVisuals(const FVector& currentPosition) const
{
   // Draw only a line connecting the current position to the setpoint (convert m â†’ cm for world debug)
   DrawDebugLine(
       dronePawn->GetWorld(),
       currentPosition * 100.0f,
       setPoint * 100.0f,
       FColor::Green,
       /*bPersistent=*/false,
       /*LifeTime=*/0.0f
   );
}
void UQuadDroneController::DrawDebugVisualsVel(const FVector& horizontalVelocity) const
 {
	if (!bDebugVisualsEnabled || !dronePawn || !dronePawn->DroneBody) return;

    FVector dronePos;
    if (dronePawn->SensorManager && dronePawn->SensorManager->AreSensorsInitialized())
    {
        const FSensorData Data = dronePawn->SensorManager->GetCurrentSensorData();
        const float zCm = (Data.bBaroValid ? Data.BaroAltitudeM : Data.GPSPosMeters.Z) * 100.0f;
        dronePos = FVector(Data.GPSPosMeters.X * 100.0f,
                           Data.GPSPosMeters.Y * 100.0f,
                           zCm);
    }
    else
    {
        dronePos = dronePawn->GetActorLocation();
    }
	const float scaleXYZ = 0.5f;

 	// Velocity debug lines
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(desiredNewVelocity.X, 0, 0) * scaleXYZ, FColor::Red, false, -1.0f, 0, 2.0f);
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, desiredNewVelocity.Y, 0) * scaleXYZ, FColor::Green, false, -1.0f, 0, 2.0f);
 	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, 0, desiredNewVelocity.Z) * scaleXYZ, FColor::Blue, false, -1.0f, 0, 2.0f);
	
 	// Motor labels
 	for (int i = 0; i < dronePawn->Thrusters.Num(); i++) {
 		FVector MotorPos = dronePawn->Thrusters[i]->GetComponentLocation();
 		FString DirText = dronePawn->MotorClockwiseDirections[i] ? TEXT("CW") : TEXT("CCW");
 		DrawDebugString(dronePawn->GetWorld(), MotorPos + FVector(0, 0, 15),
 			FString::Printf(TEXT("M%d\n%s"), i, *DirText),
 			nullptr, FColor::White, 0.0f, true, 1.2f);
 	}
 }
void UQuadDroneController::DrawMagneticDebugVisuals()
{
    if (!dronePawn || !bDebugVisualsEnabled) return;
    
    FVector DronePos = dronePawn->GetActorLocation();
    
    // Get georeferencing system
    AGeoReferencingSystem* GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(dronePawn->GetWorld());
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
                    FString::Printf(TEXT("MAG NORTH (%.1fÂ°)"), Declination), 
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
                    FString::Printf(TEXT("DRONE (%.1fÂ°)"), HeadingToTrueNorth), 
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
		FString::Printf(TEXT("True Heading: %.1fÂ°"), HeadingToTrueNorth), 
		nullptr, FColor::Green, 0.0f, true, 1.5f);
    
	DrawDebugString(GetWorld(), TextPos - UpVector * 30.0f, 
		FString::Printf(TEXT("Magnetic Heading: %.1fÂ°"), HeadingToTrueNorth + Declination), 
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

    // Position seed: actor location in meters (avoid sensor/world lookups here)
    const FVector currPos = dronePawn->GetActorLocation() / 100.0f;
    currentFlightMode = NewMode;

    // Enable direct gamepad control path for joystick-driven modes
    bGamepadModeUI = (NewMode == EFlightMode::JoyStickAngleControl ||
                      NewMode == EFlightMode::JoyStickAcroControl);
    // On selecting AutoWaypoint, generate and load the figure-8 navigation plan
    if (NewMode == EFlightMode::AutoWaypoint && dronePawn)
    {
        // Generate waypoints from pawn's position (meters)
        TArray<FVector> plan = dronePawn->GenerateFigureEightWaypoints(currPos);
        if (UNavigationComponent* Nav = dronePawn->FindComponentByClass<UNavigationComponent>())
        {
            Nav->SetNavigationPlan(plan);
        }
        if (plan.Num() > 0)
        {
            // Make the initial setpoint visible in the HUD immediately
            setPoint = plan[0];
        }
        // No debug drawing here to keep SetFlightMode side-effect free
    }
}

//=========================== PX4 Implementation =========================== //
void UQuadDroneController::ApplyMotorCommands(const TArray<float>& MotorCmd01)
{
    // Only accept PX4 motor commands when external controller mode is enabled via UI
    if (!dronePawn || !bUseExternalController) return;

    const float MassKg   = dronePawn->GetMass(); // kg
    // Use world gravity to stay consistent with your physics settings
    const float g_cm_s2  = FMath::Abs(GetWorld()->GetGravityZ()); // ~980 cm/s^2
    const float g_m_s2   = g_cm_s2 * 0.01f;                        // 9.8 m/s^2

    const float TotalHover_N       = MassKg * g_m_s2;      // N
    const float HoverPerMotor_N    = TotalHover_N / 4.0f;  // N
    const float MaxPerMotor_N      = HoverPerMotor_N * 2.0f;  // 2x hover is a good start

    // Optional: small slew-rate limit to avoid spikes from PX4 startup
    static float prevN[4] = {0,0,0,0};
    const float maxSlewN  = HoverPerMotor_N * 0.25f; // N per call (tune)

    // Debug every ~50 calls
    static int32 C=0; const bool bLog = ((C++ % 50) == 0);
    if (bLog)
        UE_LOG(LogTemp, Warning, TEXT("ApplyMotorCommands: Mass=%.2f kg, Hover/motor=%.2f N  cmds=[%.3f %.3f %.3f %.3f]"),
               MassKg, HoverPerMotor_N,
               MotorCmd01.IsValidIndex(0)?MotorCmd01[0]:0,
               MotorCmd01.IsValidIndex(1)?MotorCmd01[1]:0,
               MotorCmd01.IsValidIndex(2)?MotorCmd01[2]:0,
               MotorCmd01.IsValidIndex(3)?MotorCmd01[3]:0);

    for (int i=0; i<4; ++i)
    {
        if (!dronePawn->Thrusters.IsValidIndex(i) || !dronePawn->Thrusters[i]) continue;

        const float cmd01    = FMath::Clamp(MotorCmd01[i], 0.f, 1.f);

        // Map 0..1 â†’ 0..MaxPerMotor_N
        float thrustN = cmd01 * MaxPerMotor_N;

        // simple slew
        const float delta = FMath::Clamp(thrustN - prevN[i], -maxSlewN, maxSlewN);
        thrustN = prevN[i] + delta;
        prevN[i]= thrustN;

        // Convert Newtons â†’ Unreal force units (kg*cm/s^2). 1 N = 100 Unreal units
        const float unrealForce = thrustN * 100.0f;
        const float appliedForce = FMath::Clamp(unrealForce, 0.0f, maxThrust);

        // Update controller's Thrusts array so HUD/state data can display values
        if (Thrusts.IsValidIndex(i)) Thrusts[i] = appliedForce;

        // Apply to physics
        dronePawn->Thrusters[i]->ApplyForce(appliedForce);

        if (bLog)
            UE_LOG(LogTemp, Warning, TEXT("Motor %d: cmd=%.3f -> %.2f N"), i, cmd01, thrustN);
    }
}

//=========================== ROS2 Implementation =========================== //
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

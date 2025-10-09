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
	ControllerSet.RollRatePID->SetGains(0.0035f, 0.0016f, 0.0025f);
	
	ControllerSet.PitchRatePID = new QuadPIDController();
	ControllerSet.PitchRatePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	ControllerSet.PitchRatePID->SetGains(0.0035f, 0.0016f, 0.0025f);
	
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
    // Reset per-frame debug draw guards
    bDrewMagDebugThisFrame = false;
    
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
	float Altitude = SensorData.BaroAltitudeM;
	const FVector  currVel = SensorData.IMUVelMS;          
	const FRotator currRot = SensorData.IMUAttitude;
    const FVector imuRateDeg = FMath::RadiansToDegrees(SensorData.IMUAngVelRADS);
    localAngularRateDeg = imuRateDeg;
	
	/* ---------------- altitude / throttle ---------------- */
	hoverTargetAltitude += GP.Throttle * 1.5f * DeltaTime;          // �1 m s-�
	UE_LOG(LogTemp, Warning, TEXT("Hover Output: %f m"), hoverTargetAltitude);

	double altVelSetpoint = AltitudePID->Calculate(hoverTargetAltitude, Altitude, DeltaTime);
	double zEffort = PIDSet.ZPID->Calculate(altVelSetpoint, currVel.Z, DeltaTime);

	desiredPitch      =  -GP.Pitch  *  maxAngle;
	desiredRoll       =  GP.Roll   *  maxAngle;    
    const double pitchOut = PIDSet.PitchPID->Calculate(desiredPitch,  currRot.Pitch, DeltaTime);
    const double rollOut  = PIDSet.RollPID ->Calculate(desiredRoll,   currRot.Roll,  DeltaTime);
	
    desiredNewPitchRate =  -GP.Pitch *  maxAngleRate;
    desiredNewRollRate  =  GP.Roll  *  maxAngleRate;
    desiredPitchRate = (currentFlightMode == EFlightMode::JoyStickAcroControl)
                        ? desiredNewPitchRate
                        : FMath::Clamp(pitchOut, -maxAngleRate, maxAngleRate);
    desiredRollRate  = (currentFlightMode == EFlightMode::JoyStickAcroControl)
                        ? desiredNewRollRate
                        : FMath::Clamp(rollOut, -maxAngleRate, maxAngleRate);

    const double pitchRateOut = PIDSet.PitchRatePID->Calculate(desiredPitchRate, localAngularRateDeg.Y, DeltaTime);
    const double rollRateOut  = PIDSet.RollRatePID ->Calculate(desiredRollRate,  localAngularRateDeg.X, DeltaTime);
	
	/* ---------------- PID cascades (same gains as normal) ---------- */
	desiredYawRate = GP.Yaw*maxAngleRate;
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
	
	/* ───── World-space state ───── */
	FVector GPSData = SensorData.GPSPosMeters;
	float Altitude = SensorData.BaroAltitudeM;
	const FVector  currPos = {GPSData.X, GPSData.Y, Altitude};     
	const FVector  currVel = SensorData.IMUVelMS;          
	const FRotator currRot = SensorData.IMUAttitude;
	const FRotator yawOnlyRot(0.f, currRot.Yaw, 0.f);
	FVector AngularRate = SensorData.IMUAngVelDEGS;
	localAngularRateDeg = AngularRate;
	
	
	// Get world angular velocity and transform it to local frame using yaw-only rotation

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
            // ───── Update / fetch next set-point ─────
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
	const double zOut = CurrentSet->ZPID -> Calculate(desiredLocalVelocity.Z,currentLocalVelocity.Z, DeltaTime);
	
	/*-------- Angle P Control -------- */ 
	desiredPitch = (currentFlightMode == EFlightMode::AngleControl) ? desiredNewPitch: FMath::Clamp( xOut, -maxAngle,  maxAngle);
	desiredRoll  = (currentFlightMode == EFlightMode::AngleControl) ? desiredNewRoll: FMath::Clamp( yOut, -maxAngle,  maxAngle);
	
	const double rollOut  = CurrentSet ->RollPID->Calculate(desiredRoll,currRot.Roll , DeltaTime);
	const double pitchOut = CurrentSet ->PitchPID->Calculate(desiredPitch,currRot.Pitch, DeltaTime);
	// UE_LOG(LogTemp, Warning, TEXT("Roll Output: %f deg/s"), rollOut);
	// UE_LOG(LogTemp, Warning, TEXT("Pitch Output: %f deg/s"), pitchOut);

	/*-------- Angle Rate PID Control -------- */ 

	desiredPitchRate = (currentFlightMode == EFlightMode::RateControl) ? desiredNewPitchRate: FMath::Clamp(pitchOut, -maxAngleRate, maxAngleRate); // Implement switch here for angle control using DesiredNewPitchRate
	desiredRollRate = (currentFlightMode == EFlightMode::RateControl) ? desiredNewRollRate: FMath::Clamp(rollOut, -maxAngleRate,  maxAngleRate); // Implement switch here for angle control 
	//
	const double rollRateOut  = CurrentSet->RollRatePID->Calculate(desiredRollRate,localAngularRateDeg.X,  DeltaTime);
	const double pitchRateOut = CurrentSet->PitchRatePID->Calculate(desiredPitchRate,localAngularRateDeg.Y, DeltaTime);

	const float yawOutput = YawRateControl(DeltaTime);

    //  Mix & apply motor thrusts / torques (use commanded tilt for compensation)
    ThrustMixer(desiredPitch, desiredRoll, zOut, rollRateOut, pitchRateOut, yawOutput);
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
        // (removed duplicate velocity debug draw to prevent double visuals per frame)

}

//=========================== Thrusts =========================== //

void UQuadDroneController::ThrustMixer(double xOut, double yOut, double zOut, double rollOutput, double pitchOutput, double yawOutput)
{
	float droneMass = dronePawn->DroneBody->GetMass();
	const float gravity = 980.f	;
	const float hoverThrust = (droneMass * gravity) / 4.0f; 
 
	float baseThrust = hoverThrust + (zOut*100) / 4.0f;
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
   // Draw only a line connecting the current position to the setpoint (convert m → cm for world debug)
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
    // Avoid duplicate draws within the same frame (can be called from UI and controller)
    if (bDrewMagDebugThisFrame) return;
    bDrewMagDebugThisFrame = true;
    UWorld* World = dronePawn->GetWorld();
    if (!World) return;
    const FVector DronePos = dronePawn->GetActorLocation();
    // Georeferencing and geographic context
    AGeoReferencingSystem* GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(World);
    if (!GeoRef) return;
    FGeographicCoordinates GeoCoords;
    GeoRef->EngineToGeographic(DronePos, GeoCoords);
    const float Declination = FGeoMagDeclination::GetMagDeclinationDegrees(GeoCoords.Latitude, GeoCoords.Longitude);
    // Local ENU basis vectors
    FVector EastVector, NorthVector, UpVector;
    GeoRef->GetENUVectorsAtGeographicLocation(GeoCoords, EastVector, NorthVector, UpVector);
    // Arrows: True North (green), Magnetic North (red), Drone heading (blue)
    const FVector TrueNorthEnd = DronePos + NorthVector * 400.0f;
    DrawDebugDirectionalArrow(World, DronePos, TrueNorthEnd, 80.0f, FColor::Green, false, 0.0f, 0, 4.0f);
    const FQuat DeclinationRot(UpVector, FMath::DegreesToRadians(Declination));
    const FVector MagNorthVector = DeclinationRot.RotateVector(NorthVector);
    const FVector MagNorthEnd = DronePos + MagNorthVector * 400.0f;
    DrawDebugDirectionalArrow(World, DronePos, MagNorthEnd, 80.0f, FColor::Red, false, 0.0f, 0, 4.0f);
    const FVector DroneForward = dronePawn->GetActorForwardVector();
    const FVector DroneHeadingEnd = DronePos + DroneForward * 350.0f;
    DrawDebugDirectionalArrow(World, DronePos, DroneHeadingEnd, 72.0f, FColor::Blue, false, 0.0f, 0, 3.0f);
    // Heading relative to True North (deg)
    const float HeadingTrueDeg = FMath::RadiansToDegrees(FMath::Atan2(
        FVector::DotProduct(DroneForward, EastVector),
        FVector::DotProduct(DroneForward, NorthVector)));
    // Magnetic field vector (purple)
    if (dronePawn->SensorManager && dronePawn->SensorManager->Magnetometer)
    {
        const FVector MagFieldBody  = dronePawn->SensorManager->Magnetometer->GetLastMagField();
        const FVector MagFieldWorld = dronePawn->GetActorRotation().RotateVector(MagFieldBody);
        const FVector MagFieldScaled = MagFieldWorld * 1000.0f; // scale for visibility
        DrawDebugLine(World, DronePos, DronePos + MagFieldScaled, FColor::Purple, false, 0.0f, 0, 2.5f);
    }
    // Compass ring (smaller) and ticks
    const float CompassRadius = 220.0f;
    DrawDebugCircle(World, DronePos, CompassRadius, 64, FColor::White, false, 0.0f, 0, 1.8f,
                    EastVector, NorthVector, false);
    // Cardinal ticks
    const float TickLongLen = 18.0f;  const float TickShortLen = 10.0f;  const float TickThick = 1.5f;
    const FVector NPos = DronePos + NorthVector * CompassRadius;
    const FVector SPos = DronePos - NorthVector * CompassRadius;
    const FVector EPos = DronePos + EastVector  * CompassRadius;
    const FVector WPos = DronePos - EastVector  * CompassRadius;
    DrawDebugLine(World, NPos - NorthVector * TickLongLen, NPos + NorthVector * TickShortLen, FColor::White, false, 0.0f, 0, TickThick);
    DrawDebugLine(World, SPos + NorthVector * TickLongLen, SPos - NorthVector * TickShortLen, FColor::White, false, 0.0f, 0, TickThick);
    DrawDebugLine(World, EPos - EastVector  * TickLongLen, EPos + EastVector  * TickShortLen, FColor::White, false, 0.0f, 0, TickThick);
    DrawDebugLine(World, WPos + EastVector  * TickLongLen, WPos - EastVector  * TickShortLen, FColor::White, false, 0.0f, 0, TickThick);
    // Inter-cardinal ticks
    const FVector NE = (NorthVector + EastVector).GetSafeNormal();
    const FVector SE = (-NorthVector + EastVector).GetSafeNormal();
    const FVector SW = (-NorthVector - EastVector).GetSafeNormal();
    const FVector NW = (NorthVector - EastVector).GetSafeNormal();
    DrawDebugLine(World, DronePos + NE * CompassRadius - NE * TickShortLen, DronePos + NE * CompassRadius + NE * TickShortLen, FColor(220,220,220), false, 0.0f, 0, TickThick * 0.9f);
    DrawDebugLine(World, DronePos + SE * CompassRadius - SE * TickShortLen, DronePos + SE * CompassRadius + SE * TickShortLen, FColor(220,220,220), false, 0.0f, 0, TickThick * 0.9f);
    DrawDebugLine(World, DronePos + SW * CompassRadius - SW * TickShortLen, DronePos + SW * CompassRadius + SW * TickShortLen, FColor(220,220,220), false, 0.0f, 0, TickThick * 0.9f);
    DrawDebugLine(World, DronePos + NW * CompassRadius - NW * TickShortLen, DronePos + NW * CompassRadius + NW * TickShortLen, FColor(220,220,220), false, 0.0f, 0, TickThick * 0.9f);
    // Cardinal labels
    DrawDebugString(World, NPos + UpVector * 12.0f, TEXT("N"), nullptr, FColor::White, 0.0f, true, 1.2f);
    DrawDebugString(World, SPos + UpVector * 12.0f, TEXT("S"), nullptr, FColor::White, 0.0f, true, 1.2f);
    DrawDebugString(World, EPos + UpVector * 12.0f, TEXT("E"), nullptr, FColor::White, 0.0f, true, 1.2f);
    DrawDebugString(World, WPos + UpVector * 12.0f, TEXT("W"), nullptr, FColor::White, 0.0f, true, 1.2f);
    // Screen-space legend in top-right (approximate via camera-relative anchor)
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
    {
        if (PC->PlayerCameraManager)
        {
            FVector CamLoc; FRotator CamRot;
            PC->PlayerCameraManager->GetCameraViewPoint(CamLoc, CamRot);
            const FVector Fwd   = CamRot.Vector();
            const FVector Right = FRotationMatrix(CamRot).GetScaledAxis(EAxis::Y);
            const FVector Up    = FRotationMatrix(CamRot).GetScaledAxis(EAxis::Z);
            const FVector LegendBase = CamLoc + Fwd * 600.0f + Right * 420.0f + Up * 260.0f;
            const float LineStep = 20.0f;
            DrawDebugString(World, LegendBase + (-Up * (0.0f * LineStep)),
                            FString::Printf(TEXT("True Heading: %.1f deg"), HeadingTrueDeg),
                            nullptr, FColor::Green, 0.0f, true, 1.2f);
            DrawDebugString(World, LegendBase + (-Up * (1.0f * LineStep)),
                            FString::Printf(TEXT("Mag Heading: %.1f deg"), HeadingTrueDeg + Declination),
                            nullptr, FColor::Red, 0.0f, true, 1.2f);
            if (dronePawn->SensorManager && dronePawn->SensorManager->Magnetometer)
            {
                const FVector MagField = dronePawn->SensorManager->Magnetometer->GetLastMagField();
                DrawDebugString(World, LegendBase + (-Up * (2.0f * LineStep)),
                                FString::Printf(TEXT("Body Mag X: %.3f Y: %.3f Z: %.3f"), MagField.X, MagField.Y, MagField.Z),
                                nullptr, FColor::Purple, 0.0f, true, 1.2f);
            }
        }
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
        hoverTargetAltitude = dronePawn ? (dronePawn->GetActorLocation().Z / 100.0f) : 0.f;
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

        // Map 0..1 → 0..MaxPerMotor_N
        float thrustN = cmd01 * MaxPerMotor_N;

        // simple slew
        const float delta = FMath::Clamp(thrustN - prevN[i], -maxSlewN, maxSlewN);
        thrustN = prevN[i] + delta;
        prevN[i]= thrustN;

        // Convert Newtons → Unreal force units (kg*cm/s^2). 1 N = 100 Unreal units
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


#include "Sensors/IMUSensor.h"

// EVERYTHING IN CM 

UIMUSensor::UIMUSensor()
{

	PrimaryComponentTick.bCanEverTick = false;
	LastAccelerometer = FVector::ZeroVector;
	LastGyroscope = FVector::ZeroVector;
	PreviousVelocity = FVector::ZeroVector;
}

void UIMUSensor::Initialize()
{
	if (GetAttachParent() && GetAttachParent()->IsSimulatingPhysics())
	{
		AttachedBody = Cast<UPrimitiveComponent>(GetAttachParent());
	}
	else if (GetOwner())
	{
		UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
		if (RootPrim && RootPrim->IsSimulatingPhysics())
		{
			AttachedBody = RootPrim;
		}
	}
    
	if (AttachedBody)
	{
		PreviousVelocity = AttachedBody->GetPhysicsLinearVelocity();
		bInitialized = true;
	}
	else
	{
			UE_LOG(LogTemp, Warning, TEXT("IMUSensor: No physics body found!"));
	}
}

FVector UIMUSensor::SampleRawAcceleration(float DeltaTime)
{
	if (!bInitialized || !AttachedBody || DeltaTime <= 0.f)
		return FVector::ZeroVector;

	FVector CurrVel = AttachedBody->GetPhysicsLinearVelocity();
	FVector RawAccel = (CurrVel - PreviousVelocity) / DeltaTime;
	PreviousVelocity = CurrVel;

	// Convert to m/s^2 and transform to body frame
	return AttachedBody->GetComponentTransform().InverseTransformVectorNoScale(RawAccel / 100.0f);
}

FVector UIMUSensor::SampleRawAngularVelocity()
{
	if (!bInitialized || !AttachedBody)
		return FVector::ZeroVector;

	// UE world ω (rad/s) -> body(FRU) ω (rad/s)
	const FVector world_w_rad = AttachedBody->GetPhysicsAngularVelocityInRadians();
	const FVector w_fru = AttachedBody->GetComponentTransform().InverseTransformVectorNoScale(world_w_rad);
	const FVector pqr_frd =  FVector(-w_fru.X,w_fru.Y,w_fru.Z);
	return pqr_frd;  // X=roll rate, Y=pitch rate, Z=yaw rate (UE, FRD)
}

FVector UIMUSensor::SampleRawVelocity()
{
	if (!bInitialized || !AttachedBody) return FVector::ZeroVector;

	const FVector v_world_cms = AttachedBody->GetPhysicsLinearVelocity(); // cm/s
	const FVector v_world     = v_world_cms / 100.f;                      // m/s

	// Bring to body(FRU)
	const FVector v_fru = AttachedBody->GetComponentTransform().InverseTransformVectorNoScale(v_world);

	// FRU -> FRD (X = X, Y = Y, Z = -Z)
	return FVector(v_fru.X, v_fru.Y, -v_fru.Z);
}


// FVector UIMUSensor::SampleRawVelocity()
// {
// 	if (!bInitialized || !AttachedBody)
// 		return FVector::ZeroVector;
//
// 	const FVector v_world = AttachedBody->GetPhysicsLinearVelocity() / 100.f; // m/s
// 	const float   yaw_deg = AttachedBody->GetComponentRotation().Yaw;
// 	const FRotator yawOnly(0.f, yaw_deg, 0.f);
// 	return yawOnly.UnrotateVector(v_world); // X=fwd-in-heading, Y=lateral, Z=up
// }

FRotator UIMUSensor::SampleRawAttitude()
{
	if (!bInitialized || !AttachedBody) return FRotator::ZeroRotator;

	const FRotator fru = AttachedBody->GetComponentRotation(); // UE: Roll+=RWD, Pitch+=nose down, Yaw+=turn right (about +Up)
	FRotator frd;
	frd.Roll  =  fru.Roll;   // same
	frd.Pitch = -fru.Pitch;  // nose up positive
	frd.Yaw   = fru.Yaw;    // right-turn positive in NED
	return frd;
}

// FRotator UIMUSensor::SampleRawAttitude()
// {
// 	if (!bInitialized || !AttachedBody)
// 		return FRotator::ZeroRotator;
//
// 	return AttachedBody->GetComponentRotation(); // UE: Pitch += nose down, Roll += right
// }

void UIMUSensor::UpdateSensor(float DeltaTime, bool bNoise)
{
	AccumulatedTime += DeltaTime;
	const float Period = 1.f / UpdateRate;
	if (AccumulatedTime < Period)
		return;
	AccumulatedTime -= Period;
	if (UWorld* World = GetWorld())
	{
		LastUpdateTime = World->GetTimeSeconds();
	}
	// Sample all sensor data
	FVector Accel = SampleRawAcceleration(Period);
	FVector Gyro = SampleRawAngularVelocity();
	FVector Velocity = SampleRawVelocity();
	FRotator Attitude = SampleRawAttitude();

	// Apply noise if requested
	if (bNoise)
	{
		// Acceleration noise
		Accel.X += SensorNoise() * AccelVelNoiseStdDev;
		Accel.Y += SensorNoise() * AccelVelNoiseStdDev;
		Accel.Z += SensorNoise() * AccelVelNoiseStdDev;

		// Angular velocity noise
		Gyro.X += SensorNoise() * GyroAttNoiseStdDev;
		Gyro.Y += SensorNoise() * GyroAttNoiseStdDev;
		Gyro.Z += SensorNoise() * GyroAttNoiseStdDev;
       
		// Velocity noise (using same noise level as acceleration)
		Velocity.X += SensorNoise() * AccelVelNoiseStdDev;
		Velocity.Y += SensorNoise() * AccelVelNoiseStdDev;
		Velocity.Z += SensorNoise() * AccelVelNoiseStdDev;
       
		// Attitude noise (using same noise level as gyro)
		Attitude.Roll += SensorNoise() * FMath::RadiansToDegrees(GyroAttNoiseStdDev);
		Attitude.Pitch += SensorNoise() * FMath::RadiansToDegrees(GyroAttNoiseStdDev);
		Attitude.Yaw += SensorNoise() * FMath::RadiansToDegrees(GyroAttNoiseStdDev);
	}
    if (bNeedsFirstAccelSample)
    {
        LastAccelerometer = Accel; // seed with first valid sample
        bNeedsFirstAccelSample = false;
    }
    else
    {
        LastAccelerometer = Accel; // update every period to avoid stale accel
    }
    // Store all sensor readings
    LastGyroscope = Gyro;
    LastVelocity = Velocity;  // You'll need to add this member variable
    LastAttitude = Attitude;  // You'll need to add this member variable
}

float UIMUSensor::SensorNoise()
{
	static float V1, V2, S;
	static bool bPhase = true;
	float X;

	if (bPhase) 
	{
		do {
			float U1 = FMath::FRand(); 
			float U2 = FMath::FRand();
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || FMath::Abs(S) < 1e-8f);

		X = V1 * FMath::Sqrt(-2.0f * FMath::Loge(S) / S);
	} 
	else 
	{
		X = V2 * FMath::Sqrt(-2.0f * FMath::Loge(S) / S);
	}

	bPhase = !bPhase;
	return X;
}

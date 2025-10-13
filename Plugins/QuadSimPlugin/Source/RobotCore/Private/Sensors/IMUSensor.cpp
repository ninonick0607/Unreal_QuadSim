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

	// UE returns angular velocity in WORLD space (rad/s)
	const FVector world_w_rad = AttachedBody->GetPhysicsAngularVelocityInRadians();

	// Convert to BODY frame (p,q,r) using full inverse rotation (no scale)
	const FVector w_fru = AttachedBody->GetComponentTransform().InverseTransformVectorNoScale(world_w_rad);

	// FRU → FLU (flip Y)
	return FVector(-w_fru.X, w_fru.Y, w_fru.Z);
}
FVector UIMUSensor::SampleRawVelocity(){
	if (!bInitialized || !AttachedBody)
		return FVector::ZeroVector;
	
	FVector CurrVel = AttachedBody->GetPhysicsLinearVelocity()/100;
	FRotator ComponentRot = AttachedBody->GetComponentRotation();
	FRotator YawOnlyRot(0.f, ComponentRot.Yaw, 0.f);
    
	// Transform using only yaw
	return YawOnlyRot.UnrotateVector(CurrVel);
}

FRotator UIMUSensor::SampleRawAttitude(){
	if (!bInitialized || !AttachedBody)
		return FRotator::ZeroRotator;
	FRotator WorldRotation = AttachedBody->GetComponentRotation();
	FRotator AdjustedRotation = WorldRotation;
	AdjustedRotation.Pitch = -WorldRotation.Pitch;
    return AdjustedRotation;
}

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

// SensorAdapterComponent.h
#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Messages/SensorBus.h"
#include "SensorManagerComponent.generated.h"

class UIMUSensor;
class UGPSSensor;
class UBaroSensor;
class UMagSensor;

USTRUCT()
struct FSimpleState {
	GENERATED_BODY()
	FVector pos_ned_m      = FVector::ZeroVector;
	FVector vel_ned_mps    = FVector::ZeroVector;
	FQuat   q_nb           = FQuat::Identity;
	FRotator euler_frd_deg = FRotator::ZeroRotator;
	FVector gyro_frd_rps   = FVector::ZeroVector;
	FVector accel_frd_mps2 = FVector::ZeroVector;
	float   baro_alt_msl_m = NAN;
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class USensorManagerComponent : public UActorComponent
{
	GENERATED_BODY()
public:
	USensorManagerComponent();
    bool GetLatestSimpleState(FSimpleState& out) const;

	UPROPERTY(EditAnywhere) float ImuRateHz  = 200.f;
	UPROPERTY(EditAnywhere) float GpsRateHz  = 10.f;
	UPROPERTY(EditAnywhere) float BaroRateHz = 50.f;
	UPROPERTY(EditAnywhere) float MagRateHz  = 50.f;

	virtual void BeginPlay() override;
	virtual void TickComponent(float dt, ELevelTick, FActorComponentTickFunction*) override;

private:
	// sources
	UIMUSensor*  Imu = nullptr;
	UGPSSensor*  Gps = nullptr;
	UBaroSensor* Baro = nullptr;
	UMagSensor*  Mag = nullptr;

	// bus
	USensorBus* Bus = nullptr;

	// period accumulators
	double accImu=0, accGps=0, accBaro=0, accMag=0;
	uint32 seqImu=0, seqGps=0, seqBaro=0, seqMag=0;

	// helpers
	double NowSec() const;
	void PublishImu(double period);
	void PublishGps(double period);
	void PublishBaro(double period);
	void PublishMag(double period);
};

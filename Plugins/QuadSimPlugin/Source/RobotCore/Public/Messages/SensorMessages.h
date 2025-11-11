// SensorMessages.h
#pragma once
#include "CoreMinimal.h"
#include "SensorMessages.generated.h"

USTRUCT(BlueprintType)
struct FMsgHeader
{
	GENERATED_BODY()
	UPROPERTY() double stamp_sec = 0.0;     // seconds
	UPROPERTY() uint32 seq = 0;
	UPROPERTY() FName  frame_id = NAME_None;
};

UENUM(BlueprintType)
enum class EGpsFixType : uint8
{
	NO_FIX=0, FIX_2D=1, FIX_3D=2, RTK_FIX=3, RTK_FLOAT=4
};

USTRUCT(BlueprintType)
struct FImuMsg
{
	GENERATED_BODY()
	UPROPERTY() FMsgHeader header;
	UPROPERTY() FVector accel_mps2_b_frd = FVector::ZeroVector;  // specific force, body FRD
	UPROPERTY() FVector gyro_rps_b_frd   = FVector::ZeroVector;  // body FRD
	UPROPERTY() FVector bias_accel_mps2  = FVector::ZeroVector;  // optional
	UPROPERTY() FVector bias_gyro_rps    = FVector::ZeroVector;  // optional
	UPROPERTY() FQuat   q_nb             = FQuat::Identity;     // body FRD -> world NED
	UPROPERTY() FVector vel_ned_mps      = FVector::ZeroVector; // world NED velocity
	UPROPERTY() FRotator euler_frd_deg   = FRotator::ZeroRotator; // optional for legacy PIDs
	
};

USTRUCT(BlueprintType)
struct FGpsFixMsg
{
	GENERATED_BODY()
	UPROPERTY() FMsgHeader header;
	UPROPERTY() FVector pos_ned_m   = FVector::ZeroVector;  // N,E,D
	UPROPERTY() FVector vel_ned_mps = FVector::ZeroVector;  // Doppler if available, else derived
	UPROPERTY() EGpsFixType fix_type = EGpsFixType::FIX_3D;
	UPROPERTY() FMatrix  cov_pos = FMatrix::Identity;  // optional 3x3
	UPROPERTY() FMatrix  cov_vel = FMatrix::Identity;  // optional 3x3
};

USTRUCT(BlueprintType)
struct FBaroMsg
{
	GENERATED_BODY()
	UPROPERTY() FMsgHeader header;
	UPROPERTY() float pressure_Pa = 101325.f;
	UPROPERTY() float temperature_C = 15.f;
	UPROPERTY() float alt_msl_m = NAN;  // optional derived
};

USTRUCT(BlueprintType)
struct FMagMsg
{
	GENERATED_BODY()
	UPROPERTY() FMsgHeader header;
	UPROPERTY() FVector mag_T_b_frd = FVector::ZeroVector; // Tesla, body FRD
	UPROPERTY() float declination_deg = 0.f;
};

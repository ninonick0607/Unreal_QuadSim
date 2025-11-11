// SensorAdapterComponent.cpp
#include "Sensors/SensorManagerComponent.h"
#include "Sensors/IMUSensor.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/BaroSensor.h"
#include "Sensors/MagSensor.h"
#include "CoordinateTransform.h"

USensorManagerComponent::USensorManagerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void USensorManagerComponent::BeginPlay()
{
    Super::BeginPlay();
    Bus = USensorBus::Get(GetWorld());

    // find sensors on the same actor
    AActor* A = GetOwner();
    Imu  = A->FindComponentByClass<UIMUSensor>();
    Gps  = A->FindComponentByClass<UGPSSensor>();
    Baro = A->FindComponentByClass<UBaroSensor>();
    Mag  = A->FindComponentByClass<UMagSensor>();
}

double USensorManagerComponent::NowSec() const
{
    const UWorld* W = GetWorld();
    return W ? W->GetTimeSeconds() : 0.0;
}

void USensorManagerComponent::TickComponent(float dt, ELevelTick, FActorComponentTickFunction*)
{
    accImu  += dt;
    accGps  += dt;
    accBaro += dt;
    accMag  += dt;

    const double pImu  = 1.0 / FMath::Max(1e-6f, ImuRateHz);
    const double pGps  = 1.0 / FMath::Max(1e-6f, GpsRateHz);
    const double pBaro = 1.0 / FMath::Max(1e-6f, BaroRateHz);
    const double pMag  = 1.0 / FMath::Max(1e-6f, MagRateHz);

    while (accImu  >= pImu)  { PublishImu(pImu);   accImu  -= pImu;  }
    while (accGps  >= pGps)  { PublishGps(pGps);   accGps  -= pGps;  }
    while (accBaro >= pBaro) { PublishBaro(pBaro); accBaro -= pBaro; }
    while (accMag  >= pMag)  { PublishMag(pMag);   accMag  -= pMag;  }
}

bool USensorManagerComponent::GetLatestSimpleState(FSimpleState& out) const
{
    bool any=false;
    FImuMsg imu; FGpsFixMsg gps; FBaroMsg baro;

    if (Bus->ImuQ.PeekLatest(imu)) {
        out.vel_ned_mps    = imu.vel_ned_mps;
        out.q_nb           = imu.q_nb;
        out.euler_frd_deg  = imu.euler_frd_deg;
        out.gyro_frd_rps   = imu.gyro_rps_b_frd;
        out.accel_frd_mps2 = imu.accel_mps2_b_frd;
        any = true;
    }
    if (Bus->GpsQ.PeekLatest(gps)) {
        out.pos_ned_m = gps.pos_ned_m;
        any = true;
    }
    if (Bus->BaroQ.PeekLatest(baro)) {
        out.baro_alt_msl_m = baro.alt_msl_m;
        any = true;
    }
    return any;
}

// IMU: publish specific force and gyro in FRD, no attitude, no velocity
void USensorManagerComponent::PublishImu(double /*period*/)
{
    if (!Imu || !GetOwner()) return;

    // Raw samples from your IMUSensor are UE world-based, already removing g in world and transforming to body FRU
    // We will enforce final body FRD and SI units here
    const FTransform T_wb = GetOwner()->GetRootComponent()->GetComponentTransform();

    // Accel: your SampleRawAcceleration returns body frame already, but confirm FRU vs FRD
    // If your LastAccelerometer is in FRU, convert to FRD here. If it is already FRD, remove the conversion.
    const FVector a_b_fru = Imu->GetLastAccelerometer();       // m/s^2, body FRU
    //const FVector a_b_frd = Frames::FRU_to_FRD(a_b_fru);

    // Gyro: you built pqr_frd in SampleRawAngularVelocity, but to be robust, convert FRU->FRD at edge
    const FVector w_b_fru = Imu->GetLastGyroscope();//Frames::FRD_to_FRU(Imu->GetLastGyroscope()); // if LastGyroscope was FRD, invert to FRU then convert back next line
    //const FVector w_b_frd = Frames::FRU_to_FRD(w_b_fru);

    FImuMsg msg;
    msg.header.stamp_sec = NowSec();
    msg.header.seq = ++seqImu;
    msg.header.frame_id = FName(TEXT("base_link_frd"));

    msg.accel_mps2_b_frd = a_b_fru;
    msg.gyro_rps_b_frd   = Imu->GetLastGyroscope();   // already FRD
    msg.vel_ned_mps      = Imu->GetLastVelocity(); // world NED
    msg.q_nb             = Imu->GetLastQuatNB();      // FRD->NED
    msg.euler_frd_deg    = Imu->GetLastAttitude();    // for legacy PIDs

    Bus->ImuQ.Push(msg);
}

// GPS: publish NED position and Doppler velocity in NED if available
void USensorManagerComponent::PublishGps(double /*period*/)
{
    if (!Gps || !GetOwner()) return;

    const FVector pos_world_cm = GetOwner()->GetActorLocation();
    const FVector pos_ned_m = Frames::UEWorldVec_to_NED(pos_world_cm);

    FVector vel_ned_mps = Imu->GetLastVelocity();
    
    FGpsFixMsg msg;
    msg.header.stamp_sec = NowSec();
    msg.header.seq = ++seqGps;
    msg.header.frame_id = FName(TEXT("world_ned"));
    msg.pos_ned_m   = pos_ned_m;
    msg.vel_ned_mps = vel_ned_mps;
    msg.fix_type    = EGpsFixType::FIX_3D; // replace with your fix state
    Bus->GpsQ.Push(msg);
}

// Baro: publish pressure Pa and temperature C, optional altitude for debugging
void USensorManagerComponent::PublishBaro(double /*period*/)
{
    if (!Baro) return;
    FBaroMsg msg;
    msg.header.stamp_sec = NowSec();
    msg.header.seq = ++seqBaro;
    msg.header.frame_id = FName(TEXT("base_link_frd"));
    msg.pressure_Pa   = Baro->GetLastPressure();
    msg.temperature_C = Baro->GetLastTemperature();
    msg.alt_msl_m     = Baro->GetEstimatedAltitude(); // optional
    Bus->BaroQ.Push(msg);
}

// Mag: publish mag field in body FRD Tesla, plus declination
void USensorManagerComponent::PublishMag(double /*period*/)
{
    if (!Mag || !GetOwner()) return;

    // Your Mag sensor stores body field likely in FRU, confirm and convert
    const FVector b_fru_T = Mag->GetLastMagField();         // Tesla, body FRU
    const FVector b_frd_T = Frames::FRU_to_FRD(b_fru_T);

    FMagMsg msg;
    msg.header.stamp_sec = NowSec();
    msg.header.seq = ++seqMag;
    msg.header.frame_id = FName(TEXT("base_link_frd"));
    msg.mag_T_b_frd      = b_frd_T;
    msg.declination_deg  = Mag->GetDeclination();
    Bus->MagQ.Push(msg);
}

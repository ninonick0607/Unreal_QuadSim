// SensorManagerComponent.cpp
#include "Sensors/SensorManagerComponent.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/IMUSensor.h"
#include "Sensors/MagSensor.h"
#include "Sensors/BaroSensor.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"

USensorManagerComponent::USensorManagerComponent()
{
    PrimaryComponentTick.bCanEverTick = false; 
    bSensorsInitialized = false;
    
    GPS = CreateDefaultSubobject<UGPSSensor>(TEXT("GPSSensor"));
    IMU = CreateDefaultSubobject<UIMUSensor>(TEXT("IMUSensor"));
	Magnetometer = CreateDefaultSubobject<UMagSensor>(TEXT("MagnetometerSensor"));
	Barometer = CreateDefaultSubobject<UBaroSensor>(TEXT("BarometerSensor"));
}

void USensorManagerComponent::BeginPlay()
{
	Super::BeginPlay();
	
	if (GPS && !GPS->GetAttachParent())
	{
		GPS->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Attached GPS to SensorManager"));
	}
	if (IMU && !IMU->GetAttachParent())
	{
		IMU->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Attached IMU to SensorManager"));
	}
	if (Magnetometer && !Magnetometer->GetAttachParent())
	{
		Magnetometer->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Attached Magnetometer to SensorManager"));
	}
	if (Barometer && !Barometer->GetAttachParent())
	{
		Barometer->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Attached Barometer to SensorManager"));
	}
}

void USensorManagerComponent::InitializeSensors()
{
	if (bSensorsInitialized)
	{
		UE_LOG(LogTemp, Warning, TEXT("SensorManager: Sensors already initialized"));
		return;
	}
    
	if (IMU)
	{
		IMU->Initialize();
		UE_LOG(LogTemp, Display, TEXT("SensorManager: IMU initialized"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("SensorManager: IMU component is null!"));
	}
	if (GPS)
	{
		GPS->Initialize();  // Add this line
		UE_LOG(LogTemp, Display, TEXT("SensorManager: GPS initialized"));
	}
	if (GPS)
	{
		UE_LOG(LogTemp, Display, TEXT("SensorManager: GPS ready"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("SensorManager: GPS component is null!"));
	}
    
	if (Magnetometer)
	{
		Magnetometer->Initialize();
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Magnetometer initialized"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("SensorManager: Magnetometer component is null!"));
	}
    
	if (Barometer)
	{
		Barometer->Initialize();
		UE_LOG(LogTemp, Display, TEXT("SensorManager: Barometer initialized"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("SensorManager: Barometer component is null!"));
	}
    
	bSensorsInitialized = true;
	UE_LOG(LogTemp, Display, TEXT("SensorManager: All sensors initialized"));
}

void USensorManagerComponent::UpdateAllSensors(float DeltaTime, bool bAddNoise)
{
    if (!bSensorsInitialized)
    {
        UE_LOG(LogTemp, Warning, TEXT("SensorManager: Attempting to update uninitialized sensors"));
        return;
    }
    
    // GPS updates normally (10Hz)
    if (GPS)
    {
        GPS->UpdateSensor(DeltaTime, bAddNoise);
    }
    
    // IMU needs to update at 250Hz, so we may need multiple updates per frame
	if (IMU)
	{
		// reset our “first sample” marker
		IMU->BeginFrame();

		const float IMUPeriod = 1.0f / 250.0f;
		float RemainingTime = DeltaTime;
		while (RemainingTime > 0.0f)
		{
			float StepTime = FMath::Min(RemainingTime, IMUPeriod);
			IMU->UpdateSensor(StepTime, bAddNoise);
			RemainingTime -= IMUPeriod;
		}
	}
	if (Magnetometer)
	{
		const float MagPeriod = 1.0f / 100.0f; // 0.01 seconds (10ms)
		float RemainingTime = DeltaTime;

		while (RemainingTime > 0.0f)
		{
			float StepTime = FMath::Min(RemainingTime, MagPeriod);
			Magnetometer->UpdateSensor(StepTime, bAddNoise);
			RemainingTime -= StepTime;
		}
	}
	if (Barometer)
	{
		const float BaroPeriod = 1.0f / 20.0f; // 0.05 seconds (50ms)
		float RemainingTime = DeltaTime;

		while (RemainingTime > 0.0f)
		{
			float StepTime = FMath::Min(RemainingTime, BaroPeriod);
			Barometer->UpdateSensor(StepTime, bAddNoise);
			RemainingTime -= StepTime;
		}
	}
	CachedSensorData = GetCurrentSensorData();

}

FSensorData USensorManagerComponent::GetCurrentSensorData() const
{
	FSensorData Data;
    
	if (IMU && IMU->IsInitialized())
	{
		Data.IMUAngVelRADS = IMU->GetLastGyroscope();
		Data.IMULinearAccelMS2 = IMU->GetLastAccelerometer();
		Data.IMUAttitude = IMU->GetLastAttitude();
		Data.IMUVelMS = IMU->GetLastVelocity();
		Data.IMUTimestamp = IMU->GetLastUpdateTime();
		Data.bIMUValid = true;
	}
    
	if (GPS && GPS->IsInitialized())
	{
		Data.GPSPosMeters = GPS->GetLastGPS();
		Data.GPSLatLong = GPS->GetGeographicCoordinates();
		Data.GPSTimestamp = GPS->GetLastUpdateTime();
		Data.bGPSValid = GPS->HasFix();
	}
    
	if (Barometer && Barometer->IsInitialized())
	{
		Data.BaroAltitudeM = Barometer->GetEstimatedAltitude();
		Data.BaroTemp = Barometer->GetLastTemperature();
		Data.BaroLastPressureHPa = Barometer->GetLastPressureHPa();
		Data.bBaroValid = true;
	}
    
	if (Magnetometer && Magnetometer->IsInitialized())
	{
		Data.MagFieldGauss = Magnetometer->GetLastMagField();
		Data.MagHeadingDeg = Magnetometer->GetHeading();
		Data.MagDeclinationDeg = Magnetometer->GetDeclination();
		Data.bMagValid = !Magnetometer->IsCalibrating();
	}
    
	return Data;
}
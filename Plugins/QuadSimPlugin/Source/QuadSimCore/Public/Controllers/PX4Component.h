#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Sockets.h"
#include "PX4Component.generated.h"

class UQuadDroneController;

/**
 * Sleek, no-lockstep PX4 HIL component.
 * NOTE: Do NOT include or forward-declare any MAVLink types in this header.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMCORE_API UPX4Component : public UActorComponent
{
	GENERATED_BODY()
public:
	UPX4Component();

	// UE lifecycle
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Settings
    UPROPERTY(EditAnywhere, Category="PX4")
    bool bEnablePX4 = false;

	UPROPERTY(EditAnywhere, Category="PX4")
	int32 SimulatorTcpPort = 4560; // PX4 SITL connects here

	// Status
    UPROPERTY(VisibleAnywhere, Category="PX4")
    bool bPX4Connected = false;

    // Motor order mapping (PX4 index -> Our thruster index)
    // Default (Iris/Gazebo-style): PX4 [FR, BL, FL, BR] -> Ours [FL, FR, BL, BR]
    // Map used: [2, 0, 1, 3]
    UPROPERTY(EditAnywhere, Category="PX4")
    TArray<int32> Px4ToOurMotorMap;

    // Optional logging to verify mapping at runtime
    UPROPERTY(EditAnywhere, Category="PX4|Debug")
    bool bLogActuatorMapping = true;

	// Public control (new)
	UFUNCTION(BlueprintCallable, Category="PX4")
	void Connect();

	UFUNCTION(BlueprintCallable, Category="PX4")
	void Disconnect();

	// --- Compatibility shims (to keep other modules compiling) ---
	UFUNCTION(BlueprintCallable, Category="PX4")
	void SetPX4Active(bool bActive) { bEnablePX4 = bActive; if (bActive) Connect(); else Disconnect(); }

	// Legacy called by DroneManager; no-op in non-lockstep design (kept for ABI)
	UFUNCTION(BlueprintCallable, Category="PX4")
	void SimulationUpdate(float FixedDeltaTime);
	
	// Please use IsPX4Active() instead of bIsActive() to avoid UE bitfield conflict
	UFUNCTION(BlueprintCallable, Category="PX4")
	bool IsPX4Active() const { return bEnablePX4 && bPX4Connected; }

	// Your sensor pull — keep your existing implementation body
	void UpdateCurrentState();

	// Called by comm thread
	void ProcessIncomingTCP();
	void ThreadStep_RealTime();
	void SendHeartbeat();
	void SendHILSensor();
	void SendHILStateQuaternion();
	void SendHILGPS();
	void SendHILRC();

	// Actuator decoded -> game thread
	void EnqueueMotorCommand(const TArray<float>& Norm01);

	// Time helpers
	uint64 NowUsec() const;
	uint32 TimeBootMs() const { return static_cast<uint32>(NowUsec()/1000ULL); }

	// Handshake state (no MAVLink types here)
	void OnHeartbeat_Parsed(uint8 SrcSys, uint8 SrcComp);

	// Ready?
	bool IsPX4Ready() const { return bPX4Connected && bSawHeartbeat; }

	uint64 GetFirstHeartbeatUsec() const { return FirstHeartbeatUsec; }

private:
	// TCP
	FSocket* ListenSocket = nullptr;
	FSocket* ClientSocket = nullptr;
	bool bListening = false;
	bool bTCPConnected = false;

	// Comm thread
	class FPX4CommThread* Comm = nullptr;

	// IDs
	uint8 SysId = 1;     // this component
	uint8 CompId = 200;  // onboard controller
	uint8 TargetSys = 1; // PX4 fills after heartbeat
	uint8 TargetComp = 1;

	// Heartbeat gating
	bool   bSawHeartbeat = false;
	uint64 FirstHeartbeatUsec = 0;

	// Thread-safe state from sensors (set in UpdateCurrentState)
	mutable FCriticalSection StateCS;
	bool    bStateValid = false;
	FVector PositionNED = FVector::ZeroVector;     // m
	FVector VelocityNED = FVector::ZeroVector;     // m/s
	FVector AccelFRD    = FVector::ZeroVector;     // m/s^2 (linear accel, no gravity)
	FRotator RotNED     = FRotator::ZeroRotator;   // deg
	FVector AngVelFRD   = FVector::ZeroVector;     // rad/s
	FVector GeoLLA      = FVector::ZeroVector;     // lat, lon, alt[m]
	float   PressurePa  = 101325.f;
	float   TemperatureC= 20.f;
	float   AltM        = 0.f;
	FVector MagGauss    = FVector(0.3f,0,0.5f);

	// Game-thread motor queue
	struct FMotorCmd { TArray<float> Values; double Stamp; };
	TQueue<FMotorCmd, EQueueMode::Mpsc> MotorQueue;

	// Internals
	void CreateTCPServer();
	void AcceptOnce();
	void CloseSockets();
	void SendBytes(const uint8* Data, int32 Len);


};

// ---------- Comm Thread ----------
class FPX4CommThread : public FRunnable
{
public:
	explicit FPX4CommThread(UPX4Component* InOwner) : Owner(InOwner) {}
	virtual bool    Init() override { return true; }
	virtual uint32  Run() override;
	virtual void    Stop() override { bStop = true; }
	void            Start();
	void            Join();
	double Last250 = 0.0;
	double Last50  = 0.0;
	double LastHB  = 0.0;

private:
	UPX4Component* Owner = nullptr;
	FRunnableThread* Thread = nullptr;
	FThreadSafeBool bStop = false;
};

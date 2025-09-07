#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/World.h"
#include "Sockets.h"
#include "Common/UdpSocketBuilder.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/ThreadSafeBool.h"
#include "HAL/CriticalSection.h"

#include <mavlink_types.h>

#include "PX4Component.generated.h"

// Forward declarations
class UQuadDroneController;
class FPX4CommunicationThread;

UENUM(BlueprintType)
enum class EPX4ControlMode : uint8
{
    Disabled    UMETA(DisplayName = "Disabled"),
    Position    UMETA(DisplayName = "Position Control"),
    Velocity    UMETA(DisplayName = "Velocity Control"), 
    Attitude    UMETA(DisplayName = "Attitude Control"),
    Manual      UMETA(DisplayName = "Manual Control")
};

// Communication Thread Class
class FPX4CommunicationThread : public FRunnable
{
public:
    FPX4CommunicationThread(class UPX4Component* InPX4Component);
    virtual ~FPX4CommunicationThread();

    // FRunnable interface
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

    void StartThread();
    void StopThread();
    bool IsRunning() const { return Thread != nullptr; }

private:
    class UPX4Component* PX4Component;
    FRunnableThread* Thread;
    FThreadSafeBool bStopRequested;
    
    static const int32 TARGET_FREQUENCY_HZ = 250;  // Increase to 250Hz for lockstep
    static const double TARGET_INTERVAL; // Will be 0.004 seconds (4ms)
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMCORE_API UPX4Component : public UActorComponent
{
    GENERATED_BODY()

public:
    UPX4Component();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // Configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Connection")
    bool bUsePX4 = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Connection")
    FString PX4_IP = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Connection")
    int32 PX4_Port = 4560;  // Standard PX4 simulator port (TCP)
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Connection")
    int32 ControlPortLocal = 14540;  // Local UDP port for MAVLink control
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Connection")
    int32 ControlPortRemote = 14580;  // Remote UDP port for MAVLink control

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Control")
    EPX4ControlMode ControlMode = EPX4ControlMode::Attitude;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Control")
    float StateUpdateRate = 100.0f; // Hz

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Control")
    float HeartbeatRate = 2.0f; // Hz - Send heartbeat every 0.5 seconds
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PX4 Control", meta = (DisplayName = "Use Lockstep Mode"))
    bool bUseLockstepMode = true;

    // Status
    UPROPERTY(BlueprintReadOnly, Category = "PX4 Status")
    bool bConnectedToPX4 = false;

    UPROPERTY(BlueprintReadOnly, Category = "PX4 Status")
    float LastHeartbeatTime = 0.0f;

    // Control Functions
    UFUNCTION(BlueprintCallable, Category = "PX4")
    void SetPX4Active(bool bActive);

    UFUNCTION(BlueprintCallable, Category = "PX4")
    void ConnectToPX4();

    UFUNCTION(BlueprintCallable, Category = "PX4")
    void DisconnectFromPX4();
    
    // Lockstep control
    UFUNCTION(BlueprintCallable, Category = "PX4")
    void SetLockstepMode(bool bEnabled);

    // Thread-safe methods (called by communication thread)
    bool IsConnectedToPX4() const;
    void SendHeartbeat();
	void SimulationUpdate(float FixedDeltaTime);
	bool bIsActive() const { return bUsePX4 && bConnectedToPX4; }
	bool IsLockstepMode() const { return bUseLockstep; }
	void ProcessIncomingMAVLinkData();

	// Frame-independent update for the communication thread
	void ThreadSimulationStep();
	void UpdateCurrentState();

private:
	struct FMotorCommand
	{
		TArray<float> Commands;
		double Timestamp;
	};
    
	FCriticalSection MotorCommandMutex;
	TQueue<FMotorCommand, EQueueMode::Mpsc> PendingMotorCommands;
	
    // TCP Communication (for initial handshake)
    FSocket* TCPListenSocket;  // Server socket that listens for connections
    FSocket* TCPClientSocket;  // Actual connection socket to PX4
    TSharedPtr<FInternetAddr> PX4TCPAddress;

    // UDP Communication (for MAVLink messages)
    FSocket* UDPSendSocket;
    FSocket* UDPRecvSocket;
    TSharedPtr<FInternetAddr> PX4UDPAddress;
    TSharedPtr<FInternetAddr> LocalUDPAddress;
    
    // Communication state
    bool bTCPListening = false;
    bool bTCPConnected = false;
    bool bUDPReady = false;

    // MAVLink System IDs
    uint8 SystemID = 1;
    uint8 ComponentID = 1;
    uint8 TargetSystem = 1;
    uint8 TargetComponent = 1;
    uint64_t HILTimestamp = 0;
    const uint64_t HIL_INTERVAL_US = 10000; // 250Hz = 4ms = 4000 microseconds

    // Message sequence counter
    uint8 MessageSequence = 0;

    // Timing
    float HeartbeatTimer = 0.0f;
    float ConnectionTimeoutTimer = 0.0f;
    static constexpr float ConnectionTimeout = 10.0f;

    // State Storage (main thread)
    FVector CurrentPosition = FVector::ZeroVector;
    FVector CurrentVelocity = FVector::ZeroVector;
    FRotator CurrentRotation = FRotator::ZeroRotator;
    FVector CurrentAngularVelocity = FVector::ZeroVector;
	FVector CurrentGeoCoords = FVector::ZeroVector;
	FVector CurrentMagData = FVector::ZeroVector;
	FVector CurrentAccelData = FVector::ZeroVector;
	float CurrentPressure = 0.0f;
	float CurrentTemperature = 0.0f;
	float CurrentAltitude = 0.f;

    // Threading
    FPX4CommunicationThread* CommunicationThread;
    mutable FCriticalSection StateMutex; // Protects shared state access
    
    // Thread-safe state copies
    FVector ThreadSafePosition;
    FVector ThreadSafeVelocity;
    FRotator ThreadSafeRotation;
    FVector ThreadSafeAngularVelocity;
	FVector ThreadSafeGeoCoords;
	FVector ThreadSafeMagData;
	FVector ThreadSafeAccelData;
	float ThreadSafePressureData;
	float ThreadSafeTemperatureData;
	float ThreadSafeAltitudeData;
	
    bool bThreadSafeDataValid;

    // QuadDroneController reference
    UPROPERTY()
    UQuadDroneController* QuadController;

    // MAVLink Communication Functions
    void SetupTCPServer();
    void AcceptTCPConnection();
    void SetupUDPSockets();
    void CleanupSockets();
    void SendMAVLinkMessage(const uint8* MessageBuffer, uint16 MessageLength);
    void ParseMAVLinkData(const uint8* Data, int32 DataLength);
    void SendHILStateQuaternion();
    void SendHILSensor();
    void SendHILGPS();
    void SendHILRCInputs();


    // MAVLink Message Handlers
    void HandleActuatorOutputs(const uint8* MessageBuffer, uint16 MessageLength);
    void HandleHeartbeat(const uint8* MessageBuffer, uint16 MessageLength);
    
    // Helper Functions
    UQuadDroneController* FindQuadController();
    void UpdateConnectionStatus();

	uint64 BaseTimestamp = 0;
	uint64 TimestampOffset = 0;
	FCriticalSection TimestampMutex;
    
	// Add heartbeat to communication thread
	double LastHeartbeatSentTime = 0.0;
    
	// Track send failures
	int32 ConsecutiveSendFailures = 0;
    
	// Get synchronized timestamp
	uint64 GetSynchronizedTimestamp();

	bool bUseLockstep = true;
	uint64 LockstepCounter = 1;

	mavlink_system_t mavlink_system = {1, 1}; // system_id = 1, component_id = 1
	uint64 SimulationStepCounter = 0;
	
	// Thread-safe lockstep timing
	FCriticalSection LockstepMutex;
	double LastLockstepTime = 0.0;
	const double LOCKSTEP_INTERVAL = 0.004; // 4ms = 250Hz

};
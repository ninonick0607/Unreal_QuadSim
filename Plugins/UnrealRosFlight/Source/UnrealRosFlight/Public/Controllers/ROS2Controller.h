#pragma once
// ---------- rclUE Core ----------
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "ROS2Subscriber.h"
#include "ROS2Service.h"              // NEW
#include "rclcUtilities.h"

// ---------- Msgs ----------
#include "Msgs/ROS2Odom.h"
#include "Msgs/ROS2Imu.h"             // NEW
#include "Msgs/ROS2TFMsg.h"
#include "Msgs/ROS2PoseStamped.h"
#include "Msgs/ROS2TwistStamped.h"    // NEW
#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2Header.h"
#include "Msgs/ROS2Time.h"

// ---------- Srvs ----------
#include "Srvs/ROS2SetBool.h"         // NEW (/pause)
#include "Srvs/ROS2Trigger.h"         // NEW (/reset, /step)
#include "ROS2Controller.generated.h"
class ASimulationManager;
class UTimeController;

UCLASS(Blueprintable, BlueprintType)
class UNREALROSFLIGHT_API AROS2Controller : public AActor
{
    GENERATED_BODY()
public:
    AROS2Controller();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // Loop pubs
    UFUNCTION() void UpdateOdometryMessage(class UROS2GenericMsg* InMessage);
    UFUNCTION() void UpdateImuMessage(class UROS2GenericMsg* InMessage);       // NEW
    UFUNCTION() void UpdateTFMessage(class UROS2GenericMsg* InMessage);
    UFUNCTION() void UpdateClockMessage(class UROS2GenericMsg* InMessage);     // NEW
    UFUNCTION() void UpdateDesiredVel(class UROS2GenericMsg* InMessage);       // NEW

    // Sub handlers
    UFUNCTION() void HandleCmdRateStamped(const class UROS2GenericMsg* InMessage); // NEW TwistStamped
    UFUNCTION() void HandleResetCommand(const class UROS2GenericMsg* InMessage);
    UFUNCTION() void HandleHoverCommand(const class UROS2GenericMsg* InMessage);
    UFUNCTION() void HandleAttitudeEuler(const class UROS2GenericMsg* InMessage);
    UFUNCTION() void HandleVelocityCommand(const class UROS2GenericMsg* InMessage);

    // Service handlers
    UFUNCTION() void SrvPause_SetBool(class UROS2GenericSrv* InService);
    UFUNCTION() void SrvReset_Trigger(class UROS2GenericSrv* InService);
    UFUNCTION() void SrvStep_Trigger (class UROS2GenericSrv* InService);


    UFUNCTION() FString GetDroneID() const;
    
private:
    UPROPERTY() TWeakObjectPtr<class ASimulationManager> SimManager;
    UPROPERTY() TWeakObjectPtr<class AQuadPawn> CachedQuadPawn;
    bool bNodeStarted = false;

    // Helper to get QuadPawn (works with ChildActorComponent)
    class AQuadPawn* GetQuadPawn();

    // Rates
    UPROPERTY(EditAnywhere, Category="ROS2|Rates", meta=(ClampMin="1.0"))
    float OdometryFrequencyHz = 100.f;
    UPROPERTY(EditAnywhere, Category="ROS2|Rates", meta=(ClampMin="1.0"))
    float ImuFrequencyHz = 200.f;          // NEW
    UPROPERTY(EditAnywhere, Category="ROS2|Rates", meta=(ClampMin="1.0"))
    float TFFrequencyHz = 50.f;
    UPROPERTY(EditAnywhere, Category="ROS2|Rates", meta=(ClampMin="1.0"))
    float ClockFrequencyHz = 100.f;        // NEW

    // Flat topics (no per-drone suffix)
    UPROPERTY(VisibleAnywhere, Category="ROS2|Topics")
    FString T_Odom   = TEXT("/quadsim/odom");
    UPROPERTY(VisibleAnywhere, Category="ROS2|Topics")
    FString T_Imu    = TEXT("/quadsim/imu");
    UPROPERTY(VisibleAnywhere, Category="ROS2|Topics")
    FString T_CmdRate= TEXT("/resnet/cmd_vel_residual");
    UPROPERTY(VisibleAnywhere, Category="ROS2|Topics")
    FString T_RefVel= TEXT("/ref/vel_local");
    
    // Node & Pubs
    UPROPERTY() UROS2NodeComponent* Node            = nullptr;
    UPROPERTY() UROS2Publisher*     OdomPublisher   = nullptr;
    UPROPERTY() UROS2Publisher*     ImuPublisher    = nullptr; // NEW
    UPROPERTY() UROS2Publisher*     VelRef    = nullptr;
    UPROPERTY() UROS2Publisher*     TfPublisher     = nullptr;
    UPROPERTY() UROS2Publisher*     TfStaticPublisher = nullptr; // NEW (one-shot)
    UPROPERTY() UROS2Publisher*     ClockPublisher  = nullptr; // NEW

    // Sub
    UPROPERTY() UROS2Subscriber*    CmdRateSubscriber = nullptr;

    // Srvs
    UPROPERTY() UROS2Service*       PauseService   = nullptr;
    UPROPERTY() UROS2Service*       ResetService   = nullptr;
    UPROPERTY() UROS2Service*       StepService    = nullptr;

    // Helpers
    FROSTime GetSimTimeStamp() const;
    void PublishTFStaticOnce();  // NEW

    // Optional: hook to your time controller if/when you wire it
    UPROPERTY(EditAnywhere, Category="ROS2|Time")
    TObjectPtr<class UTimeController> TimeController = nullptr;
};

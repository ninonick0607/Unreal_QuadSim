#include "Controllers/ROS2Controller.h"

// --- Standard Includes ---
#include "Kismet/GameplayStatics.h"
#include "Async/Async.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TimerManager.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Camera/CameraComponent.h"
#include "Misc/App.h"
#include "Core/SimulationManager.h"
#include "Core/TimeController.h"
#include "EngineUtils.h"      

// --- ROS 2 Includes ---
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "ROS2Subscriber.h"
#include "rclcUtilities.h"

#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2TFMsg.h"        
#include "Msgs/ROS2TFStamped.h"    
#include "Msgs/ROS2PoseStamped.h"  
#include "Msgs/ROS2Odom.h"
#include "Msgs/ROS2Point.h"        
#include "Msgs/ROS2Img.h"
#include "Msgs/ROS2Float64.h"
#include "Msgs/ROS2Twist.h"
#include "Msgs/ROS2Str.h"
#include "Msgs/ROS2Empty.h"
#include "Msgs/ROS2TF.h"          
#include "Msgs/ROS2Header.h"      
#include "Msgs/ROS2Time.h"        
#include "Msgs/ROS2Pose.h"        
#include "Msgs/ROS2Quat.h"        

// C-Struct Headers (Needed for utility function inputs & low-level assignment)
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"
#include "rosidl_runtime_c/string_functions.h" // For string assign/init/fini

// --- Generic Unreal Includes (no QuadSimCore dependencies) ---
#include "GameFramework/Pawn.h"
#include "Components/ActorComponent.h"
#include "Msgs/ROS2Vec3Stamped.h"


// Quad controller definition
#include "Controllers/QuadDroneController.h"
// Obstacle manager from QuadSimCore
#include "Utility/ObstacleManager.h"
// QuadPawn + RobotCore sensors
#include "Msgs/ROS2Clock.h"
#include "Msgs/ROS2TwistStamped.h"
#include "Pawns/QuadPawn.h"
#include "ROS2Service.h"
#include "Sensors/BaroSensor.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/IMUSensor.h"
#include "Sensors/SensorManagerComponent.h"
#include "Srvs/ROS2SetBool.h"
#include "Srvs/ROS2Trigger.h"

AROS2Controller::AROS2Controller()
{
    PrimaryActorTick.bCanEverTick = false;
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
}

void AROS2Controller::BeginPlay()
{
    Super::BeginPlay();

    if (bNodeStarted) { return; }
    bNodeStarted = true;
    // -- Node init --
    Node = Node ? Node : CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2Node"));
    Node->Name = TEXT("quadsim_node");
    Node->Namespace = TEXT(""); // flat topics exactly as specified
    Node->Init();

    if (UWorld* W = GetWorld())
    {
        if (AActor* A = UGameplayStatics::GetActorOfClass(W, ASimulationManager::StaticClass()))
        {
            SimManager = Cast<ASimulationManager>(A);
            if (!TimeController && SimManager.IsValid())
            {
                TimeController = SimManager->GetTimeController();
                UE_LOG(LogTemp, Display, TEXT("ROS2Controller: Bound TimeController (%s)"),
                    IsValid(TimeController) ? TEXT("OK") : TEXT("null"));
            }
        }
    }

    
    auto ComputeSafeHz = [](float DesiredHz)->float {
        return FMath::Clamp(DesiredHz, 1.f, 400.f);
    };

    const float OdomHz = ComputeSafeHz(OdometryFrequencyHz);
    const float ImuHz  = ComputeSafeHz(ImuFrequencyHz);
    const float TFHz   = ComputeSafeHz(TFFrequencyHz);
    const float ClkHz  = ComputeSafeHz(ClockFrequencyHz);

    // ---------- Publishers ----------
    // /quadsim/odom
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, T_Odom,
        UROS2Publisher::StaticClass(), UROS2OdomMsg::StaticClass(),
        OdomHz, &AROS2Controller::UpdateOdometryMessage,
        UROS2QoS::SensorData, OdomPublisher);

    // /quadsim/imu
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, T_Imu,
        UROS2Publisher::StaticClass(), UROS2ImuMsg::StaticClass(),
        ImuHz, &AROS2Controller::UpdateImuMessage,
        UROS2QoS::SensorData, ImuPublisher);
    
    // /baseline/cmd_vel - Velocity PID output for ResNet enhancement
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, TEXT("/baseline/cmd_vel"),
        UROS2Publisher::StaticClass(), UROS2TwistStampedMsg::StaticClass(),
        100.0f, &AROS2Controller::UpdateBaselineCommand,
        UROS2QoS::Default, BaselineCmdPublisher);

    // /ref/vel_local - Desired velocity reference/setpoint
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, T_RefVel,
        UROS2Publisher::StaticClass(), UROS2TwistStampedMsg::StaticClass(),
        50.0f, &AROS2Controller::UpdateReferenceVelocity,
        UROS2QoS::Default, VelRef);
    
    // /tf
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, TEXT("/tf"),
        UROS2Publisher::StaticClass(), UROS2TFMsgMsg::StaticClass(),
        TFHz, &AROS2Controller::UpdateTFMessage,
        UROS2QoS::DynamicBroadcaster, TfPublisher);

    // /clock
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, TEXT("/clock"),
        UROS2Publisher::StaticClass(), UROS2ClockMsg::StaticClass(),
        ClkHz, &AROS2Controller::UpdateClockMessage,
        UROS2QoS::Default, ClockPublisher);

    // ---------- Subscribers ----------
    // /resnet/cmd_vel_residual - Receive enhanced commands from ResNet
    ROS2_CREATE_SUBSCRIBER(
        Node, this, T_VelRes,
        UROS2TwistStampedMsg::StaticClass(),
        &AROS2Controller::HandleResNetCommand);
    
    // ---------- Services (rclUE macro) ----------
    ROS2_CREATE_SERVICE_SERVER(
        Node, this,
        TEXT("/quadsim/pause"),
        UROS2SetBoolSrv::StaticClass(),
        &AROS2Controller::SrvPause_SetBool);

    ROS2_CREATE_SERVICE_SERVER(
        Node, this,
        TEXT("/quadsim/reset"),
        UROS2TriggerSrv::StaticClass(),
        &AROS2Controller::SrvReset_Trigger);

    ROS2_CREATE_SERVICE_SERVER(
        Node, this,
        TEXT("/quadsim/step"),
        UROS2TriggerSrv::StaticClass(),
        &AROS2Controller::SrvStep_Trigger);

    // Verify QuadPawn connection
    AQuadPawn* QP = GetQuadPawn();
    if (!QP)
    {
        UE_LOG(LogTemp, Error, TEXT("[ROS2Controller] CRITICAL: Could not find QuadPawn! Sensor data will be zeros."));
        UE_LOG(LogTemp, Error, TEXT("[ROS2Controller] Make sure this ROS2Controller is added as a ChildActorComponent to QuadPawn."));
    }
    else
    {
        UE_LOG(LogTemp, Display, TEXT("[ROS2Controller] Successfully connected to QuadPawn: %s"), *QP->GetName());

        if (!QP->SensorManager)
        {
            UE_LOG(LogTemp, Error, TEXT("[ROS2Controller] WARNING: QuadPawn's SensorManager is NULL!"));
        }
        else if (!QP->SensorManager->IMU)
        {
            UE_LOG(LogTemp, Error, TEXT("[ROS2Controller] WARNING: SensorManager's IMU is NULL!"));
        }
        else if (!QP->SensorManager->GPS)
        {
            UE_LOG(LogTemp, Error, TEXT("[ROS2Controller] WARNING: SensorManager's GPS is NULL!"));
        }
        else
        {
            UE_LOG(LogTemp, Display, TEXT("[ROS2Controller] All sensors verified OK."));
        }
    }

    UE_LOG(LogTemp, Display, TEXT("[ROS2] Minimal map ready: /clock, /tf(/_static), /quadsim/odom, /quadsim/imu, srv(/pause,/reset,/step), sub /resnet/cmd_vel_residual"));
}


void AROS2Controller::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller EndPlay called."));
}

AQuadPawn* AROS2Controller::GetQuadPawn()
{
    // Return cached reference if valid
    if (CachedQuadPawn.IsValid())
    {
        return CachedQuadPawn.Get();
    }

    // Try multiple methods to find the QuadPawn
    AQuadPawn* QP = nullptr;

    // Method 1: Check if we're owned by a QuadPawn (ChildActorComponent case)
    if (AActor* Owner = GetOwner())
    {
        QP = Cast<AQuadPawn>(Owner);
        if (QP)
        {
            UE_LOG(LogTemp, Display, TEXT("ROS2Controller: Found QuadPawn via GetOwner(): %s"), *QP->GetName());
            CachedQuadPawn = QP;
            return QP;
        }
    }

    // Method 2: Check if we're attached to a QuadPawn (direct attachment case)
    if (AActor* Parent = GetAttachParentActor())
    {
        QP = Cast<AQuadPawn>(Parent);
        if (QP)
        {
            UE_LOG(LogTemp, Display, TEXT("ROS2Controller: Found QuadPawn via GetAttachParentActor(): %s"), *QP->GetName());
            CachedQuadPawn = QP;
            return QP;
        }
    }

    // Method 3: Search for QuadPawn in the same level (fallback)
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<AQuadPawn> It(World); It; ++It)
        {
            QP = *It;
            UE_LOG(LogTemp, Warning, TEXT("ROS2Controller: Using fallback - found QuadPawn in level: %s"), *QP->GetName());
            CachedQuadPawn = QP;
            return QP;
        }
    }

    static bool bLoggedError = false;
    if (!bLoggedError)
    {
        UE_LOG(LogTemp, Error, TEXT("ROS2Controller: Could not find QuadPawn via any method!"));
        bLoggedError = true;
    }

    return nullptr;
}

void AROS2Controller::UpdateImuMessage(UROS2GenericMsg* InMessage)
{
    auto* QP  = GetQuadPawn();
    auto* Msg = Cast<UROS2ImuMsg>(InMessage);

    if (!QP)
    {
        static bool bLoggedOnce = false;
        if (!bLoggedOnce)
        {
            UE_LOG(LogTemp, Error, TEXT("UpdateImuMessage: QuadPawn is NULL. Is ROS2Controller set up correctly?"));
            bLoggedOnce = true;
        }
        return;
    }
    if (!QP->SensorManager)
    {
        static bool bLoggedOnce = false;
        if (!bLoggedOnce)
        {
            UE_LOG(LogTemp, Error, TEXT("UpdateImuMessage: SensorManager is NULL."));
            bLoggedOnce = true;
        }
        return;
    }
    if (!QP->SensorManager->IMU)
    {
        static bool bLoggedOnce = false;
        if (!bLoggedOnce)
        {
            UE_LOG(LogTemp, Error, TEXT("UpdateImuMessage: IMU sensor is NULL."));
            bLoggedOnce = true;
        }
        return;
    }
    if (!Msg)
    {
        UE_LOG(LogTemp, Error, TEXT("UpdateImuMessage: Message is NULL."));
        return;
    }

    FROSImu I{};
    I.Header.Stamp   = GetSimTimeStamp();
    I.Header.FrameId = TEXT("imu_link");

    // Orientation from your IMU (deg → quat done inside Quaternion())
    const FRotator att_d = QP->SensorManager->IMU->GetLastAttitude();
    I.Orientation = att_d.Quaternion();

    // Angular velocity [rad/s]
    I.AngularVelocity = QP->SensorManager->IMU->GetLastGyroscope();

    // Linear acceleration [m/s^2]
    I.LinearAcceleration = QP->SensorManager->IMU->GetLastAccelerometer();

    Msg->SetMsg(I);
}

void AROS2Controller::UpdateBaselineCommand(UROS2GenericMsg* InMessage)
{
    auto* QP = GetQuadPawn();
    auto* Msg = Cast<UROS2TwistStampedMsg>(InMessage);
    FVector CurrentVelPID;
    if (QP)
    {
        CurrentVelPID = QP->QuadController->GetVelPID();
    }
    if (!QP || !Msg || !QP->QuadController) return;
    FROSTwistStamped TS{};
    TS.Header.Stamp = GetSimTimeStamp();
    TS.Header.FrameId = TEXT("base_link");
    
    FVector VelCmd = CurrentVelPID;

    TS.Twist.Linear.X = VelCmd.X;
    TS.Twist.Linear.Y = VelCmd.Y;
    TS .Twist.Linear.Z = VelCmd.Z;

    Msg->SetMsg(TS);
}

void AROS2Controller::UpdateReferenceVelocity(UROS2GenericMsg* InMessage)
{
    auto* QP = GetQuadPawn();
    auto* Msg = Cast<UROS2TwistStampedMsg>(InMessage);
    if (!QP || !Msg || !QP->QuadController) return;

    FROSTwistStamped TS{};
    TS.Header.Stamp = GetSimTimeStamp();
    TS.Header.FrameId = TEXT("base_link");

    FVector DesiredVel = QP->QuadController->GetDesiredVelocity(); 
    TS.Twist.Linear.X = DesiredVel.X;
    TS.Twist.Linear.Y = DesiredVel.Y;
    TS.Twist.Linear.Z = DesiredVel.Z;

    Msg->SetMsg(TS);
}

void AROS2Controller::HandleResNetCommand(const UROS2GenericMsg* InMessage)
{
    const UROS2TwistStampedMsg* Wrap = Cast<const
    UROS2TwistStampedMsg>(InMessage);
    if (!Wrap) return;

    FROSTwistStamped TS{};
    Wrap->GetMsg(TS);

    // This contains: baseline_cmd + resnet_correction
    const FVector EnhancedVelCmd(TS.Twist.Linear.X, TS.Twist.Linear.Y, TS.Twist.Linear.Z);

    if (AQuadPawn* QP = GetQuadPawn())
    {
        if (QP->QuadController)
        {
            QP->QuadController->SetVelocityEnhanced(EnhancedVelCmd);
        }
    }
}

void AROS2Controller::HandleHoverCommand(const UROS2GenericMsg* InMsg)
{
    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleHoverMessage: InMsg is null")); return; }
    const UROS2Float64Msg* Float64MsgWrapper = Cast<UROS2Float64Msg>(InMsg);
    FROSFloat64 HoverData;
    Float64MsgWrapper->GetMsg(HoverData);
    const int32 HoverHeight = FMath::RoundToInt(HoverData.Data);
    UE_LOG(LogTemp, Log, TEXT("Received Hover Height: %d"), HoverHeight);

    AQuadPawn* QP = GetQuadPawn();
    if (!QP) { UE_LOG(LogTemp, Error, TEXT("HandleHoverCommand: QuadPawn invalid")); return; }

    UE_LOG(LogTemp, Log, TEXT("ROS2Controller: Hover command received - Height: %.2f"), (float)HoverHeight);
    QP->SetExternalHoverHeight((float)HoverHeight);
}

void AROS2Controller::HandleAttitudeEuler(const UROS2GenericMsg* InMsg)
{
    if (!InMsg)
    {
        UE_LOG(LogTemp, Error, TEXT("HandleAttitudeEulerStamped: InMsg is null"));
        return;
    }

    const UROS2Vec3StampedMsg* Vec3StampedMsg = Cast<UROS2Vec3StampedMsg>(InMsg);
    if (!Vec3StampedMsg)
    {
        UE_LOG(LogTemp, Error, TEXT("HandleAttitudeEulerStamped: Cast failed"));
        return;
    }

    AQuadPawn* QP = GetQuadPawn();
    if (!QP)
    {
        UE_LOG(LogTemp, Error, TEXT("HandleAttitudeEuler: QuadPawn invalid"));
        return;
    }

    FROSVec3Stamped StampedData;
    Vec3StampedMsg->GetMsg(StampedData);

    float RollDeg =  FMath::RadiansToDegrees(StampedData.Vector.X);
    float PitchDeg = FMath::RadiansToDegrees(StampedData.Vector.Y);
    float YawDeg =   FMath::RadiansToDegrees(StampedData.Vector.Z);

    UE_LOG(LogTemp, Log, TEXT("ROS2Controller: Attitude command - Roll: %.2f, Pitch: %.2f, Yaw: %.2f"), RollDeg, PitchDeg, YawDeg);
    QP->SetExternalAttitudeCommand(RollDeg, PitchDeg);
}

void AROS2Controller::HandleVelocityCommand(const UROS2GenericMsg* InMsg)
{
    const UROS2TwistMsg* TwistMsgWrapper = Cast<UROS2TwistMsg>(InMsg);
    if (!TwistMsgWrapper) return;

    FROSTwist Twist{};
    TwistMsgWrapper->GetMsg(Twist);

    // NEW semantics:
    // angular = [p,q,r] rad/s (body rates)
    // linear.z = thrust (normalized 0..1 or N)  <-- document which one you use!
    const FVector BodyRateRps(Twist.Angular.X, Twist.Angular.Y, Twist.Angular.Z);
    const float   ThrustCmd = Twist.Linear.Z;

    if (AQuadPawn* QP = GetQuadPawn())
    {
        // TODO: provide a method that applies body-rate + thrust on next step
        // QP->ApplyExternalRateThrust(BodyRateRps, ThrustCmd);
    }
}

void AROS2Controller::HandleResetCommand(const UROS2GenericMsg* InMsg)
{
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Processing 'reset' command (received Empty message)..."));

    AQuadPawn* QP = GetQuadPawn();
    if (!QP) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: QuadPawn invalid")); return; }

    UE_LOG(LogTemp, Log, TEXT("ROS2Controller: Reset command received"));
    QP->ResetRotation();
    QP->ResetPosition();
    if (QP->QuadController) QP->QuadController->ResetPID();
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Reset command processed (direct calls)."));
}

void AROS2Controller::UpdateOdometryMessage(UROS2GenericMsg* InMessage)
{
    auto* QP = GetQuadPawn();
    auto* Msg = Cast<UROS2OdomMsg>(InMessage);

    if (!QP)
    {
        static bool bLoggedOnce = false;
        if (!bLoggedOnce)
        {
            UE_LOG(LogTemp, Error, TEXT("UpdateOdometryMessage: QuadPawn is NULL. Is ROS2Controller set up correctly?"));
            bLoggedOnce = true;
        }
        return;
    }
    if (!QP->SensorManager)
    {
        static bool bLoggedOnce = false;
        if (!bLoggedOnce)
        {
            UE_LOG(LogTemp, Error, TEXT("UpdateOdometryMessage: SensorManager is NULL."));
            bLoggedOnce = true;
        }
        return;
    }
    if (!Msg)
    {
        UE_LOG(LogTemp, Error, TEXT("UpdateOdometryMessage: Message is NULL."));
        return;
    }

    FROSOdom O{};
    O.Header.Stamp = GetSimTimeStamp();
    O.Header.FrameId = TEXT("odom");
    O.ChildFrameId   = TEXT("base_link");

    const FVector pos_m  = QP->SensorManager->GPS ? QP->SensorManager->GPS->GetLastGPS() : FVector::ZeroVector;
    const FRotator att_d = QP->SensorManager->IMU ? QP->SensorManager->IMU->GetLastAttitude() : FRotator::ZeroRotator;
    const FVector v_lin  = QP->SensorManager->IMU ? QP->SensorManager->IMU->GetLastVelocity() : FVector::ZeroVector;
    const FVector w_rps  = QP->SensorManager->IMU ? QP->SensorManager->IMU->GetLastGyroscope() : FVector::ZeroVector;

    O.Pose.Pose.Position = pos_m;
    O.Pose.Pose.Orientation = att_d.Quaternion();

    O.Twist.Twist.Linear  = v_lin;
    O.Twist.Twist.Angular = w_rps;

    Msg->SetMsg(O);
}

FString AROS2Controller::GetDroneID() const
{
    // Note: const_cast needed because GetQuadPawn() is non-const (caches reference)
    AQuadPawn* QP = const_cast<AROS2Controller*>(this)->GetQuadPawn();
    return QP ? QP->GetName() : FString(TEXT("Unknown"));
}

void AROS2Controller::UpdateTFMessage(UROS2GenericMsg* InMsg)
{
    static bool bEmittedStatic = false;

    auto* TfMsg = Cast<UROS2TFMsgMsg>(InMsg);
    auto* Pawn  = GetQuadPawn();
    if (!TfMsg || !Pawn) return;

    FROSTFMsg tfmsg;

    if (!bEmittedStatic)
    {
        geometry_msgs__msg__TransformStamped s;
        geometry_msgs__msg__TransformStamped__init(&s);
        rosidl_runtime_c__String__assign(&s.header.frame_id, "world");
        rosidl_runtime_c__String__assign(&s.child_frame_id,  "odom");
        s.header.stamp.sec = 0; s.header.stamp.nanosec = 0;
        s.transform.rotation.w = 1.0;
        FROSTFStamped us; us.SetFromROS2(s);
        tfmsg.Transforms.Add(us);
        geometry_msgs__msg__TransformStamped__fini(&s);
        bEmittedStatic = true;
    }

    geometry_msgs__msg__TransformStamped d;
    geometry_msgs__msg__TransformStamped__init(&d);
    const FROSTime t = GetSimTimeStamp();
    d.header.stamp.sec = t.Sec; d.header.stamp.nanosec = t.Nanosec;
    rosidl_runtime_c__String__assign(&d.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&d.child_frame_id,  "base_link");
    d.transform = UROS2Utils::TransformUEToROS(Pawn->GetActorTransform());

    FROSTFStamped ud; ud.SetFromROS2(d);
    tfmsg.Transforms.Add(ud);
    TfMsg->SetMsg(tfmsg);

    geometry_msgs__msg__TransformStamped__fini(&d);
}

void AROS2Controller::UpdateClockMessage(UROS2GenericMsg* InMessage)
{
    auto* Msg = Cast<UROS2ClockMsg>(InMessage);
    if (!Msg) return;

    // Prefer your UTimeController if available
    FROSTime t = GetSimTimeStamp();
    FROSClock C{}; C.Clock = t;
    Msg->SetMsg(C);
}

FROSTime AROS2Controller::GetSimTimeStamp() const
{
    double seconds = 0.0;
    if (SimManager.IsValid())
    {
        seconds = SimManager->GetCurrentSimTimeSeconds();
    }
    else
    {
        // fallback to wall-clock if manager not present
        const FTimespan TS = FDateTime::UtcNow().GetTimeOfDay();
        seconds = TS.GetTotalSeconds();
    }
    FROSTime Out{};
    Out.Sec     = static_cast<int32>(seconds);
    Out.Nanosec = static_cast<uint32>((seconds - Out.Sec) * 1e9);
    return Out;
}

void AROS2Controller::PublishTFStaticOnce()
{
    if (!IsValid(TfStaticPublisher) || !IsValid(TfStaticPublisher->TopicMessage)) return;

    FROSTFMsg tfmsg;

    // world -> odom (identity)
    {
        geometry_msgs__msg__TransformStamped tf_cs;
        geometry_msgs__msg__TransformStamped__init(&tf_cs);
        rosidl_runtime_c__String__assign(&tf_cs.header.frame_id, "world");
        rosidl_runtime_c__String__assign(&tf_cs.child_frame_id,  "odom");
        tf_cs.header.stamp.sec = 0;
        tf_cs.header.stamp.nanosec = 0;
        tf_cs.transform.rotation.w = 1.0; // identity

        FROSTFStamped ue_stamp; ue_stamp.SetFromROS2(tf_cs);
        tfmsg.Transforms.Add(ue_stamp);

        geometry_msgs__msg__TransformStamped__fini(&tf_cs);
    }

    // Reuse the publisher’s existing message object
    if (auto* Wrap = Cast<UROS2TFMsgMsg>(TfStaticPublisher->TopicMessage))
    {
        Wrap->SetMsg(tfmsg);
        TfStaticPublisher->Publish(); // <- no template args, no TUEMessage issues
    }
}


void AROS2Controller::SrvPause_SetBool(UROS2GenericSrv* InService)
{
    UROS2SetBoolSrv* S = Cast<UROS2SetBoolSrv>(InService);
    if (!S) return;

    FROSSetBoolReq req;  S->GetRequest(req);
    bool ok = false;

    if (SimManager.IsValid())
    {
        if (req.bData) SimManager->PausePhysics();
        else           SimManager->ResumePhysics();
        ok = true;
    }
    else
    {
        // Fallback: world dilation
        if (UWorld* W = GetWorld()) { W->GetWorldSettings()->SetTimeDilation(req.bData ? 0.0001f : 1.0f); ok = true; }
    }

    FROSSetBoolRes res;
    res.bSuccess = ok;
    res.Message  = req.bData ? TEXT("Paused (sim)") : TEXT("Resumed (sim)");
    S->SetResponse(res);
}

void AROS2Controller::SrvReset_Trigger(UROS2GenericSrv* InService)
{
    UROS2TriggerSrv* S = Cast<UROS2TriggerSrv>(InService);
    if (!S) return;

    bool ok = false;
    if (AQuadPawn* QP = GetQuadPawn())
    {
        QP->ResetRotation();
        QP->ResetPosition();
        if (QP->QuadController) QP->QuadController->ResetPID();
        ok = true;
    }

    FROSTriggerRes res;
    res.bSuccess = ok;
    res.Message  = ok ? TEXT("Reset done") : TEXT("Reset failed (no pawn)");
    S->SetResponse(res);
}

void AROS2Controller::SrvStep_Trigger(UROS2GenericSrv* InService)
{
    UROS2TriggerSrv* S = Cast<UROS2TriggerSrv>(InService);
    if (!S) return;

    bool ok = false;
    if (SimManager.IsValid())
    {
        SimManager->RequestSimulationStep();
        ok = true;
    }
    else if (UWorld* W = GetWorld())
    {
        W->Tick(LEVELTICK_All, (TimeController ? TimeController->GetFixedDeltaTime() : 1.f/200.f));
        ok = true;
    }

    FROSTriggerRes res; res.bSuccess = ok; res.Message = ok ? TEXT("Step requested") : TEXT("No sim manager/world");
    S->SetResponse(res);
}

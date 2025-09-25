#include "Controllers/PX4Component.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "Sensors/SensorManagerComponent.h"
#include "Sensors/IMUSensor.h"
#include "Sensors/GPSSensor.h"
#include "Sensors/BaroSensor.h"
#include "Sensors/MagSensor.h"
#include "CoordinateTransform.h"

#pragma warning(push)
#pragma warning(disable: 4005 4996)
#include "common/mavlink.h"       // <-- ONLY here (in .cpp)
#pragma warning(pop)

DEFINE_LOG_CATEGORY_STATIC(LogPX4, Log, All);

// --------- UE lifecycle ---------
UPX4Component::UPX4Component()
{
    PrimaryComponentTick.bCanEverTick = true;
    // Default motor mapping: PX4 [FR, BL, FL, BR] -> Ours [FL, FR, BL, BR]
    Px4ToOurMotorMap = {1, 3, 0, 2};
}

void UPX4Component::BeginPlay()
{
	Super::BeginPlay();
	if (bEnablePX4) Connect();
}

void UPX4Component::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Comm) { Comm->Join(); delete Comm; Comm=nullptr; }
	Disconnect();
	Super::EndPlay(EndPlayReason);
}

void UPX4Component::TickComponent(float, ELevelTick, FActorComponentTickFunction*)
{
    // Inactive when PX4 button is off
    if (!bEnablePX4)
    {
        return;
    }

    UpdateCurrentState();

    if (bListening && !bTCPConnected) AcceptOnce();

    // deliver motor commands on game thread only when enabled
    FMotorCmd C;
    while (MotorQueue.Dequeue(C))
    {
        if (auto* Pawn = Cast<AQuadPawn>(GetOwner()))
        if (auto* Ctrl = Pawn->QuadController)
        {
            Ctrl->ApplyMotorCommands(C.Values);
        }
    }
}

// --------- Connection control ---------
void UPX4Component::Connect()
{
	if (bListening || bTCPConnected) return;
	CreateTCPServer();
	if (!Comm) { Comm = new FPX4CommThread(this); Comm->Start(); }
}

void UPX4Component::Disconnect()
{
	if (Comm) { Comm->Join(); delete Comm; Comm=nullptr; }
	CloseSockets();
	bPX4Connected = false;
	bTCPConnected = false;
	bListening    = false;
	bSawHeartbeat = false;
}

// --------- TCP helpers ---------
void UPX4Component::CreateTCPServer()
{
	CloseSockets();
	ListenSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("PX4SimTCP"), false);
	if (!ListenSocket) { UE_LOG(LogPX4, Error, TEXT("TCP create failed")); return; }

	ListenSocket->SetNonBlocking(true);
	ListenSocket->SetReuseAddr(true);

	auto BindAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	BindAddr->SetAnyAddress(); BindAddr->SetPort(SimulatorTcpPort);

	if (!ListenSocket->Bind(*BindAddr))
	{
		UE_LOG(LogPX4, Error, TEXT("TCP bind %d failed"), SimulatorTcpPort);
		CloseSockets(); return;
	}
	if (!ListenSocket->Listen(1))
	{
		UE_LOG(LogPX4, Error, TEXT("TCP listen %d failed"), SimulatorTcpPort);
		CloseSockets(); return;
	}
	bListening = true;
	UE_LOG(LogPX4, Log, TEXT("Listening for PX4 on TCP %d"), SimulatorTcpPort);
}

void UPX4Component::AcceptOnce()
{
	bool bHasPending=false;
	if (ListenSocket->HasPendingConnection(bHasPending) && bHasPending)
	{
		auto Remote = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
		ClientSocket = ListenSocket->Accept(*Remote, TEXT("PX4Client"));
		if (ClientSocket)
		{
			ClientSocket->SetNonBlocking(true);
			ClientSocket->SetNoDelay(true);
			int32 S=0,R=0; ClientSocket->SetSendBufferSize(1<<16,S); ClientSocket->SetReceiveBufferSize(1<<16,R);
			bTCPConnected = true; bPX4Connected = true;
			UE_LOG(LogPX4, Log, TEXT("PX4 connected from %s"), *Remote->ToString(true));
			ListenSocket->Close(); ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket); ListenSocket=nullptr; bListening=false;
		}
	}
}

void UPX4Component::CloseSockets()
{
	auto* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	if (ClientSocket) { ClientSocket->Close(); SS->DestroySocket(ClientSocket); ClientSocket=nullptr; }
	if (ListenSocket) { ListenSocket->Close(); SS->DestroySocket(ListenSocket); ListenSocket=nullptr; }
}

void UPX4Component::SendBytes(const uint8* Data, int32 Len)
{
	if (!ClientSocket || !bTCPConnected) return;
	int32 Sent=0; const uint8* P = Data; int32 Left = Len;
	while (Left > 0)
	{
		if (!ClientSocket->Send(P, Left, Sent) || Sent <= 0)
		{
			auto err = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
			if (err == SE_EWOULDBLOCK) { FPlatformProcess::Sleep(0.0001f); continue; }
			UE_LOG(LogPX4, Error, TEXT("TCP send failed err=%d"), (int32)err);
			break;
		}
		P += Sent; Left -= Sent;
	}
}

uint64 UPX4Component::NowUsec() const
{
	return static_cast<uint64>(FPlatformTime::Seconds() * 1'000'000.0);
}

// --------- Sensors → thread-safe state (use your body exactly) ---------

void UPX4Component::UpdateCurrentState()
{
    if (AQuadPawn* QuadPawn = Cast<AQuadPawn>(GetOwner()))
    {
        // --- Your sensors exactly as you wrote ---
        FVector GPSPositionMeters = QuadPawn->SensorManager->GPS->GetLastGPS();                  // meters (Unreal world FLU)
        FVector GeographicCoords  = QuadPawn->SensorManager->GPS->GetGeographicCoordinates();    // lat,lon,alt (deg,deg,m)

        FVector IMUVelocity       = QuadPawn->SensorManager->IMU->GetLastVelocity();             // m/s (FLU world)
        FVector AccelData         = QuadPawn->SensorManager->IMU->GetLastAccelerometer();        // m/s^2 (IMU body FLU)
        FRotator IMUAttitude      = QuadPawn->SensorManager->IMU->GetLastAttitude();             // deg (FLU world)
        FVector IMUAngularVelDeg  = QuadPawn->SensorManager->IMU->GetLastGyroscopeDegrees();     // deg/s (IMU body FLU)

        float   Pressure          = QuadPawn->SensorManager->Barometer->GetLastPressure();       // Pa
        float   Temperature       = QuadPawn->SensorManager->Barometer->GetLastTemperature();    // C
        float   BaroAltitude      = QuadPawn->SensorManager->Barometer->GetEstimatedAltitude();  // m

        FVector MagData           = QuadPawn->SensorManager->Magnetometer->GetLastMagField();    // Gauss (IMU body frame)

        // --- Coordinate transforms (match what MAVLink expects) ---
        // Inputs here are already meters and m/s from sensors
    	const FVector  PosNED         = UCoordinateTransform::UnrealMetersToNED(GPSPositionMeters);
    	const FVector  VelNED         = UCoordinateTransform::UnrealMetersVelocityToNED(IMUVelocity);
    	const FRotator RotNED_Local   = UCoordinateTransform::UnrealRotationToNED(IMUAttitude);
    	const FVector  AccelFRD_Local = UCoordinateTransform::UnrealBodyAccelToFRD(AccelData);
    	const FVector  AngVelFRD_Local= UCoordinateTransform::UnrealBodyAngVelToFRD(IMUAngularVelDeg);
        FVector  MagFRD_Local  = UCoordinateTransform::UnrealBodyToFRD(MagData);


        // --- Write thread-safe cache that TX reads ---
	    {
        	FScopeLock Lock(&StateCS);

        	// Use _Local suffix for locals, and write into members via this->
        	this->PositionNED   = PosNED;          
        	this->VelocityNED   = VelNED;          
        	this->AccelFRD      = AccelFRD_Local;  
        	this->RotNED        = RotNED_Local;    
        	this->AngVelFRD     = AngVelFRD_Local; 

        	this->GeoLLA        = GeographicCoords;
        	this->AltM          = BaroAltitude;
        	// If magnetometer is unavailable or zeroed (no GeoRef), synthesize a sane Earth field
        	// Typical magnitude ~0.25..0.65 Gauss. Choose mid-lat vector in NED then rotate to FRD.
        	if (MagFRD_Local.Size() < 0.05f)
        	{
        		// Rough mid-latitude field components in NED: North=0.2 G, East=0.0 G, Down=0.45 G
        		const FVector magNED(0.20f, 0.00f, 0.45f);
        		const FQuat qNED_local(RotNED_Local);
        		MagFRD_Local = qNED_local.UnrotateVector(magNED);
        	}
        	this->MagGauss      = MagFRD_Local;
        	this->PressurePa    = Pressure;
        	this->TemperatureC  = Temperature;

        	this->bStateValid   = true;
	    }
    }
}

void UPX4Component::SimulationUpdate(float /*FixedDeltaTime*/)
{
	// intentionally empty in non-lockstep mode
}

// --------- RX path ---------
void UPX4Component::ProcessIncomingTCP()
{
	if (!ClientSocket || !bTCPConnected) return;
	uint32 Pending=0;
	if (!ClientSocket->HasPendingData(Pending) || Pending==0) return;

	uint8 Buf[4096]; int32 Read=0;
	const int32 Want = FMath::Min<int32>((int32)Pending, (int32)sizeof(Buf));
	if (ClientSocket->Recv(Buf, Want, Read) && Read > 0)
	{
		mavlink_message_t msg; mavlink_status_t st;
		for (int i=0;i<Read;i++)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, Buf[i], &msg, &st))
			{
				switch (msg.msgid)
				{
					case MAVLINK_MSG_ID_HEARTBEAT:
					{
						mavlink_heartbeat_t hb; mavlink_msg_heartbeat_decode(&msg, &hb);
						OnHeartbeat_Parsed(msg.sysid, msg.compid);
					} break;

                case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                {
                    mavlink_hil_actuator_controls_t ac; mavlink_msg_hil_actuator_controls_decode(&msg, &ac);
                    // Use configurable mapping from PX4 indices to our thruster indices
                    TArray<float> u; u.SetNum(4);
                    for (int ourIdx = 0; ourIdx < 4; ++ourIdx)
                    {
                        int px4Idx = (Px4ToOurMotorMap.IsValidIndex(ourIdx) ? Px4ToOurMotorMap[ourIdx] : ourIdx);
                        float v = (px4Idx >=0 && px4Idx < 16) ? ac.controls[px4Idx] : 0.0f;
                        u[ourIdx] = FMath::Clamp(v, 0.f, 1.f);
                    }
                    static int logc=0; if (bLogActuatorMapping && ((logc++ % 50)==0))
                    {
                        UE_LOG(LogPX4, Warning, TEXT("HIL_ACT: raw=[%.3f %.3f %.3f %.3f] mapped=[%.3f %.3f %.3f %.3f] map=[%d %d %d %d]"),
                            ac.controls[0], ac.controls[1], ac.controls[2], ac.controls[3],
                            u[0], u[1], u[2], u[3],
                            Px4ToOurMotorMap.IsValidIndex(0)?Px4ToOurMotorMap[0]:-1,
                            Px4ToOurMotorMap.IsValidIndex(1)?Px4ToOurMotorMap[1]:-1,
                            Px4ToOurMotorMap.IsValidIndex(2)?Px4ToOurMotorMap[2]:-1,
                            Px4ToOurMotorMap.IsValidIndex(3)?Px4ToOurMotorMap[3]:-1);
                    }
                    EnqueueMotorCommand(u);
                } break;

					default: break;
				}
			}
		}
	}
}

void UPX4Component::OnHeartbeat_Parsed(uint8 SrcSys, uint8 SrcComp)
{
	TargetSys = SrcSys; TargetComp = SrcComp;
	if (!bSawHeartbeat) { bSawHeartbeat = true; FirstHeartbeatUsec = NowUsec(); }
}

void UPX4Component::EnqueueMotorCommand(const TArray<float>& Norm01)
{
	FMotorCmd C; C.Values = Norm01; C.Stamp = FPlatformTime::Seconds();
	MotorQueue.Enqueue(C);
}

// --------- TX: MAVLink (sensors are actually used) ---------
void UPX4Component::SendHeartbeat()
{
	mavlink_message_t m; mavlink_heartbeat_t hb{};
	hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
	hb.autopilot = MAV_AUTOPILOT_INVALID;
	hb.base_mode = 0;
	hb.system_status = MAV_STATE_ACTIVE;
	mavlink_msg_heartbeat_encode(SysId, CompId, &m, &hb);
	uint8 out[MAVLINK_MAX_PACKET_LEN]; uint16 n = mavlink_msg_to_send_buffer(out, &m);
	SendBytes(out, n);
}

void UPX4Component::SendHILSensor()
{
	// Read state from sensor cache
	FVector posNED, velNED, accFRD, mag; FRotator rotNED; FVector angFRD; float pressPa, tempC, altM;
	{
		FScopeLock L(&StateCS);
		if (!bStateValid) return;
		posNED=PositionNED;
		velNED=VelocityNED;
		accFRD=AccelFRD;
		rotNED=RotNED;
		angFRD=AngVelFRD;
		
		mag=MagGauss;
		pressPa=PressurePa;
		tempC=TemperatureC;
		altM=AltM;
	}

	const FQuat qNED = FQuat(rotNED);
	const FVector gNED(0,0,9.81f);                  
	const FVector gBodyFRD = qNED.UnrotateVector(gNED);
	// 'accFRD' = body FRD linear accel (m/s^2) from your cache
	const FVector specific = accFRD - gBodyFRD;

	mavlink_hil_sensor_t s{}; 
	s.time_usec = NowUsec();
	
	s.xacc = specific.X;
	s.yacc = specific.Y;
	s.zacc = specific.Z;
	
	s.xgyro=angFRD.X;
	s.ygyro=angFRD.Y;
	s.zgyro=angFRD.Z;

	s.xmag = mag.X;
	s.ymag = mag.Y;
	s.zmag = mag.Z;
	
	s.abs_pressure = FMath::Clamp(pressPa/100.f, 300.f, 1200.f);
	s.diff_pressure = 0.f;
	s.pressure_alt = altM;
	s.temperature  = FMath::Clamp(tempC, -40.f, 85.f);
	s.fields_updated = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<12);

	mavlink_message_t m; mavlink_msg_hil_sensor_encode(SysId, CompId, &m, &s);
	uint8 out[MAVLINK_MAX_PACKET_LEN]; uint16 n = mavlink_msg_to_send_buffer(out, &m);
	SendBytes(out, n);
}

void UPX4Component::SendHILStateQuaternion()
{
	FVector velNED; FRotator rotNED; FVector angFRD; FVector lla;
	{
		FScopeLock L(&StateCS);
		if (!bStateValid) return;
		velNED=VelocityNED;
		rotNED=RotNED;
		angFRD=AngVelFRD;
		lla=GeoLLA;
	}
	FQuat q = FQuat(rotNED);
	q.Normalize();

	mavlink_hil_state_quaternion_t h{};
	h.time_usec = NowUsec();
	h.attitude_quaternion[0]=q.W;
	h.attitude_quaternion[1]=q.X;
	h.attitude_quaternion[2]=q.Y;
	h.attitude_quaternion[3]=q.Z;
	
	h.rollspeed=angFRD.X;
	h.pitchspeed=angFRD.Y;
	h.yawspeed=angFRD.Z;
	
	h.lat=(int32)(lla.X*1e7);
	h.lon=(int32)(lla.Y*1e7);
	h.alt=(int32)(lla.Z*1000.0f);
	
	h.vx=(int16)velNED.X*100;
	h.vy=(int16)velNED.Y*100;
	h.vz=(int16)velNED.Z*100;

	mavlink_message_t m; mavlink_msg_hil_state_quaternion_encode(SysId, CompId, &m, &h);
	uint8 out[MAVLINK_MAX_PACKET_LEN]; uint16 n = mavlink_msg_to_send_buffer(out, &m);
	SendBytes(out, n);
}

void UPX4Component::SendHILGPS()
{
	FVector velNED; FVector lla;
	{
		FScopeLock L(&StateCS);
		if (!bStateValid) return;
		velNED=VelocityNED; lla=GeoLLA;
	}
	mavlink_hil_gps_t g{};
	g.time_usec = NowUsec();
	g.lat=(int32)(lla.X*1e7);
	g.lon=(int32)(lla.Y*1e7);
	g.alt=(int32)(lla.Z*1000.0f);
	g.eph=120;
	g.epv=150;
	
	g.vn=(int16)velNED.X*100;
	g.ve=(int16)velNED.Y*100;
	g.vd=(int16)velNED.Z*100;
	
	const float gs = FMath::Sqrt(velNED.X*velNED.X + velNED.Y*velNED.Y);
	g.vel = (uint16)(gs*100.0f);
	const float cog_deg = FMath::RadiansToDegrees(FMath::Atan2(velNED.Y, velNED.X));
	g.cog = (uint16)(FMath::Fmod(cog_deg + 360.f, 360.f) * 100.0f);
	g.fix_type=3;
	g.satellites_visible=12;

	mavlink_message_t m; mavlink_msg_hil_gps_encode(SysId, CompId, &m, &g);
	uint8 out[MAVLINK_MAX_PACKET_LEN]; uint16 n = mavlink_msg_to_send_buffer(out, &m);
	SendBytes(out, n);
}

void UPX4Component::SendHILRC()
{
	mavlink_hil_rc_inputs_raw_t rc{};
	rc.time_usec = NowUsec();
	rc.chan1_raw=1500; rc.chan2_raw=1500; rc.chan3_raw=1500; rc.chan4_raw=1500;
	rc.chan5_raw=2000; rc.chan6_raw=1500; rc.chan7_raw=1500; rc.chan8_raw=2000;
	rc.rssi = 255;

	mavlink_message_t m; mavlink_msg_hil_rc_inputs_raw_encode(SysId, CompId, &m, &rc);
	uint8 out[MAVLINK_MAX_PACKET_LEN]; uint16 n = mavlink_msg_to_send_buffer(out, &m);
	SendBytes(out, n);
}

// --------- Comm thread ---------
uint32 FPX4CommThread::Run()
{
	const double dt250 = 1.0/250.0, dt50=1.0/50.0, dtHB=0.5;
	double now = FPlatformTime::Seconds();
	Last250 = Last50 = LastHB = now;

	while (!bStop)
	{
		Owner->ProcessIncomingTCP();

		if (Owner->IsPX4Ready())
		{
			// grace period after the very first heartbeat
			if (const uint64 hb0 = Owner->GetFirstHeartbeatUsec();
				hb0 && (Owner->NowUsec() - hb0) > 300'000ULL)
			{
				now = FPlatformTime::Seconds();

				if ((now - Last250) >= dt250) {
					Owner->SendHILSensor();
					Owner->SendHILStateQuaternion();
					Last250 = now;
				}
				if ((now - Last50) >= dt50) {
					Owner->SendHILGPS();
					Owner->SendHILRC();
					Last50 = now;
				}
				if ((now - LastHB) >= dtHB) {
					Owner->SendHeartbeat();
					LastHB = now;
				}
			}
		}

		FPlatformProcess::Sleep(1.0f/250.0f);
	}
	return 0;
}


void FPX4CommThread::Start()
{
	if (!Thread) Thread = FRunnableThread::Create(this, TEXT("PX4Comm"), 512*1024, TPri_AboveNormal);
}
void FPX4CommThread::Join()
{
	if (Thread) { Thread->Kill(true); delete Thread; Thread=nullptr; }
}

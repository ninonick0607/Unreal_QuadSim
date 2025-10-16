#include "SimulationCore/Public/Core/SimulationManager.h"
#include "SimulationCore/Public/Core/TimeController.h"
#include "SimulationCore/Public/Interfaces/ISimulatable.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Camera/PlayerCameraManager.h"
#include "imgui.h"
#include "EngineUtils.h"                 // TActorIterator
#include "GameFramework/Pawn.h"          // APawn
#include "Camera/CameraComponent.h"      // UCameraComponent
#include "Camera/CameraActor.h"       
#include "Physics/PhysicsInterfaceCore.h"

ASimulationManager::ASimulationManager()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    
    CurrentSimulationMode = ESimulationMode::Realtime;
    CurrentSimulationTime = 0.0f;
    CurrentEpisode = 0;
    CurrentStep = 0;
    bWaitingForExternalCommand = false;
    MaxStepsPerFrame = 10;
    bShowImGuiWindow = true;
    SelectedRobotIndex = 0;

    bStepRequested = false;
    bIsStepping = false;

}

void ASimulationManager::BeginPlay()
{
    Super::BeginPlay();
    
    // Create Time Controller
    TimeController = NewObject<UTimeController>(this, TEXT("TimeController"));
    
    // Configure physics settings for better control
    if (UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get())
    {
        // Store original settings
        OriginalMaxPhysicsStep = PhysicsSettings->MaxPhysicsDeltaTime;
        OriginalSubstepping = PhysicsSettings->bSubstepping;
        
        // Enable substepping for more deterministic physics
        PhysicsSettings->bSubstepping = true;
        PhysicsSettings->MaxSubstepDeltaTime = 0.01667f; // 60Hz
        PhysicsSettings->MaxSubsteps = 6;
    }
    
    UE_LOG(LogTemp, Warning, TEXT("SimulationManager initialized with mode: %s"), 
           *UEnum::GetValueAsString(CurrentSimulationMode));
    
    // Find existing robots in the scene
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsWithInterface(GetWorld(), USimulatable::StaticClass(), FoundActors);
    
    for (AActor* Actor : FoundActors)
    {
        RegisterRobot(Actor);
    }
    
    UE_LOG(LogTemp, Warning, TEXT("Found and registered %d robots"), RegisteredRobots.Num());
}

void ASimulationManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Restore original physics settings
    if (UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get())
    {
        PhysicsSettings->MaxPhysicsDeltaTime = OriginalMaxPhysicsStep;
        PhysicsSettings->bSubstepping = OriginalSubstepping;
    }
    
    RegisteredRobots.Empty();
    Super::EndPlay(EndPlayReason);
}

// In SimulationManager.cpp

void ASimulationManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Handle different simulation modes
	switch (CurrentSimulationMode)
	{
	case ESimulationMode::Realtime:
		// Normal real-time simulation
		StepSimulation(DeltaTime);
		break;
        
	case ESimulationMode::FastForward:
		// Accelerated simulation
		StepSimulation(DeltaTime);
		break;
        
	case ESimulationMode::Paused:
		// Only step when requested
		if (bStepRequested)
		{
			ExecuteSimulationStep(TimeController->GetFixedDeltaTime());
			bStepRequested = false;
		}
		break;
        
	case ESimulationMode::Lockstep:
		// Step once, then wait for external command
		if (!bWaitingForExternalCommand)
		{
			ExecuteSimulationStep(TimeController->GetFixedDeltaTime());
			bWaitingForExternalCommand = true;
		}
		break;
	}
}
void ASimulationManager::StepSimulation(float DeltaTime)
{
    if (!TimeController)
    {
        return;
    }
    
    // For lockstep and paused modes, use fixed timestep directly
    if (CurrentSimulationMode == ESimulationMode::Lockstep || 
        CurrentSimulationMode == ESimulationMode::Paused)
    {
        ExecuteSimulationStep(TimeController->GetFixedDeltaTime());
    }
    else
    {
        // For realtime and fast forward, use the accumulator pattern
        TimeController->AccumulateTime(DeltaTime);
        
        int32 StepsExecuted = 0;
        while (TimeController->ShouldStep() && StepsExecuted < MaxStepsPerFrame)
        {
            float FixedDeltaTime = TimeController->GetFixedDeltaTime();
            ExecuteSimulationStep(FixedDeltaTime);
            TimeController->ConsumeTime();
            StepsExecuted++;
        }
        
        // Only warn if we truly still have leftover time to consume.
        // Hitting the per-frame cap with no leftover is expected when fast-forwarding.
        if (StepsExecuted >= MaxStepsPerFrame && TimeController->ShouldStep())
        {
            UE_LOG(LogTemp, Warning, TEXT("Simulation falling behind: hit per-frame cap %d and still have leftover (accum=%.4fs, fixed=%.4fs)"),
                   MaxStepsPerFrame,
                   TimeController->GetAccumulatedTime(),
                   TimeController->GetFixedDeltaTime());
        }
    }
}

void ASimulationManager::ExecuteSimulationStep(float FixedDeltaTime)
{
    // Update simulation time
    CurrentSimulationTime += FixedDeltaTime;
    CurrentStep++;
    
    // Update all robots with fixed timestep
    UpdateAllRobots(FixedDeltaTime);
}

void ASimulationManager::UpdateAllRobots(float DeltaTime)
{
    for (AActor* Robot : RegisteredRobots)
    {
        if (Robot && Robot->GetClass()->ImplementsInterface(USimulatable::StaticClass()))
        {
            ISimulatable::Execute_SimulationUpdate(Robot, DeltaTime);
        }
    }
}

void ASimulationManager::SetSimulationMode(ESimulationMode NewMode)
{
    if (CurrentSimulationMode != NewMode)
    {
        // Restore normal time dilation when leaving non-realtime modes
        if (CurrentSimulationMode != ESimulationMode::Realtime)
        {
            GetWorld()->GetWorldSettings()->SetTimeDilation(1.0f);
        }
        
        CurrentSimulationMode = NewMode;
        bWaitingForExternalCommand = false;
        
        UE_LOG(LogTemp, Display, TEXT("Simulation mode changed to: %s"), 
               *UEnum::GetValueAsString(NewMode));
        
        // Reset time controller when switching modes
        if (TimeController)
        {
            TimeController->Reset();
        }
        
        // Apply initial settings for new mode
        switch (NewMode)
        {
        case ESimulationMode::Paused:
        case ESimulationMode::Lockstep:
            GetWorld()->GetWorldSettings()->SetTimeDilation(0.0001f);
            break;
        case ESimulationMode::FastForward:
            // Use the TimeController to scale time; keep world dilation at 1.0 to avoid double scaling
            GetWorld()->GetWorldSettings()->SetTimeDilation(1.0f);
            break;
        default:
            GetWorld()->GetWorldSettings()->SetTimeDilation(1.0f);
            break;
        }
    }
}

void ASimulationManager::RegisterRobot(AActor* Robot)
{
    if (!Robot || RegisteredRobots.Contains(Robot))
    {
        return;
    }
    
    // Check if it implements ISimulatable
    if (Robot->GetClass()->ImplementsInterface(USimulatable::StaticClass()))
    {
        RegisteredRobots.Add(Robot);
        UE_LOG(LogTemp, Display, TEXT("Registered robot: %s"), *Robot->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Actor %s does not implement ISimulatable interface"), 
               *Robot->GetName());
    }
}

void ASimulationManager::UnregisterRobot(AActor* Robot)
{
    if (Robot)
    {
        RegisteredRobots.Remove(Robot);
        UE_LOG(LogTemp, Display, TEXT("Unregistered robot: %s"), *Robot->GetName());
    }
}

void ASimulationManager::SetTimeScale(float NewTimeScale)
{
    if (TimeController)
    {
        // Drive global sim speed via world time dilation so physics and game systems follow.
        // Avoid double-scaling by keeping the internal TimeController at 1x.
        TimeController->SetTimeScale(1.0f);

        const float Clamped = FMath::Clamp(NewTimeScale, 0.0001f, 100.0f);
        if (UWorld* World = GetWorld())
        {
            if (AWorldSettings* WS = World->GetWorldSettings())
            {
                WS->SetTimeDilation(Clamped);
            }
        }
    }
}

void ASimulationManager::ResetSimulation()
{
    CurrentSimulationTime = 0.0f;
    CurrentStep = 0;
    
    if (TimeController)
    {
        TimeController->Reset();
    }
    
    // Reset all robots
    for (AActor* Robot : RegisteredRobots)
    {
        if (Robot && Robot->GetClass()->ImplementsInterface(USimulatable::StaticClass()))
        {
            ISimulatable::Execute_ResetRobot(Robot);
        }
    }
    
    UE_LOG(LogTemp, Display, TEXT("Simulation reset"));
}

void ASimulationManager::PausePhysics()
{
    SetSimulationMode(ESimulationMode::Paused);
}

void ASimulationManager::ResumePhysics()
{
    SetSimulationMode(ESimulationMode::Realtime);
}

void ASimulationManager::RequestSimulationStep()
{
    if (CurrentSimulationMode == ESimulationMode::Lockstep && bWaitingForExternalCommand)
    {
        bWaitingForExternalCommand = false;
        UE_LOG(LogTemp, Verbose, TEXT("External step command received"));
    }
    else if (CurrentSimulationMode == ESimulationMode::Paused)
    {
        // Setting the flag instead of calling the functions directly
        bStepRequested = true;
    }
}

void ASimulationManager::StartNewEpisode()
{
    CurrentEpisode++;
    CurrentStep = 0;
    ResetSimulation();
    
    UE_LOG(LogTemp, Display, TEXT("Started episode %d"), CurrentEpisode);
}

void ASimulationManager::GetAvailableCameras(TArray<AActor*>& OutCameras) const
{
    OutCameras.Reset();

    UWorld* World = GetWorld();
    if (!World) return;

    // 1) Any pawn that has a CameraComponent (eg. your TopDownCameraPawn or custom pawns)
    for (TActorIterator<APawn> It(World); It; ++It)
    {
        APawn* P = *It;
        if (!IsValid(P)) continue;
        if (P->FindComponentByClass<UCameraComponent>())
        {
            OutCameras.Add(P);
        }
    }

    // 2) Standalone ACameraActor
    for (TActorIterator<ACameraActor> It(World); It; ++It)
    {
        ACameraActor* C = *It;
        if (!IsValid(C)) continue;
        OutCameras.Add(C);
    }
}

bool ASimulationManager::PossessCamera(AActor* CameraActor)
{
    if (!IsValid(CameraActor)) return false;

    UWorld* World = GetWorld();
    if (!World) return false;

    APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
    if (!PC) return false;

    if (APawn* PawnCam = Cast<APawn>(CameraActor))
    {
        PC->Possess(PawnCam);
        PC->SetViewTarget(PawnCam);
        return true;
    }

    if (ACameraActor* CamActor = Cast<ACameraActor>(CameraActor))
    {
        PC->SetViewTarget(CamActor);
        return true;
    }

    // Fallback: if it has a camera component but isn't a pawn/camera actor, just SetViewTarget
    if (CameraActor->FindComponentByClass<UCameraComponent>())
    {
        PC->SetViewTarget(CameraActor);
        return true;
    }

    return false;
}

bool ASimulationManager::GetCurrentCamera(AActor*& OutCamera) const
{
    OutCamera = nullptr;

    if (APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0))
    {
        AActor* VT = PC->GetViewTarget();
        if (IsValid(VT))
        {
            OutCamera = VT;
            return true;
        }
    }
    return false;
}


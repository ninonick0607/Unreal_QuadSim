#include "SimulationCore/Public/Core/SimulationManager.h"
#include "SimulationCore/Public/Core/TimeController.h"
#include "SimulationCore/Public/Interfaces/ISimulatable.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "imgui.h"
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
    
	DrawImGuiWindow();
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
        
        if (StepsExecuted >= MaxStepsPerFrame)
        {
            UE_LOG(LogTemp, Warning, TEXT("Simulation falling behind! Executed max steps (%d) this frame"), 
                   MaxStepsPerFrame);
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
            GetWorld()->GetWorldSettings()->SetTimeDilation(TimeController->GetTimeScale());
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
        TimeController->SetTimeScale(NewTimeScale);
        
        // Update time dilation if in fast forward mode
        if (CurrentSimulationMode == ESimulationMode::FastForward)
        {
            GetWorld()->GetWorldSettings()->SetTimeDilation(NewTimeScale);
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

void ASimulationManager::DrawImGuiWindow()
{
    if (!bShowImGuiWindow)
    {
        return;
    }
    
    ImGui::Begin("Simulation Manager", &bShowImGuiWindow);
    
    // Simulation Info
    ImGui::Text("Simulation Time: %.2f s", CurrentSimulationTime);
    ImGui::Text("Episode: %d, Step: %d", CurrentEpisode, CurrentStep);
    ImGui::Text("Registered Robots: %d", RegisteredRobots.Num());
    
    // Show current time dilation
    float CurrentTimeDilation = GetWorld()->GetWorldSettings()->TimeDilation;
    ImGui::Text("Time Dilation: %.3f", CurrentTimeDilation);
    
    ImGui::Separator();
    
    // Mode Selection
    ImGui::Text("Simulation Mode:");
    const char* modes[] = { "Realtime", "Lockstep", "Fast Forward", "Paused" };
    int currentMode = (int)CurrentSimulationMode;
    if (ImGui::Combo("Mode", &currentMode, modes, IM_ARRAYSIZE(modes)))
    {
        SetSimulationMode((ESimulationMode)currentMode);
    }
    
    // Mode-specific controls
    if (CurrentSimulationMode == ESimulationMode::FastForward)
    {
        float timeScale = TimeController ? TimeController->GetTimeScale() : 1.0f;
        if (ImGui::SliderFloat("Time Scale", &timeScale, 0.1f, 10.0f))
        {
            SetTimeScale(timeScale);
        }
    }
    
    // Fixed Timestep
    if (TimeController)
    {
        float fixedTimestep = TimeController->GetFixedDeltaTime();
        if (ImGui::InputFloat("Fixed Timestep", &fixedTimestep, 0.001f, 0.01f, "%.4f"))
        {
            TimeController->SetFixedTimestep(FMath::Clamp(fixedTimestep, 0.001f, 0.1f));
        }
        ImGui::Text("Frequency: %.1f Hz", 1.0f / fixedTimestep);
    }
    
    ImGui::Separator();
    
    // Control Buttons
    if (ImGui::Button("Reset Simulation"))
    {
        ResetSimulation();
    }
    
    ImGui::SameLine();
    
    if (ImGui::Button("New Episode"))
    {
        StartNewEpisode();
    }
    
    // Manual Step (for Lockstep/Paused modes)
    if (CurrentSimulationMode == ESimulationMode::Lockstep || CurrentSimulationMode == ESimulationMode::Paused)
    {
        if (ImGui::Button("Step"))
        {
            RequestSimulationStep();
        }
        
        if (CurrentSimulationMode == ESimulationMode::Lockstep)
        {
            ImGui::SameLine();
            ImGui::Text(bWaitingForExternalCommand ? "Waiting for command..." : "Processing...");
        }
    }
    
    ImGui::Separator();
    
    // Robot List
    ImGui::Text("Registered Robots:");
    ImGui::BeginChild("RobotList", ImVec2(0, 150), true);
    for (int32 i = 0; i < RegisteredRobots.Num(); i++)
    {
        if (RegisteredRobots[i])
        {
            bool isSelected = (i == SelectedRobotIndex);
            if (ImGui::Selectable(TCHAR_TO_UTF8(*RegisteredRobots[i]->GetName()), isSelected))
            {
                SelectedRobotIndex = i;
            }
        }
    }
    ImGui::EndChild();
    
    ImGui::End();
}
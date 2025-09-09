#include "Core/DroneManager.h"
// Controller includes
#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "imgui.h"
#include "Engine/Engine.h"
#include "Controllers/PX4Component.h"

// Static accessor for the DroneManager in the world
ADroneManager* ADroneManager::Get(UWorld* World)
{
    if (!World) return nullptr;
    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(World, ADroneManager::StaticClass(), Found);
    return Found.Num() > 0 ? Cast<ADroneManager>(Found[0]) : nullptr;
}

ADroneManager::ADroneManager()
{
    PrimaryActorTick.bCanEverTick = true;
    SelectedDroneIndex = 0;
    // Default to C++ pawn class if no Blueprint subclass is set
}

void ADroneManager::BeginPlay()
{
    Super::BeginPlay();
    
    if (UWorld* World = GetWorld())
    {
        // Subscribe to actor-spawn events.
        OnActorSpawnedHandle = World->AddOnActorSpawnedHandler(
            FOnActorSpawned::FDelegate::CreateLambda([this](AActor* SpawnedActor)
            {
                this->OnActorSpawned(SpawnedActor);
            })
        );
    }

    // Populate the list with any already existing QuadPawns.
    TArray<AActor*> FoundDrones;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AQuadPawn::StaticClass(), FoundDrones);
    for (AActor* Actor : FoundDrones)
    {
        if (AQuadPawn* Pawn = Cast<AQuadPawn>(Actor))
        {
            AllDrones.Add(Pawn);
        }
    }
    // Initialize last spawn location to the most recently found drone (assumed ground level)
    if (AllDrones.Num() > 0)
    {
        if (AQuadPawn* LastPawn = AllDrones.Last().Get())
        {
            LastSpawnLocation = LastPawn->GetActorLocation();
        }
    }
}

void ADroneManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (UWorld* World = GetWorld())
    {
        World->RemoveOnActorSpawnedHandler(OnActorSpawnedHandle);
    }
    Super::EndPlay(EndPlayReason);
}

void ADroneManager::OnActorSpawned(AActor* SpawnedActor)
{
    if (AQuadPawn* Pawn = Cast<AQuadPawn>(SpawnedActor))
    {
        AllDrones.Add(Pawn);
    }
}


// Register a Quad Drone Controller to receive global flight mode broadcasts
void ADroneManager::RegisterDroneController(UQuadDroneController* Controller)
{
    if (Controller)
    {
        OnGlobalFlightModeChanged.AddUObject(Controller, &UQuadDroneController::SetFlightMode);
    }
}

// Enable or disable swarm mode
void ADroneManager::SetSwarmMode(bool bEnable)
{
    bSwarmMode = bEnable;
}
bool ADroneManager::IsSwarmMode() const
{
    return bSwarmMode;
}

// Get the index of the given drone within AllDrones
int32 ADroneManager::GetDroneIndex(AQuadPawn* Pawn) const
{
    for (int32 i = 0; i < AllDrones.Num(); ++i)
    {
        if (AllDrones[i].Get() == Pawn)
        {
            return i;
        }
    }
    return INDEX_NONE;
}

void ADroneManager::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Clean up any invalid drone entries
    for (int32 i = AllDrones.Num() - 1; i >= 0; i--)
    {
        if (!AllDrones[i].IsValid())
        {
            AllDrones.RemoveAt(i);
        }
    }
}

AQuadPawn* ADroneManager::SpawnDrone(const FVector& SpawnLocation, const FRotator& SpawnRotation)
{
    if (!QuadPawnClass)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPawnClass not set in DroneManager!"));
        return nullptr;
    }

    UWorld* World = GetWorld();
    if (World)
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.Owner = this;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        // Always spawn at PlayerStart if present; otherwise fall back to origin (0,0,0)
        FVector UseLoc = FVector::ZeroVector;
        FRotator UseRot = SpawnRotation;
        if (AActor* PS = UGameplayStatics::GetActorOfClass(World, APlayerStart::StaticClass()))
        {
            UseLoc = PS->GetActorLocation();
            UseRot = PS->GetActorRotation();
        }

        AQuadPawn* NewDrone = World->SpawnActor<AQuadPawn>(QuadPawnClass, UseLoc, UseRot, SpawnParams);
        if (NewDrone)
        {
            // Set selection to the newly spawned drone
            SelectedDroneIndex = GetDroneIndex(NewDrone);
        }
        return NewDrone;
    }
    return nullptr;
}



TArray<AQuadPawn*> ADroneManager::GetDroneList() const
{
    TArray<AQuadPawn*> DroneList;
    for (const TWeakObjectPtr<AQuadPawn>& DronePtr : AllDrones)
    {
        if (AQuadPawn* Drone = DronePtr.Get())
        {
            DroneList.Add(Drone);
        }
    }
    return DroneList;
}

void ADroneManager::SimulationUpdate_Implementation(float FixedDeltaTime)
{
	// Update all drones with fixed timestep
	for (TWeakObjectPtr<AQuadPawn> DronePtr : AllDrones)
	{
		if (AQuadPawn* Drone = DronePtr.Get())
		{
			// Make sure PX4Component updates are synchronized
			if (UPX4Component* PX4Comp = Drone->FindComponentByClass<UPX4Component>())
			{
				if (PX4Comp->bIsActive())
				{
					PX4Comp->SimulationUpdate(FixedDeltaTime);
				}
			}
            
			// Then update control
			Drone->bIsSimulationControlled = true;
			Drone->UpdateControl(FixedDeltaTime);
			Drone->bIsSimulationControlled = false;
		}
	}
}

void ADroneManager::ResetRobot_Implementation()
{
    // Find the Player Start actor
    AActor* PlayerStartActor = UGameplayStatics::GetActorOfClass(GetWorld(), APlayerStart::StaticClass());
    FTransform ResetTransform = FTransform::Identity; // Default to origin (0,0,0)

    if (PlayerStartActor)
    {
        ResetTransform = PlayerStartActor->GetActorTransform();
    }

    // Reset all drones to the PlayerStart location
    for (TWeakObjectPtr<AQuadPawn> DronePtr : AllDrones)
    {
        if (AQuadPawn* Drone = DronePtr.Get())
        {
            // Teleport the drone to the reset location
            Drone->SetActorTransform(ResetTransform, false, nullptr, ETeleportType::ResetPhysics);

            // Also call your controller's reset logic for other things (like motor values)
            if (UQuadDroneController* Controller = Drone->QuadController)
            {
                Controller->ResetDroneOrigin(); // A new function that doesn't change position
            }
        }
    }
}

FString ADroneManager::GetRobotState_Implementation()
{
    // Return drone count and swarm mode status
    return FString::Printf(TEXT("{\"drone_count\": %d, \"swarm_mode\": %s}"),
        AllDrones.Num(),
        bSwarmMode ? TEXT("true") : TEXT("false"));
}

void ADroneManager::ApplyCommand_Implementation(const FString& Command)
{
    // For future use - parse and apply commands
    UE_LOG(LogTemp, Display, TEXT("DroneManager received command: %s"), *Command);
}

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

    // Optionally spawn the Obstacle Manager at startup if enabled
    if (bSpawnObstacles && ObstacleManagerClass)
    {
        if (UWorld* World = GetWorld())
        {
            FActorSpawnParameters Params;
            Params.Owner = this;
            Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            AActor* ObMgr = World->SpawnActor<AActor>(ObstacleManagerClass, GetActorLocation(), GetActorRotation(), Params);
            if (ObMgr)
            {
                SpawnedObstacleManager = ObMgr;
            }
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

        // Spawn 1 meter from the currently possessed drone if any; otherwise at PlayerStart
        FVector UseLoc = FVector::ZeroVector;
        FRotator UseRot = SpawnRotation;

        if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
        {
            if (AQuadPawn* Cur = Cast<AQuadPawn>(PC->GetPawn()))
            {
                // Offset 1 meter to the right to avoid overlap
                const FVector Right = Cur->GetActorRightVector();
                UseLoc = Cur->GetActorLocation() + Right * 100.0f; // 100 cm = 1 m
                UseRot = Cur->GetActorRotation();
            }
        }

        if (UseLoc.IsZero())
        {
            if (AActor* PS = UGameplayStatics::GetActorOfClass(World, APlayerStart::StaticClass()))
            {
                UseLoc = PS->GetActorLocation();
                UseRot = PS->GetActorRotation();
            }
        }

        AQuadPawn* NewDrone = World->SpawnActor<AQuadPawn>(QuadPawnClass, UseLoc, UseRot, SpawnParams);
        if (NewDrone)
        {
            // Set selection to the newly spawned drone
            SelectedDroneIndex = GetDroneIndex(NewDrone);

            // Possess the newly spawned drone and switch to its camera
            if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
            {
                PC->Possess(NewDrone);
                PC->SetViewTarget(NewDrone);
                NewDrone->ForceFPVCameraActive();
            }
        }
        return NewDrone;
    }
    return nullptr;
}

void ADroneManager::SelectDroneByIndex(int32 Index, bool bAlsoPossess)
{
    if (Index < 0 || Index >= AllDrones.Num())
        return;

    if (AQuadPawn* Target = AllDrones[Index].Get())
    {
        SelectedDroneIndex = Index;
        if (bAlsoPossess)
        {
            if (UWorld* World = GetWorld())
            {
                if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
                {
                    if (PC->GetPawn() != Target)
                    {
                        PC->Possess(Target);
                    }
                    PC->SetViewTarget(Target);
                    Target->ForceFPVCameraActive();
                }
            }
        }
    }
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

void ADroneManager::SetQuadPawnClass(TSubclassOf<AQuadPawn> InClass, bool bRespawn)
{
    QuadPawnClass = InClass;

    if (!bRespawn)
    {
        return;
    }

    // Respawn is optional; not implemented here because manager doesn't own a single pawn.
    // If needed, caller should destroy/recreate drones via their own flow.
}

void ADroneManager::SetObstacleManagerClass(TSubclassOf<AActor> InClass, bool bRespawn)
{
    ObstacleManagerClass = InClass;

    if (!bRespawn)
    {
        return;
    }

    if (!GetWorld())
    {
        return;
    }

    if (SpawnedObstacleManager.IsValid())
    {
        if (AActor* Ob = SpawnedObstacleManager.Get())
        {
            Ob->Destroy();
        }
        SpawnedObstacleManager.Reset();
    }

    if (bSpawnObstacles && ObstacleManagerClass)
    {
        FActorSpawnParameters Params;
        Params.Owner = this;
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SpawnedObstacleManager = GetWorld()->SpawnActor<AActor>(ObstacleManagerClass, GetActorLocation(), GetActorRotation(), Params);
    }
}

void ADroneManager::SetSpawnObstacles(bool bEnabled, bool bApplyImmediately)
{
    bSpawnObstacles = bEnabled;

    if (!bApplyImmediately || !GetWorld())
    {
        return;
    }

    // Turning off: destroy if exists
    if (!bSpawnObstacles)
    {
        if (AActor* Ob = SpawnedObstacleManager.Get())
        {
            Ob->Destroy();
        }
        SpawnedObstacleManager.Reset();
        return;
    }

    // Turning on: spawn if not present
    if (!SpawnedObstacleManager.IsValid() && ObstacleManagerClass)
    {
        FActorSpawnParameters Params;
        Params.Owner = this;
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SpawnedObstacleManager = GetWorld()->SpawnActor<AActor>(ObstacleManagerClass, GetActorLocation(), GetActorRotation(), Params);
    }
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
				if (PX4Comp->IsPX4Active())
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
            // Teleport the drone to the reset location but preserve current scale
            const FVector CurrentScale = Drone->GetActorScale3D();
            Drone->SetActorLocationAndRotation(
                ResetTransform.GetLocation(),
                ResetTransform.GetRotation().Rotator(),
                false,
                nullptr,
                ETeleportType::ResetPhysics);
            Drone->SetActorScale3D(CurrentScale);

            Drone->ResetPosition();
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

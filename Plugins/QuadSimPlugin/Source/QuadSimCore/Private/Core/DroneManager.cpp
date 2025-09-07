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

    // Prepare the drone labels for the ImGui interface
    ImGui::Begin("Global Drone Manager");

    // Swarm vs. Independent mode toggle
    ImGui::Checkbox("Swarm Mode", &bSwarmMode);

    // SPAWN BUTTON - ALWAYS VISIBLE
    if (ImGui::Button("Spawn Drone"))
    {
        // Determine spawn location
        FVector SpawnLocation;
        if (LastSpawnLocation.IsZero())
        {
            // First drone spawns at default location
            SpawnLocation = GetActorLocation();
        }
        else
        {
            // Subsequent drones spawn offset from last
            SpawnLocation = LastSpawnLocation + FVector(300.f, 0.f, 0.f);
            SpawnLocation.Z = LastSpawnLocation.Z;
        }

        // Default rotation
        FRotator SpawnRotation = FRotator::ZeroRotator;

        // Try to match rotation of last drone if available
        if (AllDrones.Num() > 0)
        {
            if (AQuadPawn* LastPawn = AllDrones.Last().Get())
            {
                SpawnRotation = LastPawn->GetActorRotation();
            }
        }

        // Spawn the drone
        if (AQuadPawn* NewDrone = SpawnDrone(SpawnLocation, SpawnRotation))
        {
            UE_LOG(LogTemp, Display, TEXT("Spawned new drone at %s"), *SpawnLocation.ToString());
            LastSpawnLocation = SpawnLocation;
        }
    }

    ImGui::Separator();

    // Drone selection and control UI
    int32 NumDrones = AllDrones.Num();
    if (NumDrones > 0)
    {
        ImGui::Text("Select which drone to possess:");

        // Clamp the selected index to valid range
        if (SelectedDroneIndex < 0 || SelectedDroneIndex >= NumDrones)
        {
            SelectedDroneIndex = 0;
        }

        // Display current drone ID
        AQuadPawn* CurrentPawn = AllDrones[SelectedDroneIndex].Get();
        FString CurrentID = CurrentPawn ? CurrentPawn->DroneID : FString::Printf(TEXT("Drone%d"), SelectedDroneIndex + 1);

        // Dropdown to select active drone
        if (ImGui::BeginCombo("Active Drone", TCHAR_TO_UTF8(*CurrentID)))
        {
            for (int32 i = 0; i < NumDrones; ++i)
            {
                AQuadPawn* Drone = AllDrones[i].Get();
                FString DroneID = Drone ? Drone->DroneID : FString::Printf(TEXT("Drone%d"), i + 1);
                FString Label = FString::Printf(TEXT("%s##%d"), *DroneID, i);
                bool bSelected = (SelectedDroneIndex == i);

                if (ImGui::Selectable(TCHAR_TO_UTF8(*Label), bSelected))
                {
                    SelectedDroneIndex = i;
                }

                if (bSelected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        // Possess selected drone
        if (APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0))
        {
            AQuadPawn* NewPawn = AllDrones[SelectedDroneIndex].Get();
            if (NewPawn && PC->GetPawn() != NewPawn)
            {
                PC->Possess(NewPawn);
            }
        }
    }
    else
    {
        ImGui::Text("No drones spawned yet. Click 'Spawn Drone' above!");
    }

    ImGui::End();
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
        
        // Spawn the drone.
        AQuadPawn* NewDrone = World->SpawnActor<AQuadPawn>(QuadPawnClass, SpawnLocation, SpawnRotation, SpawnParams);
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
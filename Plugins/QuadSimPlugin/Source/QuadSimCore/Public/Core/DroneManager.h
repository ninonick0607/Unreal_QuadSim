#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SimulationCore/Public/Interfaces/ISimulatable.h"
#include "DroneManager.generated.h"

class AQuadPawn;
// Forward declaration for flight modes
enum class EFlightMode : uint8;

UCLASS(Blueprintable, BlueprintType)
class QUADSIMCORE_API ADroneManager : public AActor, public ISimulatable
{
	GENERATED_BODY()

public:
    ADroneManager();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
    
    UFUNCTION(BlueprintCallable, Category = "Drone Manager")
    AQuadPawn* SpawnDrone(const FVector& SpawnLocation, const FRotator& SpawnRotation);

    UFUNCTION(BlueprintCallable, Category = "Drone Manager")
    TArray<AQuadPawn*> GetDroneList() const;

    // Settings-driven API -------------------------------------------------
    // Allow settings/HUD to assign the drone pawn class
    UFUNCTION(BlueprintCallable, Category = "Drone Manager|Settings")
    void SetQuadPawnClass(TSubclassOf<AQuadPawn> InClass, bool bRespawn = false);

    // Set the Obstacle Manager blueprint/class to spawn
    UFUNCTION(BlueprintCallable, Category = "Drone Manager|Settings")
    void SetObstacleManagerClass(TSubclassOf<AActor> InClass, bool bRespawn = false);

    // Toggle obstacle spawning; optionally apply immediately by spawning/destroying
    UFUNCTION(BlueprintCallable, Category = "Drone Manager|Settings")
    void SetSpawnObstacles(bool bEnabled, bool bApplyImmediately = false);

    
    // Register a quad-drone controller for global flight mode broadcasts
    void RegisterDroneController(class UQuadDroneController* Controller);

    // Toggle and query swarm mode
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    void SetSwarmMode(bool bEnable);
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    bool IsSwarmMode() const;

    // Get index of the given drone in the manager list
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    int32 GetDroneIndex(AQuadPawn* Pawn) const;

    // Static accessor for the drone manager in the world
    static ADroneManager* Get(UWorld* World);

    // Delegate to broadcast global flight mode changes
    DECLARE_MULTICAST_DELEGATE_OneParam(FOnGlobalFlightModeChanged, EFlightMode /*NewMode*/);
    FOnGlobalFlightModeChanged OnGlobalFlightModeChanged;

    UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
    int32 SelectedDroneIndex;

    // Fixed spawn origin for new drones (editable in BP). Defaults to (0,0,0).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
    FVector SpawnOrigin = FVector::ZeroVector;

    UFUNCTION(BlueprintCallable, Category = "Drone Manager")
    void SetSpawnOrigin(const FVector& InOrigin) { SpawnOrigin = InOrigin; }

    UFUNCTION(BlueprintPure, Category = "Drone Manager")
    FVector GetSpawnOrigin() const { return SpawnOrigin; }

    // Select a drone by index and optionally possess it (switch camera)
    UFUNCTION(BlueprintCallable, Category = "Drone Manager")
    void SelectDroneByIndex(int32 Index, bool bAlsoPossess = true);


    virtual void SimulationUpdate_Implementation(float FixedDeltaTime) override;
    virtual void ResetRobot_Implementation() override;
    virtual FString GetRobotState_Implementation() override;
    virtual void ApplyCommand_Implementation(const FString& Command) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
    TSubclassOf<AQuadPawn> QuadPawnClass;

    // Class of the obstacle manager to spawn (conditionally)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager|Obstacles")
    TSubclassOf<AActor> ObstacleManagerClass;

    // Whether obstacles should spawn at BeginPlay (can be toggled by settings)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager|Obstacles")
    bool bSpawnObstacles = true;


	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

    
private:
    // Whether swarm mode is enabled.
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category="Swarm", meta=(AllowPrivateAccess="true"))
    bool bSwarmMode = false;

    void OnActorSpawned(AActor* SpawnedActor);

    FDelegateHandle OnActorSpawnedHandle;
    // Last spawn location used for positioning new drones
    FVector LastSpawnLocation = FVector::ZeroVector;

    // Runtime reference to spawned obstacle manager (if any)
    UPROPERTY(Transient)
    TWeakObjectPtr<AActor> SpawnedObstacleManager;
};

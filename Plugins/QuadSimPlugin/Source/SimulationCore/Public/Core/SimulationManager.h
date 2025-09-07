#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "SimulationManager.generated.h"

// Forward declarations
class UTimeController;
class ISimulatable;

UENUM(BlueprintType)
enum class ESimulationMode : uint8
{
    Realtime        UMETA(DisplayName = "Realtime"),
    Lockstep        UMETA(DisplayName = "Lockstep"),
    FastForward     UMETA(DisplayName = "Fast Forward"),
    Paused          UMETA(DisplayName = "Paused")
};

UCLASS(Blueprintable)
class SIMULATIONCORE_API ASimulationManager : public AActor
{
    GENERATED_BODY()

public:
    ASimulationManager();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
    virtual void Tick(float DeltaTime) override;

    // Robot Management
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void RegisterRobot(AActor* Robot);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void UnregisterRobot(AActor* Robot);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    TArray<AActor*> GetRegisteredRobots() const { return RegisteredRobots; }

    // Simulation Control
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void StepSimulation(float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetSimulationMode(ESimulationMode NewMode);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    ESimulationMode GetSimulationMode() const { return CurrentSimulationMode; }

    // Time Control
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetTimeScale(float NewTimeScale);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    float GetSimulationTime() const { return CurrentSimulationTime; }

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();

    // Physics Control
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PausePhysics();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumePhysics();

    // External Control
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void RequestSimulationStep();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    bool IsWaitingForExternalCommand() const { return bWaitingForExternalCommand; }

    // Episode Management (for RL)
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    int32 GetCurrentEpisode() const { return CurrentEpisode; }

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    int32 GetCurrentStep() const { return CurrentStep; }

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void StartNewEpisode();
	UFUNCTION(BlueprintCallable, Category = "Simulation")
	
	bool IsControllingSimulation() const 
	{ 
		return CurrentSimulationMode != ESimulationMode::Realtime; 
	}
    
	// Get simulation time in microseconds (for PX4)
	UFUNCTION(BlueprintCallable, Category = "Simulation")
	double GetSimulationTimeMicroseconds() const 
	{ 
		return CurrentSimulationTime * 1000000.0; 
	}
    
	// Static helper to find SimulationManager in world
	static ASimulationManager* Get(UWorld* World)
	{
		if (!World) return nullptr;
		TArray<AActor*> Found;
		UGameplayStatics::GetAllActorsOfClass(World, ASimulationManager::StaticClass(), Found);
		return Found.Num() > 0 ? Cast<ASimulationManager>(Found[0]) : nullptr;
	}
protected:
    // Robot Registry
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
    TArray<AActor*> RegisteredRobots;

    // Time Controller
    UPROPERTY()
    UTimeController* TimeController;

    // Simulation Mode
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
    ESimulationMode CurrentSimulationMode;

    // Current simulation time
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
    float CurrentSimulationTime;

    // Episode tracking
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
    int32 CurrentEpisode;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
    int32 CurrentStep;

    // External control
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
    bool bWaitingForExternalCommand;

    // Frame skip protection
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
    int32 MaxStepsPerFrame;

    // Physics settings backup
    float OriginalMaxPhysicsStep;
    bool OriginalSubstepping;

private:
    void ExecuteSimulationStep(float FixedDeltaTime);
    void UpdateAllRobots(float DeltaTime);
    //void ManualPhysicsStep(float FixedDeltaTime);
    
    // ImGui
    void DrawImGuiWindow();
    bool bShowImGuiWindow;
    bool bStepRequested; 
    bool bIsStepping; 

    // Selected robot for UI
    int32 SelectedRobotIndex;
};
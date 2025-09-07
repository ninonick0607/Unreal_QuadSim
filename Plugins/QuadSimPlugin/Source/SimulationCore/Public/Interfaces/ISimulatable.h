#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "ISimulatable.generated.h"

UINTERFACE(MinimalAPI, Blueprintable)
class USimulatable : public UInterface
{
	GENERATED_BODY()
};

class SIMULATIONCORE_API ISimulatable
{
	GENERATED_BODY()

public:
	// Called by SimulationManager with fixed timestep
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Simulation")
	void SimulationUpdate(float FixedDeltaTime);
    
	// Reset the robot/actor to initial state
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Simulation")
	void ResetRobot();
    
	// Get current state for external systems
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Simulation")
	FString GetRobotState();
    
	// Apply external command
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Simulation")
	void ApplyCommand(const FString& Command);
};
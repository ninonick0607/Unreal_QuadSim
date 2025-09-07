#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "TimeController.generated.h"

UCLASS()
class SIMULATIONCORE_API UTimeController : public UObject
{
    GENERATED_BODY()

public:
    UTimeController();

    // Time accumulation
    void AccumulateTime(float DeltaTime);
    
    // Check if we should execute a fixed timestep
    bool ShouldStep() const;
    
    // Consume one fixed timestep from accumulator
    void ConsumeTime();
    
    // Reset the time controller
    void Reset();
    
    // Getters
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    float GetFixedDeltaTime() const { return FixedTimestep; }
    
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    float GetTimeScale() const { return TimeScale; }
    
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    float GetAccumulatedTime() const { return TimeAccumulator; }
    
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    bool IsPaused() const { return bPaused; }
    
    // Setters
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    void SetFixedTimestep(float NewTimestep);
    
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    void SetTimeScale(float NewTimeScale);
    
    UFUNCTION(BlueprintCallable, Category = "Time Control")
    void SetPaused(bool bNewPaused);

protected:
    // Fixed timestep value (default 0.01 = 100Hz)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Control")
    float FixedTimestep;
    
    // Time accumulator
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Time Control")
    float TimeAccumulator;
    
    // Time scale multiplier
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Control")
    float TimeScale;
    
    // Pause state
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Time Control")
    bool bPaused;
    
    // Maximum accumulated time (prevents spiral of death)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Control")
    float MaxAccumulatedTime;
};
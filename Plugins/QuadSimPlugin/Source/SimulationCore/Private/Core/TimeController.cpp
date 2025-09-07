#include "SimulationCore/Public/Core/TimeController.h"

UTimeController::UTimeController()
{
	FixedTimestep = 0.01f;      // 100Hz default
	TimeAccumulator = 0.0f;
	TimeScale = 1.0f;
	bPaused = false;
	MaxAccumulatedTime = 0.1f;  // Max 100ms of accumulated time
}

void UTimeController::AccumulateTime(float DeltaTime)
{
	if (bPaused)
	{
		return;
	}
    
	// Apply time scale
	float ScaledDeltaTime = DeltaTime * TimeScale;
    
	// Add to accumulator with maximum limit
	TimeAccumulator += ScaledDeltaTime;
	TimeAccumulator = FMath::Min(TimeAccumulator, MaxAccumulatedTime);
}

bool UTimeController::ShouldStep() const
{
	return TimeAccumulator >= FixedTimestep;
}

void UTimeController::ConsumeTime()
{
	TimeAccumulator -= FixedTimestep;
    
	// Ensure we don't go negative
	TimeAccumulator = FMath::Max(0.0f, TimeAccumulator);
}

void UTimeController::Reset()
{
	TimeAccumulator = 0.0f;
	UE_LOG(LogTemp, Display, TEXT("TimeController reset"));
}

void UTimeController::SetFixedTimestep(float NewTimestep)
{
	if (NewTimestep > 0.0f)
	{
		FixedTimestep = NewTimestep;
		UE_LOG(LogTemp, Display, TEXT("Fixed timestep set to: %.4f seconds (%.1f Hz)"), 
			   FixedTimestep, 1.0f / FixedTimestep);
	}
}

void UTimeController::SetTimeScale(float NewTimeScale)
{
	TimeScale = FMath::Clamp(NewTimeScale, 0.0f, 100.0f);
	UE_LOG(LogTemp, Display, TEXT("Time scale set to: %.2fx"), TimeScale);
}

void UTimeController::SetPaused(bool bNewPaused)
{
	bPaused = bNewPaused;
	if (bPaused)
	{
		// Clear accumulator when pausing
		TimeAccumulator = 0.0f;
	}
}
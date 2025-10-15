#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "InputCoreTypes.h"
#include "ImGuiToggleInputSubsystem.generated.h"

class FImGuiTogglePreprocessor;

UCLASS()
class QUADSIMCORE_API UImGuiToggleInputSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	// Called by preprocessor
	void ToggleImGuiInput();

	// If you want to call explicitly from somewhere else:
	void SetImGuiInput(bool bEnable);

	// Re-scan keys bound to the action mapping if you change it at runtime
	void RefreshToggleKeys();

private:
	TSet<FKey> ToggleKeys;
	bool bImGuiInputActive = false;

	double LastToggleTime = -1000.0;
	double DebounceSeconds = 0.08;

	TSharedPtr<IInputProcessor> Preprocessor;
	void ExecImGuiCmd(const TCHAR* Cmd);
	bool HasAnyImGuiCVar() const;
	bool ReadImGuiCVar() const;

	friend class FImGuiTogglePreprocessor;
};

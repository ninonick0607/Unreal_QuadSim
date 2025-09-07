#pragma once
#include "UObject/Interface.h"
#include "Styles/SimControlLayout.h"
#include "SimUIControlSink.generated.h"

UINTERFACE(BlueprintType)
class USimUIControlSink : public UInterface { GENERATED_BODY() };

class ISimUIControlSink {
	GENERATED_BODY()
public:
	virtual void OnModeChanged(EControlMode Mode, bool bGamepad) {}
	virtual void OnAxisChanged(EAxisChannel Channel, float Value) {}
};

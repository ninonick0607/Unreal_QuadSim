#include "Modules/ModuleManager.h"
#include "Logging/LogMacros.h"

class FSimHUDModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        UE_LOG(LogTemp, Warning, TEXT("SimHUD Module Started"));
    }

    virtual void ShutdownModule() override
    {
        UE_LOG(LogTemp, Warning, TEXT("SimHUD Module Shutdown"));
    }
};

IMPLEMENT_MODULE(FSimHUDModule, SimHUD)


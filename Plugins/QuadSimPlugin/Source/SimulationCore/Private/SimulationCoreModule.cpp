#include "Modules/ModuleManager.h"

class FSimulationCoreModule : public IModuleInterface
{
public:
    virtual void StartupModule() override 
    {
        UE_LOG(LogTemp, Warning, TEXT("SimulationCore Module Started"));
    }
    
    virtual void ShutdownModule() override 
    {
        UE_LOG(LogTemp, Warning, TEXT("SimulationCore Module Shutdown"));
    }
};

IMPLEMENT_MODULE(FSimulationCoreModule, SimulationCore)
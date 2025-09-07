#include "SimHUD.h"
#include "Modules/ModuleManager.h"

void FSimHUDModule::StartupModule()
{
    UE_LOG(LogTemp, Warning, TEXT("SimHUD Module Started"));
}

void FSimHUDModule::ShutdownModule()
{
    UE_LOG(LogTemp, Warning, TEXT("SimHUD Module Shutdown"));
}

IMPLEMENT_MODULE(FSimHUDModule, SimHUD)

#include "RobotCore.h"
#include "Modules/ModuleManager.h"

void FRobotCoreModule::StartupModule()
{
    UE_LOG(LogTemp, Warning, TEXT("RobotCore Module Started"));
}

void FRobotCoreModule::ShutdownModule()
{
    UE_LOG(LogTemp, Warning, TEXT("RobotCore Module Shutdown"));
}

IMPLEMENT_MODULE(FRobotCoreModule, RobotCore)

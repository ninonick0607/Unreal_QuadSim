#include "QuadSimGameModePlugin.h"
#include "QuadSimPlayerController.h"
#include "Pawns/QuadPawn.h"

#include "ImGuiDelegates.h"
#include "imgui.h"

#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Paths.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "UObject/SoftObjectPath.h"
#include "UObject/SoftObjectPtr.h"

#include "SimulationCore/Public/Core/SimulationManager.h"
#include "Core/DroneManager.h"

AQuadSimGameMode::AQuadSimGameMode()
{
    PlayerControllerClass = AQuadSimPlayerController::StaticClass();
    // Do not auto-spawn a default pawn at PlayerStart; we will possess a placed camera pawn.
    DefaultPawnClass = nullptr;
    bStartPlayersAsSpectators = true;
}

static TSubclassOf<ADroneManager> LoadDroneManagerClass()
{
    const FSoftClassPath SoftPath(TEXT("/QuadSimPlugin/Blueprints/Core/BP_DroneManager_.BP_DroneManager__C"));
    return SoftPath.TryLoadClass<ADroneManager>();
}

void AQuadSimGameMode::BeginPlay()
{
    Super::BeginPlay();

    // ---- Frame rate & vsync
    if (GEngine)
    {
        GEngine->bUseFixedFrameRate = false;
        GEngine->bSmoothFrameRate   = false;

        if (IConsoleVariable* CVarMaxFPS = IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))
            CVarMaxFPS->Set(60);
        if (IConsoleVariable* CVarVSync = IConsoleManager::Get().FindConsoleVariable(TEXT("r.VSync")))
            CVarVSync->Set(0);

        UE_LOG(LogTemp, Log, TEXT("Framerate configured: t.MaxFPS=60, r.VSync=0"));
    }

    // ---- Physics substepping
    if (UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get())
    {
        PhysicsSettings->bSubstepping        = true;
        PhysicsSettings->MaxSubstepDeltaTime = 0.016667f;
        PhysicsSettings->MaxSubsteps         = 6;
        UE_LOG(LogTemp, Log, TEXT("Physics substepping enabled (dt=%.6f, max=%d)"),
            PhysicsSettings->MaxSubstepDeltaTime, PhysicsSettings->MaxSubsteps);
    }

    UWorld* World = GetWorld();
    if (!World) return;

    // ---- Spawn location
    FVector  SpawnLocation = FVector::ZeroVector;
    FRotator SpawnRotation = FRotator::ZeroRotator;

    if (AActor* PlayerStart = UGameplayStatics::GetActorOfClass(World, APlayerStart::StaticClass()))
    {
        SpawnLocation = PlayerStart->GetActorLocation();
        SpawnRotation = PlayerStart->GetActorRotation();
        UE_LOG(LogTemp, Log, TEXT("PlayerStart at %s"), *SpawnLocation.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No PlayerStart found. Using world origin."));
    }

    // Helper to avoid duplicates
    auto FindExisting = [&](TSubclassOf<AActor> Class) -> AActor*
    {
        for (TActorIterator<AActor> It(World, Class); It; ++It) { return *It; }
        return nullptr;
    };

    // 1) SimulationManager
    if (!FindExisting(ASimulationManager::StaticClass()))
    {
        World->SpawnActor<ASimulationManager>(ASimulationManager::StaticClass(), SpawnLocation, SpawnRotation);
        UE_LOG(LogTemp, Log, TEXT("Spawned SimulationManager at %s"), *SpawnLocation.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("SimulationManager already present, reusing."));
    }

    // 2) DroneManager (store to member)
    DroneManagerRef = Cast<ADroneManager>(FindExisting(ADroneManager::StaticClass()));
    if (!DroneManagerRef)
    {
        if (TSubclassOf<ADroneManager> DMClass = LoadDroneManagerClass())
        {
            DroneManagerRef = World->SpawnActor<ADroneManager>(DMClass, SpawnLocation, SpawnRotation);
            UE_LOG(LogTemp, Log, TEXT("Spawned DroneManager from BP at %s"), *SpawnLocation.ToString());
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to load BP_DroneManager_. Check mount point/path."));
        }
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("DroneManager already present, reusing."));
    }

    // Camera handoff: possess a placed camera pawn if present
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
    {
        APawn* CameraPawn = nullptr;
        // Prefer an actor tagged "StartupCamera"
        for (TActorIterator<APawn> It(World); It; ++It)
        {
            if (It->ActorHasTag(FName("StartupCamera"))) { CameraPawn = *It; break; }
        }
        // Fallback: first pawn found
        if (!CameraPawn)
        {
            for (TActorIterator<APawn> It(World); It; ++It) { CameraPawn = *It; break; }
        }
        if (CameraPawn)
        {
            PC->Possess(CameraPawn);
            PC->SetViewTarget(CameraPawn);
        }
    }
    
}

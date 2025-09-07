// QuadSimGameModePlugin.cpp
#include "QuadSimGameModePlugin.h"
#include "QuadSimPlayerController.h"
#include "Pawns/QuadPawn.h"

#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Paths.h"
#include "HAL/IConsoleManager.h"              // <-- needed for console vars
#include "PhysicsEngine/PhysicsSettings.h"
#include "UObject/SoftObjectPath.h"
#include "UObject/SoftObjectPtr.h"

#include "SimulationCore/Public/Core/SimulationManager.h"
#include "Core/DroneManager.h"

AQuadSimGameMode::AQuadSimGameMode()
{
    PlayerControllerClass = AQuadSimPlayerController::StaticClass();
    // Optional: DefaultPawnClass = AQuadPawn::StaticClass();
}

static TSubclassOf<ADroneManager> LoadDroneManagerClass()
{
    // Prefer a TSoftClassPtr so the path is validated and can be changed in one place.
    // Adjust if your mount point differs. Your plugin must have "CanContainContent": true
    // and a proper MountPoint in the .uplugin.
    const FSoftClassPath SoftPath(TEXT("/QuadSimPlugin/Blueprints/Core/BP_DroneManager_.BP_DroneManager__C"));
    UClass* Loaded = SoftPath.TryLoadClass<ADroneManager>();
    return Loaded;
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
        {
            CVarMaxFPS->Set(60);   // 60 Hz for stable lockstep (change if you need)
        }
        if (IConsoleVariable* CVarVSync = IConsoleManager::Get().FindConsoleVariable(TEXT("r.VSync")))
        {
            CVarVSync->Set(0);
        }
        UE_LOG(LogTemp, Log, TEXT("Framerate configured: t.MaxFPS=60, r.VSync=0"));
    }

    // ---- Physics substepping (runtime tweakable)
    if (UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get())
    {
        PhysicsSettings->bSubstepping         = true;
        PhysicsSettings->MaxSubstepDeltaTime  = 0.016667f; // 60 Hz
        PhysicsSettings->MaxSubsteps          = 6;
        UE_LOG(LogTemp, Log, TEXT("Physics substepping enabled (dt=%.6f, max=%d)"),
            PhysicsSettings->MaxSubstepDeltaTime, PhysicsSettings->MaxSubsteps);
    }

    UWorld* World = GetWorld();
    if (!World) return;

    // ---- Spawn location (PlayerStart if available)
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

    // ---- Do not spawn duplicates if they already exist (useful during PIE)
    auto FindExisting = [&](TSubclassOf<AActor> Class) -> AActor*
    {
        for (TActorIterator<AActor> It(World, Class); It; ++It)
        {
            return *It;
        }
        return nullptr;
    };

    // 1) SimulationManager (native class)
    ASimulationManager* SimManager = Cast<ASimulationManager>(FindExisting(ASimulationManager::StaticClass()));
    if (!SimManager)
    {
        SimManager = World->SpawnActor<ASimulationManager>(ASimulationManager::StaticClass(), SpawnLocation, SpawnRotation);
        UE_LOG(LogTemp, Log, TEXT("Spawned SimulationManager at %s"), *SpawnLocation.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("SimulationManager already present, reusing."));
    }

    // 2) DroneManager (BP class from plugin content)
    ADroneManager* DroneManager = Cast<ADroneManager>(FindExisting(ADroneManager::StaticClass()));
    if (!DroneManager)
    {
        TSubclassOf<ADroneManager> DMClass = LoadDroneManagerClass();
        if (DMClass)
        {
            DroneManager = World->SpawnActor<ADroneManager>(DMClass, SpawnLocation, SpawnRotation);
            UE_LOG(LogTemp, Log, TEXT("Spawned DroneManager from BP at %s"), *SpawnLocation.ToString());
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to load BP_DroneManager_. Check the mount point and path."));
        }
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("DroneManager already present, reusing."));
    }

    // ---- Camera handoff (only works if the target has a camera)
    if (DroneManager)
    {
        if (APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0))
        {
            PC->SetViewTargetWithBlend(DroneManager, 0.35f);
        }
    }
}

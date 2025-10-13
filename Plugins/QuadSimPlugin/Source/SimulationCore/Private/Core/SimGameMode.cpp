#include "Core/SimGameMode.h"
#include "Camera/TopDownCameraPawn.h"
#include "UObject/SoftObjectPath.h"

#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "Core/SimulationManager.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerStart.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/PhysicsSettings.h"

ASimGameMode::ASimGameMode()
{
    // Set PlayerController via soft reference to avoid module dependency cycles
    const FSoftClassPath PCPath(TEXT("/Script/QuadSimCore.QuadSimPlayerController"));
    if (UClass* PCC = PCPath.TryLoadClass<APlayerController>())
    {
        PlayerControllerClass = PCC;
    }

    DefaultPawnClass = nullptr; // we possess a camera pawn explicitly
    bStartPlayersAsSpectators = true;
}

void ASimGameMode::ConfigureEngineAndPhysics() const
{
    if (GEngine)
    {
        GEngine->bUseFixedFrameRate = false;
        GEngine->bSmoothFrameRate   = false;

        if (IConsoleVariable* CVarMaxFPS = IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"))) CVarMaxFPS->Set(60);
        if (IConsoleVariable* CVarVSync = IConsoleManager::Get().FindConsoleVariable(TEXT("r.VSync")))   CVarVSync->Set(0);
    }

    if (UPhysicsSettings* PS = UPhysicsSettings::Get())
    {
        PS->bSubstepping        = true;
        PS->MaxSubstepDeltaTime = 0.016667f;
        PS->MaxSubsteps         = 6;
    }
}

APawn* ASimGameMode::FindPlacedStartupCamera(UWorld* World) const
{
    if (!World) return nullptr;
    for (TActorIterator<APawn> It(World); It; ++It)
    {
        if (It->ActorHasTag(TEXT("StartupCamera"))) return *It;
    }
    return nullptr;
}

APawn* ASimGameMode::SpawnDefaultCamera(UWorld* World, const FVector& LookAt, float Height, float PitchDeg)
{
    if (!World) return nullptr;

    // choose the class as UClass*
    UClass* CamClass = DefaultCameraPawnClass
        ? DefaultCameraPawnClass.Get()
        : ATopDownCameraPawn::StaticClass();

    const FVector  Loc = LookAt + FVector(0,0,Height);
    const FRotator Rot(PitchDeg, 0.f, 0.f);

    FActorSpawnParameters P;
    P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    APawn* Cam = World->SpawnActor<APawn>(CamClass, Loc, Rot, P);
    if (auto* TD = Cast<ATopDownCameraPawn>(Cam)) TD->SetCameraTransform(Loc, Rot);
    return Cam;
}

void ASimGameMode::PossessCamera(APawn* CameraPawn) const
{
    if (!CameraPawn) return;
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0))
    {
        PC->Possess(CameraPawn);
        PC->SetViewTarget(CameraPawn);
    }
}

AActor* ASimGameMode::FindExistingByClass(UWorld* World, UClass* Class) const
{
    if (!World || !Class) return nullptr;
    for (TActorIterator<AActor> It(World, Class); It; ++It) { return *It; }
    return nullptr;
}

AActor* ASimGameMode::FindExistingByTag(UWorld* World, FName Tag) const
{
    if (!World || Tag.IsNone()) return nullptr;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        if (It->ActorHasTag(Tag)) return *It;
    }
    return nullptr;
}

AActor* ASimGameMode::SpawnActorSoft(UWorld* World, const TSoftClassPtr<AActor>& SoftClass, const FTransform& Xform) const
{
    if (!World || !SoftClass.IsValid())
    {
        // Try to load on-demand if not already
        UClass* Loaded = SoftClass.LoadSynchronous();
        if (!Loaded) return nullptr;
    }

    FActorSpawnParameters P;
    P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    return World->SpawnActor<AActor>(SoftClass.Get(), Xform, P);
}


void ASimGameMode::BeginPlay()
{
    Super::BeginPlay();

    ConfigureEngineAndPhysics();

    UWorld* World = GetWorld();
    if (!World) return;

    // Look-at from PlayerStart if present
    FVector LookAt = FVector::ZeroVector;
    if (AActor* PS = UGameplayStatics::GetActorOfClass(World, APlayerStart::StaticClass()))
        LookAt = PS->GetActorLocation();

    APawn* Cam = bPreferPlacedStartupCamera ? FindPlacedStartupCamera(World) : nullptr;
    if (!Cam) Cam = SpawnDefaultCamera(World, LookAt, DefaultCameraHeight, DefaultCameraPitchDeg);
    PossessCamera(Cam);
    
    FTransform SpawnXf = FTransform::Identity;
    if (AActor* PS = UGameplayStatics::GetActorOfClass(World, APlayerStart::StaticClass()))
    {
        SpawnXf.SetLocation(PS->GetActorLocation());
        SpawnXf.SetRotation(PS->GetActorQuat());
    }

    // Your existing SimulationManager/DroneManager spawning can stay in GM or move to SimManager.
    // Minimal change: keep it here for now.
    auto FindExisting = [&](TSubclassOf<AActor> Class) -> AActor*
    {
        for (TActorIterator<AActor> It(World, Class); It; ++It) return *It;
        return nullptr;
    };

    if (!FindExisting(ASimulationManager::StaticClass()))
    {
        World->SpawnActor<ASimulationManager>(ASimulationManager::StaticClass(), FTransform::Identity);
    }

    if (DefaultDroneManagerClass.IsValid() || !DefaultDroneManagerClass.ToSoftObjectPath().IsNull())
    {
        UClass* MaybeLoaded = DefaultDroneManagerClass.Get();         // may be null if not yet loaded
        AActor* Existing     = nullptr;

        if (MaybeLoaded)
        {
            Existing = FindExistingByClass(World, MaybeLoaded);
        }
        else
        {
            // Optional: look by tag if your BP sets a tag "DroneManager"
            Existing = FindExistingByTag(World, FName(TEXT("DroneManager")));
        }

        if (Existing)
        {
            DroneManagerRef = Existing;
            UE_LOG(LogTemp, Log, TEXT("DroneManager already present, reusing: %s"), *Existing->GetName());
        }
        else
        {
            if (AActor* Spawned = SpawnActorSoft(World, DefaultDroneManagerClass, SpawnXf))
            {
                DroneManagerRef = Spawned;
                UE_LOG(LogTemp, Log, TEXT("Spawned DroneManager from soft class at %s"),
                       *SpawnXf.GetLocation().ToString());
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("Failed to spawn DroneManager. "
                    "Set 'DefaultDroneManagerClass' on BP_SimGameMode to your BP_DroneManager_ class."));
            }
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("DefaultDroneManagerClass is not set on BP_SimGameMode; skipping DroneManager spawn."));
    }

}

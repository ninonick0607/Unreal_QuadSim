// SimWorldSubsystem.cpp
#include "Runtime/SimWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GeoReferencingSystem.h"
#include "Kismet/GameplayStatics.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "Utils/GeoRefHelpers.h"

static void ApplyGeoDefaults(AGeoReferencingSystem* Geo,
    bool bRound, double Lon, double Lat, double H, const FString& Geog, const FString& Proj)
{
    if (!Geo) return;

    bool bSetGeog = SetPropIfExists<FString>(Geo, TEXT("GeographicCRS"), Geog);
    bool bSetProj = SetPropIfExists<FString>(Geo, TEXT("ProjectedCRS"),  Proj);

    bool bRoundApplied =
        SetPropIfExists<bool>(Geo, TEXT("bUseRoundEarth"), bRound) ||
        SetPropIfExists<bool>(Geo, TEXT("bOriginAtPlanetCenter"), bRound) ||
        SetPropIfExists<bool>(Geo, TEXT("bEnableECEF"), bRound) ||
        SetPropIfExists<bool>(Geo, TEXT("bUseECEF"), bRound);

    bool bLon = SetPropIfExists<double>(Geo, TEXT("OriginLongitude"), Lon);
    bool bLat = SetPropIfExists<double>(Geo, TEXT("OriginLatitude"),  Lat);
    bool bHgt = SetPropIfExists<double>(Geo, TEXT("OriginAltitude"),  H) ||
                SetPropIfExists<double>(Geo, TEXT("OriginHeight"),     H);

    UE_LOG(LogTemp, Log, TEXT("GeoRef set: round=%d, lon=%.6f, lat=%.6f, h=%.2f, geog=%s, proj=%s (props: geog=%d proj=%d round=%d lon=%d lat=%d h=%d)"),
        bRound, Lon, Lat, H, *Geog, *Proj, bSetGeog, bSetProj, bRoundApplied, bLon, bLat, bHgt);
}

void USimWorldSubsystem::EnsureGeoReferencing()
{
    if (UWorld* World = GetWorld())
    {
        if (AGeoReferencingSystem* Geo = FindOrSpawnGeoRef(World))
        {
            ApplyGeoDefaults(Geo, bRoundPlanet, OriginLonDeg, OriginLatDeg, OriginHeightM, GeographicCRS, ProjectedCRS);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create GeoReferencingSystem"));
        }
    }
}

void USimWorldSubsystem::EnsureSimulationManager()
{
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<ASimulationManager> It(World); It; ++It) return;
        World->SpawnActor<ASimulationManager>(ASimulationManager::StaticClass(), FTransform::Identity);
        UE_LOG(LogTemp, Log, TEXT("Spawned SimulationManager"));
    }
}

void USimWorldSubsystem::InitializeWorld()
{
    EnsureGeoReferencing();
    EnsureSimulationManager();
}

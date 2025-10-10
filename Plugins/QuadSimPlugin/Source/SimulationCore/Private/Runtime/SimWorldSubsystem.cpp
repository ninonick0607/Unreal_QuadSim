// SimWorldSubsystem.cpp
#include "Runtime/SimWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GeoReferencingSystem.h"
#include "Kismet/GameplayStatics.h"
#include "SimulationCore/Public/Core/SimulationManager.h"
#include "UObject/UnrealType.h"
#include <type_traits>

template<typename T>
inline static bool SetPropIfExists(UObject* Obj, const TCHAR* PropName, const T& Value)
{
    if (!Obj) return false;
    FProperty* P = Obj->GetClass()->FindPropertyByName(FName(PropName));
    if (!P) return false;

    if constexpr (std::is_same_v<T, bool>)
    {
        if (FBoolProperty* BP = CastField<FBoolProperty>(P))
        { BP->SetPropertyValue_InContainer(Obj, Value); return true; }
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        if (FDoubleProperty* DP = CastField<FDoubleProperty>(P))
        { DP->SetPropertyValue_InContainer(Obj, Value); return true; }
        if (FFloatProperty* FP = CastField<FFloatProperty>(P))
        { FP->SetPropertyValue_InContainer(Obj, static_cast<float>(Value)); return true; }
    }
    else if constexpr (std::is_same_v<T, FString>)
    {
        if (FStrProperty* SP = CastField<FStrProperty>(P))
        { SP->SetPropertyValue_InContainer(Obj, Value); return true; }
    }
    return false;
}

inline static AGeoReferencingSystem* FindOrSpawnGeoRef(UWorld* World)
{
    if (!World) return nullptr;
    for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It) return *It;

    FActorSpawnParameters P;
    P.Name = TEXT("GeoReferencingSystem");
    P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    return World->SpawnActor<AGeoReferencingSystem>(
        AGeoReferencingSystem::StaticClass(), FTransform::Identity, P);
}

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

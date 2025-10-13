// GeoRefHelpers.h - Shared utility functions for GeoReferencing setup
#pragma once

#include "CoreMinimal.h"
#include "UObject/UnrealType.h"
#include "GeoReferencingSystem.h"
#include "EngineUtils.h"
#include <type_traits>

// Tiny reflection setter to be version-safe across Unreal Engine versions
template<typename T>
inline static bool SetPropIfExists(UObject* Obj, const TCHAR* PropName, const T& Value)
{
    if (!Obj) return false;
    FProperty* P = Obj->GetClass()->FindPropertyByName(FName(PropName));
    if (!P)
    {
        UE_LOG(LogTemp, Warning, TEXT("SetPropIfExists: Property '%s' not found on %s"), PropName, *Obj->GetClass()->GetName());
        return false;
    }

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

// Helper function to list all available properties on GeoReferencingSystem
inline static void LogGeoRefProperties(AGeoReferencingSystem* Geo)
{
    if (!Geo) return;

    UE_LOG(LogTemp, Log, TEXT("=== GeoReferencingSystem Available Properties ==="));
    for (TFieldIterator<FProperty> PropIt(Geo->GetClass()); PropIt; ++PropIt)
    {
        FProperty* Prop = *PropIt;
        FString PropType = TEXT("Unknown");

        if (CastField<FBoolProperty>(Prop)) PropType = TEXT("Bool");
        else if (CastField<FDoubleProperty>(Prop)) PropType = TEXT("Double");
        else if (CastField<FFloatProperty>(Prop)) PropType = TEXT("Float");
        else if (CastField<FStrProperty>(Prop)) PropType = TEXT("String");

        UE_LOG(LogTemp, Log, TEXT("  - %s (%s)"), *Prop->GetName(), *PropType);
    }
    UE_LOG(LogTemp, Log, TEXT("=== End Properties ==="));
}

// Find existing GeoReferencingSystem or spawn a new one with immediate property setup
inline static AGeoReferencingSystem* FindOrSpawnGeoRef(UWorld* World, double Lat = 0.0, double Lon = 0.0, double Alt = 0.0)
{
    if (!World) return nullptr;

    // Check if one already exists
    AGeoReferencingSystem* Existing = nullptr;
    for (TActorIterator<AGeoReferencingSystem> It(World); It; ++It)
    {
        Existing = *It;
        break;
    }

    if (Existing)
    {
        UE_LOG(LogTemp, Warning, TEXT("GeoReferencingSystem already exists - cannot change coordinates at runtime. Destroying and respawning..."));
        Existing->Destroy();
        Existing = nullptr;
    }

    // Spawn a new one with deferred construction
    FActorSpawnParameters P;
    P.Name = TEXT("GeoReferencingSystem");
    P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    P.bDeferConstruction = true; // CRITICAL: Defer construction to set properties first!

    AGeoReferencingSystem* GeoRef = World->SpawnActor<AGeoReferencingSystem>(
        AGeoReferencingSystem::StaticClass(), FTransform::Identity, P);

    if (GeoRef && Lat != 0.0 && Lon != 0.0)
    {
        // Set properties BEFORE FinishSpawning (while still deferred)
        SetPropIfExists<double>(GeoRef, TEXT("OriginLatitude"), Lat);
        SetPropIfExists<double>(GeoRef, TEXT("OriginLongitude"), Lon);
        SetPropIfExists<double>(GeoRef, TEXT("OriginAltitude"), Alt);
        SetPropIfExists<double>(GeoRef, TEXT("OriginHeight"), Alt);

        UE_LOG(LogTemp, Log, TEXT("Set GeoRef origin BEFORE initialization: Lat=%.6f, Lon=%.6f, Alt=%.2f"), Lat, Lon, Alt);
    }

    // Now finish spawning - this will initialize with the correct values
    if (GeoRef)
    {
        GeoRef->FinishSpawning(FTransform::Identity);
        UE_LOG(LogTemp, Log, TEXT("Spawned and initialized new GeoReferencingSystem"));
    }

    return GeoRef;
}

// Force GeoReferencingSystem to rebuild/apply its settings
inline static void RebuildGeoRef(AGeoReferencingSystem* Geo)
{
    if (!Geo) return;

    // Try to call any rebuild/apply methods that might exist
    // Different UE versions have different method names

    // Method 1: Check if there's an ApplySettings function
    UFunction* ApplyFunc = Geo->GetClass()->FindFunctionByName(FName(TEXT("ApplySettings")));
    if (ApplyFunc)
    {
        Geo->ProcessEvent(ApplyFunc, nullptr);
        UE_LOG(LogTemp, Log, TEXT("Called GeoRef->ApplySettings()"));
    }

    // Method 2: Try Rebuild
    UFunction* RebuildFunc = Geo->GetClass()->FindFunctionByName(FName(TEXT("Rebuild")));
    if (RebuildFunc)
    {
        Geo->ProcessEvent(RebuildFunc, nullptr);
        UE_LOG(LogTemp, Log, TEXT("Called GeoRef->Rebuild()"));
    }

    // Method 3: Force a property change notification
    Geo->PostEditChange();
    UE_LOG(LogTemp, Log, TEXT("Called GeoRef->PostEditChange()"));
}

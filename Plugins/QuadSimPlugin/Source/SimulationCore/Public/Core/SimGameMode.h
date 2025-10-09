#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

UCLASS(BlueprintType, Blueprintable) // ← add these
class SIMULATIONCORE_API ASimGameMode : public AGameModeBase
{
	GENERATED_BODY()
public:
	ASimGameMode();
	UPROPERTY(EditAnywhere, Category="Sim|Vehicles")
	TSoftClassPtr<AActor> DefaultDroneManagerClass;

	UPROPERTY()
	TWeakObjectPtr<AActor> DroneManagerRef;
	// Camera policy (tweakable in BP_SimGameMode)
	UPROPERTY(EditAnywhere, Category="Sim|Camera") float DefaultCameraHeight = 1500.f;
	UPROPERTY(EditAnywhere, Category="Sim|Camera") float DefaultCameraPitchDeg = 60.f;
	UPROPERTY(EditAnywhere, Category="Sim|Camera") bool  bPreferPlacedStartupCamera = true;
	UPROPERTY(EditAnywhere, Category="Sim|Camera") TSubclassOf<APawn> DefaultCameraPawnClass;

	// GeoRef defaults (your request)
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") bool   bRoundPlanet = true;
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") double OriginLonDeg = -86.5732;
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") double OriginLatDeg =  30.4752;
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") double OriginHeightM = 0.0;
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") FString GeographicCRS = TEXT("EPSG:4326");
	UPROPERTY(EditAnywhere, Category="Sim|GeoRef") FString ProjectedCRS  = TEXT("EPSG:32631");

	virtual void BeginPlay() override;

protected:
	// helpers
	class APawn* FindPlacedStartupCamera(UWorld* World) const;
	class APawn* SpawnDefaultCamera(UWorld* World, const FVector& LookAt, float Height, float PitchDeg);
	void PossessCamera(APawn* CameraPawn) const;
	void ConfigureEngineAndPhysics() const;

private:
	void EnsureGeoReferencingAndApplyDefaults(UWorld* World);
	AActor* FindExistingByClass(UWorld* World, UClass* Class) const;
	AActor* FindExistingByTag(UWorld* World, FName Tag) const;
	AActor* SpawnActorSoft(UWorld* World, const TSoftClassPtr<AActor>& SoftClass, const FTransform& Xform) const;


};

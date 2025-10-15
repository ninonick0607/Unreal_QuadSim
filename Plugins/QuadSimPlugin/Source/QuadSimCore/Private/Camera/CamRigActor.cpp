// Camera/CamRigActor.cpp
#include "Camera/CamRigActor.h"
#include "Camera/CameraRigComponent.h"
#include "Components/SceneComponent.h"

ACamRigActor::ACamRigActor()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Rig = CreateDefaultSubobject<UCameraRigComponent>(TEXT("CameraRig"));
	Rig->SetupAttachment(Root);
	PrimaryActorTick.bCanEverTick = true;
}

void ACamRigActor::BeginPlay()
{
	Super::BeginPlay();
	Rig->InitializeRig(Root, NAME_None); // no FPV socket; this is a free rig
}

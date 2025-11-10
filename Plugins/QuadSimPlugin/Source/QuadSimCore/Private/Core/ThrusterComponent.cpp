// ThrusterComponent.cpp

#include "Core/ThrusterComponent.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"

UThrusterComponent::UThrusterComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UThrusterComponent::BeginPlay()
{
    Super::BeginPlay();
}

void UThrusterComponent::ApplyForce(double Force)
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: No owner, cannot apply force."));
        return;
    }

    UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!RootPrim || !RootPrim->IsSimulatingPhysics())
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: Root is not simulating physics!"));
        return;
    }
    double UE_Force = Force * 100;
    const FVector Direction = GetComponentTransform().GetUnitAxis(EAxis::X);
    const FVector ForceVector = Direction * UE_Force;

    const FVector ForceLocation = GetComponentLocation();
    RootPrim->AddForceAtLocation(ForceVector, ForceLocation);

}
void UThrusterComponent::ApplyTorque(const FVector& Torque, bool bIsDegrees /*= true*/)
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: No owner, cannot apply torque."));
        return;
    }

    UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!RootPrim || !RootPrim->IsSimulatingPhysics())
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: Root is not simulating physics!"));
        return;
    }
    RootPrim->AddTorqueInRadians(Torque, NAME_None, true);
    
}

void UThrusterComponent::ApplyTorque(float TorqueValue, bool bIsDegrees)
{
    FVector TorqueVector(0.0f, 0.0f, TorqueValue);
    ApplyTorque(TorqueVector, bIsDegrees);
}

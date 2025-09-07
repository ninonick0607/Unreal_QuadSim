#include "Core/DroneGlobalState.h"
#include "Controllers/QuadDroneController.h" // Include here instead of in header

DroneGlobalState::DroneGlobalState()
    : DesiredVelocity(FVector::ZeroVector)
{
}

DroneGlobalState::~DroneGlobalState()
{
}

void DroneGlobalState::SetDesiredVelocity(const FVector& NewVelocity)
{
    DesiredVelocity = NewVelocity;

    for (UQuadDroneController* Controller : BoundControllers)
    {
        if (Controller)
        {
            Controller->SetDesiredVelocity(NewVelocity);
        }
    }
}

void DroneGlobalState::BindController(UQuadDroneController* Controller)
{
    // Register controller if not already bound
    if (Controller && !BoundControllers.Contains(Controller))
    {
        BoundControllers.Add(Controller);
    }
}

void DroneGlobalState::UnbindController(UQuadDroneController* Controller)
{
    if (Controller)
    {
        BoundControllers.RemoveSingle(Controller);
    }
}
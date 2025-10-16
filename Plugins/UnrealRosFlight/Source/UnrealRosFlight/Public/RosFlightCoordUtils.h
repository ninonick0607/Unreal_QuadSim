// RosflightCoordUtils.h
#pragma once
#include "Math/Vector.h"
#include "Math/Quat.h"

// Unreal  →  ROS (NED-style X-Fwd / Y-Left / Z-Up)  --------------------------------
FORCEINLINE FVector UnrealToRos(const FVector& In)
{
	// Unreal: +X Fwd, +Y Right, +Z Up  (LHS)
	// ROS   : +X Fwd, +Y Left , +Z Up  (RHS)
	return FVector( In.X, -In.Y,  In.Z );
}

FORCEINLINE FQuat UnrealToRos(const FQuat& Q)
{
	// flip y & z to change handedness
	return FQuat(  Q.X, -Q.Y, -Q.Z,  Q.W );
}

// Add these functions to RosflightCoordUtils.h

// ROS (NED-style) → Unreal ---------------------------------------------------------
FORCEINLINE FVector RosToUnreal(const FVector& In)
{
	// ROS   : +X Fwd, +Y Left , +Z Up  (RHS)
	// Unreal: +X Fwd, +Y Right, +Z Up  (LHS)
	return FVector( In.X, -In.Y,  In.Z );
}

FORCEINLINE FQuat RosToUnreal(const FQuat& Q)
{
	// flip y & z to change handedness
	return FQuat(  Q.X, -Q.Y, -Q.Z,  Q.W );
}
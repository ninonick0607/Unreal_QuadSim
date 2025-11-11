#pragma once
#include "CoreMinimal.h"

namespace Frames
{
    // ENU(x=East, y=North, z=Up)  <->  NED(x=North, y=East, z=Down)
    FORCEINLINE FVector ENU_to_NED(const FVector& v_enu)
    {
        return FVector(v_enu.Y, v_enu.X, -v_enu.Z);
    }
    FORCEINLINE FVector NED_to_ENU(const FVector& v_ned)
    {
        return FVector(v_ned.Y, v_ned.X, -v_ned.Z);
    }

    // Body FRU(x=Fwd, y=Right, z=Up)  <->  FRD(x=Fwd, y=Right, z=Down)
    FORCEINLINE FVector FRU_to_FRD(const FVector& v_fru)
    {
        return FVector(v_fru.X, v_fru.Y, -v_fru.Z);
    }
    FORCEINLINE FVector FRD_to_FRU(const FVector& v_frd)
    {
        return FVector(v_frd.X, v_frd.Y, -v_frd.Z);
    }

    // World vector UE ENU to core NED
    FORCEINLINE FVector UEWorldVec_to_NED(const FVector& v_world_enu)
    {
        // UE units are cm by default, convert to meters and swap axes
        return ENU_to_NED(v_world_enu / 100.f);
    }

    // World angular velocity is already rad/s, only axis swap ENU->NED is needed if you ever express as world
    // Body transform helpers using component transform
    FORCEINLINE FVector WorldToBody_FRU(const FTransform& T_wb, const FVector& v_world)
    {
        return T_wb.InverseTransformVectorNoScale(v_world);
    }
    FORCEINLINE FVector Body_FRU_to_FRD(const FVector& v_fru)
    {
        return FRU_to_FRD(v_fru);
    }

    // Quaternion ENU->NED. Build from rotation matrix P that permutes axes.
    FORCEINLINE FQuat Q_ENU_to_NED()
    {
        // P = [[0,1,0],[1,0,0],[0,0,-1]]
        const FMatrix P(
            FPlane(0, 1,  0, 0),
            FPlane(1, 0,  0, 0),
            FPlane(0, 0, -1, 0),
            FPlane(0, 0,  0, 1));
        return FQuat(P);
    }

    FORCEINLINE FMatrix S_FRD_to_FRU() {
        return FMatrix(
            FPlane( 1, 0,  0, 0),
            FPlane( 0, 1,  0, 0),
            FPlane( 0, 0, -1, 0),
            FPlane( 0, 0,  0, 1));
    }
}

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"

constexpr float SAMPLING_RES = 10.0f;
constexpr float SAMPLING_MIN_LAT = -90.0f;
constexpr float SAMPLING_MAX_LAT = 90.0f;
constexpr float SAMPLING_MIN_LON = -180.0f;
constexpr float SAMPLING_MAX_LON = 180.0f;

constexpr int32 LAT_DIM = 19;
constexpr int32 LON_DIM = 37;

class ROBOTCORE_API FGeoMagDeclination
{
public:
	static float GetMagDeclinationDegrees(float LatitudeDeg, float LongitudeDeg);
	static float GetMagInclinationDegrees(float LatitudeDeg, float LongitudeDeg);
	static float GetMagStrengthGauss(float LatitudeDeg, float LongitudeDeg);
	static float GetMagStrengthTesla(float LatitudeDeg, float LongitudeDeg);
	static FVector GetMagFieldVector(float LatitudeDeg, float LongitudeDeg);
    
private:
	static float GetTableData(float LatitudeDeg, float LongitudeDeg, const int16 Table[LAT_DIM][LON_DIM], float Scale);
	static uint32 GetLookupTableIndex(float& Val, float Min, float Max);
};
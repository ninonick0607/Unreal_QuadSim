// Fill out your copyright notice in the Description page of Project Settings.

#include "WorldMagneticModel/GeoMagDeclination.h"
#include "WorldMagneticModel/GeoMagneticTables.h"
#include "Math/UnrealMathUtility.h"

uint32 FGeoMagDeclination::GetLookupTableIndex(float& Val, float Min, float Max)
{
	// For the rare case of hitting the bounds exactly
	// Limit to table bounds - required for maxima even when table spans full globe range
	// Limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1)
	Val = FMath::Clamp(Val, Min, Max - SAMPLING_RES);
    
	return static_cast<uint32>((Val - Min) / SAMPLING_RES);
}

float FGeoMagDeclination::GetTableData(float LatitudeDeg, float LongitudeDeg, const int16 Table[LAT_DIM][LON_DIM], float Scale)
{
	// Clamp latitude to valid range
	LatitudeDeg = FMath::Clamp(LatitudeDeg, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
    
	// Wrap longitude to valid range
	if (LongitudeDeg > SAMPLING_MAX_LON)
	{
		LongitudeDeg -= 360.0f;
	}
	if (LongitudeDeg < SAMPLING_MIN_LON)
	{
		LongitudeDeg += 360.0f;
	}
    
	// Round down to nearest sampling resolution
	float MinLat = FMath::FloorToFloat(LatitudeDeg / SAMPLING_RES) * SAMPLING_RES;
	float MinLon = FMath::FloorToFloat(LongitudeDeg / SAMPLING_RES) * SAMPLING_RES;
    
	// Find index of nearest low sampling point
	uint32 MinLatIndex = GetLookupTableIndex(MinLat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	uint32 MinLonIndex = GetLookupTableIndex(MinLon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);
    
	// Get the four corner values for bilinear interpolation
	const float DataSW = static_cast<float>(Table[MinLatIndex][MinLonIndex]);
	const float DataSE = static_cast<float>(Table[MinLatIndex][MinLonIndex + 1]);
	const float DataNE = static_cast<float>(Table[MinLatIndex + 1][MinLonIndex + 1]);
	const float DataNW = static_cast<float>(Table[MinLatIndex + 1][MinLonIndex]);
    
	// Perform bilinear interpolation
	const float LatScale = FMath::Clamp((LatitudeDeg - MinLat) / SAMPLING_RES, 0.0f, 1.0f);
	const float LonScale = FMath::Clamp((LongitudeDeg - MinLon) / SAMPLING_RES, 0.0f, 1.0f);
    
	const float DataMin = LonScale * (DataSE - DataSW) + DataSW;
	const float DataMax = LonScale * (DataNE - DataNW) + DataNW;
    
	return (LatScale * (DataMax - DataMin) + DataMin) * Scale;
}

float FGeoMagDeclination::GetMagDeclinationDegrees(float LatitudeDeg, float LongitudeDeg)
{
	return GetTableData(LatitudeDeg, LongitudeDeg, DeclinationTable, WMM_DECLINATION_SCALE_TO_DEGREES);
}

float FGeoMagDeclination::GetMagInclinationDegrees(float LatitudeDeg, float LongitudeDeg)
{
	return GetTableData(LatitudeDeg, LongitudeDeg, InclinationTable, WMM_INCLINATION_SCALE_TO_DEGREES);
}

float FGeoMagDeclination::GetMagStrengthGauss(float LatitudeDeg, float LongitudeDeg)
{
	// 1 Gauss = 1e4 Tesla
	return GetMagStrengthTesla(LatitudeDeg, LongitudeDeg) * 1e4f;
}

float FGeoMagDeclination::GetMagStrengthTesla(float LatitudeDeg, float LongitudeDeg)
{
	// Table stored as scaled nanotesla
	return GetTableData(LatitudeDeg, LongitudeDeg, TotalIntensityTable, WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA) * 1e-9f;
}

FVector FGeoMagDeclination::GetMagFieldVector(float LatitudeDeg, float LongitudeDeg)
{
	// Get magnetic field parameters
	const float DeclinationRad = FMath::DegreesToRadians(GetMagDeclinationDegrees(LatitudeDeg, LongitudeDeg));
	const float InclinationRad = FMath::DegreesToRadians(GetMagInclinationDegrees(LatitudeDeg, LongitudeDeg));
	const float FieldStrengthGauss = GetMagStrengthGauss(LatitudeDeg, LongitudeDeg);
    
	// Calculate magnetic field vector in NED frame
	// Using rotation matrices similar to PX4's approach
	// First rotate by inclination around East axis, then by declination around Down axis
    
	// Initial field points North with full strength
	float FieldNorth = FieldStrengthGauss * FMath::Cos(InclinationRad) * FMath::Cos(DeclinationRad);
	float FieldEast = FieldStrengthGauss * FMath::Cos(InclinationRad) * FMath::Sin(DeclinationRad);
	float FieldDown = FieldStrengthGauss * FMath::Sin(InclinationRad);
    
	// Return in Unreal's coordinate system (adjust if needed)
	// Assuming X=North, Y=East, Z=Down for now
	return FVector(FieldNorth, FieldEast, FieldDown);
}
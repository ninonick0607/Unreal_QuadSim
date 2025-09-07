// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "QuadHUDWidget.generated.h"

class UTextureRenderTarget2D;

UCLASS()
class QUADSIMCORE_API UQuadHUDWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	// This creates an event in Blueprint that we can call directly from C++.
	// It is much safer than calling by name.
	UFUNCTION(BlueprintImplementableEvent, Category = "HUD")
	void UpdateHUDTexture(UTextureRenderTarget2D* NewTexture);
};
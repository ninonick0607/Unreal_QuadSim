// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "RobotPanelBase.generated.h"

/**
 * Base class for robot-specific UI panels.
 *
 * Extend this class in your robot plugin to create custom UI panels.
 * Each panel can maintain its own state and render logic.
 *
 * Example:
 * - UQuadControlPanel extends URobotPanelBase
 * - UGroundRobotControlPanel extends URobotPanelBase
 *
 * The SimHUD subsystem will call DrawPanel() each frame for open panels.
 */
UCLASS(Abstract, Blueprintable)
class SIMULATIONCORE_API URobotPanelBase : public UObject
{
	GENERATED_BODY()

public:
	/**
	 * Called every frame to render the panel using ImGui.
	 * Override this to implement your panel's UI.
	 *
	 * @param DeltaTime - Time since last frame in seconds
	 */
	virtual void DrawPanel(float DeltaTime) { }

	/**
	 * Get the panel's display title.
	 * @return Title to show in the window title bar
	 */
	virtual FString GetPanelTitle() const { return TEXT("Robot Panel"); }

	/**
	 * Should this panel have a button in the taskbar?
	 * @return true if a taskbar button should be shown
	 */
	virtual bool WantsTaskbarButton() const { return true; }

	/**
	 * Get the taskbar button label.
	 * @return Label for the taskbar button
	 */
	virtual FString GetTaskbarButtonLabel() const { return GetPanelTitle(); }

	/**
	 * Get the panel's unique ID (for ImGui window management).
	 * @return Unique identifier string
	 */
	virtual FString GetPanelID() const
	{
		return FString::Printf(TEXT("%s##%s"), *GetPanelTitle(), *GetName());
	}

	/**
	 * Called when the panel is opened.
	 * Override to perform initialization logic.
	 */
	virtual void OnPanelOpened() { }

	/**
	 * Called when the panel is closed.
	 * Override to perform cleanup logic.
	 */
	virtual void OnPanelClosed() { }

	/** Toggle the panel's open state */
	void ToggleOpen()
	{
		SetOpen(!bIsOpen);
	}

	/** Set the panel's open state */
	void SetOpen(bool bNewOpen)
	{
		if (bIsOpen != bNewOpen)
		{
			bIsOpen = bNewOpen;
			if (bIsOpen)
			{
				OnPanelOpened();
			}
			else
			{
				OnPanelClosed();
			}
		}
	}

	/** Check if the panel is currently open */
	bool IsOpen() const { return bIsOpen; }

public:
	/** Is this panel currently open/visible? */
	UPROPERTY()
	bool bIsOpen = false;

	/** Panel position in screen space (set to -1,-1 for auto-positioning) */
	UPROPERTY()
	FVector2D PanelPosition = FVector2D(-1.f, -1.f);

	/** Panel size in pixels */
	UPROPERTY()
	FVector2D PanelSize = FVector2D(400.f, 600.f);

	/** Should the panel size be fixed (non-resizable)? */
	UPROPERTY()
	bool bFixedSize = false;

	/** Minimum panel size (if resizable) */
	UPROPERTY()
	FVector2D MinSize = FVector2D(200.f, 200.f);

	/** Maximum panel size (if resizable) */
	UPROPERTY()
	FVector2D MaxSize = FVector2D(1920.f, 1080.f);
};

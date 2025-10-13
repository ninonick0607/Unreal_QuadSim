// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RobotUIProvider.generated.h"

class URobotPanelBase;

/**
 * Interface for robots to provide their own UI panels.
 * Implement this in your robot plugin to integrate with SimHUD.
 *
 * Example usage:
 * - QuadSimCore implements UQuadUIProvider
 * - GroundRobotCore implements UGroundRobotUIProvider
 * - Each provides their own control panels, state data panels, etc.
 */
UINTERFACE(MinimalAPI, Blueprintable)
class URobotUIProvider : public UInterface
{
	GENERATED_BODY()
};

class SIMULATIONCORE_API IRobotUIProvider
{
	GENERATED_BODY()

public:
	/**
	 * Get the robot's display name for UI purposes.
	 * @return Display name (e.g., "Quadcopter #1", "Ground Robot A")
	 */
	virtual FString GetRobotDisplayName() const = 0;

	/**
	 * Get the robot's type identifier.
	 * @return Type string (e.g., "Quadcopter", "GroundRobot", "RoboticArm")
	 */
	virtual FString GetRobotType() const = 0;

	/**
	 * Get the control panel for this robot (if supported).
	 * @return Control panel instance, or nullptr if not supported
	 */
	virtual URobotPanelBase* GetControlPanel() { return nullptr; }

	/**
	 * Get the state data panel for this robot (if supported).
	 * @return State data panel instance, or nullptr if not supported
	 */
	virtual URobotPanelBase* GetStateDataPanel() { return nullptr; }

	/**
	 * Get the configuration panel for this robot (if supported).
	 * @return Configuration panel instance, or nullptr if not supported
	 */
	virtual URobotPanelBase* GetConfigPanel() { return nullptr; }

	/**
	 * Called when this robot is selected in the manager.
	 * Override to perform selection-specific logic.
	 */
	virtual void OnSelected() {}

	/**
	 * Called when this robot is deselected.
	 * Override to perform deselection-specific logic.
	 */
	virtual void OnDeselected() {}

	/**
	 * Check if this provider is still valid (robot hasn't been destroyed).
	 * @return true if the provider and its robot are valid
	 */
	virtual bool IsValid() const { return true; }
};

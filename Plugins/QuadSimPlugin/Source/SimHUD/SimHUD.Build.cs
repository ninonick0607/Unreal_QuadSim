// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class SimHUD : ModuleRules
{
	public SimHUD(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "SimulationCore",
            "RobotCore",
            "QuadSimCore",
            "ImGui",
            "ImPlotLibrary",
            "Projects"
        });

	}
}

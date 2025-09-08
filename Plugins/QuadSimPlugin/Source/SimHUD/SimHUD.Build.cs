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
            "UMG",        
            "Slate",      
            "SlateCore",  
            "QuadSimCore",
            "SimulationCore",
            "RobotCore",
            "ImGui",
            "ImPlotLibrary",
            "Projects"
        });
		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// keep if you like, but not strictly required once in Public:
			"Slate",
			"SlateCore",
			"UMG",
			"ImGui"
		});

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}

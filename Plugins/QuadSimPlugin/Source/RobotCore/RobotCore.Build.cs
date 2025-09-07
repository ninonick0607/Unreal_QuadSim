using UnrealBuildTool;

public class RobotCore : ModuleRules
{
    public RobotCore(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            "ImGui",
            "GeoReferencing"
        });
    }
}


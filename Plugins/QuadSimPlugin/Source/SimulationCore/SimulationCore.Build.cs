using UnrealBuildTool;

public class SimulationCore : ModuleRules
{
    public SimulationCore(ReadOnlyTargetRules Target) : base(Target)
    {
        PrivateDependencyModuleNames.AddRange(new string[] { "GeoReferencing" });
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            "ImGui",
            "GeoReferencing"
        });
        
        // We'll add QuadSimCore dependency later if needed
    }
}
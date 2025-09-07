using UnrealBuildTool;
using System.IO;

public class QuadSimCore : ModuleRules
{
    public QuadSimCore(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput",
            "ChaosVehicles", "PhysicsCore", "RenderCore", "RHI",
            "Sockets", "Networking", "ImGui", "Slate", "SlateCore", 
            "UMG", "Json", "JsonUtilities","SimulationCore","RobotCore","GeoReferencing"

        });
        PrivateDependencyModuleNames.AddRange(new string[] { "Projects", "AITestSuite" });
        
        bEnableExceptions = true;
        CppStandard = CppStandardVersion.Cpp20;
        
        // MAVLink configuration - point to the root MAVLink directory
        string MAVLinkPath = Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "MAVLink");
        
        if (Directory.Exists(MAVLinkPath))
        {
            // Add the MAVLink root directory to include paths
            PublicIncludePaths.Add(MAVLinkPath);
            
            // Define which MAVLink dialect to use (common is the standard one)
            PublicDefinitions.Add("MAVLINK_DIALECT=common");
            
            // Disable some warnings that MAVLink headers might trigger
            if (Target.Platform == UnrealTargetPlatform.Win64)
            {
                PrivateDefinitions.Add("_CRT_SECURE_NO_WARNINGS");
            }
            
            System.Console.WriteLine("MAVLink found at: " + MAVLinkPath);
        }
        else
        {
            System.Console.WriteLine("MAVLink headers not found at: " + MAVLinkPath);
            System.Console.WriteLine("Please ensure MAVLink is downloaded to QuadSimPlugin/ThirdParty/MAVLink/");
        }
    }
}
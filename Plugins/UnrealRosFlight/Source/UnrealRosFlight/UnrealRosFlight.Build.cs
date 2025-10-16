using UnrealBuildTool;
using System.IO;

public class UnrealRosFlight : ModuleRules
{
	public UnrealRosFlight(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// Public dependencies are for things included in your PUBLIC headers
		PublicDependencyModuleNames.AddRange(
			new string[] { "Core", "rclUE", "Networking" });

		// Private dependencies are for things used only in your PRIVATE .cpp files
		PrivateDependencyModuleNames.AddRange(
			new string[] { "CoreUObject", "Engine", "Projects", "PhysicsCore", "Chaos", "Sockets", "Networking", "RenderCore", "RHI", "SimulationCore" });

		// Add QuadSimCore as a dependency for the communication interfaces
		string ProjectPluginsDirectory = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../")); 
		string QuadSimPluginPath = Path.Combine(ProjectPluginsDirectory, "QuadSimPlugin");
		string QuadSimCorePublicPath = Path.Combine(QuadSimPluginPath, "Source", "QuadSimCore", "Public");
		string RobotCorePublicPath = Path.Combine(QuadSimPluginPath, "Source", "RobotCore", "Public");
		
		if (Directory.Exists(QuadSimCorePublicPath))
		{
			PublicDependencyModuleNames.Add("QuadSimCore");
			PublicIncludePaths.Add(QuadSimCorePublicPath);
		}

		if (Directory.Exists(RobotCorePublicPath))
		{
			PublicDependencyModuleNames.Add("RobotCore");
			PublicIncludePaths.Add(RobotCorePublicPath);
		}

		string RosRoot     = Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "ros");
		string RosInclude  = Path.Combine(RosRoot, "include");
		string RosLib      = Path.Combine(RosRoot, "lib");

		PublicIncludePaths.Add(RosInclude);

		// add every package folder inside <include>
		foreach (var dir in Directory.GetDirectories(RosInclude))
			PublicIncludePaths.Add(dir);

#if PLATFORM_LINUX
        foreach (string so in Directory.GetFiles(RosLib, "*.so"))
        {
            PublicAdditionalLibraries.Add(so);
            RuntimeDependencies.Add(so);
        }
#endif
	}
}

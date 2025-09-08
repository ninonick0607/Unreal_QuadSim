using System.IO;
using UnrealBuildTool;

public class ImPlotLibrary : ModuleRules
{
#if WITH_FORWARDED_MODULE_RULES_CTOR
	public ImPlotLibrary(ReadOnlyTargetRules Target) : base(Target)
#else
    public ImPlotLibrary(TargetInfo Target)
#endif
	{
		// You can keep this External; UBT still discovers .cpps under this module's Source tree.
		Type = ModuleType.External;

		// Make headers visible to dependents and ensure Private sources see our macro.
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

		// The key: cut out the demo, which is where MSVC overflows happen.
		PrivateDefinitions.Add("IMPLOT_DISABLE_DEMO");

		// Optional: if you also want to hide Dear ImGui's own demo window globally:
		// PublicDefinitions.Add("IMGUI_DISABLE_DEMO_WINDOWS");
	}
}

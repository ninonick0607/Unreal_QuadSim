#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Misc/ConfigCacheIni.h"
#include "GameFramework/InputSettings.h" 

class FQuadSimCoreModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        if (TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("QuadSimCore")))
        {
            const FString ConfigPath = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Config/DefaultInput.ini"));
            if (FPaths::FileExists(ConfigPath))
            {
                GConfig->LoadFile(ConfigPath);
            }
        }

        if (UInputSettings* Settings = const_cast<UInputSettings*>(GetDefault<UInputSettings>()))
        {
            Settings->LoadConfig();  

        }
    }

    virtual void ShutdownModule() override {}
};

IMPLEMENT_MODULE(FQuadSimCoreModule, QuadSimCore)

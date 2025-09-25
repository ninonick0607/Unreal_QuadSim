
// DroneJSONConfig.cpp
#include "Core/DroneJSONConfig.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonWriter.h"
#include "Serialization/JsonSerializer.h"

UDroneJSONConfig* UDroneJSONConfig::Instance = nullptr;

// Include plugin manager to locate plugin directory for config files
#include "Interfaces/IPluginManager.h"
UDroneJSONConfig::UDroneJSONConfig()
{
    // Set safe defaults before attempting to load from disk
    // These mirror the defaults shipped in Plugins/QuadSimPlugin/Config/DroneConfig.json
    Config.FlightParams.MaxVelocityBound   = 8.f;
    Config.FlightParams.MaxVelocity        = 8.f;
    Config.FlightParams.MaxAngleBound      = 30.f;
    Config.FlightParams.MaxAngle           = 15.f;
    Config.FlightParams.MaxAngleRate       = 10.f;
    Config.FlightParams.MaxPIDOutput       = 10000.f;
    Config.FlightParams.MaxThrust          = 700.f; // Prevents accidental zeroing on failed load
    Config.FlightParams.AltitudeThreshold  = 0.6f;
    Config.FlightParams.MinAltitudeLocal   = 500.f;
    Config.FlightParams.AcceptableDistance = 200.f;

    Config.ControllerParams.AltitudeRate       = 400.f;
    Config.ControllerParams.YawRate            = 90.f;
    Config.ControllerParams.MinVelocityForYaw  = 10.f;

    Config.ObstacleParams.OuterBoundarySize = 10000.f;
    Config.ObstacleParams.InnerBoundarySize = 10000.f;
    Config.ObstacleParams.SpawnHeight       = 0.f;

    LoadConfig();
}

UDroneJSONConfig& UDroneJSONConfig::Get()
{
    if (!Instance)
    {
        Instance = NewObject<UDroneJSONConfig>();
        Instance->AddToRoot(); 
    }
    return *Instance;
}

FString UDroneJSONConfig::GetConfigFilePath() const
{
    // Attempt to load config from this plugin's Config directory
    TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("QuadSimPlugin"));
    if (Plugin.IsValid())
    {
        return FPaths::Combine(Plugin->GetBaseDir(), TEXT("Config"), TEXT("DroneConfig.json"));
    }
    // Fallback to project Config directory
    return FPaths::ProjectConfigDir() / TEXT("DroneConfig.json");
}

bool UDroneJSONConfig::LoadConfig()
{
    FString JsonString;
    const FString Path = GetConfigFilePath();
    if (!FFileHelper::LoadFileToString(JsonString, *Path))
    {
        UE_LOG(LogTemp, Warning, TEXT("DroneJSONConfig: Failed to load config file: %s. Using defaults."), *Path);
        return false;
    }

    TSharedPtr<FJsonObject> JsonObject;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);

    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("DroneJSONConfig: Failed to parse JSON from %s. Using defaults."), *Path);
        return false;
    }

    const TSharedPtr<FJsonObject>* FlightParams;
    if (JsonObject->TryGetObjectField(TEXT("flight_parameters"), FlightParams))
    {
        (*FlightParams)->TryGetNumberField(TEXT("max_velocity_bound"), Config.FlightParams.MaxVelocityBound); 
        (*FlightParams)->TryGetNumberField(TEXT("max_velocity"), Config.FlightParams.MaxVelocity); 
        (*FlightParams)->TryGetNumberField(TEXT("max_angle_bound"), Config.FlightParams.MaxAngleBound);
        (*FlightParams)->TryGetNumberField(TEXT("max_angle"),        Config.FlightParams.MaxAngle);
        (*FlightParams)->TryGetNumberField(TEXT("max_angle_rate"), Config.FlightParams.MaxAngleRate);
        (*FlightParams)->TryGetNumberField(TEXT("max_pid_output"), Config.FlightParams.MaxPIDOutput);
        (*FlightParams)->TryGetNumberField(TEXT("max_thrust"), Config.FlightParams.MaxThrust);
        (*FlightParams)->TryGetNumberField(TEXT("altitude_threshold"), Config.FlightParams.AltitudeThreshold);
        (*FlightParams)->TryGetNumberField(TEXT("min_altitude_local"), Config.FlightParams.MinAltitudeLocal);
        (*FlightParams)->TryGetNumberField(TEXT("acceptable_distance"), Config.FlightParams.AcceptableDistance);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("DroneJSONConfig: 'flight_parameters' not found in %s. Keeping defaults."), *Path);
    }
    
    const TSharedPtr<FJsonObject>* ControllerParams;
    if (JsonObject->TryGetObjectField(TEXT("controller"), ControllerParams))
    {
        (*ControllerParams)->TryGetNumberField(TEXT("altitude_rate"), Config.ControllerParams.AltitudeRate);
        (*ControllerParams)->TryGetNumberField(TEXT("yaw_rate"), Config.ControllerParams.YawRate);
        (*ControllerParams)->TryGetNumberField(TEXT("min_velocity_for_yaw"), Config.ControllerParams.MinVelocityForYaw);
    }
    else
    {
        UE_LOG(LogTemp, Verbose, TEXT("DroneJSONConfig: 'controller' not found in %s. Keeping defaults."), *Path);
    }
    
    const TSharedPtr<FJsonObject>* ObstacleParams;
    if (JsonObject->TryGetObjectField(TEXT("obstacle_parameters"), ObstacleParams))
    {
        (*ObstacleParams)->TryGetNumberField(TEXT("outer_boundary"), Config.ObstacleParams.OuterBoundarySize);
        (*ObstacleParams)->TryGetNumberField(TEXT("inner_boundary"), Config.ObstacleParams.InnerBoundarySize);
        (*ObstacleParams)->TryGetNumberField(TEXT("spawn_height"), Config.ObstacleParams.SpawnHeight);
    }
    else
    {
        UE_LOG(LogTemp, Verbose, TEXT("DroneJSONConfig: 'obstacle_parameters' not found in %s. Keeping defaults."), *Path);
    }
    return true;
}

bool UDroneJSONConfig::ReloadConfig()
{
    return LoadConfig();
}

bool UDroneJSONConfig::SaveConfig()
 {
     // Build JSON object
     TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
 
     // Flight parameters
     TSharedPtr<FJsonObject> FlightParamsObj = MakeShared<FJsonObject>();
     FlightParamsObj->SetNumberField(TEXT("max_velocity_bound"),    Config.FlightParams.MaxVelocityBound);
     FlightParamsObj->SetNumberField(TEXT("max_velocity"),          Config.FlightParams.MaxVelocity);
     FlightParamsObj->SetNumberField(TEXT("max_angle_bound"),       Config.FlightParams.MaxAngleBound);
     FlightParamsObj->SetNumberField(TEXT("max_angle"),             Config.FlightParams.MaxAngle);
     FlightParamsObj->SetNumberField(TEXT("max_angle_rate"),        Config.FlightParams.MaxAngleRate);
     FlightParamsObj->SetNumberField(TEXT("max_pid_output"),        Config.FlightParams.MaxPIDOutput);
     FlightParamsObj->SetNumberField(TEXT("max_thrust"),            Config.FlightParams.MaxThrust);
     FlightParamsObj->SetNumberField(TEXT("altitude_threshold"),    Config.FlightParams.AltitudeThreshold);
     FlightParamsObj->SetNumberField(TEXT("min_altitude_local"),    Config.FlightParams.MinAltitudeLocal);
     FlightParamsObj->SetNumberField(TEXT("acceptable_distance"),   Config.FlightParams.AcceptableDistance);
     JsonObject->SetObjectField(TEXT("flight_parameters"), FlightParamsObj);
 
     // Controller parameters
     TSharedPtr<FJsonObject> ControllerParamsObj = MakeShared<FJsonObject>();
     ControllerParamsObj->SetNumberField(TEXT("altitude_rate"),        Config.ControllerParams.AltitudeRate);
     ControllerParamsObj->SetNumberField(TEXT("yaw_rate"),             Config.ControllerParams.YawRate);
     ControllerParamsObj->SetNumberField(TEXT("min_velocity_for_yaw"), Config.ControllerParams.MinVelocityForYaw);
     JsonObject->SetObjectField(TEXT("controller"), ControllerParamsObj);
 
     // Obstacle parameters
     TSharedPtr<FJsonObject> ObstacleParamsObj = MakeShared<FJsonObject>();
     ObstacleParamsObj->SetNumberField(TEXT("outer_boundary"), Config.ObstacleParams.OuterBoundarySize);
     ObstacleParamsObj->SetNumberField(TEXT("inner_boundary"), Config.ObstacleParams.InnerBoundarySize);
     ObstacleParamsObj->SetNumberField(TEXT("spawn_height"),   Config.ObstacleParams.SpawnHeight);
     JsonObject->SetObjectField(TEXT("obstacle_parameters"), ObstacleParamsObj);
 
     // Serialize JSON to string
     FString OutputString;
     TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
     if (FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer))
     {
         // Save to file
         const FString OutPath = GetConfigFilePath();
         const bool bOk = FFileHelper::SaveStringToFile(OutputString, *OutPath);
         if (!bOk)
         {
             UE_LOG(LogTemp, Error, TEXT("DroneJSONConfig: Failed to save config to %s"), *OutPath);
         }
         return bOk;
     }
     return false;
 }

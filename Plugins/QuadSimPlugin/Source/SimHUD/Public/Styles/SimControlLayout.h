#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SimControlLayout.generated.h"

// Forward decl; the .cpp includes the real header
class UDroneJSONConfig;

UENUM(BlueprintType)
enum class EControlMode : uint8
{
    Position,
    Velocity,
    Angle,
    Acro
};

UENUM(BlueprintType)
enum class EAxisChannel : uint8
{
    X, Y, Z,
    Roll, Pitch, Yaw,
    YawRate,
    Throttle
};

USTRUCT(BlueprintType)
struct FAxisSpec
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite) EAxisChannel Channel = EAxisChannel::Yaw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) FText        Label;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) FText        Units;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float        Min = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float        Max = 1.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float        Default = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float        Step = 0.1f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) bool         bWrap = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) bool         bShowSpin = true;
};

USTRUCT(BlueprintType)
struct FModeLayout
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FAxisSpec> Axes;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnLayoutChanged);

UCLASS(BlueprintType)
class SIMHUD_API USimControlLayout : public UDataAsset
{
    GENERATED_BODY()

public:
    USimControlLayout();

    /** Main layouts per mode */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Layout")
    TMap<EControlMode, FModeLayout> Layouts;

    /** Optional gamepad variants (kept for parity) */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Layout")
    FModeLayout GamepadAngle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Layout")
    FModeLayout GamepadAcro;

    /** Live copy of JSON knobs (mirror what ImGui used) */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float MaxVelocityBound = 8.f;     // flight_parameters.max_velocity_bound

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float MaxVelocity      = 1.f;     // flight_parameters.max_velocity (SliderMaxVelocity)

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float MaxAngle         = 15.f;    // flight_parameters.max_angle (SliderMaxAngle)  [deg]

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float MaxAngleRate     = 10.f;    // flight_parameters.max_angle_rate (SliderMaxAngleRate) [deg/s]

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float MaxThrust        = 700.f;   // flight_parameters.max_thrust (kept for future use)

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Config")
    float YawRateLimit     = 50.f;    // controller.yaw_rate [deg/s]

    /** Notify listeners (ControlPanel, Taskbar) that limits changed */
    UPROPERTY(BlueprintAssignable)
    FOnLayoutChanged OnLayoutChanged;

    /** JSON reload entry (same loader you already use) */
    UFUNCTION(BlueprintCallable, Category="Layout")
    void RefreshFromConfig(bool bForceReloadFile = false);

    /** ImGui-equivalent setters — rebuild Layouts + broadcast */
    UFUNCTION(BlueprintCallable, Category="Layout") void SetMaxVelocity(float V);
    UFUNCTION(BlueprintCallable, Category="Layout") void SetMaxAngle(float A);
    UFUNCTION(BlueprintCallable, Category="Layout") void SetMaxAngleRate(float R);
    UFUNCTION(BlueprintCallable, Category="Layout") void SetYawRateLimit(float Y);

    UFUNCTION(BlueprintCallable, BlueprintPure, Category="Layout")
    const FModeLayout& GetLayout(EControlMode Mode, bool bGamepadOnly) const;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& E) override;
#endif

private:
    void BuildLayoutsFromCurrentConfig();
    void LoadConfigValues(bool bForceReloadFile);
    void BroadcastChanged() { OnLayoutChanged.Broadcast(); }
};

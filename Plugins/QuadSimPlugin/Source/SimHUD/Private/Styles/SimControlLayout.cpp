#include "Styles/SimControlLayout.h"
#include "Internationalization/Text.h"
#include "Core/DroneJSONConfig.h"   // your existing JSON loader

#define LOCTEXT_NAMESPACE "SimControlLayout"

static FAxisSpec MakeAxis(
    EAxisChannel Channel,
    const FString& Label,
    const FString& Units,
    float Min, float Max,
    float Default = 0.f,
    float Step = 0.1f,
    bool bWrap = false,
    bool bShowSpin = true)
{
    FAxisSpec A;
    A.Channel   = Channel;
    A.Label     = FText::FromString(Label);
    A.Units     = FText::FromString(Units);
    A.Min       = Min;
    A.Max       = Max;
    A.Default   = Default;
    A.Step      = Step;
    A.bWrap     = bWrap;
    A.bShowSpin = bShowSpin;
    return A;
}

USimControlLayout::USimControlLayout()
{
    LoadConfigValues(false);
    BuildLayoutsFromCurrentConfig();
}

void USimControlLayout::LoadConfigValues(bool bForceReloadFile)
{
    UDroneJSONConfig& Cfg = UDroneJSONConfig::Get();
    if (bForceReloadFile)
    {
        Cfg.ReloadConfig();
    }

    // Mirror ImGui knobs:
    MaxVelocityBound = Cfg.Config.FlightParams.MaxVelocityBound;
    MaxThrust        = Cfg.Config.FlightParams.MaxThrust;

    MaxVelocity      = Cfg.Config.FlightParams.MaxVelocity;     // SliderMaxVelocity
    MaxAngle         = Cfg.Config.FlightParams.MaxAngle;        // SliderMaxAngle
    MaxAngleRate     = Cfg.Config.FlightParams.MaxAngleRate;    // SliderMaxAngleRate
    YawRateLimit     = Cfg.Config.ControllerParams.YawRate;     // controller.yaw_rate

    // Safety/fallbacks:
    if (MaxVelocityBound <= 0.f) MaxVelocityBound = 8.f;
    MaxVelocity  = FMath::Clamp(MaxVelocity, 0.f, MaxVelocityBound);
    if (MaxAngle      <= 0.f) MaxAngle      = 15.f;
    if (MaxAngleRate  <= 0.f) MaxAngleRate  = 10.f;
    if (YawRateLimit  <= 0.f) YawRateLimit  = 50.f;
}

void USimControlLayout::BuildLayoutsFromCurrentConfig()
{
    Layouts.Empty();

    // Position: XYZ (m) + YawRate (deg/s)
    {
        FModeLayout M;
        M.Axes = {
            MakeAxis(EAxisChannel::X,       "X Coordinate", "m",    -50.f, 50.f, 0.f, 0.1f),
            MakeAxis(EAxisChannel::Y,       "Y Coordinate", "m",    -50.f, 50.f, 0.f, 0.1f),
            MakeAxis(EAxisChannel::Z,       "Z Coordinate", "m",      0.f, 50.f, 0.f, 0.1f),
            MakeAxis(EAxisChannel::YawRate, "Yaw Rate",     "deg/s", -YawRateLimit, YawRateLimit, 0.f, 1.f)
        };
        Layouts.Add(EControlMode::Position, M);
    }

    // Velocity: Vx Vy Vz (m/s) ± MaxVelocity; YawRate ± YawRateLimit
    {
        FModeLayout M;
        M.Axes = {
            MakeAxis(EAxisChannel::X,       "X Velocity", "m/s", -MaxVelocity, MaxVelocity, 0.f, 0.1f),
            MakeAxis(EAxisChannel::Y,       "Y Velocity", "m/s", -MaxVelocity, MaxVelocity, 0.f, 0.1f),
            MakeAxis(EAxisChannel::Z,       "Z Velocity", "m/s", -MaxVelocity, MaxVelocity, 0.f, 0.1f),
            MakeAxis(EAxisChannel::YawRate, "Yaw Rate",   "deg/s", -YawRateLimit, YawRateLimit, 0.f, 1.f)
        };
        Layouts.Add(EControlMode::Velocity, M);
    }

    // Angle: Roll Pitch (deg) ± MaxAngle; YawRate ± YawRateLimit; ZVel ± MaxVelocity
    {
        FModeLayout M;
        M.Axes = {
            MakeAxis(EAxisChannel::Roll,    "Roll",     "deg",   -MaxAngle, MaxAngle, 0.f, 0.5f),
            MakeAxis(EAxisChannel::Pitch,   "Pitch",    "deg",   -MaxAngle, MaxAngle, 0.f, 0.5f),
            MakeAxis(EAxisChannel::YawRate, "Yaw Rate", "deg/s", -YawRateLimit, YawRateLimit, 0.f, 1.f),
            MakeAxis(EAxisChannel::Z,       "Z Velocity","m/s",  -MaxVelocity, MaxVelocity, 0.f, 0.1f)
        };
        Layouts.Add(EControlMode::Angle, M);
        GamepadAngle = M;
    }

    // Acro/Rate: RollRate/PitchRate/YawRate (deg/s) ± MaxAngleRate; ZVel ± MaxVelocity
    {
        FModeLayout M;
        M.Axes = {
            MakeAxis(EAxisChannel::Roll,    "Roll Rate",  "deg/s", -MaxAngleRate, MaxAngleRate, 0.f, 1.f),
            MakeAxis(EAxisChannel::Pitch,   "Pitch Rate", "deg/s", -MaxAngleRate, MaxAngleRate, 0.f, 1.f),
            MakeAxis(EAxisChannel::YawRate, "Yaw Rate",   "deg/s", -MaxAngleRate, MaxAngleRate, 0.f, 1.f),
            MakeAxis(EAxisChannel::Z,       "Z Velocity", "m/s",   -MaxVelocity,  MaxVelocity,  0.f, 0.1f)
        };
        Layouts.Add(EControlMode::Acro, M);
        GamepadAcro = M;
    }

    // Sanitize
    auto Sanitize = [](FModeLayout& ML)
    {
        for (FAxisSpec& A : ML.Axes)
        {
            if (A.Max < A.Min) { Swap(A.Min, A.Max); }
            A.Default = FMath::Clamp(A.Default, A.Min, A.Max);
            A.Step    = FMath::Max(KINDA_SMALL_NUMBER, A.Step);
        }
    };
    for (auto& Pair : Layouts) { Sanitize(Pair.Value); }
    Sanitize(GamepadAngle);
    Sanitize(GamepadAcro);
}

const FModeLayout& USimControlLayout::GetLayout(EControlMode Mode, bool bGamepadOnly) const
{
    if (bGamepadOnly)
    {
        if (Mode == EControlMode::Angle) return GamepadAngle;
        if (Mode == EControlMode::Acro)  return GamepadAcro;
    }
    if (const FModeLayout* Found = Layouts.Find(Mode))
    {
        return *Found;
    }
    static const FModeLayout Empty;
    return Empty;
}

void USimControlLayout::RefreshFromConfig(bool bForceReloadFile /*=false*/)
{
    LoadConfigValues(bForceReloadFile);
    BuildLayoutsFromCurrentConfig();
    BroadcastChanged();
}

void USimControlLayout::SetMaxVelocity(float V)
{
    MaxVelocity = FMath::Clamp(V, 0.f, MaxVelocityBound);
    BuildLayoutsFromCurrentConfig();
    BroadcastChanged();
}
void USimControlLayout::SetMaxAngle(float A)
{
    MaxAngle = FMath::Max(0.f, A);
    BuildLayoutsFromCurrentConfig();
    BroadcastChanged();
}
void USimControlLayout::SetMaxAngleRate(float R)
{
    MaxAngleRate = FMath::Max(0.f, R);
    BuildLayoutsFromCurrentConfig();
    BroadcastChanged();
}
void USimControlLayout::SetYawRateLimit(float Y)
{
    YawRateLimit = FMath::Max(0.f, Y);
    BuildLayoutsFromCurrentConfig();
    BroadcastChanged();
}

#if WITH_EDITOR
void USimControlLayout::PostEditChangeProperty(FPropertyChangedEvent& E)
{
    Super::PostEditChangeProperty(E);

    if (!E.Property) return;

    static const FName Names[] = {
        GET_MEMBER_NAME_CHECKED(USimControlLayout, MaxVelocityBound),
        GET_MEMBER_NAME_CHECKED(USimControlLayout, MaxVelocity),
        GET_MEMBER_NAME_CHECKED(USimControlLayout, MaxAngle),
        GET_MEMBER_NAME_CHECKED(USimControlLayout, MaxAngleRate),
        GET_MEMBER_NAME_CHECKED(USimControlLayout, MaxThrust),
        GET_MEMBER_NAME_CHECKED(USimControlLayout, YawRateLimit)
    };
    for (const FName& N : Names)
    {
        if (E.Property->GetFName() == N)
        {
            // Keep invariants, then rebuild + notify
            MaxVelocity = FMath::Clamp(MaxVelocity, 0.f, MaxVelocityBound);
            BuildLayoutsFromCurrentConfig();
            BroadcastChanged();
            break;
        }
    }
}
#endif

#undef LOCTEXT_NAMESPACE

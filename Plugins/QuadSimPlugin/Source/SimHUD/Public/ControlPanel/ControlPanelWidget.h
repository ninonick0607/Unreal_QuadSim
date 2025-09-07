#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styles/SimControlLayout.h"
#include "ControlPanelWidget.generated.h"

// fwd
class UCheckBox;
class UScrollBox;
class UVerticalBox;
class USizeBox;
class UPanelWidget;
class UModeSelector;
class USliderRow;
class UWidget;
class UButton;
class UIconButtonGeneral;
class UQuadDroneController;
class ADroneManager;
class AQuadPawn;
enum class EFlightMode : uint8;

/** Per-mode parameter change */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(
    FOnControlParamChanged, EControlMode, Mode, EAxisChannel, Channel, float, Value);

/** Control-settings identifiers */
UENUM(BlueprintType)
enum class EControlSetting : uint8
{
    MaxVelocity     UMETA(DisplayName="Max Velocity"),
    MaxAngle        UMETA(DisplayName="Max Angle"),
    MaxAngleRate    UMETA(DisplayName="Max Angle Rate"),
};

/** Settings change */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
    FOnControlSettingChanged, EControlSetting, Setting, float, Value);

UCLASS()
class SIMHUD_API UControlPanelWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintAssignable, Category="ControlPanel")
    FOnControlParamChanged OnControlParamChanged;

    UPROPERTY(BlueprintAssignable, Category="ControlPanel|Settings")
    FOnControlSettingChanged OnControlSettingChanged;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="ControlPanel")
    USimControlLayout* Layout = nullptr;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category="ControlPanel")
    TSubclassOf<USliderRow> SliderRowClass;

    UPROPERTY(BlueprintReadOnly, Category="ControlPanel")
    EControlMode CurrentMode = EControlMode::Position;

    UPROPERTY(BlueprintReadOnly, Category="ControlPanel")
    bool bGamepadOnly = false;

    UFUNCTION(BlueprintCallable, Category="ControlPanel")
    void SetMode(EControlMode NewMode);

    UFUNCTION(BlueprintCallable, Category="ControlPanel")
    void SetGamepadOnly(bool bOn);

    UFUNCTION(BlueprintCallable, Category="ControlPanel")
    void SetChannelValue(EAxisChannel Channel, float Value, bool bSilent=false);

    UFUNCTION(BlueprintPure, Category="ControlPanel|Settings")
    void GetControlSettings(float& OutMaxVel, float& OutMaxAngle, float& OutMaxAngleRate) const
    { OutMaxVel = MaxVel; OutMaxAngle = MaxAng; OutMaxAngleRate = MaxRate; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ControlPanel|Targeting", meta=(ExposeOnSpawn=true))
	UQuadDroneController* ControllerOverride = nullptr;

	// If true and a DroneManager reports swarm mode, we’ll apply to all drones
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ControlPanel|Targeting")
	bool bApplyToSwarm = false;
protected:
    virtual void NativeConstruct() override;
	UFUNCTION() void HandleLayoutChangedExternal();
    // Dynamic, per-mode rows live here
    UPROPERTY(meta=(BindWidgetOptional))   UScrollBox*   Scroll_Sliders = nullptr;
    UPROPERTY(meta=(BindWidget))           UVerticalBox* VBox_Sliders   = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   USizeBox*     SizeBox_BodyLimiter = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   UCheckBox*    ChkGamepad = nullptr;

    /** Control Settings section */
    UPROPERTY(meta=(BindWidgetOptional))   UVerticalBox* VB_Settings = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   USliderRow*   Row_MaxVelocity = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   USliderRow*   Row_MaxAngle = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   USliderRow*   Row_MaxAngleRate = nullptr;

    /** “Flight Mode Controls” extras you want to show/hide per mode */
	UPROPERTY(meta=(BindWidgetOptional)) UIconButtonGeneral* Btn_ActivateHoverMode = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   USliderRow*   Row_HoverAltitude = nullptr;
    UPROPERTY(meta=(BindWidgetOptional))   UPanelWidget* HBox_Resets = nullptr;   // container for Reset buttons

	UPROPERTY(meta=(BindWidgetOptional))
	UModeSelector* ModeSelector = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ControlPanel", meta=(ExposeOnSpawn=true))
	UModeSelector* ModeSelectorOverride = nullptr;
    // --- UI handlers
    UFUNCTION() void OnGamepadChanged(bool bChecked);
    UFUNCTION() void HandleRowChanged(EAxisChannel Channel, float Value);  // mode-driven row change
    UFUNCTION() void HandleModeChanged(EControlMode NewMode);              // from UModeSelector

    // Control Settings handlers
    UFUNCTION() void OnMaxVelocityChanged(EAxisChannel Channel, float Value);
    UFUNCTION() void OnMaxAngleChanged(EAxisChannel Channel, float Value);
    UFUNCTION() void OnMaxAngleRateChanged(EAxisChannel Channel, float Value);

    // Extras
    UFUNCTION() void OnActivateHoverModeClicked();
	UFUNCTION() void ApplyParamToController(EControlMode Mode, EAxisChannel Channel, float Value);


private:
	UPROPERTY(Transient)
	UQuadDroneController* ResolvedController = nullptr;
	
    UPROPERTY(EditAnywhere, Category="ControlPanel")
    bool bSyncModeSelectorUI = true;

    TMap<EAxisChannel, float> Values_Position;
    TMap<EAxisChannel, float> Values_Velocity;
    TMap<EAxisChannel, float> Values_Angle;
    TMap<EAxisChannel, float> Values_Acro;

    float MaxVel  = 10.f;   // m/s
    float MaxAng  = 30.f;   // deg
    float MaxRate = 120.f;  // deg/s
	void BindToModeSelector();

    TMap<EAxisChannel,float>& ValuesFor(EControlMode Mode);
    const TMap<EAxisChannel,float>& ValuesFor(EControlMode Mode) const;

    void RebuildSliderList();
    void ApplyGamepadFilterToButtons();
    void UpdateModeSpecificVisibility();   // NEW: toggle extras per mode
    UModeSelector* FindFirstModeSelector() const;

	// ---- Helpers ----
	void ResolveControllers();
	void ForEachTargetController(TFunctionRef<void(UQuadDroneController*)> Fn);

	// Pushers
	void PushVelocityVector();     // (Vx,Vy,Vz) → SetDesiredVelocity
	void PushYawRate();            // yaw rate   → SetDesiredYawRate
	void PushAngleTargets(); 
	// Convenience getters from our value map
	float GetValueOrDefault(EAxisChannel Ch, float Default = 0.f) const;

	void PushFlightModeToControllers();
	EFlightMode MapControlModeToFlightMode(EControlMode Mode) const;
	UPROPERTY(Transient)
	bool bUserPickedMode = false;
};

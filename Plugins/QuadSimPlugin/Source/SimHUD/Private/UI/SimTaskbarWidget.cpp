#include "UI/SimTaskbarWidget.h"
#include "Components/Button.h"
#include "Blueprint/WidgetTree.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerStart.h"
#include "GameFramework/PlayerController.h"

#include "SimulationCore/Public/Core/SimulationManager.h"
#include "Core/DroneManager.h"
#include "Pawns/QuadPawn.h"
#include "QuadSimCore/Public/QuadSimPlayerController.h"

// ------ Manager init ------

void USimTaskbarWidget::InitializeManagers()
{
    // Preferred: use the pointers assigned in the widget instance
    if (SimulationManagerRef) SimMgr = SimulationManagerRef;
    if (DroneManagerRef)      DroneMgr = DroneManagerRef;

    // Fallbacks if not assigned
    if (!SimMgr.IsValid())
    {
        if (UWorld* W = GetWorld())
        {
            if (ASimulationManager* Found = ASimulationManager::Get(W))
            {
                SimMgr = Found;
            }
            else
            {
                TArray<AActor*> FoundActors;
                UGameplayStatics::GetAllActorsOfClass(W, ASimulationManager::StaticClass(), FoundActors);
                if (FoundActors.Num() > 0) SimMgr = Cast<ASimulationManager>(FoundActors[0]);
            }
        }
    }

    if (!DroneMgr.IsValid())
    {
        if (UWorld* W = GetWorld())
        {
            if (ADroneManager* Found = ADroneManager::Get(W))
            {
                DroneMgr = Found;
            }
            else
            {
                TArray<AActor*> FoundActors;
                UGameplayStatics::GetAllActorsOfClass(W, ADroneManager::StaticClass(), FoundActors);
                if (FoundActors.Num() > 0) DroneMgr = Cast<ADroneManager>(FoundActors[0]);
            }
        }
    }

    UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Managers: Sim=%p, DM=%p"),
        SimMgr.Get(), DroneMgr.Get());
}

// ------ Inner button resolution for WBP_* children ------

UButton* USimTaskbarWidget::ResolveInnerButton(UUserWidget* Child, const FName& InnerName)
{
    if (!Child) return nullptr;
    if (UWidget* W = Child->GetWidgetFromName(InnerName))
    {
        if (UButton* B = Cast<UButton>(W)) return B;
    }
    return nullptr;
}

// ------ Life cycle ------

void USimTaskbarWidget::NativeConstruct()
{
    Super::NativeConstruct();

    InitializeManagers();

    // Bind Pause/Play (WBP_ToggleIconButton)
    if (UButton* B = ResolveInnerButton(BtnPausePlay, InnerButtonName))
    {
        B->OnClicked.AddDynamic(this, &USimTaskbarWidget::OnPausePlayClicked);
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Bound BtnPausePlay"));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[SimTaskbar] BtnPausePlay inner button not found (name=%s)"), *InnerButtonName.ToString());
    }

    // Bind Faster / Slower / Step (WBP_IconButton)
    if (UButton* B = ResolveInnerButton(BtnFaster, InnerButtonName))
    {
        B->OnClicked.AddDynamic(this, &USimTaskbarWidget::OnFasterClicked);
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Bound BtnFaster"));
    }
    else UE_LOG(LogTemp, Warning, TEXT("[SimTaskbar] BtnFaster inner button not found"));

    if (UButton* B = ResolveInnerButton(BtnSlower, InnerButtonName))
    {
        B->OnClicked.AddDynamic(this, &USimTaskbarWidget::OnSlowerClicked);
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Bound BtnSlower"));
    }
    else UE_LOG(LogTemp, Warning, TEXT("[SimTaskbar] BtnSlower inner button not found"));

    if (UButton* B = ResolveInnerButton(BtnStep, InnerButtonName))
    {
        B->OnClicked.AddDynamic(this, &USimTaskbarWidget::OnStepClicked);
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Bound BtnStep"));
    }
    else UE_LOG(LogTemp, Warning, TEXT("[SimTaskbar] BtnStep inner button not found"));

    // Bind Spawn (direct UButton instance)
    if (BtnSpawn)
    {
        BtnSpawn->OnClicked.AddDynamic(this, &USimTaskbarWidget::OnSpawnClicked);
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] Bound BtnSpawn"));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[SimTaskbar] BtnSpawn not assigned"));
    }
	UpdateSpeedText(); 
}
void USimTaskbarWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);
	UpdateSpeedText();
}
// ------ Helpers ------

float USimTaskbarWidget::ReadTimeScale() const
{
    if (const UWorld* W = GetWorld())
    if (const AWorldSettings* WS = W->GetWorldSettings())
        return WS->TimeDilation;

    return 1.f;
}

void USimTaskbarWidget::SetTimeScaleClamped(float s)
{
    if (ASimulationManager* SM = GetSim())
    {
        SM->SetTimeScale(FMath::Clamp(s, 0.1f, 10.0f));
        UE_LOG(LogTemp, Display, TEXT("[SimTaskbar] TimeScale -> %.2f"), ReadTimeScale());
    }
}

// ------ Handlers ------

void USimTaskbarWidget::OnPausePlayClicked()
{
	if (ASimulationManager* SM = GetSim())
	{
		const auto Mode = SM->GetSimulationMode();
		if (Mode == ESimulationMode::Paused) SM->SetSimulationMode(ESimulationMode::Realtime);
		else                                 SM->SetSimulationMode(ESimulationMode::Paused);
		UpdateSpeedText();
	}
}

static FString FormatScaleLabel(ESimulationMode Mode, float TimeDilation)
{
	// What to show:
	// - Paused: show "x0.00"
	// - Realtime: "x1.00"
	// - FastForward: actual dilation
	// - Lockstep: show fixed dt stepping; label like "x1.00" is okay, or customize if you prefer
	float s = 1.0f;

	switch (Mode)
	{
	case ESimulationMode::Paused:      s = 0.0f; break;
	case ESimulationMode::Realtime:    s = 1.0f; break;
	case ESimulationMode::FastForward: s = TimeDilation; break;
	case ESimulationMode::Lockstep:    s = 1.0f; break;
	default:                           s = 1.0f; break;
	}

	// Clamp for display; UE sets ~0.0001 in paused, show 0.00 to user
	if (s < 0.005f) s = 0.0f;

	return FString::Printf(TEXT("x%.2f"), s);
}

void USimTaskbarWidget::UpdateSpeedText()
{
	if (!TxtSpeed) return;

	ASimulationManager* SM = GetSim();
	const UWorld* W = GetWorld();
	if (!SM || !W || !W->GetWorldSettings()) return;

	const ESimulationMode Mode = SM->GetSimulationMode();
	const float TD = W->GetWorldSettings()->TimeDilation;

	const FString Label = FormatScaleLabel(Mode, TD);

	// Avoid redundant SetText calls
	float NumericShown = 1.f;
	if (Mode == ESimulationMode::Paused)      NumericShown = 0.f;
	else if (Mode == ESimulationMode::Realtime) NumericShown = 1.f;
	else if (Mode == ESimulationMode::FastForward) NumericShown = TD;
	else NumericShown = 1.f;

	if (!FMath::IsNearlyEqual(NumericShown, LastShownScale, 0.001f))
	{
		TxtSpeed->SetText(FText::FromString(Label));
		LastShownScale = NumericShown;
	}
}


void USimTaskbarWidget::OnFasterClicked()
{
	if (ASimulationManager* SM = GetSim())
	{
		if (SM->GetSimulationMode() != ESimulationMode::FastForward)
			SM->SetSimulationMode(ESimulationMode::FastForward);

		const float cur = ReadTimeScale();
		SetTimeScaleClamped(cur + 0.25f);
		UpdateSpeedText();
	}
}


void USimTaskbarWidget::OnSlowerClicked()
{
	if (ASimulationManager* SM = GetSim())
	{
		if (SM->GetSimulationMode() != ESimulationMode::FastForward)
			SM->SetSimulationMode(ESimulationMode::FastForward);

		const float cur = ReadTimeScale();
		SetTimeScaleClamped(cur - 0.10f);
		UpdateSpeedText();
	}
}

void USimTaskbarWidget::OnStepClicked()
{
	if (ASimulationManager* SM = GetSim())
	{
		if (SM->GetSimulationMode() != ESimulationMode::Paused)
			SM->SetSimulationMode(ESimulationMode::Paused);

		SM->RequestSimulationStep();
		UpdateSpeedText();
	}
}
void USimTaskbarWidget::OnSpawnClicked()
{
    if (ADroneManager* DM = GetDM())
    {
        // Same logic as your ImGui path
        FVector  SpawnLoc = FVector::ZeroVector;
        FRotator SpawnRot = FRotator::ZeroRotator;

        if (AQuadPawn* NewDrone = DM->SpawnDrone(SpawnLoc, SpawnRot))
        {
            if (APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0))
            {
                if (PC && NewDrone && PC->GetPawn() != NewDrone)
                {
                    PC->Possess(NewDrone);
                    PC->SetViewTarget(NewDrone);
                    if (AQuadSimPlayerController* QPC = Cast<AQuadSimPlayerController>(PC))
                    {
                        QPC->ApplyGameOnly();
                    }
                    else
                    {
                        FInputModeGameOnly Mode; Mode.SetConsumeCaptureMouseDown(true);
                        PC->SetInputMode(Mode);
                        PC->bShowMouseCursor = false;
                    }
                }
            }
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[Taskbar] Spawn failed (QuadPawnClass?)"));
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[Taskbar] No DroneManager"));
    }
}


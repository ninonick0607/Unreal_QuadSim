#include "MainWidget.h"
#include "Components/Button.h"
#include "Components/PanelWidget.h"
#include "Components/Border.h"
#include "Blueprint/UserWidget.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"

void UMainWidget::NativeConstruct()
{
    Super::NativeConstruct();

    ControlPanelButton = ResolveControlPanelButton();
    if (ControlPanelButton)
    {
        ControlPanelButton->OnClicked.Clear();
        ControlPanelButton->OnClicked.AddDynamic(this, &UMainWidget::HandleControlPanelClicked);
        UE_LOG(LogTemp, Log, TEXT("[MainWidget] Bound click for '%s'"), *ControlPanelButtonName.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[MainWidget] Could not find a UButton under widget named '%s'"),
               *ControlPanelButtonName.ToString());
    }
}

void UMainWidget::NativeDestruct()
{
    if (ControlPanelInstance)
    {
        ControlPanelInstance->RemoveFromParent();
        ControlPanelInstance = nullptr;
    }
    Super::NativeDestruct();
}

void UMainWidget::HandleControlPanelClicked()
{
    ToggleControlPanel();
}

void UMainWidget::ToggleControlPanel()
{
    // Create on first use
    if (!ControlPanelInstance)
    {
        if (!ControlPanelClass)
        {
            UE_LOG(LogTemp, Warning, TEXT("[MainWidget] ControlPanelClass not set on %s."), *GetName());
            return;
        }

        APlayerController* PC = GetOwningPlayer();
        if (!PC)
        {
            UE_LOG(LogTemp, Warning, TEXT("[MainWidget] No owning player."));
            return;
        }

        ControlPanelInstance = CreateWidget<UUserWidget>(PC, ControlPanelClass);
        if (ControlPanelInstance)
        {
            ControlPanelInstance->AddToViewport(ControlPanelZOrder);
            ControlPanelInstance->SetVisibility(ESlateVisibility::Visible);

            // Make sure UI gets focus & mouse
            PC->bShowMouseCursor = true;
            PC->bEnableClickEvents = true;
            PC->bEnableMouseOverEvents = true;

            FInputModeGameAndUI Mode;
            Mode.SetWidgetToFocus(ControlPanelInstance->TakeWidget()); // <-- focus it
            Mode.SetHideCursorDuringCapture(false);
            Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
            PC->SetInputMode(Mode);

            UE_LOG(LogTemp, Log, TEXT("[MainWidget] Control Panel created & shown."));
        }
        return;
    }

    // Already created -> toggle visibility
    const ESlateVisibility Vis = ControlPanelInstance->GetVisibility();
    if (Vis == ESlateVisibility::Visible)
    {
        ControlPanelInstance->SetVisibility(ESlateVisibility::Collapsed);
        UE_LOG(LogTemp, Log, TEXT("[MainWidget] Control Panel collapsed."));
    }
    else
    {
        if (!ControlPanelInstance->IsInViewport())
        {
            if (APlayerController* PC = GetOwningPlayer())
            {
                ControlPanelInstance->AddToViewport(ControlPanelZOrder);
            }
        }
        ControlPanelInstance->SetVisibility(ESlateVisibility::Visible);

        // Refocus when reopening
        if (APlayerController* PC = GetOwningPlayer())
        {
            FInputModeGameAndUI Mode;
            Mode.SetWidgetToFocus(ControlPanelInstance->TakeWidget());
            Mode.SetHideCursorDuringCapture(false);
            Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
            PC->SetInputMode(Mode);
        }

        UE_LOG(LogTemp, Log, TEXT("[MainWidget] Control Panel shown."));
    }
}

void UMainWidget::ShowControlPanel(bool bShow)
{
    if (!ControlPanelInstance)
    {
        if (bShow)
        {
            ToggleControlPanel(); // create + show
        }
        return;
    }

    ControlPanelInstance->SetVisibility(bShow ? ESlateVisibility::Visible
                                              : ESlateVisibility::Collapsed);
    if (bShow)
    {
        if (APlayerController* PC = GetOwningPlayer())
        {
            FInputModeGameAndUI Mode;
            Mode.SetWidgetToFocus(ControlPanelInstance->TakeWidget());
            Mode.SetHideCursorDuringCapture(false);
            Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
            PC->SetInputMode(Mode);
        }
    }
}

UButton* UMainWidget::ResolveControlPanelButton()
{
    if (!ControlPanelButtonName.IsNone())
    {
        if (UWidget* Named = GetWidgetFromName(ControlPanelButtonName))
        {
            if (UButton* AsBtn = Cast<UButton>(Named))
            {
                return AsBtn;
            }
            if (UButton* Deep = FindFirstButtonDeep(Named))
            {
                return Deep;
            }
        }
    }

    // Fallback: search the whole tree for a button with "ControlPanel" in its name
    if (UWidget* Root = GetRootWidget())
    {
        TFunction<UButton*(UWidget*)> FindByString = [&](UWidget* W) -> UButton*
        {
            if (!W) return nullptr;
            if (UButton* B = Cast<UButton>(W))
            {
                if (W->GetName().Contains(TEXT("ControlPanel")))
                    return B;
            }
            if (UPanelWidget* Panel = Cast<UPanelWidget>(W))
            {
                for (int32 i = 0; i < Panel->GetChildrenCount(); ++i)
                    if (UButton* B = FindByString(Panel->GetChildAt(i))) return B;
            }
            if (UUserWidget* UW = Cast<UUserWidget>(W))
            {
                if (UWidget* Inner = UW->GetRootWidget())
                    if (UButton* B = FindByString(Inner)) return B;
            }
            if (UBorder* Border = Cast<UBorder>(W))
            {
                if (UWidget* C = Border->GetContent())
                    if (UButton* B = FindByString(C)) return B;
            }
            return nullptr;
        };
        if (UButton* B = FindByString(Root)) return B;
    }

    return nullptr;
}

UButton* UMainWidget::FindFirstButtonDeep(UWidget* Root) const
{
    if (!Root) return nullptr;
    if (UButton* B = Cast<UButton>(Root)) return B;

    if (UUserWidget* UW = Cast<UUserWidget>(Root))
    {
        if (UWidget* Inner = UW->GetRootWidget())
            if (UButton* B = FindFirstButtonDeep(Inner)) return B;
    }

    if (UPanelWidget* Panel = Cast<UPanelWidget>(Root))
    {
        for (int32 i = 0; i < Panel->GetChildrenCount(); ++i)
            if (UButton* B = FindFirstButtonDeep(Panel->GetChildAt(i))) return B;
    }

    if (UBorder* Border = Cast<UBorder>(Root))
    {
        if (UWidget* C = Border->GetContent())
            if (UButton* B = FindFirstButtonDeep(C)) return B;
    }

    return nullptr;
}

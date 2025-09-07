#include "IconButtonGeneral.h"
#include "Components/Button.h"

void UIconButtonGeneral::NativeConstruct()
{
	Super::NativeConstruct();

	if (Button)
	{
		// Cache current (BP) style as Default
		DefaultStyle = Button->WidgetStyle;

		// Build the "selected" style -> make Normal look like Hovered
		SelectedStyle = DefaultStyle;
		SelectedStyle.Normal   = DefaultStyle.Hovered;
		// (Optional) keep Pressed as-is so clicks still feel clicky
		// SelectedStyle.Pressed = DefaultStyle.Pressed;

		// Bind events
		Button->OnClicked.AddDynamic(this, &UIconButtonGeneral::HandleClicked);
		Button->OnHovered.AddDynamic(this, &UIconButtonGeneral::HandleHovered);
		Button->OnUnhovered.AddDynamic(this, &UIconButtonGeneral::HandleUnhovered);

		// Paint initial state
		ApplyCurrentStyle();
	}
}

void UIconButtonGeneral::SetSelected(bool bInSelected)
{
	if (bSelected == bInSelected) return;
	bSelected = bInSelected;
	ApplyCurrentStyle();
}

void UIconButtonGeneral::ApplyCurrentStyle()
{
	if (!Button) return;
	Button->SetStyle(bSelected ? SelectedStyle : DefaultStyle);
}

void UIconButtonGeneral::HandleClicked()
{
	// Don’t toggle here—ModeSelector is the source of truth.
	// Just broadcast; ModeSelector will call SetSelected() on the right one.
	OnPressed.Broadcast();
}

void UIconButtonGeneral::HandleHovered()
{
	// No-op; UE will temporarily show Hovered/Pressed based on current style.
}

void UIconButtonGeneral::HandleUnhovered()
{
	// No-op; selected state keeps the hovered look via SelectedStyle.
}

// SimImGuiStyle.h
#pragma once
#include "CoreMinimal.h"
#include "imgui.h"

struct FSimImGuiStyle
{
    // Global scaling
    float FontScale = 1.00f;   // 1.0 for crisp, non-scaled text
    float UiScale   = 1.00f;   // keep native sizes to avoid blur

    // Base palette (dark theme)
    ImVec4 WindowBg     = ImVec4(0.09f, 0.10f, 0.12f, 0.98f);
    ImVec4 ChildBg      = ImVec4(0.11f, 0.12f, 0.15f, 0.98f);
    ImVec4 PopupBg      = ImVec4(0.08f, 0.09f, 0.11f, 0.98f);
    ImVec4 Border       = ImVec4(0.20f, 0.22f, 0.26f, 1.00f);
    ImVec4 Text         = ImVec4(0.93f, 0.96f, 1.00f, 1.00f);

    // Generic widgets
    ImVec4 FrameBg      = ImVec4(0.16f, 0.18f, 0.22f, 1.00f);
    ImVec4 FrameHovered = ImVec4(0.22f, 0.24f, 0.28f, 1.00f);
    ImVec4 FrameActive  = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);

    // Default button palette (uniform dark blue gradient)
    ImVec4 Button       = ImVec4(0.18f, 0.24f, 0.35f, 1.00f); // base dark blue
    ImVec4 ButtonHover  = ImVec4(0.24f, 0.32f, 0.48f, 1.00f); // lighter on hover
    ImVec4 ButtonActive = ImVec4(0.16f, 0.28f, 0.46f, 1.00f); // vivid when pressed

    // Accents for taskbar buttons
    ImVec4 AccentReverse= ImVec4(0.95f, 0.60f, 0.15f, 1.00f); // orange
    ImVec4 AccentPlay   = ImVec4(0.20f, 0.80f, 0.35f, 1.00f); // green
    ImVec4 AccentFast   = ImVec4(0.23f, 0.47f, 0.90f, 1.00f); // blue
    ImVec4 AccentStep   = ImVec4(0.70f, 0.50f, 0.95f, 1.00f); // purple
    ImVec4 AccentReset  = ImVec4(0.90f, 0.25f, 0.30f, 1.00f); // red

    void Apply() const
    {
        ImGuiStyle& Style = ImGui::GetStyle();
        ImGuiIO&    IO    = ImGui::GetIO();

        // Idempotent scaling: avoid compounding ScaleAllSizes each frame.
        static float PrevUiScale = 1.0f;
        static bool  bScaledOnce = false;
        if (!bScaledOnce)
        {
            Style.ScaleAllSizes(UiScale);
            PrevUiScale = UiScale;
            bScaledOnce = true;
        }
        else if (FMath::Abs(UiScale - PrevUiScale) > 0.001f)
        {
            const float Ratio = UiScale / PrevUiScale;
            Style.ScaleAllSizes(Ratio);
            PrevUiScale = UiScale;
        }
        IO.FontGlobalScale = FontScale;

        auto& Col = Style.Colors;
        Col[ImGuiCol_Text]           = Text;
        Col[ImGuiCol_WindowBg]       = WindowBg;
        Col[ImGuiCol_ChildBg]        = ChildBg;
        Col[ImGuiCol_PopupBg]        = PopupBg;
        Col[ImGuiCol_Border]         = Border;

        Col[ImGuiCol_FrameBg]        = FrameBg;
        Col[ImGuiCol_FrameBgHovered] = FrameHovered;
        Col[ImGuiCol_FrameBgActive]  = FrameActive;

        Col[ImGuiCol_Button]         = Button;
        Col[ImGuiCol_ButtonHovered]  = ButtonHover;
        Col[ImGuiCol_ButtonActive]   = ButtonActive;

        // Subtle rounding and padding for a modern look
        Style.FrameRounding   = 5.f;
        Style.GrabRounding    = 5.f;
        Style.WindowBorderSize= 1.f;
        Style.FrameBorderSize = 0.f;
        Style.ItemSpacing     = ImVec2(7.f, 5.f);
        Style.FramePadding    = ImVec2(7.f, 4.f); // tighter buttons/controls
    }
};

#pragma once
#include "CoreMinimal.h"

class FSimImGuiLayer
{
public:
	void Register();
	void Unregister();

private:
	void Draw(); // bound to FImGuiDelegates::OnDraw

private:
	FDelegateHandle DrawHandle;
	bool bRegistered = false;
};

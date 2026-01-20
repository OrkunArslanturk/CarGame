// Copyright Epic Games, Inc. All Rights Reserved.

#include "CarTest2GameMode.h"
#include "CarTest2PlayerController.h"

ACarTest2GameMode::ACarTest2GameMode()
{
	PlayerControllerClass = ACarTest2PlayerController::StaticClass();
}

// Copyright Epic Games, Inc. All Rights Reserved.

#include "CarTest2WheelRear.h"
#include "UObject/ConstructorHelpers.h"

UCarTest2WheelRear::UCarTest2WheelRear()
{
	AxleType = EAxleType::Rear;
	bAffectedByHandbrake = true;
	bAffectedByEngine = true;
}
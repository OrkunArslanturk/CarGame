// Copyright Epic Games, Inc. All Rights Reserved.

#include "CarTest2WheelFront.h"
#include "UObject/ConstructorHelpers.h"

UCarTest2WheelFront::UCarTest2WheelFront()
{
	AxleType = EAxleType::Front;
	bAffectedBySteering = true;
	MaxSteerAngle = 40.f;
}
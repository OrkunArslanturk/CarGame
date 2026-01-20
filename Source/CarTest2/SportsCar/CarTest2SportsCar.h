// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CarTest2Pawn.h"
#include "CarTest2SportsCar.generated.h"

/**
 *  Sports car wheeled vehicle implementation
 */
UCLASS(abstract)
class ACarTest2SportsCar : public ACarTest2Pawn
{
	GENERATED_BODY()
	
public:

	ACarTest2SportsCar();
};

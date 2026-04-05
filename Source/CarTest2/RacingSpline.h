// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "RacingSpline.generated.h"

UCLASS()
class CARTEST2_API ARacingSpline : public AActor
{
	GENERATED_BODY()
public:

	ARacingSpline();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	USplineComponent* Spline;
};

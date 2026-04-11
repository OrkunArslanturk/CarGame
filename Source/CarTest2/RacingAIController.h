// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "RacingAIController.generated.h"

class AAI_ArcadeCar;
class ARacingSpline;
/**
 * 
 */
UCLASS()
class CARTEST2_API ARacingAIController : public AAIController
{
	GENERATED_BODY()
	
public:

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void OnPossess(APawn* InPawn) override;
	
	FVector SmoothedTargetLocation;
protected:

	UPROPERTY()
	AAI_ArcadeCar* Car;

	UPROPERTY()
	ARacingSpline* RacingSpline;

	UPROPERTY(EditAnywhere)
	float LookAheadDistance = 1200.f;
	
	UPROPERTY()
	TArray<AAI_ArcadeCar*> AllCars;

	float CurrentLaneOffset = 0.f;
	float TargetLaneOffset = 0.f;
	
	float StuckTime = 0.f;
	FVector LastLocation;

	FVector LastSafeLocation;
	FRotator LastSafeRotation;
	float SafeUpdateTimer = 0.f;
	
};
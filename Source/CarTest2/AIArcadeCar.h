// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "AIArcadeCar.generated.h"

/**
 * 
 */
UCLASS()
class CARTEST2_API AAIArcadeCar : public AWheeledVehiclePawn
{
	GENERATED_BODY()
public:

	AAIArcadeCar();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	void SetThrottle(float Value);
	void SetSteer(float Value);
	void SetBrake(float Value);

protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI Car")
	float EnginePower = 520.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI Car")
	float MaxRPM = 5500.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI Car")
	float MaxSpeedKMH = 210.f;

};
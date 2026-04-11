// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ArcadeCar/ArcadeCar.h"
#include "AI_ArcadeCar.generated.h"

/**
 * 
 */
UCLASS()
class CARTEST2_API AAI_ArcadeCar : public AArcadeCar
{
	GENERATED_BODY()
	
public:

	AAI_ArcadeCar();
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	
	void SetThrottle(float Value);
	void SetSteer(float Value);
	void SetBrake(float Value);
	float GetCurrentSteer() const { return CurrentSteer; }
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI")
	float LaneOffset = 0.f;
private:

	//  Smooth steering 
	float CurrentSteer = 0.f;
	float AIThrottle = 0.f;
	float AIBrake = 0.f;
};
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ArcadeCar/ArcadeCar.h"
#include "AI_ArcadeCar.generated.h"

UENUM(BlueprintType)
enum class EAILevel : uint8
{
	Normal UMETA(DisplayName="Normal"),
	VeryGood UMETA(DisplayName="VeryGood")
};

UCLASS()
class CARTEST2_API AAI_ArcadeCar : public AArcadeCar
{
	GENERATED_BODY()
	
public:

	AAI_ArcadeCar();
	virtual void Tick(float DeltaTime) override;
	virtual void BeginPlay() override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	
	void SetThrottle(float Value);
	void SetSteer(float Value);
	void SetBrake(float Value);
	float GetCurrentSteer() const { return CurrentSteer; }
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI")
	float LaneOffset = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI")
	EAILevel AILevel = EAILevel::Normal;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI")
	float Skill = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI")
	float Aggression = 1.0f;
private:

	float CurrentSteer = 0.f;
	float AIThrottle = 0.f;
	float AIBrake = 0.f;
};
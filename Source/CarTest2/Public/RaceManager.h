// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Blueprint/UserWidget.h"
#include "RaceManager.generated.h"

class AArcadeCar;
class ARaceCheckpoint;

UCLASS()
class CARTEST2_API ARaceManager : public AActor
{
	GENERATED_BODY()

public:
	ARaceManager();
	
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	UPROPERTY()
	TArray<AArcadeCar*> Cars;
	
	FTimerHandle InitTimer;
	void InitCars();

	void UpdateRanking();
	
	UPROPERTY(EditAnywhere)
	TSubclassOf<UUserWidget> RaceHUDClass;

	UUserWidget* RaceHUD;

	FString BuildRankingText();
};
// RacingGameMode.h
// Game mode for arcade racing
// For AGP Racing Game Assignment - Futuregames 2026

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "RacingGameMode.generated.h"

UCLASS()
class CARTEST2_API ARacingGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ARacingGameMode();

	virtual void BeginPlay() override;

	// ==================== RACE STATE ====================

	UPROPERTY(BlueprintReadOnly, Category = "Race")
	bool bRaceStarted = false;

	UPROPERTY(BlueprintReadOnly, Category = "Race")
	float RaceTime = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Race")
	int32 TotalLaps = 3;

	// ==================== FUNCTIONS ====================

	UFUNCTION(BlueprintCallable, Category = "Race")
	void StartRace();

	UFUNCTION(BlueprintCallable, Category = "Race")
	void EndRace();

	UFUNCTION(BlueprintCallable, Category = "Race")
	FString GetFormattedRaceTime() const;
};
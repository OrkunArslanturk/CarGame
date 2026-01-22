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
    virtual void Tick(float DeltaTime) override;

    // --- Race State ---

    UPROPERTY(BlueprintReadOnly, Category = "Race")
    bool bRaceActive = false;

    UPROPERTY(BlueprintReadOnly, Category = "Race")
    float RaceTime = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Race")
    int32 TotalLaps = 3;

    // --- Functions ---

    UFUNCTION(BlueprintCallable, Category = "Race")
    void StartRace();

    UFUNCTION(BlueprintCallable, Category = "Race")
    void StopRace();

    UFUNCTION(BlueprintPure, Category = "Race")
    FString GetFormattedTime() const;
};
#include "RacingGameMode.h"
#include "RacingPlayerController.h"
#include "ArcadeCar.h"

ARacingGameMode::ARacingGameMode()
{
    DefaultPawnClass = AArcadeCar::StaticClass();
    PlayerControllerClass = ARacingPlayerController::StaticClass();
    PrimaryActorTick.bCanEverTick = true;
}

void ARacingGameMode::BeginPlay()
{
    Super::BeginPlay();
}

void ARacingGameMode::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bRaceActive)
    {
        RaceTime += DeltaTime;
    }
}

void ARacingGameMode::StartRace()
{
    bRaceActive = true;
    RaceTime = 0.f;
}

void ARacingGameMode::StopRace()
{
    bRaceActive = false;
}

FString ARacingGameMode::GetFormattedTime() const
{
    int32 Minutes = FMath::FloorToInt(RaceTime / 60.f);
    int32 Seconds = FMath::FloorToInt(FMath::Fmod(RaceTime, 60.f));
    int32 Millis = FMath::FloorToInt(FMath::Fmod(RaceTime * 1000.f, 1000.f));
    
    return FString::Printf(TEXT("%02d:%02d.%03d"), Minutes, Seconds, Millis);
}
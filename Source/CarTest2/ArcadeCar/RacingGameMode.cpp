// RacingGameMode.cpp
// Game mode implementation for arcade racing
// For AGP Racing Game Assignment - Futuregames 2026

#include "RacingGameMode.h"
#include "RacingPlayerController.h"
#include "ArcadeCar.h"

ARacingGameMode::ARacingGameMode()
{
	// Set default classes
	DefaultPawnClass = AArcadeCar::StaticClass();
	PlayerControllerClass = ARacingPlayerController::StaticClass();

	PrimaryActorTick.bCanEverTick = true;
}

void ARacingGameMode::BeginPlay()
{
	Super::BeginPlay();

	UE_LOG(LogTemp, Log, TEXT("RacingGameMode started"));
	UE_LOG(LogTemp, Log, TEXT("DefaultPawn: %s"), *DefaultPawnClass->GetName());
	UE_LOG(LogTemp, Log, TEXT("PlayerController: %s"), *PlayerControllerClass->GetName());
}

void ARacingGameMode::StartRace()
{
	bRaceStarted = true;
	RaceTime = 0.f;

	UE_LOG(LogTemp, Log, TEXT("Race Started!"));
}

void ARacingGameMode::EndRace()
{
	bRaceStarted = false;

	UE_LOG(LogTemp, Log, TEXT("Race Ended! Time: %s"), *GetFormattedRaceTime());
}

FString ARacingGameMode::GetFormattedRaceTime() const
{
	int32 Minutes = FMath::FloorToInt(RaceTime / 60.f);
	int32 Seconds = FMath::FloorToInt(FMath::Fmod(RaceTime, 60.f));
	int32 Milliseconds = FMath::FloorToInt(FMath::Fmod(RaceTime * 1000.f, 1000.f));

	return FString::Printf(TEXT("%02d:%02d.%03d"), Minutes, Seconds, Milliseconds);
}
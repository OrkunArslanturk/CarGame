// Fill out your copyright notice in the Description page of Project Settings.

#include "RaceCheckpoint.h"
#include "Components/BoxComponent.h"
#include "Kismet/GameplayStatics.h"
#include "RaceManager.h"
#include "ArcadeCar/ArcadeCar.h"

ARaceCheckpoint::ARaceCheckpoint()
{
	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	RootComponent = Trigger;

	Trigger->SetCollisionProfileName(TEXT("Trigger"));
	Trigger->OnComponentBeginOverlap.AddDynamic(this, &ARaceCheckpoint::OnOverlap);
}

void ARaceCheckpoint::OnOverlap(
	UPrimitiveComponent* OverlappedComp,
	AActor* OtherActor,
	UPrimitiveComponent* OtherComp,
	int32 OtherBodyIndex,
	bool bFromSweep,
	const FHitResult& SweepResult)
{
	AArcadeCar* Car = Cast<AArcadeCar>(OtherActor);
	if (!Car) return;

	int PrevCheckpoint = Car->CurrentCheckpoint;
	int NextCheckpoint = PrevCheckpoint + 1;

	if (NextCheckpoint >= Car->TotalCheckpoints)
	{
		NextCheckpoint = 0;
	}

	if (CheckpointIndex == NextCheckpoint)
	{
		Car->CurrentCheckpoint = CheckpointIndex;

		// Race start
		if (CheckpointIndex == 0 && Car->IsPlayerControlled())
		{
			if (ARaceManager* RM = Cast<ARaceManager>(
				UGameplayStatics::GetActorOfClass(GetWorld(), ARaceManager::StaticClass())))
			{
				RM->bRaceStarted = true;

				// UE_LOG(LogTemp, Warning, TEXT("Race start"));
			}
		}

		// Lap complete
		if (PrevCheckpoint == Car->TotalCheckpoints - 1 && CheckpointIndex == 0)
		{
			Car->OnLapCompleted();

			// UE_LOG(LogTemp, Warning, TEXT("Lap: %d"), Car->CurrentLap);
		}
	}
}
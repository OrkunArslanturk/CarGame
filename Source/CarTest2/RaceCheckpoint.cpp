// Fill out your copyright notice in the Description page of Project Settings.

#include "RaceCheckpoint.h"
#include "Components/BoxComponent.h"
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
	UE_LOG(LogTemp, Warning, TEXT("Total CP: %d"), Car->TotalCheckpoints);
	UE_LOG(LogTemp, Warning, TEXT("IsPlayer: %d | %s"),
	Car->IsPlayerControlled(),
	*Car->DriverName);
	int PrevCheckpoint = Car->CurrentCheckpoint;
	int NextCheckpoint = PrevCheckpoint + 1;

	if (NextCheckpoint >= Car->TotalCheckpoints)
	{
		NextCheckpoint = 0;
	}

	if (CheckpointIndex == NextCheckpoint)
	{
		Car->CurrentCheckpoint = CheckpointIndex;

		// Lap complete
		if (PrevCheckpoint == Car->TotalCheckpoints - 1 && CheckpointIndex == 0)
		{
			Car->OnLapCompleted();

			UE_LOG(LogTemp, Warning, TEXT("LAP: %d"), Car->CurrentLap);
		}
	}
}
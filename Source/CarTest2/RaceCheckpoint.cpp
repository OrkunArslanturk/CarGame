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

	// Correct checkpoint or not
	if (Car->CurrentCheckpoint == CheckpointIndex)
	{
		Car->CurrentCheckpoint++;

		// Lap finish or not
		if (Car->CurrentCheckpoint >= Car->TotalCheckpoints)
		{
			Car->CurrentCheckpoint = 0;
			Car->OnLapCompleted();
		}
	}
}
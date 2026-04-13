#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RaceCheckpoint.generated.h"

class UBoxComponent;

UCLASS()
class CARTEST2_API ARaceCheckpoint : public AActor
{
	GENERATED_BODY()

public:
	ARaceCheckpoint();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Race")
	int CheckpointIndex;

protected:
	UPROPERTY(VisibleAnywhere)
	UBoxComponent* Trigger;

	UFUNCTION()
	void OnOverlap(
		UPrimitiveComponent* OverlappedComp,
		AActor* OtherActor,
		UPrimitiveComponent* OtherComp,
		int32 OtherBodyIndex,
		bool bFromSweep,
		const FHitResult& SweepResult
	);
};

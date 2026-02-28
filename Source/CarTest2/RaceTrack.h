#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RaceTrack.generated.h"

class USplineComponent;

UCLASS()
class YOURGAME_API ARaceTrack : public AActor
{
	GENERATED_BODY()

public:
	ARaceTrack();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USplineComponent* TrackSpline;
};
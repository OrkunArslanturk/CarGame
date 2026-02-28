#include "RaceTrack.h"
#include "Components/SplineComponent.h"

ARaceTrack::ARaceTrack()
{
	PrimaryActorTick.bCanEverTick = false;

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

	TrackSpline = CreateDefaultSubobject<USplineComponent>(TEXT("TrackSpline"));
	TrackSpline->SetupAttachment(RootComponent);

	TrackSpline->SetClosedLoop(true);
}
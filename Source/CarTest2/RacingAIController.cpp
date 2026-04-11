#include "RacingAIController.h"
#include "AI_ArcadeCar.h"
#include "RacingSpline.h"
#include "Kismet/GameplayStatics.h"
#include "Components/SplineComponent.h"

void ARacingAIController::BeginPlay()
{
	Super::BeginPlay();

	RacingSpline = Cast<ARacingSpline>(
		UGameplayStatics::GetActorOfClass(GetWorld(), ARacingSpline::StaticClass())
	);

	TArray<AActor*> FoundCars;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAI_ArcadeCar::StaticClass(), FoundCars);

	for (AActor* Actor : FoundCars)
	{
		AAI_ArcadeCar* Other = Cast<AAI_ArcadeCar>(Actor);
		if (Other && Other != Car)
		{
			AllCars.Add(Other);
		}
	}

	// başlangıç lane
	CurrentLaneOffset = Car->LaneOffset;
	TargetLaneOffset = Car->LaneOffset;
}

void ARacingAIController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);

	Car = Cast<AAI_ArcadeCar>(InPawn);
}

void ARacingAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Car || !RacingSpline) return;

	FVector CarLocation = Car->GetActorLocation();
	FVector Forward = Car->GetActorForwardVector();
	FVector Right = Car->GetActorRightVector();


	// --- CLOSEST POINT ---
	float ClosestKey =
		RacingSpline->Spline->FindInputKeyClosestToWorldLocation(CarLocation);

	float ClosestDistance =
		RacingSpline->Spline->GetDistanceAlongSplineAtSplineInputKey(ClosestKey);

	AAI_ArcadeCar* BlockingCar = nullptr;
	float ClosestDist = 999999.f;

	for (AAI_ArcadeCar* Other : AllCars)
	{
		FVector ToOther = Other->GetActorLocation() - CarLocation;

		float ForwardDot = FVector::DotProduct(Forward, ToOther.GetSafeNormal());

		if (ForwardDot > 0.5f) // önünde mi?
		{
			float Dist = ToOther.Size();

			if (Dist < 800.f && Dist < ClosestDist)
			{
				ClosestDist = Dist;
				BlockingCar = Other;
			}
		}
	}
	if (BlockingCar)
	{
		float Chance = (Car->AILevel == EAILevel::VeryGood) ? 0.9f : 0.3f;

		if (FMath::FRand() < Chance)
		{
			float Side =
				(FVector::DotProduct(Right, BlockingCar->GetActorLocation() - CarLocation) > 0)
					? -1.f
					: 1.f;

			TargetLaneOffset = Car->LaneOffset + (Side * 300.f);
		}
	}
	else
	{
		TargetLaneOffset = Car->LaneOffset;
	}

	float LaneSpeed = (Car->AILevel == EAILevel::VeryGood) ? 4.0f : 2.0f;

	CurrentLaneOffset = FMath::FInterpTo(
		CurrentLaneOffset,
		TargetLaneOffset,
		DeltaTime,
		LaneSpeed
	);

	float Ahead1 = ClosestDistance + 300.f;
	float Ahead2 = ClosestDistance + 900.f;

	FVector P1 = RacingSpline->Spline->GetLocationAtDistanceAlongSpline(Ahead1, ESplineCoordinateSpace::World);
	FVector P2 = RacingSpline->Spline->GetLocationAtDistanceAlongSpline(Ahead2, ESplineCoordinateSpace::World);

	FVector Dir1 = (P1 - CarLocation).GetSafeNormal();
	FVector Dir2 = (P2 - P1).GetSafeNormal();

	float CornerSign = FVector::CrossProduct(Dir1, Dir2).Z;

	float ApexOffset = 0.f;

	float CornerStrength = FMath::Abs(CornerSign);

	// If there is cornewr
	if (CornerStrength > 0.2f)
	{
		ApexOffset = CornerSign * 200.f; // left-right offset
	}


	// SPEED
	float Speed = Car->GetVelocity().Size();
	float SpeedKMH = Speed * 0.036f;

	// LOOKAHEAD
	float LookAhead =
		FMath::GetMappedRangeValueClamped(
			FVector2D(0.f, 4000.f),
			FVector2D(800.f, 2000.f),
			Speed
		);

	float TargetDistance = ClosestDistance + LookAhead;

	FVector TargetLocation =
		RacingSpline->Spline->GetLocationAtDistanceAlongSpline(
			TargetDistance,
			ESplineCoordinateSpace::World
		);

	FVector SplineRight =
		RacingSpline->Spline->GetRightVectorAtDistanceAlongSpline(
			TargetDistance,
			ESplineCoordinateSpace::World
		);

	float FinalOffset = CurrentLaneOffset + ApexOffset;
	TargetLocation += SplineRight * FinalOffset;

	// SMOOTH 
	SmoothedTargetLocation = FMath::VInterpTo(
		SmoothedTargetLocation,
		TargetLocation,
		DeltaTime,
		6.0f
	);

	FVector ToTarget = (SmoothedTargetLocation - CarLocation).GetSafeNormal();

	// STEERING
	float SteerTarget = FVector::DotProduct(Right, ToTarget);

	// skill effect
	SteerTarget *= Car->Skill;
	// clamp
	SteerTarget = FMath::Clamp(SteerTarget, -1.f, 1.f);

	// DEADZONE
	if (FMath::Abs(SteerTarget) < 0.03f)
	{
		SteerTarget = 0.f;
	}

	// LOW SPEED FIX
	if (SpeedKMH < 30.f)
	{
		SteerTarget *= 0.6f;
	}

	Car->SetSteer(SteerTarget);

	// --- THROTTLE ---
	float TurnStrength = FMath::Abs(SteerTarget);

	float Throttle = 1.f;

	if (TurnStrength > 0.6f)
	{
		Throttle = FMath::Lerp(0.3f, 0.5f, Car->Skill);
	}
	else if (TurnStrength > 0.3f)
	{
		Throttle = FMath::Lerp(0.6f, 0.8f, Car->Skill);
	}

	if (SpeedKMH > 180.f)
	{
		Throttle = 0.5f;
	}

	Car->SetThrottle(Throttle);
	Car->SetBrake(0.f);
}

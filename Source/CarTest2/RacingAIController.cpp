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

	// --- SPLINE CLOSEST POINT ---
	float ClosestKey =
		RacingSpline->Spline->FindInputKeyClosestToWorldLocation(CarLocation);

	float ClosestDistance =
		RacingSpline->Spline->GetDistanceAlongSplineAtSplineInputKey(ClosestKey);

	// --- SPEED ---
	float Speed = Car->GetVelocity().Size();

	// --- BASE LOOKAHEAD ---
	float BaseLookAhead =
		FMath::GetMappedRangeValueClamped(
			FVector2D(0.f, 4000.f),
			FVector2D(600.f, 1800.f),
			Speed
		);

	// --- TARGET POINT ---
	float TargetDistance = ClosestDistance + BaseLookAhead;

	FVector TargetLocation =
		RacingSpline->Spline->GetLocationAtDistanceAlongSpline(
			TargetDistance,
			ESplineCoordinateSpace::World
		);

	FVector ToTarget = (TargetLocation - CarLocation).GetSafeNormal();

	// --- STEERING CALCULATION ---
	float Dot = FVector::DotProduct(Forward, ToTarget);
	float Cross = FVector::CrossProduct(Forward, ToTarget).Z;

	float Angle = FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)); // radians
	float SignedAngle = Angle * FMath::Sign(Cross);

	// Normalize to [-1,1]
	float SteerTarget = FMath::Clamp(SignedAngle * 0.8f, -1.f, 1.f);

	// --- TURN STRENGTH ---
	float TurnStrength = FMath::Abs(SteerTarget);

	// --- DYNAMIC LOOKAHEAD (virajda azalt) ---
	float DynamicLookAhead = FMath::Lerp(BaseLookAhead, BaseLookAhead * 0.5f, TurnStrength);

	TargetDistance = ClosestDistance + DynamicLookAhead;

	TargetLocation =
		RacingSpline->Spline->GetLocationAtDistanceAlongSpline(
			TargetDistance,
			ESplineCoordinateSpace::World
		);

	ToTarget = (TargetLocation - CarLocation).GetSafeNormal();

	// Recalculate steering with new target
	Dot = FVector::DotProduct(Forward, ToTarget);
	Cross = FVector::CrossProduct(Forward, ToTarget).Z;

	Angle = FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f));
	SignedAngle = Angle * FMath::Sign(Cross);

	SteerTarget = FMath::Clamp(SignedAngle * 1.5f, -1.f, 1.f);

	// --- FINAL STEER (SMOOTHED INSIDE CAR) ---
	Car->SetSteer(SteerTarget);

	// --- THROTTLE + BRAKE LOGIC ---
	
	float Throttle = 1.f;
	float Brake = 0.f;

	if (Throttle > 0.1f)
	{
		Brake = 0.f;
	}
	
	if (TurnStrength > 0.7f)
	{
		Throttle = 0.2f;
		// Brake = 0.4f;
	}
	else if (TurnStrength > 0.4f)
	{
		Throttle = 0.5f;
	}
	else
	{
		Throttle = 1.f;
	}

	// Speed limiter (çok hızlandıysa gaz kes)
	float SpeedKMH = Speed * 0.036f;
	if (SpeedKMH < 15.f)
	{
		Throttle = 1.0f; // full gazla kaldır
	}
	if (SpeedKMH > 180.f)
	{
		Throttle = 0.3f;
	}

	Car->SetThrottle(Throttle);
	Car->SetBrake(Brake);
}
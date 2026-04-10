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
    FVector Right = Car->GetActorRightVector();

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

    // --- TARGET POINT (FIRST PASS) ---
    float TargetDistance = ClosestDistance + BaseLookAhead;

    FVector TargetLocation =
        RacingSpline->Spline->GetLocationAtDistanceAlongSpline(
            TargetDistance,
            ESplineCoordinateSpace::World
        );

    // -------------------------------
    // SMOOTH TARGET
    // -------------------------------
    SmoothedTargetLocation = FMath::VInterpTo(
        SmoothedTargetLocation,
        TargetLocation,
        DeltaTime,
        5.0f
    );

    FVector ToTarget = (SmoothedTargetLocation - CarLocation).GetSafeNormal();

    // --- STEERING (STABLE VERSION) ---
    float SteerTarget = FVector::DotProduct(Right, ToTarget);
    SteerTarget = FMath::Clamp(SteerTarget, -1.f, 1.f);

    // --- TURN STRENGTH ---
    float TurnStrength = FMath::Abs(SteerTarget);

    // --- DYNAMIC LOOKAHEAD ---
    float DynamicLookAhead = FMath::Lerp(BaseLookAhead, BaseLookAhead * 0.5f, TurnStrength);

    TargetDistance = ClosestDistance + DynamicLookAhead;

    TargetLocation =
        RacingSpline->Spline->GetLocationAtDistanceAlongSpline(
            TargetDistance,
            ESplineCoordinateSpace::World
        );

    // AGAIN SMOOTH
    SmoothedTargetLocation = FMath::VInterpTo(
        SmoothedTargetLocation,
        TargetLocation,
        DeltaTime,
        5.0f
    );

    ToTarget = (SmoothedTargetLocation - CarLocation).GetSafeNormal();

    // --- FINAL STEERING ---
    SteerTarget = FVector::DotProduct(Right, ToTarget);
    SteerTarget = FMath::Clamp(SteerTarget, -1.f, 1.f);

    // DEADZONE (zigzag fix)
    if (FMath::Abs(SteerTarget) < 0.05f)
    {
        SteerTarget = 0.f;
    }

    // LOW SPEED FIX
    float SpeedKMH = Speed * 0.036f;
    if (SpeedKMH < 30.f)
    {
        SteerTarget *= 0.5f;
    }

    Car->SetSteer(SteerTarget);

    // --- THROTTLE ---
    float Throttle = 1.f;
    float Brake = 0.f;

    if (TurnStrength > 0.7f)
    {
        Throttle = 0.2f;
    }
    else if (TurnStrength > 0.4f)
    {
        Throttle = 0.5f;
    }
    else
    {
        Throttle = 1.f;
    }

    if (SpeedKMH < 15.f)
    {
        Throttle = 1.0f;
    }

    if (SpeedKMH > 180.f)
    {
        Throttle = 0.3f;
    }

    Car->SetThrottle(Throttle);
    Car->SetBrake(Brake);
}
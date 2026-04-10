// Fill out your copyright notice in the Description page of Project Settings.


#include "AI_ArcadeCar.h"
#include "RacingAIController.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Camera/CameraComponent.h"

AAI_ArcadeCar::AAI_ArcadeCar()
{
	PrimaryActorTick.bCanEverTick = true;

	AIControllerClass = ARacingAIController::StaticClass();
	AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
	
	if (Camera)
	{
		Camera->Deactivate();
		Camera->SetAutoActivate(false);
	}
}

void AAI_ArcadeCar::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
	if (!Vehicle) return;

	// Speed update (base class sync=
	SpeedKMH = Vehicle->GetForwardSpeed() * 0.036f;

	float FinalThrottle = AIThrottle;
	float FinalBrake = 0.f;

	// Launch boost
	if (SpeedKMH < 5.f && AIThrottle > 0.1f)
	{
		FinalThrottle = 1.0f;
	}

	SetAIInputs(FinalThrottle, FinalBrake, CurrentSteer);
}

void AAI_ArcadeCar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	// no input
}

void AAI_ArcadeCar::SetThrottle(float Value)
{
	AIThrottle = Value;
}

void AAI_ArcadeCar::SetSteer(float Value)
{
	if (auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent()))
	{
		// SMOOTH STEERING
		CurrentSteer = FMath::FInterpTo(CurrentSteer, Value, GetWorld()->GetDeltaSeconds(), 4.0f);

	}
}


void AAI_ArcadeCar::SetBrake(float Value)
{
	AIBrake = Value;
}

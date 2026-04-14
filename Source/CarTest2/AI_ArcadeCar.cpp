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
	float FinalBrake = AIBrake;

	// Launch boost
	if (SpeedKMH < 5.f && AIThrottle > 0.1f)
	{
		FinalThrottle = 1.0f;
	}

	SetAIInputs(FinalThrottle, FinalBrake, CurrentSteer);
	
	SetBrake(0.f);
}

void AAI_ArcadeCar::BeginPlay()
{
	Super::BeginPlay();

	switch (AILevel)
	{
	case EAILevel::Normal:
		Skill = 1.2f;
		// Aggression = 0.8f;
		break;

	case EAILevel::VeryGood:
		Skill = 1.8f;
		// Aggression = 1.5f;
		break;
	}
	
	auto* Vehicle = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	if (Vehicle)
	{
		if (AILevel == EAILevel::Normal)
		{
			Vehicle->EngineSetup.MaxTorque *= 1.2f;
			Vehicle->EngineSetup.MaxRPM *= 1.1f;
		}
		else if (AILevel == EAILevel::VeryGood)
		{
			Vehicle->EngineSetup.MaxTorque *= 1.5f;
			Vehicle->EngineSetup.MaxRPM *= 1.3f;
		}
	}
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
		float InterpSpeed = FMath::Lerp(3.0f, 8.0f, Skill - 0.8f);

		CurrentSteer = FMath::FInterpTo(CurrentSteer, Value, GetWorld()->GetDeltaSeconds(), InterpSpeed);
	}
}


void AAI_ArcadeCar::SetBrake(float Value)
{
	AIBrake = Value;
}
